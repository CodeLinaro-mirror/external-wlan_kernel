/*
 * Copyright (c) 2019 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* add additional information to our printk's */
#define pr_fmt(fmt) "%s: " fmt "\n", __func__

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <linux/platform_device.h>
#include <linux/ratelimit.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/cache.h>
#include <linux/qcn_sdio_al.h>

#define DRIVER_DESC	"QCN sdio bridge driver"

#define SDIO_RX_BYTE_COUNT		0x20
#define SDIO_RX_BYTE_COUNT_TRANS	0x21

#define RX_BUFF_SIZE			0x4000
#define TX_BUFF_SIZE			0x4000

#define SDIO_TX_BUFF_SZ_EVENT		0x20
#define SDIO_TX_BUFF_SZ_TRANS_EVENT	0x21

struct data_pkt {
	int n_read;
	char *buf;
	size_t len;
};


#define FILE_OPENED		BIT(0)

#define MAX_DATA_PKT_SIZE       16384

struct sdio_bridge {
	char *name;
	spinlock_t lock;
	dev_t cdev_start_no;
	struct cdev cdev;
	struct class *class;
	struct device *device;
	wait_queue_head_t sdb_wait_q;
	unsigned long flags;
};

static struct sdio_bridge *__sdb;
static struct sdio_al_channel_data *channel_data;
static struct sdio_al_channel_handle *channel_handle;
static struct sdio_al_client_handle *client_handle;
static struct data_pkt	*rx_pkt, *tx_pkt;
static int data_available;
static int data_remaining;
static u8 *rx_dma_buff;
static u8 *tx_dma_buff;

static void
dbg_log_event(struct sdio_bridge *sdb, char *event, int d1, int d2) {}

	static
struct data_pkt *sdb_alloc_data_pkt(size_t count, gfp_t flags)
{
	struct data_pkt *pkt;

	pkt = kzalloc(sizeof(struct data_pkt), flags);
	if (!pkt)
		return ERR_PTR(-ENOMEM);

	pkt->buf = kzalloc(count, flags);
	if (!pkt->buf) {
		kfree(pkt);
		return ERR_PTR(-ENOMEM);
	}

	pkt->len = count;

	return pkt;
}

static void sdb_free_data_pkt(struct data_pkt *pkt)
{
	kfree(pkt->buf);
	kfree(pkt);
}

static ssize_t sdb_fs_read(struct file *fp, char __user *buf,
		size_t count, loff_t *pos)
{
	struct sdio_bridge *sdb = __sdb;
	int ret = 0;
	int padded_len = 0;

read_start:
	if (!data_available) {
		ret = wait_event_interruptible(sdb->sdb_wait_q,
				data_available);
		if (ret < 0)
			return ret;
		goto read_start;
	}

	if (!data_remaining) {
		if (count > data_available)
			return -EINVAL;

		if (data_available % 4)
			padded_len = (((data_available / 4) + 1) * 4);
		else
			padded_len = data_available;
		ret = sdio_al_queue_transfer(channel_handle,
				SDIO_AL_RX, rx_dma_buff, padded_len, 0);
		if (!ret) {
			pr_err("SDIO: Read Success\n");
			ret = copy_to_user(buf, rx_dma_buff, count);
			if (ret)
				pr_err("SDIO: copy Fail Res:%d\n", ret);
			else
				data_remaining = data_available - count;
		} else
			pr_err("SDIO: Read Fail Res:%d\n", ret);
	} else {
		if (count > data_remaining)
			return -EINVAL;

		ret = copy_to_user(buf, rx_dma_buff +
				(data_available - data_remaining), count);
		if (ret)
			pr_err("SDIO: copy Fail Res:%d\n", ret);
		else {
			pr_err("SDIO: Read Success\n");
			data_remaining -= count;
			if (!data_remaining)
				data_available = 0;
		}
	}

	return count;
}

static ssize_t sdb_fs_write(struct file *fp, const char __user *buf,
		size_t count, loff_t *pos)
{
	int ret = 0;
	int padded_len = 0;
	unsigned int mdata = 0;

	if (count % 4)
		padded_len = (((count / 4) + 1) * 4);
	else
		padded_len = count;

	ret = copy_from_user(tx_dma_buff, buf, count);
	if (ret)
		return ret;

	mdata = ((SDIO_TX_BUFF_SZ_EVENT << 24) | (count & 0x3FFF));
	ret = sdio_al_meta_transfer(channel_handle, mdata);
	if (ret) {
		pr_err("%s: meta data transfer failed:%d\n", __func__, ret);
		return ret;
	}

	pr_err("%s: meta data transfer success\n", __func__);

	ret = sdio_al_queue_transfer(channel_handle,
			SDIO_AL_TX, tx_dma_buff, padded_len, 0);
	if (!ret)
		pr_err("SDIO: Write Success");
	else
		pr_err("SDIO: Write Fail Result:%d\n", ret);

	return count;
}

static int sdb_fs_open(struct inode *ip, struct file *fp)
{
	struct sdio_bridge *sdb =
		container_of(ip->i_cdev, struct sdio_bridge, cdev);

	if (IS_ERR(sdb)) {
		pr_err("SDIO: sdb device not found");
		return -ENODEV;
	}

	fp->private_data = sdb;
	set_bit(FILE_OPENED, &sdb->flags);

	return 0;
}

static unsigned int sdb_fs_poll(struct file *file, poll_table *wait)
{
	struct sdio_bridge	*sdb = file->private_data;
	int			ret = 0;

	while (!data_available)
		poll_wait(file, &sdb->sdb_wait_q, wait);

	if (data_available)
		ret = POLLIN | POLLRDNORM;

	return ret;
}

static int sdb_fs_release(struct inode *ip, struct file *fp)
{
	struct sdio_bridge	*sdb = fp->private_data;

	dbg_log_event(sdb, "FS-RELEASE", 0, 0);

	clear_bit(FILE_OPENED, &sdb->flags);
	fp->private_data = NULL;

	return 0;
}

static const struct file_operations sdb_fops = {
	.owner = THIS_MODULE,
	.read = sdb_fs_read,
	.write = sdb_fs_write,
	.open = sdb_fs_open,
	.release = sdb_fs_release,
	.poll = sdb_fs_poll,
};

void sdio_dl_meta_data_cb(struct sdio_al_client_handle *cl_handle,
							unsigned int data)
{
	struct sdio_bridge *sdb = __sdb;
	u8 event = 0;

	event = (u8)((data & 0xFF000000) >> 24);

	switch (event) {
	case SDIO_RX_BYTE_COUNT:
		data_available = (data & 0x00003FFF);
		if (data_available)
			wake_up(&sdb->sdb_wait_q);
		break;
	case SDIO_RX_BYTE_COUNT_TRANS:
		break;
	default:
		break;
	}
}

static int sdio_probe(struct sdio_al_client_handle *client_handle)
{
	struct sdio_bridge		*sdb;
	int				ret = 0;
	u8				*rx_buff = NULL;
	u8				*tx_buff = NULL;

	channel_data = kzalloc(sizeof(struct sdio_al_channel_data), GFP_KERNEL);
	if (!channel_data)
		return -ENOMEM;

	channel_data->name = "SDIO_AL_TTY_CH0";
	channel_data->client_data = client_handle->client_data;
	channel_data->dl_meta_data_cb = sdio_dl_meta_data_cb;
	channel_data->dl_data_avail_cb = NULL;
	channel_data->ul_xfer_cb = NULL;
	channel_data->dl_xfer_cb = NULL;

	channel_handle = sdio_al_register_channel(client_handle, channel_data);
	if (!channel_handle) {
		pr_err("SDIO: channel not registered");
		sdio_al_deregister_client(client_handle);
		return -EINVAL;
	}

	sdb = kzalloc(sizeof(struct sdio_bridge), GFP_KERNEL);
	if (!sdb) {
		ret =  -ENOMEM;
		goto dev_free;
	}

	__sdb = sdb;

	sdb->name = kasprintf(GFP_KERNEL, "qcn_sdio");
	if (!sdb->name) {
		pr_info("SDIO: unable to allocate name");
		ret = -ENOMEM;
		goto dev_free_name;
	}

	rx_buff = kzalloc(RX_BUFF_SIZE, GFP_KERNEL);
	if (!rx_buff)
		return -ENOMEM;

	rx_dma_buff = (u8 *)L1_CACHE_ALIGN((unsigned long)rx_buff);

	tx_buff = kzalloc(TX_BUFF_SIZE, GFP_KERNEL);
	if (!rx_buff)
		return -ENOMEM;

	tx_dma_buff = (u8 *)L1_CACHE_ALIGN((unsigned long)tx_buff);

	init_waitqueue_head(&sdb->sdb_wait_q);
	ret = alloc_chrdev_region(&sdb->cdev_start_no, 0, 1, sdb->name);
	if (ret < 0) {
		dbg_log_event(sdb, "chr reg failed", ret, 0);
		goto fail_chrdev_region;
	}

	sdb->class = class_create(THIS_MODULE, sdb->name);
	if (IS_ERR(sdb->class)) {
		dbg_log_event(sdb, "clscr failed", PTR_ERR(sdb->class), 0);
		goto fail_class_create;
	}

	cdev_init(&sdb->cdev, &sdb_fops);
	sdb->cdev.owner = THIS_MODULE;

	ret = cdev_add(&sdb->cdev, sdb->cdev_start_no, 1);
	if (ret < 0) {
		dbg_log_event(sdb, "cdev_add failed", ret, 0);
		goto fail_class_create;
	}

	sdb->device = device_create(sdb->class, NULL, sdb->cdev_start_no,
			NULL, sdb->name);
	if (IS_ERR(sdb->device)) {
		dbg_log_event(sdb, "devcrfailed", PTR_ERR(sdb->device), 0);
		goto fail_device_create;
	}

	pr_info("sdio dev connected");
	rx_pkt = sdb_alloc_data_pkt(MAX_DATA_PKT_SIZE, GFP_KERNEL);
	if (IS_ERR(rx_pkt)) {
		dev_err(sdb->device, "unable to allocate data packet");
		return PTR_ERR(rx_pkt);
	}

	tx_pkt = sdb_alloc_data_pkt(MAX_DATA_PKT_SIZE, GFP_KERNEL);
	if (IS_ERR(tx_pkt)) {
		dev_err(sdb->device, "unable to allocate data packet");
		sdb_free_data_pkt(rx_pkt);
		return PTR_ERR(tx_pkt);
	}

	return ret;

fail_device_create:
	cdev_del(&sdb->cdev);
fail_class_create:
	unregister_chrdev_region(sdb->cdev_start_no, 1);
fail_chrdev_region:
dev_free_name:
	kfree(sdb->name);
dev_free:
	kfree(sdb);
	return ret;
}

static int sdio_remove(struct sdio_al_client_handle *cl_handle)
{

	struct sdio_bridge *sdb;

	sdb = __sdb;

	if (rx_pkt)
		sdb_free_data_pkt(rx_pkt);
	if (tx_pkt)
		sdb_free_data_pkt(tx_pkt);

	if (sdb) {
		cdev_del(&sdb->cdev);
		unregister_chrdev_region(sdb->cdev_start_no, 1);

		kfree(sdb->name);
		kfree(sdb);
	}

	return 0;
}

static void sdio_lpm_notify_cb(struct sdio_al_client_handle *cl_handle,
		enum sdio_al_lpm_event event)
{}

static struct sdio_al_client_data client_data = {
	.name =			"SDIO_AL_CLIENT_TTY",
	.probe =		sdio_probe,
	.remove =		sdio_remove,
	.lpm_notify_cb =	sdio_lpm_notify_cb,
};

static struct dentry *dbg_dir;

static int sdio_client_probe(struct platform_device *pdev)
{
	int ret = 0;

	if (sdio_al_is_ready())
		return -EPROBE_DEFER;

	dbg_dir = debugfs_create_dir("sdio_bridge", NULL);
	if (IS_ERR(dbg_dir))
		pr_err("SDIO: unable to create debug dir");

	client_handle = sdio_al_register_client(&client_data);

	if (!client_handle->id) {
		pr_err("SDIO: unable to register sdio_client driver");
		goto dev_free;
	}

	return 0;

dev_free:
	if (!IS_ERR(dbg_dir))
		debugfs_remove_recursive(dbg_dir);
	return ret;

}

static int sdio_client_remove(struct platform_device *pdev)
{
	struct sdio_bridge *sdb = __sdb;

	if (!IS_ERR(dbg_dir))
		debugfs_remove_recursive(dbg_dir);

	if (channel_handle)
		sdio_al_deregister_channel(channel_handle);
	if (client_handle)
		sdio_al_deregister_client(client_handle);

	if (rx_pkt)
		sdb_free_data_pkt(rx_pkt);

	if (tx_pkt)
		sdb_free_data_pkt(tx_pkt);

	kfree(sdb->name);
	kfree(sdb);

	return 0;
}

static const struct of_device_id sdio_bridge_dt_match[] = {
	{.compatible = "qcom,sdio-bridge"},
	{}
};
MODULE_DEVICE_TABLE(of, sdio_bridge_dt_match);

static struct platform_driver sdio_bridge_driver = {
	.probe  = sdio_client_probe,
	.remove = sdio_client_remove,
	.driver = {
		.name = "sdio-bridge",
		.owner = THIS_MODULE,
		.of_match_table = sdio_bridge_dt_match,
	},
};

static int __init sdio_client_init(void)
{
	return platform_driver_register(&sdio_bridge_driver);
}

static void __exit sdio_client_exit(void)
{
	platform_driver_unregister(&sdio_bridge_driver);
}


module_init(sdio_client_init);
module_exit(sdio_client_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
