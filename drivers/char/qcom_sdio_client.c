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
#include <linux/of.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <uapi/linux/major.h>
#include <linux/ipc_logging.h>

#define DATA_ALIGNMENT			4
#define	MAX_CLIENTS			5
#define TX_BUF_SIZE			0x4000
#define RX_BUF_SIZE			0x4000
#define SDIO_RX_BYTE_COUNT		0x21
#define SDIO_RX_BYTE_COUNT_TRANS	0x22
#define	SDIO_RX_BYTE_TRANS_MODE		0x23
#define	SDIO_RX_BYTE_TX_READY		0x24

#define SDIO_SAHARA_DOORBELL_EVENT	0x20
#define SDIO_TX_BUF_SZ_EVENT		0x21
#define SDIO_TX_BUF_SZ_TRANS_EVENT	0x22

#define	QCN_IPC_LOG_PAGES		32

static bool to_console;
module_param(to_console, bool, S_IRUGO | S_IWUSR | S_IWGRP);

static bool ipc_log;
module_param(ipc_log, bool, S_IRUGO | S_IWUSR | S_IWGRP);

#define	qlog(qsb, _msg, ...) do {					     \
	if (to_console)							     \
		pr_err_ratelimited("[%s] " _msg, __func__, ##__VA_ARGS__);   \
	if (ipc_log)							     \
		ipc_log_string(qsb->ipc_log_ctxt, "[%s] " _msg, __func__,    \
							##__VA_ARGS__);	     \
} while (0)

enum qcn_sdio_cli_id {
	QCN_SDIO_CLI_ID_INVALID = 0,
	QCN_SDIO_CLI_ID_TTY,
	QCN_SDIO_CLI_ID_WLAN,
	QCN_SDIO_CLI_ID_QMI,
	QCN_SDIO_CLI_ID_DIAG,
	QCN_SDIO_CLI_ID_MAX
};

struct diag_bridge_ops {
	void *ctxt;
	void (*read_complete_cb)(void *ctxt, char *buf,
			int buf_size, int actual);
	void (*write_complete_cb)(void *ctxt, char *buf,
			int buf_size, int actual);
	int (*suspend)(void *ctxt);
	void (*resume)(void *ctxt);
};

struct qcom_sdio_bridge {
	const char *name;
	const char *ch_name;
	uint8_t id;
	struct sdio_al_channel_handle *channel_handle;
	struct sdio_al_client_handle *client_handle;
	spinlock_t lock;
	wait_queue_head_t wait_q;
	u8 *tx_dma_buf;
	u8 *rx_dma_buf;
	u8 *pre_aligned_tx_buf;
	u8 *pre_aligned_rx_buf;
	int data_avail;
	int data_remain;
	int blk_trans_mode;
	int tx_ready;
	int offset;
	struct diag_bridge_ops *ops;
	void *ipc_log_ctxt;
};

static struct device *qsb_device;
static struct class *qsb_class;
static struct qcom_sdio_bridge *qsbdev[MAX_CLIENTS];

static void sdio_dl_meta_data_cb(struct sdio_al_client_handle *cl_handle,
		unsigned int data)
{
	struct qcom_sdio_bridge *qsb = NULL;
	u8 event = 0;

	if (!cl_handle || (cl_handle->id < QCN_SDIO_CLI_ID_TTY) ||
			(cl_handle->id > QCN_SDIO_CLI_ID_DIAG)) {
		qlog(qsb, "invalid client ID\n");
		return;
	}

	qsb = qsbdev[cl_handle->id];

	event = (u8)((data & 0xFF000000) >> 24);

	switch (event) {
	case SDIO_RX_BYTE_COUNT:
		qsb->data_avail = (data & 0x00003FFF);
		if (qsb->data_avail)
			wake_up(&qsb->wait_q);
		break;
	case SDIO_RX_BYTE_COUNT_TRANS:
		break;
	case SDIO_RX_BYTE_TRANS_MODE:
		qsb->blk_trans_mode = (data & 0x00000001);
		qlog(qsb, "client %s mode = %d\n",
				qsb->name, qsb->blk_trans_mode);
		break;
	case SDIO_RX_BYTE_TX_READY:
		qsb->tx_ready = 1;
		wake_up(&qsb->wait_q);
		qlog(qsb, "client %s tx_ready\n",
				qsb->name);
		break;
	default:
		qlog(qsb, "client %s invalid event\n",
				qsb->name);
		break;
	}
}

int qcom_client_open(int id, struct diag_bridge_ops *ops)
{
	int ret = -ENODEV;
	unsigned int mdata = 0;
	struct qcom_sdio_bridge *qsb = NULL;

	if ((id < QCN_SDIO_CLI_ID_TTY) || (id > QCN_SDIO_CLI_ID_DIAG)) {
		qlog(qsb, "invalid client ID\n");
		return ret;
	}

	qsb = qsbdev[id];
	qlog(qsb, "client %s\n", qsb->name);

	qsb->ops = ops;

	mdata = (SDIO_SAHARA_DOORBELL_EVENT << 24);
	ret = sdio_al_meta_transfer(qsb->channel_handle, mdata, 0);

	return ret;
}
EXPORT_SYMBOL(qcom_client_open);

int qcom_client_close(int id)
{
	int ret = -ENODEV;
	struct qcom_sdio_bridge *qsb = NULL;

	if ((id < QCN_SDIO_CLI_ID_TTY) || (id > QCN_SDIO_CLI_ID_DIAG)) {
		qlog(qsb, "invalid client ID\n");
		return ret;
	}

	qsb = qsbdev[id];
	qlog(qsb, "client %s\n", qsb->name);

	qsb->ops = NULL;

	return 0;
}
EXPORT_SYMBOL(qcom_client_close);

int qcom_client_read(int id, char *buf, size_t count)
{
	struct qcom_sdio_bridge *qsb = NULL;
	int ret = 0;
	int padded_len = 0;

	if ((id < QCN_SDIO_CLI_ID_TTY) || (id > QCN_SDIO_CLI_ID_DIAG)) {
		qlog(qsb, "invalid client ID\n");
		return ret;
	}

	qsb = qsbdev[id];
	qlog(qsb, "client %s\n", qsb->name);

	if (id == QCN_SDIO_CLI_ID_DIAG && !qsb->ops) {
		qlog(qsb, "%s: no diag operations assigned\n", qsb->name);
		ret = -ENODEV;
		goto out;
	}

read_start:
	if (!qsb->data_avail) {
		ret = wait_event_interruptible(qsb->wait_q, qsb->data_avail);
		if (ret < 0) {
			qlog(qsb, "%s: failed waiting for data_avail event\n",
								qsb->name);
			return ret;
		}
		goto read_start;
	}

	if (!qsb->data_remain) {
		if (count > qsb->data_avail) {
			qlog(qsb, "%s: data underflow error\n",	qsb->name);
			return -EINVAL;
		}

		if (qsb->blk_trans_mode &&
			(qsb->data_avail % qsb->client_handle->block_size)) {
			padded_len = (((qsb->data_avail /
					qsb->client_handle->block_size) + 1) *
					(qsb->client_handle->block_size));
		} else {
			padded_len = qsb->data_avail;
		}

		ret = sdio_al_queue_transfer(qsb->channel_handle,
				SDIO_AL_RX, qsb->rx_dma_buf, padded_len, 0);
		if (ret) {
			qlog(qsb, "%s: data transfer failed\n", qsb->name);
			qsb->data_avail = 0;
			return -EINVAL;
		}

		if (id == QCN_SDIO_CLI_ID_TTY) {
			ret = copy_to_user(buf, qsb->rx_dma_buf, count);
			if (ret) {
				qlog(qsb, "%s: failed to copy to user buffer\n",
								qsb->name);
				return -EIO;
			}
		} else {
			memcpy(buf, qsb->rx_dma_buf, count);
		}

		qsb->data_remain = qsb->data_avail - count;
		if (!qsb->data_remain)
			qsb->data_avail = 0;

	} else {
		if (count > qsb->data_remain) {
			qlog(qsb, "%s: data underflow error\n",	qsb->name);
			return -EINVAL;
		}

		if (id == QCN_SDIO_CLI_ID_TTY) {
			ret = copy_to_user(buf, qsb->rx_dma_buf +
				(qsb->data_avail - qsb->data_remain), count);
			if (ret) {
				qlog(qsb, "%s: failed to copy to user buffer\n",
						qsb->name);
				return -EIO;
			}
		} else {
			memcpy(buf, qsb->rx_dma_buf +
				(qsb->data_avail - qsb->data_remain), count);
		}

		qsb->data_remain -= count;
		if (!qsb->data_remain)
			qsb->data_avail = 0;
	}
out:
	return count;
}
EXPORT_SYMBOL(qcom_client_read);

int qcom_client_write(int id, char *buf, size_t count)
{
	int ret = 0;
	int padded_len = 0;
	unsigned int mdata = 0;
	struct qcom_sdio_bridge *qsb = NULL;

	if ((id < QCN_SDIO_CLI_ID_TTY) || (id > QCN_SDIO_CLI_ID_DIAG)) {
		qlog(qsb, "invalid client ID\n");
		return ret;
	}

	qsb = qsbdev[id];

	qsb->tx_ready = 0;

	qlog(qsb, "client %s\n", qsb->name);

	if (id == QCN_SDIO_CLI_ID_DIAG && !qsb->ops) {
		qlog(qsb, "%s: no diag operations assigned\n", qsb->name);
		ret = -ENODEV;
		return ret;
	}

	if (qsb->blk_trans_mode && (count % qsb->client_handle->block_size))
		padded_len = (((count / qsb->client_handle->block_size) + 1) *
				(qsb->client_handle->block_size));
	else
		padded_len = count;

	if (id == QCN_SDIO_CLI_ID_TTY) {
		ret = copy_from_user(qsb->tx_dma_buf, buf, count);
		if (ret) {
			qlog(qsb, "%s: failed to copy from user buffer\n",
								qsb->name);
			return ret;
		}
	} else {
		memcpy(qsb->tx_dma_buf, buf, count);
	}

	mdata = ((SDIO_TX_BUF_SZ_EVENT << 24) | (count & 0x3FFF));
	ret = sdio_al_meta_transfer(qsb->channel_handle, mdata, 0);
	if (ret) {
		qlog(qsb, "%s: meta data transfer failed\n", qsb->name);
		return ret;
	}

	ret = wait_event_interruptible(qsb->wait_q, qsb->tx_ready);
	if (ret < 0) {
		qlog(qsb, "%s: failed waiting for tx_ready event\n", qsb->name);
		return ret;
	}

	ret = sdio_al_queue_transfer(qsb->channel_handle,
			SDIO_AL_TX, qsb->tx_dma_buf, padded_len, 0);
	if (ret) {
		qlog(qsb, "%s: data transfer failed\n", qsb->name);
		return ret;
	}

	return count;
}
EXPORT_SYMBOL(qcom_client_write);

static int qsb_dev_open(struct inode *inode, struct file *file)
{
	if (atomic_read(&inode->i_count) != 1)
		return -EBUSY;

	return qcom_client_open(QCN_SDIO_CLI_ID_TTY, NULL);
}

static ssize_t qsb_dev_read(struct file *file, char __user *buf, size_t count,
		loff_t *ppos)
{
	return qcom_client_read(QCN_SDIO_CLI_ID_TTY, (char *)buf, count);
}

static ssize_t qsb_dev_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	return qcom_client_write(QCN_SDIO_CLI_ID_TTY, (char *)buf, count);
}

static int qsb_dev_release(struct inode *inode, struct file *file)
{
	return qcom_client_close(QCN_SDIO_CLI_ID_TTY);
}

	static
unsigned int qsb_dev_poll(struct file *file, struct poll_table_struct *wait)
{
	int ret = 0;
	struct qcom_sdio_bridge *qsb = NULL;

	qsb = qsbdev[QCN_SDIO_CLI_ID_TTY];

	if (!qsb->data_avail)
		poll_wait(file, &qsb->wait_q, wait);

	if (qsb->data_avail)
		ret = POLLIN | POLLRDNORM;

	return ret;
}

static const struct file_operations qsb_dev_ops = {
	.open           =       qsb_dev_open,
	.read           =       qsb_dev_read,
	.write          =       qsb_dev_write,
	.release        =       qsb_dev_release,
	.poll		=	qsb_dev_poll,
};

int qcom_client_debug_init(int id)
{
	int ret = -EINVAL;
	struct qcom_sdio_bridge *qsb = NULL;
	char name[32] = {0};

	if ((id < QCN_SDIO_CLI_ID_TTY) || (id > QCN_SDIO_CLI_ID_DIAG)) {
		pr_err("%s : invalid client ID\n", __func__);
		return ret;
	}

	qsb = qsbdev[id];

	snprintf(name, sizeof(name), "%s_%s", "qcn_client",
			(char *)(qsb->name + 15));

	qsb->ipc_log_ctxt = ipc_log_context_create(QCN_IPC_LOG_PAGES, name, 0);
	if (!qsb->ipc_log_ctxt) {
		pr_err("failed to initialize ipc logging for client_%d", id);
		goto out;
	}

	return 0;
out:
	return ret;
}

void qcom_client_debug_deinit(int id)
{
	struct qcom_sdio_bridge *qsb = NULL;

	if ((id < QCN_SDIO_CLI_ID_TTY) || (id > QCN_SDIO_CLI_ID_DIAG)) {
		pr_err("%s : invalid client ID\n", __func__);
		return;
	}

	qsb = qsbdev[id];

	if (qsb->ipc_log_ctxt) {
		ipc_log_context_destroy(qsb->ipc_log_ctxt);
		qsb->ipc_log_ctxt = NULL;
	}
}

static int qcom_client_probe(struct sdio_al_client_handle *client_handle)
{
	int ret = -EINVAL;
	struct sdio_al_channel_handle *channel_handle = NULL;
	struct sdio_al_channel_data *channel_data = NULL;
	struct qcom_sdio_bridge *qsb = NULL;
	int major_no = 0;
	u8 *rx_buff = NULL;
	u8 *tx_buff = NULL;

	if ((client_handle->id < QCN_SDIO_CLI_ID_TTY) ||
			(client_handle->id > QCN_SDIO_CLI_ID_DIAG)) {
		pr_err("%s : invalid client ID\n", __func__);
		goto err;
	}

	qcom_client_debug_init(client_handle->id);

	qsb = qsbdev[client_handle->id];

	qlog(qsb, "probing client %s\n", qsb->name);

	channel_data = kzalloc(sizeof(struct sdio_al_channel_data), GFP_KERNEL);
	if (!channel_data) {
		qlog(qsb, "client %s failed to allocate channel_data\n",
								qsb->name);
		ret = -ENOMEM;
		goto err;
	}

	channel_data->name = kasprintf(GFP_KERNEL, qsb->ch_name);
	channel_data->client_data = client_handle->client_data;
	channel_data->dl_meta_data_cb = sdio_dl_meta_data_cb;
	channel_data->dl_data_avail_cb = NULL;
	channel_data->ul_xfer_cb = NULL;
	channel_data->dl_xfer_cb = NULL;

	channel_handle = sdio_al_register_channel(client_handle, channel_data);
	if (!channel_handle) {
		qlog(qsb, "client %s failed to register channel_handle\n",
								qsb->name);
		ret = -EINVAL;
		goto channel_data_err;
	}

	qsb->channel_handle = channel_handle;

	rx_buff = kzalloc(RX_BUF_SIZE, GFP_KERNEL);
	if (!rx_buff) {
		qlog(qsb, "client %s failed to allocate rx_buff\n", qsb->name);
		ret = -ENOMEM;
		goto channel_reg_err;
	}

	qsb->rx_dma_buf = (u8 *)L1_CACHE_ALIGN((unsigned long)rx_buff);

	tx_buff = kzalloc(TX_BUF_SIZE, GFP_KERNEL);
	if (!tx_buff) {
		qlog(qsb, "client %s failed to allocate tx_buff\n", qsb->name);
		ret = -ENOMEM;
		goto rx_err;
	}

	qsb->tx_dma_buf = (u8 *)L1_CACHE_ALIGN((unsigned long)tx_buff);

	init_waitqueue_head(&qsb->wait_q);

	if (client_handle->id == QCN_SDIO_CLI_ID_TTY) {
		major_no = register_chrdev(UNNAMED_MAJOR, "QCN", &qsb_dev_ops);
		if (major_no < 0) {
			qlog(qsb, "client %s failed to allocate major_no\n",
								qsb->name);
			ret = major_no;
			goto tx_err;
		}

		qsb_class = class_create(THIS_MODULE, "qsahara");
		if (IS_ERR(qsb_class)) {
			qlog(qsb, "client %s failed to create class\n",
								qsb->name);
			ret = PTR_ERR(qsb_class);
			goto reg_err;
		}

		qsb_device = device_create(qsb_class, NULL, MKDEV(major_no, 0),
				NULL, "qcn_sdio");
		if (IS_ERR(qsb_device)) {
			qlog(qsb, "client %s failed to create device node\n",
								qsb->name);
			ret = PTR_ERR(qsb_device);

			goto dev_err;
		}
	}

	qlog(qsb, "probed client %s\n", qsb->name);
	return 0;

	if (client_handle->id == QCN_SDIO_CLI_ID_TTY) {
dev_err:
		class_destroy(qsb_class);
reg_err:
		unregister_chrdev(major_no, "qsahara");
	}
tx_err:
	kfree(qsb->pre_aligned_tx_buf);
rx_err:
	kfree(qsb->pre_aligned_rx_buf);
channel_reg_err:
	sdio_al_deregister_channel(channel_handle);
channel_data_err:
	kfree(channel_data);
err:
	qlog(qsb, "probe failed for client %s\n", qsb->name);
	return ret;
}

static int qcom_client_remove(struct sdio_al_client_handle *client_handle)
{
	int ret = -EINVAL;
	int minor_no = 0;
	int major_no = 0;
	struct qcom_sdio_bridge *qsb = NULL;

	if ((client_handle->id < QCN_SDIO_CLI_ID_TTY) ||
			(client_handle->id > QCN_SDIO_CLI_ID_DIAG)) {
		pr_err("%s : invalid client ID\n", __func__);
		goto err;
	}

	qsb = qsbdev[client_handle->id];
	qlog(qsb, "removing client %s\n", qsb->name);
	kfree(qsb->pre_aligned_tx_buf);
	kfree(qsb->pre_aligned_rx_buf);
	qcom_client_debug_deinit(client_handle->id);

	if (qsb_device && client_handle->id == QCN_SDIO_CLI_ID_TTY) {
		minor_no = MINOR(qsb_device->devt);
		major_no = MAJOR(qsb_device->devt);
		device_destroy(qsb_class, MKDEV(major_no, minor_no));
		class_destroy(qsb_class);
		unregister_chrdev(major_no, "qsahara");
		qsb_class = NULL;
		qsb_device = NULL;
		major_no = 0;
	}

	qlog(qsb, "removed client %s\n", qsb->name);
	return 0;

err:
	qlog(qsb, "failed to removed client %s\n", qsb->name);
	return ret;
}


static int qcom_bridge_probe(struct platform_device *pdev)
{
	int ret = -EPROBE_DEFER;
	struct sdio_al_client_handle *client_handle = NULL;
	struct sdio_al_client_data *client_data = NULL;
	int id = 0;

	ret = sdio_al_is_ready();
	if (ret) {
		ret = -EPROBE_DEFER;
		goto out;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "qcom,client-id", &id);
	if (ret) {
		pr_err("qcom,client-id not found\n");
		goto out;
	}

	qsbdev[id] = kzalloc(sizeof(struct qcom_sdio_bridge), GFP_KERNEL);
	if (!qsbdev[id]) {
		ret = -ENOMEM;
		goto out;
	}

	qsbdev[id]->id = id;

	ret = of_property_read_string(pdev->dev.of_node, "qcom,ch-name",
			&(qsbdev[id]->ch_name));
	if (ret) {
		pr_err("qcom,ch-name not found\n");
		goto out;
	}

	client_data = kzalloc(sizeof(struct sdio_al_client_data), GFP_KERNEL);
	if (!client_data) {
		ret = -ENOMEM;
		goto bridge_alloc_error;
	}

	ret = of_property_read_string(pdev->dev.of_node, "qcom,client-name",
			&client_data->name);
	if (ret) {
		pr_err("qcom,client-name not found\n");
		goto bridge_alloc_error;
	}

	qsbdev[id]->name = kasprintf(GFP_KERNEL, client_data->name);

	client_data->probe = qcom_client_probe;
	client_data->remove = qcom_client_remove;

	client_handle = sdio_al_register_client(client_data);
	if (!client_handle->id) {
		ret = -EINVAL;
		goto client_error;
	}

	if (qsbdev[client_handle->id]->id != client_handle->id) {
		pr_err("probed client %d doesn't match registered client %d\n",
			qsbdev[client_handle->id]->id, client_handle->id);
		goto client_reg_error;
	}

	qsbdev[client_handle->id]->client_handle = client_handle;

	return 0;

client_reg_error:
	sdio_al_deregister_client(client_handle);
client_error:
	kfree(client_data);
bridge_alloc_error:
	kfree(qsbdev[id]);
out:
	return ret;
}

static int qcom_bridge_remove(struct platform_device *pdev)
{
	int ret = -EBUSY;

	return ret;
}

static const struct of_device_id qcom_sdio_bridge_of_match[] = {
	{.compatible	= "qcom,sdio-bridge"},
	{}
};
MODULE_DEVICE_TABLE(of, qcom_sdio_bridge_of_match);

static struct platform_driver qcom_sdio_bridge_driver = {
	.probe	= qcom_bridge_probe,
	.remove	= qcom_bridge_remove,
	.driver	= {
		.name	= "sdio_bridge",
		.owner	= THIS_MODULE,
		.of_match_table	= qcom_sdio_bridge_of_match,
	},
};

static int __init qcom_bridge_init(void)
{
	int ret = -EBUSY;

	ret = platform_driver_register(&qcom_sdio_bridge_driver);
	if (ret) {
		printk(to_console ? KERN_ERR : KERN_DEBUG
		"%s: platform_driver registeration  failed\n", __func__);
		goto out;
	}

	return 0;
out:
	return ret;
}

static void __exit qcom_bridge_exit(void)
{
	platform_driver_unregister(&qcom_sdio_bridge_driver);
}

module_init(qcom_bridge_init);
module_exit(qcom_bridge_exit);
MODULE_DESCRIPTION("Qualcomm sdio bridge driver");
MODULE_LICENSE("GPL v2");
