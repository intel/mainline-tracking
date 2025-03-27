// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2025-2026 Intel Corporation */
#include <linux/device.h>
#include <linux/dev_printk.h>
#include <linux/sched/signal.h>
#include <linux/slab.h>
#include <linux/issei.h>
#include <linux/issei_device.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/uuid.h>
#include <linux/wait.h>

#include "issei_dev.h"
#include "host_client.h"

/**
 * issei_device_open - open issei device context.
 * @match: pointer to callback that should return non-zero when matching device is found
 * @data: user data to pass to match callback
 *
 * Return: pointer to opaque context on success, <0 on error
 */
void *issei_device_open(int (*match)(struct device *, const void *), const void *data)
{
	struct issei_host_client *cl;
	struct issei_device *idev;
	struct device *dev;

	dev = class_find_device(issei_class, NULL, data, match);
	if (!dev)
		return ERR_PTR(-ENODEV);

	idev = dev_get_drvdata(dev);

	cl = issei_cl_create(idev, NULL);
	if (IS_ERR(cl))
		put_device(dev);

	return cl;
}
EXPORT_SYMBOL_GPL(issei_device_open);

/**
 * issei_device_release - release issei device context.
 * @ctx: opaque context
 */
void issei_device_release(void *ctx)
{
	struct issei_host_client *cl = (struct issei_host_client *)ctx;
	struct issei_device *idev = cl->idev;

	issei_cl_remove(cl);
	put_device(&idev->dev);
}
EXPORT_SYMBOL_GPL(issei_device_release);

/**
 * issei_device_connect - connect to the firmware client.
 * @ctx: opaque context
 * @client_uuid: UUID of firmware client to connect
 * @conn: pointer for client data to fill on successful connection
 *
 * Return: 0 on success, <0 on error
 */
int issei_device_connect(void *ctx, const uuid_t *client_uuid, struct issei_client *conn)
{
	struct issei_host_client *cl = (struct issei_host_client *)ctx;
	struct issei_device *idev = cl->idev;

	if (idev->rst_state != ISSEI_RST_STATE_DONE) {
		dev_dbg(&idev->dev, "Device is in transition\n");
		return -ENODEV;
	}
	return issei_cl_connect(cl, client_uuid, &conn->max_msg_length,
			       &conn->protocol_version, &conn->flags);
}
EXPORT_SYMBOL_GPL(issei_device_connect);

/**
 * issei_device_disconnect - disconnect from the firmware client.
 * @ctx: opaque context
 *
 * Return: 0 on success, <0 on error
 */
int issei_device_disconnect(void *ctx)
{
	struct issei_host_client *cl = (struct issei_host_client *)ctx;
	struct issei_device *idev = cl->idev;

	if (idev->rst_state != ISSEI_RST_STATE_DONE) {
		dev_dbg(&idev->dev, "Device is in transition\n");
		return -ENODEV;
	}

	return issei_cl_disconnect(cl);
}
EXPORT_SYMBOL_GPL(issei_device_disconnect);

/**
 * issei_device_write - send data to the firmware client.
 * @ctx: opaque context
 * @ubuf: data to send
 * @length: data length
 *
 * Return: >=0 data length on success, <0 on error
 */
ssize_t issei_device_write(void *ctx, const u8 *ubuf, size_t length)
{
	struct issei_host_client *cl = (struct issei_host_client *)ctx;
	struct issei_device *idev = cl->idev;
	ssize_t ret;

	if (idev->rst_state != ISSEI_RST_STATE_DONE) {
		dev_dbg(&idev->dev, "Device is in transition\n");
		return -EBUSY;
	}

	if (!length)
		return 0;

	/* sanity check */
	if (length > idev->dma.length.h2f) {
		dev_dbg(&idev->dev, "Write is too big %zu > %zu\n", length,
			idev->dma.length.h2f);
		return -EFBIG;
	}

	u8 *buf __free(kfree) = kmemdup(ubuf, length, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	do {
		ret = issei_cl_write(cl, buf, length);
		if (ret < 0 && ret != -EAGAIN)
			return ret;
		/* buf is consumed by issei_cl_write on success */
		if (ret >= 0)
			retain_and_null_ptr(buf);
		if (wait_event_interruptible(cl->write_wait, issei_cl_check_write(cl) != 1)) {
			issei_cl_clean_all_wbuf(cl);
			if (signal_pending(current))
				return -EINTR;
			return -ERESTARTSYS;
		}
	} while (ret == -EAGAIN);

	return ret;
}
EXPORT_SYMBOL_GPL(issei_device_write);

/**
 * issei_device_read - receive data from the firmware client.
 * @ctx: opaque context
 * @ubuf: buffer for data
 * @length: buffer length
 *
 * Return: >=0 data length on success, <0 on error
 */
ssize_t issei_device_read(void *ctx, u8 *ubuf, size_t length)
{
	struct issei_host_client *cl = (struct issei_host_client *)ctx;
	struct issei_device *idev = cl->idev;
	u8 *data = NULL;
	ssize_t ret;

	if (idev->rst_state != ISSEI_RST_STATE_DONE) {
		dev_dbg(&idev->dev, "Device is in transition\n");
		return -EBUSY;
	}

	if (!length)
		return 0;

	/* sanity check */
	if (length > idev->dma.length.f2h) {
		dev_dbg(&idev->dev, "Read is too big %zu > %zu\n", length,
			idev->dma.length.f2h);
		return -EFBIG;
	}

	ret = issei_cl_read(cl, &data, length);
	if (ret < 0) {
		if (ret != -ENOENT)
			return ret;

		if (wait_event_interruptible(cl->read_wait, issei_cl_check_read(cl) != 0)) {
			if (signal_pending(current))
				return -EINTR;
			return -ERESTARTSYS;
		}
		ret = issei_cl_read(cl, &data, length);
		if (ret < 0)
			return ret;
	}

	memcpy(ubuf, data, ret);
	kfree(data);
	return ret;
}
EXPORT_SYMBOL_GPL(issei_device_read);

/**
 * issei_device_dma_map - allocate DMA buffer on parent device.
 * @ctx: opaque context
 * @size: buffer size
 * @daddr: pointer for buffer physical address
 * @vaddr: pointer for buffer virtual address
 *
 * Return: 0 on success, <0 on error
 */
int issei_device_dma_map(void *ctx, size_t size, dma_addr_t *daddr, void **vaddr)
{
	struct issei_host_client *cl = (struct issei_host_client *)ctx;

	return issei_cl_dma_map(cl, size, daddr, vaddr);
}
EXPORT_SYMBOL_GPL(issei_device_dma_map);

/**
 * issei_device_dma_unmap - deallocate DMA buffer.
 * @ctx: opaque context
 */
void issei_device_dma_unmap(void *ctx)
{
	struct issei_host_client *cl = (struct issei_host_client *)ctx;

	issei_cl_dma_unmap(cl);
}
EXPORT_SYMBOL_GPL(issei_device_dma_unmap);

/**
 * issei_device_register_interface - register class interface for issei class.
 * @intf: interface structure
 *
 * Return: 0 on success, <0 on error
 */
int issei_device_register_interface(struct class_interface *intf)
{
	intf->class = issei_class;

	return class_interface_register(intf);
}
EXPORT_SYMBOL_GPL(issei_device_register_interface);
