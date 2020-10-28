// SPDX-License-Identifier: GPL-2.0-only
/*
 * xlink Core Driver.
 *
 * Copyright (C) 2018-2019 Intel Corporation
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/kref.h>

#include "xlink-defs.h"
#include "xlink-core.h"
#include "xlink-ioctl.h"

#define CHANNEL_SET_USER_BIT(chan) ((chan) |= (1 << 15))

static int copy_result_to_user(u32 *where, int rc)
{
	if (copy_to_user((void __user *)where, &rc, sizeof(rc)))
		return -EFAULT;
	return rc;
}

static enum xlink_error xlink_write_volatile_user(struct xlink_handle *handle,
						  u16 chan, u8 const *message, u32 size)
{
	enum xlink_error rc = 0;

	rc = do_xlink_write_volatile(handle, chan, message, size, 1);
	return rc;
}

int ioctl_connect(unsigned long arg)
{
	struct xlink_handle	devh	= {};
	struct xlinkconnect	con	= {};
	int rc = 0;

	if (copy_from_user(&con, (void __user *)arg,
			   sizeof(struct xlinkconnect)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)con.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	rc = xlink_connect(&devh);
	if (rc == X_LINK_SUCCESS) {
		if (copy_to_user((void __user *)con.handle,
				 &devh, sizeof(struct xlink_handle)))
			return -EFAULT;
	}

	return copy_result_to_user(con.return_code, rc);
}

int ioctl_open_channel(unsigned long arg)
{
	struct xlink_handle	devh	= {};
	struct xlinkopenchannel	op	= {};
	int rc = 0;

	if (copy_from_user(&op, (void __user *)arg,
			   sizeof(struct xlinkopenchannel)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)op.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	rc = xlink_open_channel(&devh, op.chan, op.mode, op.data_size,
				op.timeout);

	return copy_result_to_user(op.return_code, rc);
}

int ioctl_read_data(unsigned long arg)
{
	struct xlink_handle	devh	= {};
	struct xlinkreaddata	rd	= {};
	int rc = 0;
	u8 *rdaddr;
	u32 size;
	int interface;

	if (copy_from_user(&rd, (void __user *)arg,
			   sizeof(struct xlinkreaddata)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)rd.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	rc = xlink_read_data(&devh, rd.chan, &rdaddr, &size);
	if (!rc) {
		interface = get_interface_from_sw_device_id(devh.sw_device_id);
		if (interface == IPC_INTERFACE) {
			if (copy_to_user((void __user *)rd.pmessage, (void *)&rdaddr,
					 sizeof(u32)))
				return -EFAULT;
		} else {
			if (copy_to_user((void __user *)rd.pmessage, (void *)rdaddr,
					 size))
				return -EFAULT;
		}
		if (copy_to_user((void __user *)rd.size, (void *)&size, sizeof(size)))
			return -EFAULT;
	}

	return copy_result_to_user(rd.return_code, rc);
}

int ioctl_write_data(unsigned long arg)
{
	struct xlink_handle	devh	= {};
	struct xlinkwritedata	wr	= {};
	int rc = 0;

	if (copy_from_user(&wr, (void __user *)arg,
			   sizeof(struct xlinkwritedata)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)wr.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	if (wr.size > XLINK_MAX_DATA_SIZE)
		return -EFAULT;
	rc = xlink_write_data_user(&devh, wr.chan, wr.pmessage, wr.size);

	return copy_result_to_user(wr.return_code, rc);
}

int ioctl_write_volatile_data(unsigned long arg)
{
	struct xlink_handle	devh	= {};
	struct xlinkwritedata	wr	= {};
	int rc = 0;
	u8 volbuf[XLINK_MAX_BUF_SIZE]; // buffer for volatile transactions

	if (copy_from_user(&wr, (void __user *)arg,
			   sizeof(struct xlinkwritedata)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)wr.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	if (wr.size > XLINK_MAX_BUF_SIZE)
		return -EFAULT;
	if (copy_from_user(volbuf, (void __user *)wr.pmessage, wr.size))
		return -EFAULT;
	rc = xlink_write_volatile_user(&devh, wr.chan, volbuf, wr.size);

	return copy_result_to_user(wr.return_code, rc);
}

int ioctl_release_data(unsigned long arg)
{
	struct xlink_handle	devh	= {};
	struct xlinkrelease	rel	= {};
	int rc = 0;
	u8 reladdr;

	if (copy_from_user(&rel, (void __user *)arg,
			   sizeof(struct xlinkrelease)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)rel.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	if (rel.addr) {
		if (get_user(reladdr, (u32 __user *const)rel.addr))
			return -EFAULT;
		rc = xlink_release_data(&devh, rel.chan,
					(u8 *)&reladdr);
	} else {
		rc = xlink_release_data(&devh, rel.chan, NULL);
	}

	return copy_result_to_user(rel.return_code, rc);
}

int ioctl_close_channel(unsigned long arg)
{
	struct xlink_handle	devh	= {};
	struct xlinkopenchannel	op	= {};
	int rc = 0;

	if (copy_from_user(&op, (void __user *)arg,
			   sizeof(struct xlinkopenchannel)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)op.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	rc = xlink_close_channel(&devh, op.chan);

	return copy_result_to_user(op.return_code, rc);
}

int ioctl_disconnect(unsigned long arg)
{
	struct xlink_handle	devh	= {};
	struct xlinkconnect	con	= {};
	int rc = 0;

	if (copy_from_user(&con, (void __user *)arg,
			   sizeof(struct xlinkconnect)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)con.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	rc = xlink_disconnect(&devh);

	return copy_result_to_user(con.return_code, rc);
}
