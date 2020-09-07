// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/xlink_drv_inf.h>
#include "../common/core.h"
#include "epf.h"

int xlink_pcie_get_device_list(u32 *sw_device_id_list,
			       u32 *num_devices)
{
	if (xlink_sw_id != 0) {
		*num_devices = 1;
		*sw_device_id_list = xlink_sw_id;
	} else {
		*num_devices = 0;
	}

	return 0;
}
EXPORT_SYMBOL(xlink_pcie_get_device_list);

int xlink_pcie_get_device_name(u32 sw_device_id, char *device_name,
			       size_t name_size)
{
	struct xpcie *xpcie = intel_xpcie_core_get_by_id(sw_device_id);

	if (!xpcie)
		return -ENODEV;

	memset(device_name, 0, name_size);
	if (name_size > strlen(XPCIE_DRIVER_NAME))
		name_size = strlen(XPCIE_DRIVER_NAME);
	strncpy(device_name, XPCIE_DRIVER_NAME, name_size);

	return 0;
}
EXPORT_SYMBOL(xlink_pcie_get_device_name);

int xlink_pcie_get_device_status(u32 sw_device_id, u32 *device_status)
{
	struct xpcie *xpcie = intel_xpcie_core_get_by_id(sw_device_id);

	if (!xpcie)
		return -ENODEV;

	switch (xpcie->status) {
	case XPCIE_STATUS_READY:
	case XPCIE_STATUS_RUN:
		*device_status = _XLINK_DEV_READY;
		break;
	case XPCIE_STATUS_ERROR:
		*device_status = _XLINK_DEV_ERROR;
		break;
	default:
		*device_status = _XLINK_DEV_BUSY;
		break;
	}

	return 0;
}
EXPORT_SYMBOL(xlink_pcie_get_device_status);

int xlink_pcie_boot_device(u32 sw_device_id, const char *binary_name)
{
	return 0;
}
EXPORT_SYMBOL(xlink_pcie_boot_device);

int xlink_pcie_connect(u32 sw_device_id)
{
	struct xpcie *xpcie = intel_xpcie_core_get_by_id(sw_device_id);

	if (!xpcie)
		return -ENODEV;

	if (xpcie->status != XPCIE_STATUS_RUN)
		return -EIO;

	return 0;
}
EXPORT_SYMBOL(xlink_pcie_connect);

int xlink_pcie_read(u32 sw_device_id, void *data, size_t *const size,
		    u32 timeout)
{
	struct xpcie *xpcie = intel_xpcie_core_get_by_id(sw_device_id);

	if (!xpcie)
		return -ENODEV;

	return intel_xpcie_core_read(xpcie, data, size, timeout);
}
EXPORT_SYMBOL(xlink_pcie_read);

int xlink_pcie_write(u32 sw_device_id, void *data, size_t *const size,
		     u32 timeout)
{
	struct xpcie *xpcie = intel_xpcie_core_get_by_id(sw_device_id);

	if (!xpcie)
		return -ENODEV;

	return intel_xpcie_core_write(xpcie, data, size, timeout);
}
EXPORT_SYMBOL(xlink_pcie_write);

int xlink_pcie_reset_device(u32 sw_device_id)
{
	return 0;
}
EXPORT_SYMBOL(xlink_pcie_reset_device);
