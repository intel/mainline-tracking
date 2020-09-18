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

#ifdef XLINK_PCIE_REMOTE
#include "../remote_host/pci.h"
#else
#include "core.h"
#include "../local_host/epf.h"
#include "../local_host/struct.h"
#endif

/* Define xpcie driver interface API */
int xlink_pcie_get_device_list(u32 *sw_device_id_list,
			       u32 *num_devices)
{
#ifdef XLINK_PCIE_LOCAL
	if (xlink_sw_id != 0) {
		*num_devices = 1;
		*sw_device_id_list = xlink_sw_id;
	} else {
		*num_devices = 0;
	}
#else
	*num_devices = intel_xpcie_get_device_num(sw_device_id_list);
#endif

	return 0;
}
EXPORT_SYMBOL(xlink_pcie_get_device_list);

int xlink_pcie_get_device_name(u32 sw_device_id, char *device_name,
			       size_t name_size)
{
#ifdef XLINK_PCIE_LOCAL
	struct xpcie *xpcie = intel_xpcie_core_get_by_id(sw_device_id);

	if (!xpcie)
		return -ENODEV;

	memset(device_name, 0, name_size);
	if (name_size > strlen(XPCIE_DRIVER_NAME))
		name_size = strlen(XPCIE_DRIVER_NAME);
	strncpy(device_name, XPCIE_DRIVER_NAME, name_size);

	return 0;
#else
	return intel_xpcie_get_device_name_by_id(sw_device_id,
						 device_name, name_size);
#endif
}
EXPORT_SYMBOL(xlink_pcie_get_device_name);

int xlink_pcie_get_device_status(u32 sw_device_id, u32 *device_status)
{
	u32 status;

#ifdef XLINK_PCIE_LOCAL
	struct xpcie *xpcie = intel_xpcie_core_get_by_id(sw_device_id);

	if (!xpcie)
		return -ENODEV;

	status = xpcie->status;
#else
	int rc;

	rc = intel_xpcie_get_device_status_by_id(sw_device_id, &status);
	if (rc)
		return rc;
#endif
	switch (status) {
	case XPCIE_STATUS_READY:
	case XPCIE_STATUS_RUN:
		*device_status = _XLINK_DEV_READY;
		break;
	case XPCIE_STATUS_ERROR:
		*device_status = _XLINK_DEV_ERROR;
		break;
	case XPCIE_STATUS_RECOVERY:
		*device_status = _XLINK_DEV_RECOVERY;
		break;
	case XPCIE_STATUS_OFF:
		*device_status = _XLINK_DEV_OFF;
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
#ifdef XLINK_PCIE_LOCAL
	struct xpcie *xpcie = intel_xpcie_core_get_by_id(sw_device_id);

	if (!xpcie)
		return -ENODEV;

	if (xpcie->status != XPCIE_STATUS_RUN)
		return -EIO;

	return 0;
#else
	return intel_xpcie_pci_connect_device(sw_device_id);
#endif
}
EXPORT_SYMBOL(xlink_pcie_connect);

int xlink_pcie_read(u32 sw_device_id, void *data, size_t *const size,
		    u32 timeout)
{
#ifdef XLINK_PCIE_LOCAL
	struct xpcie *xpcie = intel_xpcie_core_get_by_id(sw_device_id);

	if (!xpcie)
		return -ENODEV;

	return intel_xpcie_core_read(xpcie, data, size, timeout);
#else
	return intel_xpcie_pci_read(sw_device_id, data, size, timeout);
#endif
}
EXPORT_SYMBOL(xlink_pcie_read);

int xlink_pcie_write(u32 sw_device_id, void *data, size_t *const size,
		     u32 timeout)
{
#ifdef XLINK_PCIE_LOCAL
	struct xpcie *xpcie = intel_xpcie_core_get_by_id(sw_device_id);

	if (!xpcie)
		return -ENODEV;

	return intel_xpcie_core_write(xpcie, data, size, timeout);
#else
	return intel_xpcie_pci_write(sw_device_id, data, size, timeout);
#endif
}
EXPORT_SYMBOL(xlink_pcie_write);

int xlink_pcie_reset_device(u32 sw_device_id)
{
	int ret = 0;
#ifdef XLINK_PCIE_REMOTE
	ret = intel_xpcie_pci_reset_device(sw_device_id);
#endif
	return ret;
}
EXPORT_SYMBOL(xlink_pcie_reset_device);
