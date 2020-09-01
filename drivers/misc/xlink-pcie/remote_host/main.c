// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#include "pci.h"

static const struct pci_device_id xpcie_pci_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_KEEMBAY), 0 },
	{ 0 }
};

static bool driver_unload;

static int intel_xpcie_probe(struct pci_dev *pdev,
			     const struct pci_device_id *ent)
{
	int ret = 0;
	u32 sw_devid = 0;
	u32 hw_id = 0;
	bool new_device = false;
	struct xpcie_dev *xdev;

	hw_id = ((u16)pdev->bus->number << 8) | PCI_SLOT(pdev->devfn);

	sw_devid = (XLINK_DEV_INF_PCIE << XLINK_DEV_INF_TYPE_SHIFT) |
		   (hw_id << XLINK_DEV_PHYS_ID_SHIFT) |
		   (XLINK_DEV_TYPE_KMB << XLINK_DEV_TYPE_SHIFT) |
		   (XLINK_DEV_SLICE_0 << XLINK_DEV_SLICE_ID_SHIFT) |
		   (XLINK_DEV_FUNC_VPU << XLINK_DEV_FUNC_SHIFT);

	xdev = intel_xpcie_get_device_by_id(sw_devid);
	if (!xdev) {
		xdev = intel_xpcie_create_device(sw_devid, pdev);
		if (!xdev)
			return -ENOMEM;

		new_device = true;
	}

	ret = intel_xpcie_pci_init(xdev, pdev);
	if (ret) {
		intel_xpcie_remove_device(xdev);
		return ret;
	}

	if (new_device)
		intel_xpcie_list_add_device(xdev);

	return ret;
}

static void intel_xpcie_remove(struct pci_dev *pdev)
{
	struct xpcie_dev *xdev = pci_get_drvdata(pdev);

	if (xdev) {
		intel_xpcie_pci_cleanup(xdev);
		if (driver_unload)
			intel_xpcie_remove_device(xdev);
	}
}

static struct pci_driver xpcie_driver = {
	.name = XPCIE_DRIVER_NAME,
	.id_table = xpcie_pci_table,
	.probe = intel_xpcie_probe,
	.remove = intel_xpcie_remove
};

static int __init intel_xpcie_init_module(void)
{
	return pci_register_driver(&xpcie_driver);
}

static void __exit intel_xpcie_exit_module(void)
{
	driver_unload = true;
	pci_unregister_driver(&xpcie_driver);
}

module_init(intel_xpcie_init_module);
module_exit(intel_xpcie_exit_module);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Intel");
MODULE_DESCRIPTION(XPCIE_DRIVER_DESC);
MODULE_VERSION(XPCIE_DRIVER_VERSION);
