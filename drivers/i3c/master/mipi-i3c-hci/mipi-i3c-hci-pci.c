// SPDX-License-Identifier: GPL-2.0
/*
 * PCI glue code for MIPI I3C HCI driver
 *
 * Copyright (C) 2023 Intel Corporation
 *
 * Author: Jarkko Nikula <jarkko.nikula@linux.intel.com>
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>

static int mipi_i3c_hci_pci_probe(struct pci_dev *pci,
				  const struct pci_device_id *id)
{
	struct platform_device *pdev;
	struct resource res[2];
	int ret;

	ret = pcim_enable_device(pci);
	if (ret)
		return ret;

	pci_set_master(pci);

	memset(&res, 0, sizeof(res));

	res[0].flags = IORESOURCE_MEM;
	res[0].start = pci_resource_start(pci, 0);
	res[0].end = pci_resource_end(pci, 0);

	res[1].flags = IORESOURCE_IRQ;
	res[1].start = pci->irq;
	res[1].end = pci->irq;

	pdev = platform_device_alloc("mipi-i3c-hci", 0);
	if (!pdev)
		return -ENOMEM;

	pdev->dev.parent = &pci->dev;

	ret = platform_device_add_resources(pdev, res, ARRAY_SIZE(res));
	if (ret)
		goto err;

	ret = platform_device_add(pdev);
	if (ret)
		goto err;

	pci_set_drvdata(pci, pdev);

	return 0;

err:
	platform_device_put(pdev);
	return ret;
}

static void mipi_i3c_hci_pci_remove(struct pci_dev *pci)
{
	struct platform_device *pdev = pci_get_drvdata(pci);

	platform_device_unregister(pdev);
}

static const struct pci_device_id mipi_i3c_hci_pci_devices[] = {
	/* Meteor Lake-P */
	{ PCI_VDEVICE(INTEL, 0x7e7c), },
	{ },
};
MODULE_DEVICE_TABLE(pci, mipi_i3c_hci_pci_devices);

static struct pci_driver mipi_i3c_hci_pci_driver = {
	.name = "mipi_i3c_hci_pci",
	.id_table = mipi_i3c_hci_pci_devices,
	.probe = mipi_i3c_hci_pci_probe,
	.remove = mipi_i3c_hci_pci_remove,
};

module_pci_driver(mipi_i3c_hci_pci_driver);

MODULE_AUTHOR("Jarkko Nikula <jarkko.nikula@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MIPI I3C HCI driver on PCI bus");
