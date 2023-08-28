// SPDX-License-Identifier: GPL-2.0
/*
 * PCI glue code for MIPI I3C HCI driver
 *
 * Copyright (C) 2023 Intel Corporation
 *
 * Author: Jarkko Nikula <jarkko.nikula@linux.intel.com>
 */
#include <linux/acpi.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>

struct mipi_i3c_hci_pci_info {
	int (*init)(struct pci_dev *pci);
};

#define INTEL_PRIV_OFFSET		0x2b0
#define INTEL_PRIV_SIZE			0x28
#define INTEL_PRIV_RESETS		0x04
#define INTEL_PRIV_RESETS_RESET		BIT(0)
#define INTEL_PRIV_RESETS_RESET_DONE	BIT(1)

static int mipi_i3c_hci_pci_intel_init(struct pci_dev *pci)
{
	unsigned long timeout;
	void __iomem *priv;

	priv = devm_ioremap(&pci->dev,
			    pci_resource_start(pci, 0) + INTEL_PRIV_OFFSET,
			    INTEL_PRIV_SIZE);
	if (!priv)
		return -ENOMEM;

	/* Assert reset, wait for completion and release reset */
	writel(0, priv + INTEL_PRIV_RESETS);
	timeout = jiffies + msecs_to_jiffies(10);
	while (!(readl(priv + INTEL_PRIV_RESETS) &
		 INTEL_PRIV_RESETS_RESET_DONE)) {
		if (time_after(jiffies, timeout))
			break;
		cpu_relax();
	}
	writel(INTEL_PRIV_RESETS_RESET, priv + INTEL_PRIV_RESETS);

	return 0;
}

static struct mipi_i3c_hci_pci_info intel_info = {
	.init = mipi_i3c_hci_pci_intel_init,
};

static int mipi_i3c_hci_pci_probe(struct pci_dev *pci,
				  const struct pci_device_id *id)
{
	struct mipi_i3c_hci_pci_info *info;
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
	ACPI_COMPANION_SET(&pdev->dev, ACPI_COMPANION(&pci->dev));

	ret = platform_device_add_resources(pdev, res, ARRAY_SIZE(res));
	if (ret)
		goto err;

	info = (struct mipi_i3c_hci_pci_info *)id->driver_data;
	if (info && info->init) {
		ret = info->init(pci);
		if (ret)
			goto err;
	}

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
	{ PCI_VDEVICE(INTEL, 0x7e7c), (kernel_ulong_t)&intel_info},
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
