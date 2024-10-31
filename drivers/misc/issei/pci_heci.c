// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2023-2024 Intel Corporation */
#include <linux/module.h>
#include <linux/pci.h>

#include "cdev.h"
#include "hw_heci.h"
#include "hw_heci_regs.h"

static int issei_heci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	const struct hw_heci_cfg *cfg;
	struct issei_device *idev;
	struct issei_heci_hw *hw;
	int err;

	cfg = issei_heci_get_cfg(ent->driver_data);
	if (!cfg)
		return -ENODEV;

	err = pcim_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "failed to enable pci device. err = %d\n", err);
		return err;
	}

	err = pcim_iomap_regions(pdev, BIT(0), KBUILD_MODNAME);
	if (err) {
		dev_err(&pdev->dev, "failed to get pci regions. err = %d\n", err);
		return err;
	}

	err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (err) {
		dev_err(&pdev->dev, "no usable DMA configuration, aborting. err = %d\n", err);
		return err;
	}

	pci_set_master(pdev);

	idev = issei_heci_dev_init(&pdev->dev, cfg);
	if (!idev)
		return -ENOMEM;
	hw = to_heci_hw(idev);
	hw->mem_addr = pcim_iomap_table(pdev)[0];

	err = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSI);
	if (err < 0) {
		dev_err(&pdev->dev, "pci_alloc_irq_vectors failure. err = %d\n", err);
		return err;
	}

	hw->irq = pci_irq_vector(pdev, 0);

	err = devm_request_threaded_irq(&pdev->dev, hw->irq,
				   issei_heci_irq_quick_handler,
				   NULL,
				   IRQF_ONESHOT, KBUILD_MODNAME, idev);
	if (err) {
		dev_err(&pdev->dev, "request_threaded_irq failure. err = %d, irq = %d\n",
			err, hw->irq);
		goto release_irq;
	}

	err = issei_start(idev);
	if (err)	{
		dev_err(&pdev->dev, "init hw failure. err = %d.\n", err);
		goto free_irq;
	}

	err = issei_register(idev, &pdev->dev);
	if (err) {
		dev_err(&pdev->dev, "register failure. err = %d.\n", err);
		goto stop;
	}

	pci_set_drvdata(pdev, idev);

	dev_dbg(&pdev->dev, "initialization successful.\n");

	return 0;

stop:
	issei_stop(idev);
free_irq:
	/* Manually free IRQ otherwise PCI free irq vectors will fail */
	devm_free_irq(&pdev->dev, hw->irq, idev);
release_irq:
	idev->ops->irq_disable(idev);
	pci_free_irq_vectors(pdev);
	return err;
}

static void __issei_heci_deconstruct(struct pci_dev *pdev)
{
	struct issei_device *idev = pci_get_drvdata(pdev);
	struct issei_heci_hw *hw = to_heci_hw(idev);

	issei_stop(idev);

	idev->ops->irq_disable(idev);
	/* Manually free IRQ otherwise PCI free irq vectors will fail */
	devm_free_irq(&pdev->dev, hw->irq, idev);
	pci_free_irq_vectors(pdev);
}


static void issei_heci_shutdown(struct pci_dev *pdev)
{
	dev_dbg(&pdev->dev, "shutdown\n");
	__issei_heci_deconstruct(pdev);
}

static void issei_heci_remove(struct pci_dev *pdev)
{
	dev_dbg(&pdev->dev, "stop\n");
	__issei_heci_deconstruct(pdev);

	issei_deregister(pci_get_drvdata(pdev));
}

static int __maybe_unused issei_heci_pm_suspend(struct device *device)
{
	struct issei_device *idev = dev_get_drvdata(device);

	issei_stop(idev);
	idev->ops->irq_disable(idev);

	return 0;
}

static int __maybe_unused issei_heci_pm_resume(struct device *device)
{
	struct issei_device *idev = dev_get_drvdata(device);

	return issei_start(idev);
}

static const struct dev_pm_ops issei_heci_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(issei_heci_pm_suspend, issei_heci_pm_resume)
};

static const struct pci_device_id heci_pci_tbl[] = {
	{PCI_VDEVICE(INTEL, 0xA85D)}, /* Lunar Lake M */
	{PCI_VDEVICE(INTEL, 0xE35D)}, /* Panter Lake H */
	{PCI_VDEVICE(INTEL, 0xE45D)}, /* Panter Lake P */

	{0, }
};
MODULE_DEVICE_TABLE(pci, heci_pci_tbl);

static struct pci_driver issei_heci_driver = {
	.name = KBUILD_MODNAME,
	.id_table = heci_pci_tbl,
	.probe = issei_heci_probe,
	.remove = issei_heci_remove,
	.shutdown = issei_heci_shutdown,
	.driver.pm = &issei_heci_pm_ops,
	.driver.probe_type = PROBE_PREFER_ASYNCHRONOUS,
};

module_pci_driver(issei_heci_driver);

MODULE_DESCRIPTION("Intel(R) Silicon Security Engine Interface - HECI");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("INTEL_SSEI");
