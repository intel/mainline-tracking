// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel HSW loader on ACPI systems
 *
 * Copyright (C) 2019, Intel Corporation. All rights reserved.
 */

#include <linux/acpi.h>
#include <linux/platform_device.h>
#include <sound/soc-acpi.h>
#include <sound/soc-acpi-intel-match.h>
#include "../common/sst-dsp.h"

#define SST_LPT_DSP_DMA_ADDR_OFFSET	0x0F0000
#define SST_WPT_DSP_DMA_ADDR_OFFSET	0x0FE000
#define SST_LPT_DSP_DMA_SIZE		(1024 - 1)

static struct sst_pdata hsw_desc = {
	.id = SST_DEV_ID_LYNX_POINT,
	.fw_name = "intel/IntcSST1.bin",
	.boards = snd_soc_acpi_intel_haswell_machines,
	.dma_base = SST_LPT_DSP_DMA_ADDR_OFFSET,
};

static struct sst_pdata bdw_desc = {
	.id = SST_DEV_ID_WILDCAT_POINT,
	.fw_name = "intel/IntcSST2.bin",
	.boards = snd_soc_acpi_intel_broadwell_machines,
	.dma_base = SST_WPT_DSP_DMA_ADDR_OFFSET,
};

static const struct acpi_device_id hsw_acpi_ids[] = {
	{ "INT33C8", (unsigned long)&hsw_desc },
	{ "INT3438", (unsigned long)&bdw_desc },
	{ }
};
MODULE_DEVICE_TABLE(acpi, hsw_acpi_ids);

static int hsw_acpi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sst_acpi_desc *acpi_desc;
	const struct acpi_device_id *id;

	id = acpi_match_device(dev->driver->acpi_match_table, dev);
	if (!id)
		return -ENODEV;

	acpi_desc = devm_kzalloc(dev, sizeof(*acpi_desc), GFP_KERNEL);
	if (!acpi_desc)
		return -ENOMEM;

	acpi_desc->drv_name = "haswell-pcm-audio";
	acpi_desc->pdata = (struct sst_pdata *)id->driver_data;
	acpi_desc->resindex_lpe_base = 0;
	acpi_desc->resindex_pcicfg_base = 1;
	acpi_desc->resindex_fw_base = -1;
	acpi_desc->irqindex_host_ipc = 0;
	acpi_desc->dma_engine = SST_DMA_TYPE_DW;
	acpi_desc->dma_size = SST_LPT_DSP_DMA_SIZE;
	platform_set_drvdata(pdev, acpi_desc);

	return sst_dsp_acpi_probe(pdev);
}

static struct platform_driver hsw_acpi_driver = {
	.probe = hsw_acpi_probe,
	.remove = sst_dsp_acpi_remove,
	.driver = {
		.name = "hsw-acpi",
		.acpi_match_table = ACPI_PTR(hsw_acpi_ids),
	},
};
module_platform_driver(hsw_acpi_driver);

MODULE_AUTHOR("Cezary Rojewski <cezary.rojewski@intel.com>");
MODULE_DESCRIPTION("Intel HSW loader on ACPI systems");
MODULE_LICENSE("GPL v2");
