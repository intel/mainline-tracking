// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel BYT loader on ACPI systems
 *
 * Copyright (C) 2019, Intel Corporation. All rights reserved.
 */

#include <linux/acpi.h>
#include <linux/platform_device.h>
#include <sound/soc-acpi.h>
#include <sound/soc-acpi-intel-match.h>
#include "../common/sst-dsp.h"

static struct sst_pdata byt_desc = {
	.id = SST_DEV_ID_BYT,
	.fw_name = "intel/fw_sst_0f28.bin-48kHz_i2s_master",
	.boards = snd_soc_acpi_intel_baytrail_legacy_machines,
	.dma_base = -1,
};

static const struct acpi_device_id byt_acpi_ids[] = {
	{ "80860F28", (unsigned long)&byt_desc },
	{ }
};
MODULE_DEVICE_TABLE(acpi, byt_acpi_ids);

static int byt_acpi_probe(struct platform_device *pdev)
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

	acpi_desc->drv_name = "baytrail-pcm-audio";
	acpi_desc->pdata = (struct sst_pdata *)id->driver_data;
	acpi_desc->resindex_lpe_base = 0;
	acpi_desc->resindex_pcicfg_base = 1;
	acpi_desc->resindex_fw_base = 2;
	acpi_desc->irqindex_host_ipc = 5;
	platform_set_drvdata(pdev, acpi_desc);

	return sst_dsp_acpi_probe(pdev);
}

static struct platform_driver byt_acpi_driver = {
	.probe = byt_acpi_probe,
	.remove = sst_dsp_acpi_remove,
	.driver = {
		.name = "byt-acpi",
		.acpi_match_table = ACPI_PTR(byt_acpi_ids),
	},
};
module_platform_driver(byt_acpi_driver);

MODULE_AUTHOR("Cezary Rojewski <cezary.rojewski@intel.com>");
MODULE_DESCRIPTION("Intel BYT loader on ACPI systems");
MODULE_LICENSE("GPL v2");
