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

static struct sst_acpi_desc byt_acpi_desc = {
	.drv_name = "baytrail-pcm-audio",
	.machines = snd_soc_acpi_intel_baytrail_legacy_machines,
	.resindex_lpe_base = 0,
	.resindex_pcicfg_base = 1,
	.resindex_fw_base = 2,
	.irqindex_host_ipc = 5,
	.sst_id = SST_DEV_ID_BYT,
	.resindex_dma_base = -1,
};

static const struct acpi_device_id byt_acpi_ids[] = {
	{ "80860F28", (unsigned long)&byt_acpi_desc },
	{ }
};
MODULE_DEVICE_TABLE(acpi, byt_acpi_ids);

static struct platform_driver byt_acpi_driver = {
	.probe = sst_dsp_acpi_probe,
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
