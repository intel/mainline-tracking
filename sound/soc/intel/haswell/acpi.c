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

static struct sst_acpi_desc hsw_acpi_desc = {
	.drv_name = "haswell-pcm-audio",
	.machines = snd_soc_acpi_intel_haswell_machines,
	.resindex_lpe_base = 0,
	.resindex_pcicfg_base = 1,
	.resindex_fw_base = -1,
	.irqindex_host_ipc = 0,
	.sst_id = SST_DEV_ID_LYNX_POINT,
	.dma_engine = SST_DMA_TYPE_DW,
	.resindex_dma_base = SST_LPT_DSP_DMA_ADDR_OFFSET,
	.dma_size = SST_LPT_DSP_DMA_SIZE,
};

static struct sst_acpi_desc bdw_acpi_desc = {
	.drv_name = "haswell-pcm-audio",
	.machines = snd_soc_acpi_intel_broadwell_machines,
	.resindex_lpe_base = 0,
	.resindex_pcicfg_base = 1,
	.resindex_fw_base = -1,
	.irqindex_host_ipc = 0,
	.sst_id = SST_DEV_ID_WILDCAT_POINT,
	.dma_engine = SST_DMA_TYPE_DW,
	.resindex_dma_base = SST_WPT_DSP_DMA_ADDR_OFFSET,
	.dma_size = SST_LPT_DSP_DMA_SIZE,
};

static const struct acpi_device_id hsw_acpi_ids[] = {
	{ "INT33C8", (unsigned long)&hsw_acpi_desc },
	{ "INT3438", (unsigned long)&bdw_acpi_desc },
	{ }
};
MODULE_DEVICE_TABLE(acpi, hsw_acpi_ids);

static struct platform_driver hsw_acpi_driver = {
	.probe = sst_dsp_acpi_probe,
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
