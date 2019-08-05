// SPDX-License-Identifier: GPL-2.0
/*
 * soc-apci-intel-ehl-match.c - tables and support for EHL ACPI enumeration.
 *
 * Copyright (c) 2018, Intel Corporation.
 *
 */

#include <sound/soc-acpi.h>
#include <sound/soc-acpi-intel-match.h>
#include "../skylake/skl.h"

static struct skl_machine_pdata ehl_pdata = {
	.use_tplg_pcm = true,
};

struct snd_soc_acpi_mach snd_soc_acpi_intel_ehl_machines[] = {
{
	.id = "INT34C2",
	.drv_name = "ehl_rt274",
	.fw_filename = "intel/dsp_fw_ehl.bin",
	.pdata = &ehl_pdata,
	.sof_fw_filename = "sof-ehl.ri",
	.sof_tplg_filename = "sof-ehl-rt274.tplg",
	},
	{},
};
EXPORT_SYMBOL_GPL(snd_soc_acpi_intel_ehl_machines);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel Common ACPI Match module");
