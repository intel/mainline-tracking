// SPDX-License-Identifier: GPL-2.0
/*
 * soc-apci-intel-ehl-match.c - tables and support for EHL ACPI enumeration.
 *
 * Copyright (c) 2019, Intel Corporation.
 *
 */

#include <sound/soc-acpi.h>
#include <sound/soc-acpi-intel-match.h>
#include "../skylake/skl.h"

static int imr_alloc = 1;

static struct skl_machine_pdata ehl_pdata_fpga = {
	.use_tplg_pcm = true,
	.imr_alloc = &imr_alloc,
};

static struct skl_machine_pdata ehl_pdata = {
	.use_tplg_pcm = true,
};

struct snd_soc_acpi_mach snd_soc_acpi_intel_ehl_machines[] = {

	{
		.id = "INT34C2", /* Using KBL RVP with IP FPGA */
		.drv_name = "ehl_rt5660",
		.fw_filename = "intel/dsp_fw_ehl.bin",
		.pdata = &ehl_pdata_fpga,
	},

	{
		.id = "INTC1027", /* EHL board */
		.drv_name = "ehl_rt5660",
		.fw_filename = "intel/dsp_fw_ehl.bin",
		.pdata = &ehl_pdata,
	},
	{
		.id = "10EC5682",
		.drv_name = "sof_rt5682",
		.sof_fw_filename = "sof-icl.ri",
		.sof_tplg_filename = "sof-icl-rt5682.tplg",
	},
	{},

};
EXPORT_SYMBOL_GPL(snd_soc_acpi_intel_ehl_machines);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel Common ACPI Match module");
