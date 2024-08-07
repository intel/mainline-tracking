// SPDX-License-Identifier: GPL-2.0-only
/*
 * soc-apci-intel-arl-match.c - tables and support for ARL ACPI enumeration.
 *
 * Copyright (c) 2023 Intel Corporation.
 */

#include <sound/soc-acpi.h>
#include <sound/soc-acpi-intel-match.h>

static const struct snd_soc_acpi_endpoint single_endpoint = {
	.num = 0,
	.aggregated = 0,
	.group_position = 0,
	.group_id = 0,
};

static const struct snd_soc_acpi_adr_device rt711_0_adr[] = {
	{
		.adr = 0x000020025D071100ull,
		.num_endpoints = 1,
		.endpoints = &single_endpoint,
		.name_prefix = "rt711"
	}
};

static const struct snd_soc_acpi_adr_device rt711_sdca_0_adr[] = {
	{
		.adr = 0x000030025D071101ull,
		.num_endpoints = 1,
		.endpoints = &single_endpoint,
		.name_prefix = "rt711"
	}
};

static const struct snd_soc_acpi_link_adr arl_rvp[] = {
	{
		.mask = BIT(0),
		.num_adr = ARRAY_SIZE(rt711_0_adr),
		.adr_d = rt711_0_adr,
	},
	{}
};

static const struct snd_soc_acpi_link_adr arl_sdca_rvp[] = {
	{
		.mask = BIT(0),
		.num_adr = ARRAY_SIZE(rt711_sdca_0_adr),
		.adr_d = rt711_sdca_0_adr,
	},
	{}
};

static const struct snd_soc_acpi_codecs arl_essx_83x6 = {
	.num_codecs = 3,
	.codecs = { "ESSX8316", "ESSX8326", "ESSX8336"},
};

struct snd_soc_acpi_mach snd_soc_acpi_intel_arl_machines[] = {
	{
		.comp_ids = &arl_essx_83x6,
		.drv_name = "sof-essx8336",
		.sof_tplg_filename = "sof-arl-es8336", /* the tplg suffix is added at run time */
		.tplg_quirk_mask = SND_SOC_ACPI_TPLG_INTEL_SSP_NUMBER |
			SND_SOC_ACPI_TPLG_INTEL_SSP_MSB |
			SND_SOC_ACPI_TPLG_INTEL_DMIC_NUMBER,
	},
	{},
};
EXPORT_SYMBOL_GPL(snd_soc_acpi_intel_arl_machines);

/* this table is used when there is no I2S codec present */
struct snd_soc_acpi_mach snd_soc_acpi_intel_arl_sdw_machines[] = {
	{
		.link_mask = 0x1, /* link0 required */
		.links = arl_rvp,
		.drv_name = "sof_sdw",
		.sof_tplg_filename = "sof-arl-rt711.tplg",
	},
	{
		.link_mask = 0x1, /* link0 required */
		.links = arl_sdca_rvp,
		.drv_name = "sof_sdw",
		.sof_tplg_filename = "sof-arl-rt711-l0.tplg",
	},
	{},
};
EXPORT_SYMBOL_GPL(snd_soc_acpi_intel_arl_sdw_machines);
