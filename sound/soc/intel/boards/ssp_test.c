// SPDX-License-Identifier: GPL-2.0
//
// ssp_test.c - ASoC Machine Driver for test
//
// Copyright (C) 2019, Intel Corporation. All rights reserved.

#include <linux/module.h>
#include <linux/device.h>
#include <sound/soc-acpi.h>
#include <sound/soc-acpi-intel-match.h>
#include "../skylake/skl.h"

static int imr_alloc;
module_param(imr_alloc, int, 0660);
MODULE_PARM_DESC(imr_alloc, "imr buffer address needed for FPGA platform");

static u8 ssp_test_dummy_dais[] = { 0, 1, 2, 3, 4, 5 };
static struct skl_machine_pdata ssp_test_pdata = {
	.use_tplg_pcm = true,
	.imr_alloc = &imr_alloc,
	.dummy_codec = true,
	.dummy_dais = ssp_test_dummy_dais,
	.num_dummy_dais = ARRAY_SIZE(ssp_test_dummy_dais),
};

struct snd_soc_acpi_mach snd_soc_acpi_intel_ssp_test_machine = {
	.id = "dummy",
	.drv_name = "ssp_test",
	.fw_filename = "intel/dsp_fw_test.bin",
	.pdata = &ssp_test_pdata,
};
EXPORT_SYMBOL_GPL(snd_soc_acpi_intel_ssp_test_machine);

static const struct snd_soc_dapm_route ssp_test_map[] = {
	{"ssp0 Tx", NULL, "loop0_out"},
	{"loop0_in", NULL, "ssp0 Rx"},
	{"ssp1 Tx", NULL, "loop1_out"},
	{"loop1_in", NULL, "ssp1 Rx"},
	{"ssp2 Tx", NULL, "loop2_out"},
	{"loop2_in", NULL, "ssp2 Rx"},
	{"ssp3 Tx", NULL, "loop3_out"},
	{"loop3_in", NULL, "ssp3 Rx"},
	{"ssp4 Tx", NULL, "loop4_out"},
	{"loop4_in", NULL, "ssp4 Rx"},
	{"ssp5 Tx", NULL, "loop5_out"},
	{"loop5_in", NULL, "ssp5 Rx"},
};

static int
ssp_test_add_dai_link(struct snd_soc_card *card, struct snd_soc_dai_link *link)
{
	struct snd_soc_acpi_mach *mach = card->dev->platform_data;

	link->nonatomic = 1;
	link->platform_name = mach->mach_params.platform;

	return 0;
}

static struct snd_soc_dai_link ssp_test_dailink[] = {
	{
		.name = "SSP0-Codec",
		.id = 1,
		.cpu_dai_name = "SSP0 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ignore_suspend = 1,
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "SSP1-Codec",
		.id = 2,
		.cpu_dai_name = "SSP1 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ignore_suspend = 1,
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "SSP2-Codec",
		.id = 3,
		.cpu_dai_name = "SSP2 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ignore_suspend = 1,
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "SSP3-Codec",
		.id = 4,
		.cpu_dai_name = "SSP3 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ignore_suspend = 1,
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "SSP4-Codec",
		.id = 5,
		.cpu_dai_name = "SSP4 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ignore_suspend = 1,
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "SSP5-Codec",
		.id = 6,
		.cpu_dai_name = "SSP5 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ignore_suspend = 1,
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
};

/* SoC card */
static struct snd_soc_card snd_soc_card_ssp_test = {
	.name = "ssp-test-audio",
	.dai_link = ssp_test_dailink,
	.num_links = ARRAY_SIZE(ssp_test_dailink),
	.dapm_routes = ssp_test_map,
	.num_dapm_routes = ARRAY_SIZE(ssp_test_map),
	.add_dai_link = ssp_test_add_dai_link,
	.fully_routed = true,
};

static int snd_ssp_test_probe(struct platform_device *pdev)
{
	snd_soc_card_ssp_test.dev = &pdev->dev;
	return devm_snd_soc_register_card(&pdev->dev, &snd_soc_card_ssp_test);
}

static const struct platform_device_id ssp_test_board_ids[] = {
	{ .name = "ssp_test" },
	{ }
};

static struct platform_driver snd_ssp_test_driver = {
	.driver = {
		.name = "ssp_test",
		.pm = &snd_soc_pm_ops,
	},
	.probe = snd_ssp_test_probe,
	.id_table = ssp_test_board_ids,
};

module_platform_driver(snd_ssp_test_driver);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ssp_test");
