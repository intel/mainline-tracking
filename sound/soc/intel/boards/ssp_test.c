// SPDX-License-Identifier: GPL-2.0
//
// ssp_test.c - ASoC Machine Driver for test
//
// Copyright (C) 2019, Intel Corporation. All rights reserved.

#include <linux/module.h>
#include <linux/device.h>
#include <sound/soc-acpi.h>
#include <sound/jack.h>
#include "../skylake/skl.h"
#include "../../codecs/hdac_hdmi.h"

static int imr_alloc;
module_param(imr_alloc, int, 0660);
MODULE_PARM_DESC(imr_alloc, "imr buffer address needed for FPGA platform");

#define TEST_NAME_SIZE		32
#define TEST_MAX_HDMI		3
static struct snd_soc_jack test_hdmi[TEST_MAX_HDMI];
struct test_hdmi_pcm {
	struct list_head head;
	struct snd_soc_dai *codec_dai;
	int device;
};
struct test_private {
	struct list_head hdmi_pcm_list;
	int pcm_count;
};
static struct snd_soc_dai *test_get_codec_dai(struct snd_soc_card *card,
						     const char *dai_name)
{
	struct snd_soc_pcm_runtime *rtd;

	list_for_each_entry(rtd, &card->rtd_list, list) {
		if (!strcmp(rtd->codec_dai->name, dai_name))
			return rtd->codec_dai;
	}

	return NULL;
}

SND_SOC_DAILINK_DEF(dummy_codec,
	DAILINK_COMP_ARRAY(COMP_DUMMY()));

#define PIN_NAME(ID)\
	ssp##ID

#define DAI_LINK_SSP_PIN(ID)\
SND_SOC_DAILINK_DEF(PIN_NAME(ID),\
	DAILINK_COMP_ARRAY(COMP_CPU("SSP"#ID" Pin")))

SND_SOC_DAILINK_DEF(probe_pb,
	DAILINK_COMP_ARRAY(COMP_CPU("Probe Injection0 CPU DAI")));
SND_SOC_DAILINK_DEF(probe_cp,
	DAILINK_COMP_ARRAY(COMP_CPU("Probe Extraction CPU DAI")));


SND_SOC_DAILINK_DEF(idisp1_pin,
	DAILINK_COMP_ARRAY(COMP_CPU("iDisp1 Pin")));
SND_SOC_DAILINK_DEF(idisp1_codec,
	DAILINK_COMP_ARRAY(COMP_CODEC("ehdaudio0D2", "intel-hdmi-hifi1")));

SND_SOC_DAILINK_DEF(idisp2_pin,
	DAILINK_COMP_ARRAY(COMP_CPU("iDisp2 Pin")));
SND_SOC_DAILINK_DEF(idisp2_codec,
	DAILINK_COMP_ARRAY(COMP_CODEC("ehdaudio0D2", "intel-hdmi-hifi2")));

SND_SOC_DAILINK_DEF(idisp3_pin,
	DAILINK_COMP_ARRAY(COMP_CPU("iDisp3 Pin")));
SND_SOC_DAILINK_DEF(idisp3_codec,
	DAILINK_COMP_ARRAY(COMP_CODEC("ehdaudio0D2", "intel-hdmi-hifi3")));

#define DAI_LINK(ID)\
{\
	.name = "SSP"#ID"-Codec",\
	.id = ID,\
	.ignore_suspend = 1,\
	.no_pcm = 1,\
	.dpcm_playback = 1,\
	.dpcm_capture = 1,\
	SND_SOC_DAILINK_REG(PIN_NAME(ID), dummy_codec),\
}

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
	{"hifi1", NULL, "iDisp1 Tx"},
	{"iDisp1 Tx", NULL, "iDisp1_out"},
	{"hifi2", NULL, "iDisp2 Tx"},
	{"iDisp2 Tx", NULL, "iDisp2_out"},
	{"hifi3", NULL, "iDisp3 Tx"},
	{"iDisp3 Tx", NULL, "iDisp3_out"},
};


static int
test_add_dai_link(struct snd_soc_card *card, struct snd_soc_dai_link *link)
{
	struct test_private *ctx = snd_soc_card_get_drvdata(card);
	char hdmi_dai_name[TEST_NAME_SIZE];
	struct test_hdmi_pcm *pcm;

	link->nonatomic = 1;

	/* Assuming HDMI dai link will consist the string "HDMI" */
	if (strstr(link->name, "HDMI")) {
		static int i = 1; /* hdmi codec dai name starts from index 1 */

		pcm = devm_kzalloc(card->dev, sizeof(*pcm), GFP_KERNEL);
		if (!pcm)
			return -ENOMEM;

		snprintf(hdmi_dai_name, sizeof(hdmi_dai_name),
			 "intel-hdmi-hifi%d", i++);
		pcm->codec_dai = test_get_codec_dai(card, hdmi_dai_name);
		if (!pcm->codec_dai)
			return -EINVAL;

		pcm->device = ctx->pcm_count;
		list_add_tail(&pcm->head, &ctx->hdmi_pcm_list);
	}
	ctx->pcm_count++;

	return 0;
}

static int
ssp_test_add_dai_link(struct snd_soc_card *card, struct snd_soc_dai_link *link)
{
	struct snd_soc_acpi_mach *mach = card->dev->platform_data;

	link->nonatomic = 1;
	link->platforms->name = mach->mach_params.platform;

	return test_add_dai_link(card, link);
}

DAI_LINK_SSP_PIN(0);
DAI_LINK_SSP_PIN(1);
DAI_LINK_SSP_PIN(2);
DAI_LINK_SSP_PIN(3);
DAI_LINK_SSP_PIN(4);
DAI_LINK_SSP_PIN(5);

static struct snd_soc_dai_link ssp_test_dailink[] = {
	DAI_LINK(0),
	DAI_LINK(1),
	DAI_LINK(2),
	DAI_LINK(3),
	DAI_LINK(4),
	DAI_LINK(5),
	{
		.name = "iDisp1",
		.id = 3,
		.dpcm_playback = 1,
		.no_pcm = 1,
		SND_SOC_DAILINK_REG(idisp1_pin, idisp1_codec),
	},
	{
		.name = "iDisp2",
		.id = 4,
		.dpcm_playback = 1,
		.no_pcm = 1,
		SND_SOC_DAILINK_REG(idisp2_pin, idisp2_codec),
	},
	{
		.name = "iDisp3",
		.id = 5,
		.dpcm_playback = 1,
		.no_pcm = 1,
		SND_SOC_DAILINK_REG(idisp3_pin, idisp3_codec),
	},
	{
		.name = "Compress Probe Playback",
		.init = NULL,
		.ignore_suspend = 1,
		.nonatomic = 1,
		SND_SOC_DAILINK_REG(probe_pb, dummy_codec),
	},
	{
		.name = "Compress Probe Capture",
		.init = NULL,
		.ignore_suspend = 1,
		.nonatomic = 1,
		SND_SOC_DAILINK_REG(probe_cp, dummy_codec),
	},
};

static int test_card_late_probe(struct snd_soc_card *card)
{
	struct test_private *ctx = snd_soc_card_get_drvdata(card);
	struct snd_soc_component *component = NULL;
	char jack_name[TEST_NAME_SIZE];
	struct test_hdmi_pcm *pcm;
	int err, i = 0;

	if (list_empty(&ctx->hdmi_pcm_list))
		return 0;

	list_for_each_entry(pcm, &ctx->hdmi_pcm_list, head) {
		component = pcm->codec_dai->component;
		snprintf(jack_name, sizeof(jack_name),
			"HDMI/DP, pcm=%d Jack", pcm->device);
		err = snd_soc_card_jack_new(card, jack_name,
					SND_JACK_AVOUT, &test_hdmi[i],
					NULL, 0);
		if (err)
			return err;

		err = hdac_hdmi_jack_init(pcm->codec_dai,
					  pcm->device, &test_hdmi[i]);
		if (err < 0)
			return err;

		i++;
	}

	if (!component)
		return -EINVAL;

	return hdac_hdmi_jack_port_init(component, &card->dapm);
}

/* SoC card */
static struct snd_soc_card snd_soc_card_ssp_test = {
	.name = "ssp-test-audio",
	.dai_link = ssp_test_dailink,
	.num_links = ARRAY_SIZE(ssp_test_dailink),
	.dapm_routes = ssp_test_map,
	.num_dapm_routes = ARRAY_SIZE(ssp_test_map),
	.add_dai_link = ssp_test_add_dai_link,
	.fully_routed = true,
	.late_probe = test_card_late_probe,
};

static int snd_ssp_test_probe(struct platform_device *pdev)
{
	struct test_private *ctx;

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->pcm_count = ARRAY_SIZE(ssp_test_dailink);
	INIT_LIST_HEAD(&ctx->hdmi_pcm_list);

	snd_soc_card_ssp_test.dev = &pdev->dev;
	snd_soc_card_set_drvdata(&snd_soc_card_ssp_test, ctx);

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
