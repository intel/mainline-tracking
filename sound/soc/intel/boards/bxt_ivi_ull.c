// SPDX-License-Identifier: GPL-2.0
//
// bxt_ivi_ull.c  --  Intel Broxton-P I2S ULL Machine Driver
//
// Copyright (C) 2017, Intel Corporation. All rights reserved.

#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

static const struct snd_soc_pcm_stream media1_out_params = {
	.formats = SNDRV_PCM_FMTBIT_S32_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 3,
	.channels_max = 3,
};

static const struct snd_soc_pcm_stream codec1_in_params = {
	.formats = SNDRV_PCM_FMTBIT_S32_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 6,
	.channels_max = 6,
};

static const struct snd_soc_dapm_widget broxton_widgets[] = {
	SND_SOC_DAPM_SPK("DummySpeaker1", NULL),
	SND_SOC_DAPM_SPK("DummySpeaker2", NULL),
	SND_SOC_DAPM_SPK("DummySpeaker3", NULL),
	SND_SOC_DAPM_SPK("DummySpeaker4", NULL),
	SND_SOC_DAPM_MIC("DummyMIC0", NULL),
	SND_SOC_DAPM_MIC("DummyMIC2", NULL),
	SND_SOC_DAPM_MIC("DummyMIC4", NULL),
};

static const struct snd_soc_dapm_route bxtp_ull_map[] = {
	{"8ch_pt_in3", NULL, "ssp0 Rx" },
	{"ssp0 Rx", NULL, "Dummy Capture" },
	{"Dummy Capture", NULL, "DummyMIC0"},

	{"DummySpeaker2", NULL, "Dummy Playback"},
	{"Dummy Playback", NULL, "ssp2 Tx"},
	{"ssp2 Tx", NULL, "8ch_pt_out2"},

	{"DummySpeaker1", NULL, "Dummy Playback"},
	{"Dummy Playback", NULL, "ssp1 Tx"},
	{"ssp1 Tx", NULL, "8ch_pt_out3"},

	{"8ch_pt_in2", NULL, "ssp2 Rx" },
	{"ssp2 Rx", NULL, "Dummy Capture" },
	{"Dummy Capture", NULL, "DummyMIC2"},

	{"DummySpeaker4", NULL, "Dummy Playback"},
	{"Dummy Playback", NULL, "ssp3 Tx"},
	{"ssp3 Tx", NULL, "8ch_pt_out"},

	{"8ch_pt_in", NULL, "ssp3 Rx" },
	{"ssp3 Rx", NULL, "Dummy Capture" },
	{"Dummy Capture", NULL, "DummyMIC4"},

	/* (ANC) Codec1_in - Loop pipe */
	{ "codec1_in", NULL, "ssp0-b Rx" },
	{ "ssp0-b Rx", NULL, "Dummy Capture" },

	/* Media1_out Loop Path */
	{"DummySpeaker3", NULL, "Dummy Playback"},
	{ "Dummy Playback", NULL, "ssp1-b Tx"},
	{ "ssp1-b Tx", NULL, "media1_out"},
};

SND_SOC_DAILINK_DEF(dummy,
	DAILINK_COMP_ARRAY(COMP_DUMMY()));

SND_SOC_DAILINK_DEF(system3,
	DAILINK_COMP_ARRAY(COMP_CPU("System Pin 3")));

SND_SOC_DAILINK_DEF(system4,
	DAILINK_COMP_ARRAY(COMP_CPU("System Pin 4")));

SND_SOC_DAILINK_DEF(system5,
	DAILINK_COMP_ARRAY(COMP_CPU("System Pin 5")));

SND_SOC_DAILINK_DEF(system6,
	DAILINK_COMP_ARRAY(COMP_CPU("System Pin 6")));

 /* Back End DAI */
SND_SOC_DAILINK_DEF(ssp0b_pin,
	DAILINK_COMP_ARRAY(COMP_CPU("SSP0-B Pin")));
SND_SOC_DAILINK_DEF(ssp1b_pin,
	DAILINK_COMP_ARRAY(COMP_CPU("SSP1-B Pin")));
SND_SOC_DAILINK_DEF(ssp2b_pin,
	DAILINK_COMP_ARRAY(COMP_CPU("SSP2-B Pin")));

SND_SOC_DAILINK_DEF(platform,
	DAILINK_COMP_ARRAY(COMP_PLATFORM("0000:00:0e.0")));

#define PIN_NAME(ID)\
	ssp##ID

#define DAI_LINK_SSP_PIN(ID)\
SND_SOC_DAILINK_DEF(PIN_NAME(ID),\
	DAILINK_COMP_ARRAY(COMP_CPU("SSP"#ID" Pin")))

#define DAI_LINK(ID, PB, CP)\
{\
	.name = "SSP"#ID"-Codec",\
	.id = ID,\
	.ignore_suspend = 1,\
	.ignore_pmdown_time = 1,\
	.no_pcm = 1,\
	.dpcm_playback = PB,\
	.dpcm_capture = CP,\
	SND_SOC_DAILINK_REG(PIN_NAME(ID), dummy, platform),\
}

DAI_LINK_SSP_PIN(0);
DAI_LINK_SSP_PIN(1);
DAI_LINK_SSP_PIN(2);
DAI_LINK_SSP_PIN(3);

/* broxton digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link bxtp_ull_dais[] = {
	{
		.name = "Bxt Audio Port 3",
		.stream_name = "Stereo-16K SSP3",
		.dynamic = 1,
		.nonatomic = 1,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(system3, dummy, platform),
	},
	{
		.name = "Bxt Audio Port 4",
		.stream_name = "5-ch SSP1",
		.dynamic = 1,
		.nonatomic = 1,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(system4, dummy, platform),
	},
	{
		.name = "Bxt Audio Port 5",
		.stream_name = "SSP2 Stream",
		.dynamic = 1,
		.nonatomic = 1,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(system5, dummy, platform),
	},
	{
		.name = "Bxt Audio Port 6",
		.stream_name = "8-Ch SSP0",
		.dynamic = 1,
		.nonatomic = 1,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(system6, dummy, platform),
	},
	/* CODEC<->CODEC link */
	{
		.name = "Bxtn SSP0 Port",
		.stream_name = "Bxtn SSP0",
		.params = &codec1_in_params,
		SND_SOC_DAILINK_REG(ssp0b_pin, dummy, platform),
	},
	{
		.name = "Bxtn SSP2 port",
		.stream_name = "Bxtn SSP2",
		.params = &codec1_in_params,
		SND_SOC_DAILINK_REG(ssp2b_pin, dummy, platform),
	},
	{
		.name = "Bxtn SSP1 port",
		.stream_name = "Bxtn SSP2",
		.params = &media1_out_params,
		.params = &codec1_in_params,
		SND_SOC_DAILINK_REG(ssp1b_pin, dummy, platform),
	},

	/* Back End DAI links */
	DAI_LINK(3, 1, 1),
	DAI_LINK(1, 1, 0),
	DAI_LINK(2, 1, 1),
	DAI_LINK(0, 0, 1),
};

static int bxt_add_dai_link(struct snd_soc_card *card,
			struct snd_soc_dai_link *link)
{
	link->platforms->name = "0000:00:0e.0";
	link->nonatomic = 1;
	return 0;
}

/* broxton audio machine driver for ULL Dummy Codec*/
static struct snd_soc_card bxtp_ull = {
	.name = "bxtp-ull",
	.owner = THIS_MODULE,
	.dai_link = bxtp_ull_dais,
	.num_links = ARRAY_SIZE(bxtp_ull_dais),
	.dapm_widgets = broxton_widgets,
	.num_dapm_widgets = ARRAY_SIZE(broxton_widgets),
	.dapm_routes = bxtp_ull_map,
	.num_dapm_routes = ARRAY_SIZE(bxtp_ull_map),
	.fully_routed = false,
	.add_dai_link = bxt_add_dai_link,
};

static int broxton_audio_probe(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "%s registering %s\n", __func__, pdev->name);
	bxtp_ull.dev = &pdev->dev;
	return snd_soc_register_card(&bxtp_ull);
}

static int broxton_audio_remove(struct platform_device *pdev)
{
	snd_soc_unregister_card(&bxtp_ull);
	return 0;
}

static struct platform_driver broxton_audio = {
	.probe = broxton_audio_probe,
	.remove = broxton_audio_remove,
	.driver = {
		.name = "bxt_tdf8532",
		.pm = &snd_soc_pm_ops,
	},
};

module_platform_driver(broxton_audio)

/* Module information */
MODULE_DESCRIPTION("Intel SST Audio for Broxton ULL Machine");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bxt_tdf8532");
