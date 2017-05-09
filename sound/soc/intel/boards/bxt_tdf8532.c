// SPDX-License-Identifier: GPL-2.0
//
// bxt_tdf8532.c  --  Intel Broxton-P I2S Machine Driver for IVI reference
// platform
//
// Copyright (C) 2017 Intel Corporation

#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

static const struct snd_kcontrol_new broxton_tdf8532_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speaker"),
};

static const struct snd_soc_dapm_widget broxton_tdf8532_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_MIC("DiranaCp", NULL),
	SND_SOC_DAPM_HP("DiranaPb", NULL),
	SND_SOC_DAPM_MIC("HdmiIn", NULL),
	SND_SOC_DAPM_MIC("TestPinCp", NULL),
	SND_SOC_DAPM_HP("TestPinPb", NULL),
	SND_SOC_DAPM_MIC("BtHfpDl", NULL),
	SND_SOC_DAPM_HP("BtHfpUl", NULL),
	SND_SOC_DAPM_MIC("ModemDl", NULL),
	SND_SOC_DAPM_HP("ModemUl", NULL),
};

static const struct snd_soc_dapm_route broxton_tdf8532_map[] = {

	/* Speaker BE connections */
	{ "Speaker", NULL, "ssp4 Tx"},
	{ "ssp4 Tx", NULL, "codec0_out"},

	{ "dirana_in", NULL, "ssp2 Rx"},
	{ "ssp2 Rx", NULL, "DiranaCp"},

	{ "dirana_aux_in", NULL, "ssp2 Rx"},
	{ "ssp2 Rx", NULL, "DiranaCp"},

	{ "dirana_tuner_in", NULL, "ssp2 Rx"},
	{ "ssp2 Rx", NULL, "DiranaCp"},

	{ "DiranaPb", NULL, "ssp2 Tx"},
	{ "ssp2 Tx", NULL, "dirana_out"},

	{ "hdmi_ssp1_in", NULL, "ssp1 Rx"},
	{ "ssp1 Rx", NULL, "HdmiIn"},

	{ "TestPin_ssp5_in", NULL, "ssp5 Rx"},
	{ "ssp5 Rx", NULL, "TestPinCp"},

	{ "TestPinPb", NULL, "ssp5 Tx"},
	{ "ssp5 Tx", NULL, "TestPin_ssp5_out"},

	{ "BtHfp_ssp0_in", NULL, "ssp0 Rx"},
	{ "ssp0 Rx", NULL, "BtHfpDl"},

	{ "BtHfpUl", NULL, "ssp0 Tx"},
	{ "ssp0 Tx", NULL, "BtHfp_ssp0_out"},

	{ "Modem_ssp3_in", NULL, "ssp3 Rx"},
	{ "ssp3 Rx", NULL, "ModemDl"},

	{ "ModemUl", NULL, "ssp3 Tx"},
	{ "ssp3 Tx", NULL, "Modem_ssp3_out"},
};

static int bxt_tdf8532_ssp2_fixup(struct snd_soc_pcm_runtime *rtd,
				struct snd_pcm_hw_params *params)
{
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	/* set SSP to 32 bit */
	snd_mask_none(fmt);
	snd_mask_set(fmt, SNDRV_PCM_FORMAT_S32_LE);

	return 0;
}

SND_SOC_DAILINK_DEF(dummy,
	DAILINK_COMP_ARRAY(COMP_DUMMY()));

SND_SOC_DAILINK_DEF(tdf8532,
	DAILINK_COMP_ARRAY(COMP_CODEC("i2c-INT34C3:00", "tdf8532-hifi")));

SND_SOC_DAILINK_DEF(platform,
	DAILINK_COMP_ARRAY(COMP_PLATFORM("0000:00:0e.0")));

#define PIN_NAME(ID)\
	ssp##ID

#define DAI_LINK_SSP_PIN(ID)\
SND_SOC_DAILINK_DEF(PIN_NAME(ID),\
	DAILINK_COMP_ARRAY(COMP_CPU("SSP"#ID" Pin")))

#define DAI_LINK(ID, PB, CP, BEFIXUP)\
{\
	.name = "SSP"#ID"-Codec",\
	.id = ID,\
	.ignore_suspend = 1,\
	.ignore_pmdown_time = 1,\
	.no_pcm = 1,\
	.dpcm_playback = PB,\
	.dpcm_capture = CP,\
	.be_hw_params_fixup = BEFIXUP,\
	SND_SOC_DAILINK_REG(PIN_NAME(ID), dummy, platform),\
}

DAI_LINK_SSP_PIN(0);
DAI_LINK_SSP_PIN(1);
DAI_LINK_SSP_PIN(2);
DAI_LINK_SSP_PIN(3);
DAI_LINK_SSP_PIN(4);
DAI_LINK_SSP_PIN(5);

/* broxton digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link broxton_tdf8532_dais[] = {
	/* Back End DAI links */
	DAI_LINK(0, 1, 1, NULL), /* SSP0 - BT */
	DAI_LINK(1, 0, 1, NULL), /* SSP1 - HDMI-In */
	DAI_LINK(2, 1, 1, bxt_tdf8532_ssp2_fixup), /* SSP2 - Dirana */
	DAI_LINK(3, 0, 1, NULL), /* SSP3 - Modem */
	{
		/* SSP4 - Amplifier */
		.name = "SSP4-Codec",
		.id = 4,
		.ignore_suspend = 1,
		.dpcm_playback = 1,
		.no_pcm = 1,
		SND_SOC_DAILINK_REG(ssp4, tdf8532, platform),
	},
	DAI_LINK(5, 1, 1, NULL), /* SSP5 - TestPin */
};

static int bxt_add_dai_link(struct snd_soc_card *card,
			struct snd_soc_dai_link *link)
{
	link->platforms->name = "0000:00:0e.0";
	link->nonatomic = 1;
	return 0;
}

/* broxton audio machine driver for TDF8532 */
static struct snd_soc_card broxton_tdf8532 = {
	.name = "broxton_tdf8532",
	.dai_link = broxton_tdf8532_dais,
	.num_links = ARRAY_SIZE(broxton_tdf8532_dais),
	.controls = broxton_tdf8532_controls,
	.num_controls = ARRAY_SIZE(broxton_tdf8532_controls),
	.dapm_widgets = broxton_tdf8532_widgets,
	.num_dapm_widgets = ARRAY_SIZE(broxton_tdf8532_widgets),
	.dapm_routes = broxton_tdf8532_map,
	.num_dapm_routes = ARRAY_SIZE(broxton_tdf8532_map),
	.fully_routed = true,
	.add_dai_link = bxt_add_dai_link,
};

static int broxton_tdf8532_audio_probe(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "%s registering %s\n", __func__, pdev->name);
	broxton_tdf8532.dev = &pdev->dev;
	return snd_soc_register_card(&broxton_tdf8532);
}

static int broxton_tdf8532_audio_remove(struct platform_device *pdev)
{
	snd_soc_unregister_card(&broxton_tdf8532);
	return 0;
}

static struct platform_driver broxton_tdf8532_audio = {
	.probe = broxton_tdf8532_audio_probe,
	.remove = broxton_tdf8532_audio_remove,
	.driver = {
		.name = "bxt_tdf8532",
		.pm = &snd_soc_pm_ops,
	},
};

module_platform_driver(broxton_tdf8532_audio)

/* Module information */
MODULE_DESCRIPTION("Intel SST Audio for Broxton GP MRB");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bxt_tdf8532");
