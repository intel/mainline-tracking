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
#include <sound/jack.h>
#include "../skylake/skl.h"
#include "../../codecs/hdac_hdmi.h"

#define BXT_NAME_SIZE		32
#define BXT_MAX_HDMI		3
static struct snd_soc_jack bxt_hdmi[BXT_MAX_HDMI];
struct bxt_hdmi_pcm {
	struct list_head head;
	struct snd_soc_dai *codec_dai;
	int device;
};
struct bxt_private {
	struct list_head hdmi_pcm_list;
};

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

	{"hifi1", NULL, "iDisp1 Tx"},
	{"iDisp1 Tx", NULL, "iDisp1_out"},

	{"hifi2", NULL, "iDisp2 Tx"},
	{"iDisp2 Tx", NULL, "iDisp2_out"},

	{"hifi3", NULL, "iDisp3 Tx"},
	{"iDisp3 Tx", NULL, "iDisp3_out"},
};

static int broxton_hdmi_init(struct snd_soc_pcm_runtime *rtd)
{
	struct bxt_private *ctx = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *dai = rtd->codec_dai;
	struct bxt_hdmi_pcm *pcm;

	pcm = devm_kzalloc(rtd->card->dev, sizeof(*pcm), GFP_KERNEL);
	if (!pcm)
		return -ENOMEM;

	pcm->device = rtd->dai_link->id;
	pcm->codec_dai = dai;

	list_add_tail(&pcm->head, &ctx->hdmi_pcm_list);

	return 0;
}

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

SND_SOC_DAILINK_DEF(speaker,
	DAILINK_COMP_ARRAY(COMP_CPU("Speaker Pin")));

SND_SOC_DAILINK_DEF(dirana_cp,
	DAILINK_COMP_ARRAY(COMP_CPU("Dirana Cp Pin")));

SND_SOC_DAILINK_DEF(dirana_pb,
	DAILINK_COMP_ARRAY(COMP_CPU("Dirana Pb Pin")));

SND_SOC_DAILINK_DEF(test_cp,
	DAILINK_COMP_ARRAY(COMP_CPU("TestPin Cp Pin")));

SND_SOC_DAILINK_DEF(test_pb,
	DAILINK_COMP_ARRAY(COMP_CPU("TestPin Cp Pin")));

SND_SOC_DAILINK_DEF(bthfp_cp,
	DAILINK_COMP_ARRAY(COMP_CPU("BtHfp Cp Pin")));

SND_SOC_DAILINK_DEF(bthfp_pb,
	DAILINK_COMP_ARRAY(COMP_CPU("BtHfp Pb Pin")));

SND_SOC_DAILINK_DEF(modem_pb,
	DAILINK_COMP_ARRAY(COMP_CPU("Modem Pb Pin")));

SND_SOC_DAILINK_DEF(modem_cp,
	DAILINK_COMP_ARRAY(COMP_CPU("Modem Cp Pin")));

SND_SOC_DAILINK_DEF(hdmi_cp,
	DAILINK_COMP_ARRAY(COMP_CPU("HDMI Cp Pin")));

SND_SOC_DAILINK_DEF(dirana_aux_cp,
	DAILINK_COMP_ARRAY(COMP_CPU("Dirana Aux Cp Pin")));

SND_SOC_DAILINK_DEF(dirana_tuner_cp,
	DAILINK_COMP_ARRAY(COMP_CPU("Dirana Tuner Cp Pin")));

SND_SOC_DAILINK_DEF(tdf8532,
	DAILINK_COMP_ARRAY(COMP_CODEC("i2c-INT34C3:00", "tdf8532-hifi")));

SND_SOC_DAILINK_DEF(platform,
	DAILINK_COMP_ARRAY(COMP_PLATFORM("0000:00:0e.0")));

SND_SOC_DAILINK_DEF(hdmi1,
	DAILINK_COMP_ARRAY(COMP_CPU("HDMI1 Pin")));

SND_SOC_DAILINK_DEF(hdmi2,
	DAILINK_COMP_ARRAY(COMP_CPU("HDMI2 Pin")));

SND_SOC_DAILINK_DEF(hdmi3,
	DAILINK_COMP_ARRAY(COMP_CPU("HDMI3 Pin")));

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

/* broxton digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link broxton_tdf8532_dais[] = {
	/* Front End DAI links */
	{
		.name = "Speaker Port",
		.stream_name = "Speaker",
		.nonatomic = 1,
		.dynamic = 1,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(speaker, dummy, platform),
	},
	{
		.name = "Dirana Capture Port",
		.stream_name = "Dirana Cp",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
		SND_SOC_DAILINK_REG(dirana_cp, dummy, platform),
	},
	{
		.name = "Dirana Playback Port",
		.stream_name = "Dirana Pb",
		.nonatomic = 1,
		.dynamic = 1,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(dirana_pb, dummy, platform),
	},
	{
		.name = "TestPin Capture Port",
		.stream_name = "TestPin Cp",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
		SND_SOC_DAILINK_REG(test_cp, dummy, platform),
	},
	{
		.name = "TestPin Playback Port",
		.stream_name = "TestPin Pb",
		.nonatomic = 1,
		.dynamic = 1,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(test_pb, dummy, platform),
	},
	{
		.name = "BtHfp Capture Port",
		.stream_name = "BtHfp Cp",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
		SND_SOC_DAILINK_REG(bthfp_cp, dummy, platform),
	},
	{
		.name = "BtHfp Playback Port",
		.stream_name = "BtHfp Pb",
		.nonatomic = 1,
		.dynamic = 1,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(bthfp_pb, dummy, platform),
	},
	{
		.name = "Modem Capture Port",
		.stream_name = "Modem Cp",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
		SND_SOC_DAILINK_REG(modem_cp, dummy, platform),
	},
	{
		.name = "Modem Playback Port",
		.stream_name = "Modem Pb",
		.nonatomic = 1,
		.dynamic = 1,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(modem_pb, dummy, platform),
	},
	{
		.name = "HDMI Capture Port",
		.stream_name = "HDMI Cp",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
		SND_SOC_DAILINK_REG(hdmi_cp, dummy, platform),
	},
	{
		.name = "Dirana Aux Capture Port",
		.stream_name = "Dirana Aux Cp",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
		SND_SOC_DAILINK_REG(dirana_aux_cp, dummy, platform),
	},
	{
		.name = "Dirana Tuner Capture Port",
		.stream_name = "Dirana Tuner Cp",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
		SND_SOC_DAILINK_REG(dirana_tuner_cp, dummy, platform),
	},
	{
		.name = "Bxt HDMI Port1",
		.stream_name = "Hdmi1",
		.dpcm_playback = 1,
		.init = NULL,
		.nonatomic = 1,
		.dynamic = 1,
		SND_SOC_DAILINK_REG(hdmi1, dummy, platform),
	},
	{
		.name = "Bxt HDMI Port2",
		.stream_name = "Hdmi2",
		.dpcm_playback = 1,
		.init = NULL,
		.nonatomic = 1,
		.dynamic = 1,
		SND_SOC_DAILINK_REG(hdmi2, dummy, platform),
	},
	{
		.name = "Bxt HDMI Port3",
		.stream_name = "Hdmi3",
		.dpcm_playback = 1,
		.init = NULL,
		.nonatomic = 1,
		.dynamic = 1,
		SND_SOC_DAILINK_REG(hdmi3, dummy, platform),
	},
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
	{
		.name = "iDisp1",
		.id = 6,
		.init = broxton_hdmi_init,
		.dpcm_playback = 1,
		.no_pcm = 1,
		SND_SOC_DAILINK_REG(idisp1_pin, idisp1_codec, platform),
	},
	{
		.name = "iDisp2",
		.id = 7,
		.init = broxton_hdmi_init,
		.dpcm_playback = 1,
		.no_pcm = 1,
		SND_SOC_DAILINK_REG(idisp2_pin, idisp2_codec, platform),
	},
	{
		.name = "iDisp3",
		.id = 8,
		.init = broxton_hdmi_init,
		.dpcm_playback = 1,
		.no_pcm = 1,
		SND_SOC_DAILINK_REG(idisp3_pin, idisp3_codec, platform),
	},
};

static int bxt_card_late_probe(struct snd_soc_card *card)
{
	struct bxt_private *ctx = snd_soc_card_get_drvdata(card);
	struct snd_soc_component *component = NULL;
	char jack_name[BXT_NAME_SIZE];
	struct bxt_hdmi_pcm *pcm;
	int err, i = 0;

	if (list_empty(&ctx->hdmi_pcm_list))
		return 0;

	list_for_each_entry(pcm, &ctx->hdmi_pcm_list, head) {
		component = pcm->codec_dai->component;
		snprintf(jack_name, sizeof(jack_name),
			"HDMI/DP, pcm=%d Jack", pcm->device);
		err = snd_soc_card_jack_new(card, jack_name,
					SND_JACK_AVOUT, &bxt_hdmi[i],
					NULL, 0);
		if (err)
			return err;

		err = hdac_hdmi_jack_init(pcm->codec_dai,
					  pcm->device, &bxt_hdmi[i]);
		if (err < 0)
			return err;

		i++;
	}

	if (!component)
		return -EINVAL;

	return hdac_hdmi_jack_port_init(component, &card->dapm);
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
	.late_probe = bxt_card_late_probe,
};

static int broxton_tdf8532_audio_probe(struct platform_device *pdev)
{
	struct bxt_private *ctx;

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	INIT_LIST_HEAD(&ctx->hdmi_pcm_list);

	dev_info(&pdev->dev, "%s registering %s\n", __func__, pdev->name);
	broxton_tdf8532.dev = &pdev->dev;
	snd_soc_card_set_drvdata(&broxton_tdf8532, ctx);

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
