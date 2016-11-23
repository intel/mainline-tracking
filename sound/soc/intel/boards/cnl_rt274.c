// SPDX-License-Identifier: GPL-2.0
//
// cnl_rt274.c  --  ASOC Machine driver for CNL
//
// Copyright (C) 2016 Intel Corp
// Author: Guneshwor Singh <guneshwor.o.singh@intel.com>

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/acpi.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/input.h>

#include "../../codecs/rt274.h"

static struct snd_soc_jack cnl_headset;

/* Headset jack detection DAPM pins */
static struct snd_soc_jack_pin cnl_headset_pins[] = {
	{
		.pin = "Mic Jack",
		.mask = SND_JACK_MICROPHONE,
	},
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};

static const struct snd_kcontrol_new cnl_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("Mic Jack"),
};

static const struct snd_soc_dapm_widget cnl_rt274_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_MIC("SoC DMIC", NULL),
};

static const struct snd_soc_pcm_stream dai_params_codec = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};

static int cnl_dmic_fixup(struct snd_soc_pcm_runtime *rtd,
				struct snd_pcm_hw_params *params)
{
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);
	channels->min = channels->max = 2;

	return 0;
}

static const struct snd_soc_dapm_route cnl_map[] = {
	{"Headphone Jack", NULL, "HPO Pin"},
	{"MIC", NULL, "Mic Jack"},
	{"DMic", NULL, "SoC DMIC"},
	{"DMIC01 Rx", NULL, "Capture"},
	{"dmic01_hifi", NULL, "DMIC01 Rx"},

	{"AIF1 Playback", NULL, "ssp0 Tx"},
	{"ssp0 Tx", NULL, "codec1_out"},
	{"ssp0 Tx", NULL, "codec0_out"},

	{"ssp0 Rx", NULL, "AIF1 Capture"},
	{"codec0_in", NULL, "ssp0 Rx"},
};

static int cnl_rt274_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	struct snd_soc_component *component = runtime->codec_dai->component;
	struct snd_soc_card *card = runtime->card;
	struct snd_soc_dai *codec_dai = runtime->codec_dai;

	ret = snd_soc_card_jack_new(runtime->card, "Headset",
		SND_JACK_HEADSET, &cnl_headset,
		cnl_headset_pins, ARRAY_SIZE(cnl_headset_pins));

	if (ret)
		return ret;

	snd_soc_component_set_jack(component, &cnl_headset, NULL);

	/* TDM 4 slots 24 bit, set Rx & Tx bitmask to 4 active slots */
	ret = snd_soc_dai_set_tdm_slot(codec_dai, 0xF, 0xF, 4, 24);
	if (ret < 0) {
		dev_err(runtime->dev, "can't set codec pcm format %d\n", ret);
		return ret;
	}

	card->dapm.idle_bias_off = true;

	return 0;
}

static int cnl_be_fixup(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;
	snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT));
	snd_mask_set(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
			(unsigned int __force)SNDRV_PCM_FORMAT_S24_LE);

	return 0;
}

#define CNL_FREQ_OUT 19200000

static int rt274_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret, ratio = 100;

	snd_soc_dai_set_bclk_ratio(codec_dai, ratio);

	ret = snd_soc_dai_set_pll(codec_dai, 0, RT274_PLL2_S_BCLK,
				  ratio * params_rate(params), CNL_FREQ_OUT);
	if (ret != 0) {
		dev_err(rtd->dev, "Failed to enable PLL2 with Ref Clock Loop: %d\n",
			ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, RT274_SCLK_S_PLL2, CNL_FREQ_OUT,
				     SND_SOC_CLOCK_IN);
	if (ret < 0)
		dev_err(rtd->dev, "set codec sysclk failed: %d\n", ret);

	return ret;
}

static struct snd_soc_ops rt274_ops = {
	.hw_params = rt274_hw_params,
};

static const char pname[] = "0000:00:1f.3";

SND_SOC_DAILINK_DEF(dummy,
	DAILINK_COMP_ARRAY(COMP_DUMMY()));

SND_SOC_DAILINK_DEF(ssp0_pin,
	DAILINK_COMP_ARRAY(COMP_CPU("SSP0 Pin")));
SND_SOC_DAILINK_DEF(ssp0_codec,
	DAILINK_COMP_ARRAY(COMP_CODEC("i2c-INT34C2:00", "rt274-aif1")));

SND_SOC_DAILINK_DEF(dmic01_pin,
	DAILINK_COMP_ARRAY(COMP_CPU("DMIC01 Pin")));
SND_SOC_DAILINK_DEF(dmic_codec,
	DAILINK_COMP_ARRAY(COMP_CODEC("dmic-codec", "dmic-hifi")));

SND_SOC_DAILINK_DEF(probe_pb,
	DAILINK_COMP_ARRAY(COMP_CPU("Probe Injection0 CPU DAI")));
SND_SOC_DAILINK_DEF(probe_cp,
	DAILINK_COMP_ARRAY(COMP_CPU("Probe Extraction CPU DAI")));

SND_SOC_DAILINK_DEF(platform,
	DAILINK_COMP_ARRAY(COMP_PLATFORM(pname)));

static struct snd_soc_dai_link cnl_rt274_msic_dailink[] = {
	/* back ends */
	{
		.name = "SSP0-Codec",
		.id = 1,
		.no_pcm = 1,
		.dai_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.ignore_suspend = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.init = cnl_rt274_init,
		.ops = &rt274_ops,
		.be_hw_params_fixup =  cnl_be_fixup,
		SND_SOC_DAILINK_REG(ssp0_pin, ssp0_codec, platform),
	},
	{
		.name = "dmic01",
		.id = 2,
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.no_pcm = 1,
		.be_hw_params_fixup = cnl_dmic_fixup,
		SND_SOC_DAILINK_REG(dmic01_pin, dmic_codec, platform),
	},
	/* Probe DAI links */
	{
		.name = "Compress Probe Playback",
		.init = NULL,
		.ignore_suspend = 1,
		.nonatomic = 1,
		SND_SOC_DAILINK_REG(probe_pb, dummy, platform),
	},
	{
		.name = "Compress Probe Capture",
		.init = NULL,
		.ignore_suspend = 1,
		.nonatomic = 1,
		SND_SOC_DAILINK_REG(probe_cp, dummy, platform),
	},};

static int
cnl_add_dai_link(struct snd_soc_card *card, struct snd_soc_dai_link *link)
{
	link->platforms->name = pname;
	link->nonatomic = 1;

	return 0;
}

/* SoC card */
static struct snd_soc_card snd_soc_card_cnl = {
	.name = "cnl-audio",
	.dai_link = cnl_rt274_msic_dailink,
	.num_links = ARRAY_SIZE(cnl_rt274_msic_dailink),
	.dapm_widgets = cnl_rt274_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cnl_rt274_widgets),
	.dapm_routes = cnl_map,
	.num_dapm_routes = ARRAY_SIZE(cnl_map),
	.controls = cnl_controls,
	.num_controls = ARRAY_SIZE(cnl_controls),
	.add_dai_link = cnl_add_dai_link,
};

static int snd_cnl_rt274_mc_probe(struct platform_device *pdev)
{
	snd_soc_card_cnl.dev = &pdev->dev;
	return devm_snd_soc_register_card(&pdev->dev, &snd_soc_card_cnl);
}

static const struct platform_device_id cnl_board_ids[] = {
	{ .name = "cnl_rt274" },
	{ }
};

static struct platform_driver snd_cnl_rt274_driver = {
	.driver = {
		.name = "cnl_rt274",
		.pm = &snd_soc_pm_ops,
	},
	.probe = snd_cnl_rt274_mc_probe,
	.id_table = cnl_board_ids,
};

module_platform_driver(snd_cnl_rt274_driver);

MODULE_AUTHOR("Guneshwor Singh <guneshwor.o.singh@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:cnl_rt274");
