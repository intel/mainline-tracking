// SPDX-License-Identifier: GPL-2.0
//
// bxtp_ivi_rse_rt298.c  -- Intel RSE I2S Machine Driver
//
// Copyright(c) 2018 Intel Corporation

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
#include <linux/input.h>

static const struct snd_soc_dapm_widget broxton_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_MIC("DMIC2", NULL),
};

static const struct snd_soc_dapm_route broxton_rt298_map[] = {
	{"Speaker", NULL, "Dummy Playback"},
	{"Dummy Capture", NULL, "DMIC2"},
	/* BE connections */
	{ "Dummy Playback", NULL, "ssp4 Tx"},
	{ "ssp4 Tx", NULL, "codec0_out"},
	{ "Dummy Playback", NULL, "ssp2 Tx"},
	{ "ssp2 Tx", NULL, "codec1_out"},
	{ "Dummy Playback", NULL, "ssp1 Tx"},
	{ "ssp1 Tx", NULL, "codec2_out"},
	{ "Dummy Playback", NULL, "ssp1 Tx"},
	{ "ssp1 Tx", NULL, "codec3_out"},
	{ "hdmi_ssp0_in", NULL, "ssp0 Rx" },
	{ "ssp0 Rx", NULL, "Dummy Capture" },
	/* Test connections */
	{ "Dummy Playback", NULL, "ssp3 Tx"},
	{ "ssp3 Tx", NULL, "TestSSP3_out"},
	{ "TestSSP3_in", NULL, "ssp3 Rx" },
	{ "ssp3 Rx", NULL, "Dummy Capture" },
};

static int bxtp_ssp0_gpio_init(struct snd_soc_pcm_runtime *rtd)
{
	char *gpio_addr;
	u32 gpio_value1 = 0x40900500;
	u32 gpio_value2 = 0x44000600;

	gpio_addr = (void *)ioremap_nocache(0xd0c40610, 0x30);
	if (gpio_addr == NULL)
		return(-EIO);

	memcpy_toio(gpio_addr + 0x8, &gpio_value1, sizeof(gpio_value1));
	memcpy_toio(gpio_addr + 0x10, &gpio_value2, sizeof(gpio_value2));
	memcpy_toio(gpio_addr + 0x18, &gpio_value2, sizeof(gpio_value2));
	memcpy_toio(gpio_addr + 0x20, &gpio_value2, sizeof(gpio_value2));

	iounmap(gpio_addr);
	return 0;
}

static int bxtp_ssp1_gpio_init(struct snd_soc_pcm_runtime *rtd)
{

	char *gpio_addr;
	u32 gpio_value1 = 0x44000400;

	gpio_addr = (void *)ioremap_nocache(0xd0c40660, 0x30);
	if (gpio_addr == NULL)
		return(-EIO);

	memcpy_toio(gpio_addr + 0x8, &gpio_value1, sizeof(gpio_value1));
	memcpy_toio(gpio_addr + 0x10, &gpio_value1, sizeof(gpio_value1));
	memcpy_toio(gpio_addr + 0x18, &gpio_value1, sizeof(gpio_value1));
	memcpy_toio(gpio_addr + 0x20, &gpio_value1, sizeof(gpio_value1));

	iounmap(gpio_addr);
	return 0;
}

static int bxtp_ssp4_gpio_init(struct snd_soc_pcm_runtime *rtd)
{

	char *gpio_addr;
	u32 gpio_value1 = 0x44000A00;
	u32 gpio_value2 = 0x44000800;

	gpio_addr = (void *)ioremap_nocache(0xd0c705A0, 0x30);
	if (gpio_addr == NULL)
		return(-EIO);

	memcpy_toio(gpio_addr, &gpio_value1, sizeof(gpio_value1));
	memcpy_toio(gpio_addr + 0x8, &gpio_value1, sizeof(gpio_value1));
	memcpy_toio(gpio_addr + 0x10, &gpio_value1, sizeof(gpio_value1));
	memcpy_toio(gpio_addr + 0x18, &gpio_value2, sizeof(gpio_value2));

	iounmap(gpio_addr);
	return 0;

}

static int bxtp_ssp3_gpio_init(struct snd_soc_pcm_runtime *rtd)
{

	char *gpio_addr;
	u32 gpio_value1 = 0x44000800;
	u32 gpio_value2 = 0x44000802;

	gpio_addr = (void *)ioremap_nocache(0xd0c40638, 0x30);
	if (gpio_addr == NULL)
		return(-EIO);

	memcpy_toio(gpio_addr, &gpio_value1, sizeof(gpio_value1));
	memcpy_toio(gpio_addr + 0x8, &gpio_value2, sizeof(gpio_value2));
	memcpy_toio(gpio_addr + 0x10, &gpio_value1, sizeof(gpio_value1));
	memcpy_toio(gpio_addr + 0x18, &gpio_value1, sizeof(gpio_value1));

	iounmap(gpio_addr);
	return 0;
}

static int broxton_ssp1_fixup(struct snd_soc_pcm_runtime *rtd,
		struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_CHANNELS);

	/* The ADSP will covert the FE rate to 48k, 2 Channel */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	/* set SSP1 to 16 bit */
	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
			SNDRV_PCM_HW_PARAM_FIRST_MASK],
			SNDRV_PCM_FORMAT_S16_LE);
	return 0;
}

static int broxton_ssp2_fixup(struct snd_soc_pcm_runtime *rtd,
		struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_CHANNELS);

	/* The ADSP will covert the FE rate to 44k, stereo */
	rate->min = rate->max = 44100;
	channels->min = channels->max = 2;

	/* set SSP2 to 16 bit */
	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
			SNDRV_PCM_HW_PARAM_FIRST_MASK],
			SNDRV_PCM_FORMAT_S16_LE);
	return 0;
}

static int broxton_ssp4_fixup(struct snd_soc_pcm_runtime *rtd,
		struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_CHANNELS);

	/* The ADSP will covert the FE rate to 44k, stereo */
	rate->min = rate->max = 44100;
	channels->min = channels->max = 2;

	/* set SSP4 to 16 bit */
	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
			SNDRV_PCM_HW_PARAM_FIRST_MASK],
			SNDRV_PCM_FORMAT_S16_LE);
	return 0;
}

SND_SOC_DAILINK_DEF(dummy,
	DAILINK_COMP_ARRAY(COMP_DUMMY()));

SND_SOC_DAILINK_DEF(ssp0_pin,
	DAILINK_COMP_ARRAY(COMP_CPU("SSP0 Pin")));
SND_SOC_DAILINK_DEF(ssp1_pin,
	DAILINK_COMP_ARRAY(COMP_CPU("SSP1 Pin")));
SND_SOC_DAILINK_DEF(ssp2_pin,
	DAILINK_COMP_ARRAY(COMP_CPU("SSP2 Pin")));
SND_SOC_DAILINK_DEF(ssp3_pin,
	DAILINK_COMP_ARRAY(COMP_CPU("SSP3 Pin")));
SND_SOC_DAILINK_DEF(ssp4_pin,
	DAILINK_COMP_ARRAY(COMP_CPU("SSP4 Pin")));

SND_SOC_DAILINK_DEF(platform,
	DAILINK_COMP_ARRAY(COMP_PLATFORM("0000:00:0e.0")));

struct snd_soc_dai_link broxton_rt298_dais[] = {
	/* Back End DAI links */
	{	.name = "SSP0-Codec",
		.id = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.no_pcm = 1,
		.init = bxtp_ssp0_gpio_init,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(ssp0_pin, dummy, platform),
	},
	{
		.name = "SSP1-Codec",
		.id = 2,
		.be_hw_params_fixup = broxton_ssp1_fixup,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.no_pcm = 1,
		.init = bxtp_ssp1_gpio_init,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(ssp1_pin, dummy, platform),
	},
	{
		.name = "SSP2-Codec",
		.id = 3,
		.be_hw_params_fixup = broxton_ssp2_fixup,
		.ignore_suspend = 1,
		.no_pcm = 1,
		.init = NULL,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(ssp2_pin, dummy, platform),
	},
	{
		.name = "SSP3-Codec",
		.id = 4,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.no_pcm = 1,
		.init = bxtp_ssp3_gpio_init,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(ssp3_pin, dummy, platform),
	},
	{
		.name = "SSP4-Codec",
		.id = 5,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.no_pcm = 1,
		.init =  bxtp_ssp4_gpio_init,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.be_hw_params_fixup = broxton_ssp4_fixup,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(ssp4_pin, dummy, platform),
	},
};

static int
bxt_add_dai_link(struct snd_soc_card *card, struct snd_soc_dai_link *link)
{
	link->platforms->name = "0000:00:0e.0";
	link->nonatomic = 1;

	return 0;
}

/* SoC card */
static struct snd_soc_card broxton_rt298 = {
	.name = "broxton-ivi-rse",
	.dai_link = broxton_rt298_dais,
	.num_links = ARRAY_SIZE(broxton_rt298_dais),
	.dapm_widgets = broxton_widgets,
	.num_dapm_widgets = ARRAY_SIZE(broxton_widgets),
	.dapm_routes = broxton_rt298_map,
	.num_dapm_routes = ARRAY_SIZE(broxton_rt298_map),
	.controls = NULL,
	.num_controls = 0,
	.fully_routed = true,
	.add_dai_link = bxt_add_dai_link,
};

static int broxton_audio_probe(struct platform_device *pdev)
{
	broxton_rt298.dev = &pdev->dev;
	return snd_soc_register_card(&broxton_rt298);
}

static int broxton_audio_remove(struct platform_device *pdev)
{
	snd_soc_unregister_card(&broxton_rt298);
	return 0;
}

static struct platform_driver broxton_audio = {
	.probe = broxton_audio_probe,
	.remove = broxton_audio_remove,
	.driver = {
		.name = "bxt_alc298s_i2s",
		.pm = &snd_soc_pm_ops,
	},
};

module_platform_driver(broxton_audio);

/* Module information */
MODULE_AUTHOR("Pardha Saradhi K <pardha.saradhi.kesapragada@intel.com>");
MODULE_AUTHOR("Ramesh Babu <Ramesh.Babu@intel.com>");
MODULE_AUTHOR("Senthilnathan Veppur <senthilnathanx.veppur@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bxt_alc298s_i2s");
