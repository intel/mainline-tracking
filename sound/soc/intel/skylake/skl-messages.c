// SPDX-License-Identifier: GPL-2.0-only
/*
 *  skl-message.c - HDA DSP interface for FW registration, Pipe and Module
 *  configurations
 *
 *  Copyright (C) 2015 Intel Corp
 *  Author:Rafal Redzimski <rafal.f.redzimski@intel.com>
 *	   Jeeja KP <jeeja.kp@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/slab.h>
#include <linux/pci.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/intel-nhlt.h>
#include <uapi/linux/kernel.h>
#include <uapi/sound/skl-tplg-interface.h>
#include "skl-sst-dsp.h"
#include "cnl-sst-dsp.h"
#include "skl-sst-ipc.h"
#include "skl.h"
#include "../common/sst-dsp.h"
#include "../common/sst-dsp-priv.h"
#include "skl-topology.h"

#define INVALID_PIPELINE_ID	0xFF
#define ASRC_MODE_UPLINK	BIT(0)
#define ASRC_MODE_DOWNLINK	BIT(1)
#define SKL_ENABLE_ALL_CHANNELS  0xffffffff

int skl_alloc_dma_buf(struct device *dev,
		struct snd_dma_buffer *dmab, size_t size)
{
	return snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV, dev, size, dmab);
}

int skl_free_dma_buf(struct device *dev, struct snd_dma_buffer *dmab)
{
	snd_dma_free_pages(dmab);
	return 0;
}

void skl_dsp_set_astate_cfg(struct skl_dev *skl, u32 cnt, void *data)
{
	struct skl_ipc_large_config_msg	msg = {0};

	msg.large_param_id = SKL_BASEFW_ASTATE_TABLE;
	msg.param_data_size = (cnt * sizeof(struct skl_astate_param) +
				sizeof(cnt));

	skl_ipc_set_large_config(&skl->ipc, &msg, data);
}

static int skl_dsp_setup_spib(struct device *dev, unsigned int size,
				int stream_tag, int enable)
{
	struct hdac_bus *bus = dev_get_drvdata(dev);
	struct hdac_stream *stream = snd_hdac_get_stream(bus,
			SNDRV_PCM_STREAM_PLAYBACK, stream_tag);
	struct hdac_ext_stream *estream;

	if (!stream)
		return -EINVAL;

	estream = stream_to_hdac_ext_stream(stream);
	/* enable/disable SPIB for this hdac stream */
	snd_hdac_ext_stream_spbcap_enable(bus, enable, stream->index);

	/* set the spib value */
	snd_hdac_ext_stream_set_spib(bus, estream, size);

	return 0;
}

int skl_dsp_prepare(struct device *dev, unsigned int format,
		unsigned int size, struct snd_dma_buffer *dmab, int direction)
{
	struct hdac_bus *bus = dev_get_drvdata(dev);
	struct hdac_ext_stream *estream;
	struct hdac_stream *stream;
	struct snd_pcm_substream substream;
	int ret;

	if (!bus)
		return -ENODEV;

	memset(&substream, 0, sizeof(substream));

	substream.stream = direction;

	estream = snd_hdac_ext_stream_assign(bus, &substream,
					HDAC_EXT_STREAM_TYPE_HOST);
	if (!estream)
		return -ENODEV;

	stream = hdac_stream(estream);

	/* assign decouple host dma channel */
	ret = snd_hdac_dsp_prepare(stream, format, size, dmab);
	if (ret < 0)
		return ret;

	skl_dsp_setup_spib(dev, size, stream->stream_tag, true);

	return stream->stream_tag;
}

int skl_dsp_trigger(struct device *dev, bool start, int stream_tag,
		int direction)
{
	struct hdac_bus *bus = dev_get_drvdata(dev);
	struct hdac_stream *stream;

	if (!bus)
		return -ENODEV;

	stream = snd_hdac_get_stream(bus, direction, stream_tag);
	if (!stream)
		return -EINVAL;

	snd_hdac_dsp_trigger(stream, start);

	return 0;
}

int skl_dsp_cleanup(struct device *dev,
		struct snd_dma_buffer *dmab, int stream_tag, int direction)
{
	struct hdac_bus *bus = dev_get_drvdata(dev);
	struct hdac_stream *stream;
	struct hdac_ext_stream *estream;

	if (!bus)
		return -ENODEV;

	stream = snd_hdac_get_stream(bus, direction, stream_tag);
	if (!stream)
		return -EINVAL;

	estream = stream_to_hdac_ext_stream(stream);
	skl_dsp_setup_spib(dev, 0, stream_tag, false);
	snd_hdac_ext_stream_release(estream, HDAC_EXT_STREAM_TYPE_HOST);

	snd_hdac_dsp_cleanup(stream, dmab);

	return 0;
}

int skl_init_dsp(struct skl_dev *skl, struct sst_pdata *pdata)
{
	struct sst_dsp *dsp;
	struct hdac_bus *bus = skl_to_bus(skl);
	struct device *dev = skl->dev;

	/* enable ppcap interrupt */
	snd_hdac_ext_bus_ppcap_enable(bus, true);
	snd_hdac_ext_bus_ppcap_int_enable(bus, true);

	skl->is_first_boot = true;
	spin_lock_init(&skl->trace_lock);
	INIT_LIST_HEAD(&skl->notify_kctls);
	INIT_LIST_HEAD(&skl->cfg.dmactrl_list);
	init_waitqueue_head(&skl->trace_waitq);
	init_waitqueue_head(&skl->mod_load_wait);

	pdata->id = skl->pci->device;
	pdata->irq = skl->pci->irq;
	pdata->dma_base = -1;
	pdata->dsp = skl;
	dsp = sst_dsp_new(dev, pdata);
	if (!dsp) {
		dev_err(dev, "%s: no device\n", __func__);
		return -ENODEV;
	}

	return 0;
}

int skl_free_dsp(struct skl_dev *skl)
{
	struct hdac_bus *bus = skl_to_bus(skl);

	/* disable  ppcap interrupt */
	snd_hdac_ext_bus_ppcap_int_enable(bus, false);

	skl_sst_dsp_cleanup(skl);

	kfree(skl->hw_cfg.i2s_caps.ctrl_base_addr);
	kfree(skl->cores.state);
	kfree(skl->cores.usage_count);

	if (skl->dsp->addr.lpe)
		iounmap(skl->dsp->addr.lpe);

	return 0;
}

/*
 * In the case of "suspend_active" i.e, the Audio IP being active
 * during system suspend, immediately excecute any pending D0i3 work
 * before suspending. This is needed for the IP to work in low power
 * mode during system suspend. In the case of normal suspend, cancel
 * any pending D0i3 work.
 */
int skl_suspend_late_dsp(struct skl_dev *skl)
{
	struct delayed_work *dwork;

	if (!skl)
		return 0;

	dwork = &skl->d0i3.work;

	if (dwork->work.func) {
		if (skl->suspend_active)
			flush_delayed_work(dwork);
		else
			cancel_delayed_work_sync(dwork);
	}

	return 0;
}

int skl_suspend_dsp(struct skl_dev *skl)
{
	struct hdac_bus *bus = skl_to_bus(skl);
	int ret;

	/* if ppcap is not supported return 0 */
	if (!bus->ppcap)
		return 0;

	ret = skl_dsp_sleep(skl->dsp);
	if (ret < 0)
		return ret;

	/* disable ppcap interrupt */
	snd_hdac_ext_bus_ppcap_int_enable(bus, false);
	snd_hdac_ext_bus_ppcap_enable(bus, false);

	return 0;
}

int skl_resume_dsp(struct skl_dev *skl)
{
	struct hdac_bus *bus = skl_to_bus(skl);
	int ret;

	/* if ppcap is not supported return 0 */
	if (!bus->ppcap)
		return 0;

	/* enable ppcap interrupt */
	snd_hdac_ext_bus_ppcap_enable(bus, true);
	snd_hdac_ext_bus_ppcap_int_enable(bus, true);

	/* check if DSP 1st boot is done */
	if (skl->is_first_boot)
		return 0;

	/*
	 * Disable dynamic clock and power gating during firmware
	 * and library download
	 */
	skl->enable_miscbdcge(skl->dev, false);
	skl->clock_power_gating(skl->dev, false);

	ret = skl_dsp_wake(skl->dsp);
	skl->enable_miscbdcge(skl->dev, true);
	skl->clock_power_gating(skl->dev, true);
	if (ret < 0)
		return ret;

	if (skl->cfg.astate_cfg != NULL) {
		skl_dsp_set_astate_cfg(skl, skl->cfg.astate_cfg->count,
					skl->cfg.astate_cfg);
	}

	/* Set the FW config info from topology */
	skl_tplg_fw_cfg_set(skl);

	return ret;
}

enum skl_bitdepth skl_get_bit_depth(int params)
{
	switch (params) {
	case 8:
		return SKL_DEPTH_8BIT;

	case 16:
		return SKL_DEPTH_16BIT;

	case 24:
		return SKL_DEPTH_24BIT;

	case 32:
		return SKL_DEPTH_32BIT;

	default:
		return SKL_DEPTH_INVALID;

	}
}

/*
 * Each module in DSP expects a base module configuration, which consists of
 * PCM format information, which we calculate in driver and resource values
 * which are read from widget information passed through topology binary
 * This is send when we create a module with INIT_INSTANCE IPC msg
 */
static void skl_set_base_module_format(struct skl_dev *skl,
			struct skl_module_cfg *mconfig,
			struct skl_base_cfg *base_cfg)
{
	struct skl_module *module = mconfig->module;
	struct skl_module_res *res = &module->resources[mconfig->res_idx];
	struct skl_module_iface *fmt = &module->formats[mconfig->fmt_idx];
	struct skl_module_fmt *format = &fmt->inputs[0].fmt;

	base_cfg->audio_fmt.number_of_channels = format->channels;

	base_cfg->audio_fmt.s_freq = format->s_freq;
	base_cfg->audio_fmt.bit_depth = format->bit_depth;
	base_cfg->audio_fmt.valid_bit_depth = format->valid_bit_depth;
	base_cfg->audio_fmt.ch_cfg = format->ch_cfg;
	base_cfg->audio_fmt.sample_type = format->sample_type;

	dev_dbg(skl->dev, "bit_depth=%x valid_bd=%x ch_config=%x\n",
			format->bit_depth, format->valid_bit_depth,
			format->ch_cfg);

	base_cfg->audio_fmt.channel_map = format->ch_map;

	base_cfg->audio_fmt.interleaving = format->interleaving_style;

	base_cfg->cpc = res->cpc;
	base_cfg->ibs = res->ibs;
	base_cfg->obs = res->obs;
	base_cfg->is_pages = res->is_pages;
}

static void fill_pin_params(struct skl_audio_data_format *pin_fmt,
				struct skl_module_fmt *format)
{
	pin_fmt->number_of_channels = format->channels;
	pin_fmt->s_freq = format->s_freq;
	pin_fmt->bit_depth = format->bit_depth;
	pin_fmt->valid_bit_depth = format->valid_bit_depth;
	pin_fmt->ch_cfg = format->ch_cfg;
	pin_fmt->sample_type = format->sample_type;
	pin_fmt->channel_map = format->ch_map;
	pin_fmt->interleaving = format->interleaving_style;
}

/*
 * Any module configuration begins with a base module configuration but
 * can be followed by a generic extension containing audio format for all
 * modules's pins that are in use.
 */
static void skl_set_base_ext_module_format(struct skl_dev *skl,
			struct skl_module_cfg *mconfig,
			struct skl_base_cfg_ext *base_cfg_ext)
{
	int i;
	struct skl_module_pin_resources *pin_res;
	struct skl_pin_format *pin_fmt;
	struct skl_module *module = mconfig->module;
	struct skl_module_res *res = &module->resources[mconfig->res_idx];
	struct skl_module_iface *fmt = &module->formats[mconfig->fmt_idx];
	struct skl_module_fmt *format;

	base_cfg_ext->nr_input_pins = res->nr_input_pins;
	base_cfg_ext->nr_output_pins = res->nr_output_pins;
	base_cfg_ext->priv_param_length = 0;

	for (i = 0; i < res->nr_input_pins; i++) {
		pin_res = &res->input[i];
		pin_fmt = &base_cfg_ext->pins_fmt[i];

		pin_fmt->pin_idx = pin_res->pin_index;
		pin_fmt->buf_size = pin_res->buf_size;

		format = &fmt->inputs[pin_res->pin_index].fmt;
		fill_pin_params(&pin_fmt->audio_fmt, format);
	}

	for (i = 0; i < res->nr_output_pins; i++) {
		pin_res = &res->output[i];
		pin_fmt = &base_cfg_ext->pins_fmt[res->nr_input_pins + i];

		pin_fmt->pin_idx = pin_res->pin_index;
		pin_fmt->buf_size = pin_res->buf_size;

		format = &fmt->outputs[pin_res->pin_index].fmt;
		fill_pin_params(&pin_fmt->audio_fmt, format);
	}
}

/*
 * Copies copier capabilities into copier module and updates copier module
 * config size.
 */
static void skl_copy_copier_caps(struct skl_module_cfg *mconfig,
				struct skl_cpr_cfg *cpr_mconfig)
{
	if (mconfig->formats_config[SKL_PARAM_INIT].caps_size == 0)
		return;

	memcpy(cpr_mconfig->gtw_cfg.config_data,
			mconfig->formats_config[SKL_PARAM_INIT].caps,
			mconfig->formats_config[SKL_PARAM_INIT].caps_size);

	cpr_mconfig->gtw_cfg.config_length =
			(mconfig->formats_config[SKL_PARAM_INIT].caps_size) / 4;
}

#define SKL_NON_GATEWAY_CPR_NODE_ID 0xFFFFFFFF
/*
 * Calculate the gatewat settings required for copier module, type of
 * gateway and index of gateway to use
 */
static u32 skl_get_node_id(struct skl_dev *skl,
			struct skl_module_cfg *mconfig)
{
	union skl_connector_node_id node_id = {0};
	union skl_ssp_dma_node ssp_node  = {0};
	struct skl_pipe_params *params = mconfig->pipe->p_params;

	switch (mconfig->dev_type) {
	case SKL_DEVICE_BT:
		node_id.node.dma_type =
			(SKL_CONN_SOURCE == mconfig->hw_conn_type) ?
			SKL_DMA_I2S_LINK_OUTPUT_CLASS :
			SKL_DMA_I2S_LINK_INPUT_CLASS;
		node_id.node.vindex = params->host_dma_id +
					(mconfig->vbus_id << 3);
		break;

	case SKL_DEVICE_I2S:
		node_id.node.dma_type =
			(SKL_CONN_SOURCE == mconfig->hw_conn_type) ?
			SKL_DMA_I2S_LINK_OUTPUT_CLASS :
			SKL_DMA_I2S_LINK_INPUT_CLASS;
		ssp_node.dma_node.time_slot_index = mconfig->time_slot;
		ssp_node.dma_node.i2s_instance = mconfig->vbus_id;
		node_id.node.vindex = ssp_node.val;
		break;

	case SKL_DEVICE_DMIC:
		node_id.node.dma_type = SKL_DMA_DMIC_LINK_INPUT_CLASS;
		node_id.node.vindex = mconfig->vbus_id +
					 (mconfig->time_slot);
		break;

	case SKL_DEVICE_HDALINK:
		node_id.node.dma_type =
			(SKL_CONN_SOURCE == mconfig->hw_conn_type) ?
			SKL_DMA_HDA_LINK_OUTPUT_CLASS :
			SKL_DMA_HDA_LINK_INPUT_CLASS;
		node_id.node.vindex = params->link_dma_id;
		break;

	case SKL_DEVICE_HDAHOST:
		node_id.node.dma_type =
			(SKL_CONN_SOURCE == mconfig->hw_conn_type) ?
			SKL_DMA_HDA_HOST_OUTPUT_CLASS :
			SKL_DMA_HDA_HOST_INPUT_CLASS;
		node_id.node.vindex = params->host_dma_id;
		break;

	default:
		node_id.val = 0xFFFFFFFF;
		break;
	}

	return node_id.val;
}

static void skl_setup_cpr_gateway_cfg(struct skl_dev *skl,
			struct skl_module_cfg *mconfig,
			struct skl_cpr_cfg *cpr_mconfig)
{
	u32 dma_io_buf;
	struct skl_module_res *res;
	int res_idx = mconfig->res_idx;

	cpr_mconfig->gtw_cfg.node_id = skl_get_node_id(skl, mconfig);

	if (cpr_mconfig->gtw_cfg.node_id == SKL_NON_GATEWAY_CPR_NODE_ID) {
		cpr_mconfig->cpr_feature_mask = 0;
		return;
	}

	if (skl->nr_modules) {
		res = &mconfig->module->resources[mconfig->res_idx];
		cpr_mconfig->gtw_cfg.dma_buffer_size = res->dma_buffer_size;
		goto skip_buf_size_calc;
	} else {
		res = &mconfig->module->resources[res_idx];
	}

	switch (mconfig->hw_conn_type) {
	case SKL_CONN_SOURCE:
		if (mconfig->dev_type == SKL_DEVICE_HDAHOST)
			dma_io_buf =  res->ibs;
		else
			dma_io_buf =  res->obs;
		break;

	case SKL_CONN_SINK:
		if (mconfig->dev_type == SKL_DEVICE_HDAHOST)
			dma_io_buf =  res->obs;
		else
			dma_io_buf =  res->ibs;
		break;

	default:
		dev_warn(skl->dev, "wrong connection type: %d\n",
				mconfig->hw_conn_type);
		return;
	}

	cpr_mconfig->gtw_cfg.dma_buffer_size =
				mconfig->dma_buffer_size * dma_io_buf;

	/* fallback to 2ms default value */
	if (!cpr_mconfig->gtw_cfg.dma_buffer_size) {
		if (mconfig->hw_conn_type == SKL_CONN_SOURCE)
			cpr_mconfig->gtw_cfg.dma_buffer_size = 2 * res->obs;
		else
			cpr_mconfig->gtw_cfg.dma_buffer_size = 2 * res->ibs;
	}

skip_buf_size_calc:
	cpr_mconfig->cpr_feature_mask = 0;
	cpr_mconfig->gtw_cfg.config_length  = 0;

	skl_copy_copier_caps(mconfig, cpr_mconfig);
}

#define DMA_I2S_BLOB_SIZE 21

int skl_dsp_set_dma_control(struct skl_dev *skl, u32 *caps,
				u32 caps_size, u32 node_id)
{
	struct skl_dma_control *dma_ctrl;
	struct skl_ipc_large_config_msg msg = {0};
	int err = 0;


	/*
	 * if blob size zero, then return
	 */
	if (caps_size == 0)
		return 0;

	msg.large_param_id = SKL_BASEFW_DMA_CONTROL;
	msg.param_data_size = sizeof(struct skl_dma_control) + caps_size;

	dma_ctrl = kzalloc(msg.param_data_size, GFP_KERNEL);
	if (dma_ctrl == NULL)
		return -ENOMEM;

	dma_ctrl->node_id = node_id;

	/*
	 * NHLT blob may contain additional configs along with i2s blob.
	 * firmware expects only the i2s blob size as the config_length.
	 * So fix to i2s blob size.
	 * size in dwords.
	 */
	dma_ctrl->config_length = DMA_I2S_BLOB_SIZE;

	memcpy(dma_ctrl->config_data, caps, caps_size);

	err = skl_ipc_set_large_config(&skl->ipc, &msg, (u32 *)dma_ctrl);

	kfree(dma_ctrl);

	return err;
}
EXPORT_SYMBOL_GPL(skl_dsp_set_dma_control);

static u32 skl_prepare_i2s_node_id(u32 instance, u8 dev_type,
				u32 dir, u32 time_slot)
{
	union skl_connector_node_id node_id = {0};
	union skl_ssp_dma_node ssp_node  = {0};

	node_id.node.dma_type = (dir == SNDRV_PCM_STREAM_PLAYBACK) ?
					SKL_DMA_I2S_LINK_OUTPUT_CLASS :
					SKL_DMA_I2S_LINK_INPUT_CLASS;
	ssp_node.dma_node.time_slot_index = time_slot;
	ssp_node.dma_node.i2s_instance = instance;
	node_id.node.vindex = ssp_node.val;

	return node_id.val;
}

int skl_dsp_set_dma_clk_controls(struct skl_dev *skl)
{
	struct nhlt_specific_cfg *cfg = NULL;
	struct skl_dmctrl_hdr *hdr;
	struct skl_dmactrl_node *dmactrl;
	u8 *dma_ctrl_config;
	void *i2s_config = NULL;
	u32 i2s_config_size, node_id;
	int ret = 0;

	list_for_each_entry_reverse(dmactrl,
			&skl->cfg.dmactrl_list, node) {
		hdr = &dmactrl->hdr;

		/* get nhlt specific config info */
		cfg = skl_get_nhlt_specific_cfg(skl, hdr->vbus_id,
					NHLT_LINK_SSP, hdr->fmt,
					hdr->ch, hdr->freq,
					hdr->direction, NHLT_DEVICE_I2S);
		if (!cfg || !hdr->data_size) {
			dev_warn(skl->dev,
				"Failed to get NHLT config: dma_cfg_idx=%u vbusi_id=%u ch=%d fmt=%u s_rate=%u\n",
				dmactrl->idx, hdr->vbus_id,
				hdr->ch, hdr->fmt, hdr->freq);
				continue;
		}
		print_hex_dump(KERN_DEBUG, "NHLT blob Info:",
				DUMP_PREFIX_OFFSET, 8, 4,
				cfg->caps, cfg->size, false);

		i2s_config_size = cfg->size + hdr->data_size;
		i2s_config = kzalloc(i2s_config_size, GFP_KERNEL);
		if (!i2s_config)
			return -ENOMEM;

		/* copy blob */
		memcpy(i2s_config, cfg->caps, cfg->size);

		/* copy additional dma controls informatioin */
		dma_ctrl_config = (u8 *)i2s_config + cfg->size;
		memcpy(dma_ctrl_config, hdr->data, hdr->data_size);

		print_hex_dump(KERN_DEBUG, "Blob + DMA Control Info:",
				DUMP_PREFIX_OFFSET, 8, 4,
				i2s_config, i2s_config_size, false);

		/* get node id */
		node_id = skl_prepare_i2s_node_id(hdr->vbus_id,
						SKL_DEVICE_I2S,
						hdr->direction,
						hdr->tdm_slot);

		ret = skl_dsp_set_dma_control(skl, (u32 *)i2s_config,
						i2s_config_size, node_id);

		kfree(i2s_config);

		if (ret < 0)
			dev_err(skl->dev,
				"Failed to set dma_clk_cfg, errno[%d]: dma_cfg_idx=%u vbusi_id=%u ch=%d fmt=%u s_rate=%u\n",
				ret, dmactrl->idx, hdr->vbus_id,
				hdr->ch, hdr->fmt, hdr->freq);

	}

	return 0;
}

static void skl_setup_out_format(struct skl_dev *skl,
			struct skl_module_cfg *mconfig,
			struct skl_audio_data_format *out_fmt)
{
	struct skl_module *module = mconfig->module;
	struct skl_module_iface *fmt = &module->formats[mconfig->fmt_idx];
	struct skl_module_fmt *format = &fmt->outputs[0].fmt;

	out_fmt->number_of_channels = (u8)format->channels;
	out_fmt->s_freq = format->s_freq;
	out_fmt->bit_depth = format->bit_depth;
	out_fmt->valid_bit_depth = format->valid_bit_depth;
	out_fmt->ch_cfg = format->ch_cfg;

	out_fmt->channel_map = format->ch_map;
	out_fmt->interleaving = format->interleaving_style;
	out_fmt->sample_type = format->sample_type;

	dev_dbg(skl->dev, "copier out format chan=%d fre=%d bitdepth=%d\n",
		out_fmt->number_of_channels, format->s_freq, format->bit_depth);
}

static int skl_set_gain_format(struct skl_dev *skl,
			struct skl_module_cfg *mconfig,
			struct skl_gain_module_config *gain_mconfig)
{
	struct skl_gain_data *gain_fmt = mconfig->gain_data;

	skl_set_base_module_format(skl, mconfig,
			(struct skl_base_cfg *)gain_mconfig);
	gain_mconfig->gain_cfg.channel_id = SKL_ENABLE_ALL_CHANNELS;
	gain_mconfig->gain_cfg.target_volume = gain_fmt->volume[0];
	gain_mconfig->gain_cfg.ramp_type = gain_fmt->ramp_type;
	gain_mconfig->gain_cfg.ramp_duration = gain_fmt->ramp_duration;

	return 0;
}

/*
 * DSP needs SRC module for frequency conversion, SRC takes base module
 * configuration and the target frequency as extra parameter passed as src
 * config
 */
static void skl_set_src_format(struct skl_dev *skl,
			struct skl_module_cfg *mconfig,
			struct skl_src_module_cfg *src_mconfig)
{
	struct skl_module *module = mconfig->module;
	struct skl_module_iface *iface = &module->formats[mconfig->fmt_idx];
	struct skl_module_fmt *fmt = &iface->outputs[0].fmt;

	skl_set_base_module_format(skl, mconfig,
		(struct skl_base_cfg *)src_mconfig);

	src_mconfig->src_cfg = fmt->s_freq;

}

/*
 * DSP needs ASRC module for asynchronous frequency conversion, ASRC takes
 * the same parameters as a SRC module and mode parameter as an extra parameter
 * to know in which direction stream is going
 */
static void skl_set_asrc_format(struct skl_dev *skl,
			struct skl_module_cfg *mconfig,
			struct skl_asrc_module_cfg *asrc_mconfig)
{
	skl_set_src_format(skl, mconfig, &asrc_mconfig->src_cfg);

	if (mconfig->pipe->p_params->stream == SNDRV_PCM_STREAM_PLAYBACK)
		asrc_mconfig->mode = ASRC_MODE_UPLINK;
	else
		asrc_mconfig->mode = ASRC_MODE_DOWNLINK;
}

/*
 * DSP needs updown module to do channel conversion. updown module take base
 * module configuration and channel configuration
 * It also take coefficients and now we have defaults applied here
 */
static void skl_set_updown_mixer_format(struct skl_dev *skl,
			struct skl_module_cfg *mconfig,
			struct skl_up_down_mixer_cfg *mixer_mconfig)
{
	struct skl_module *module = mconfig->module;
	struct skl_module_iface *iface = &module->formats[mconfig->fmt_idx];
	struct skl_module_fmt *fmt = &iface->outputs[0].fmt;

	skl_set_base_module_format(skl,	mconfig,
		(struct skl_base_cfg *)mixer_mconfig);
	mixer_mconfig->out_ch_cfg = fmt->ch_cfg;
	mixer_mconfig->ch_map = fmt->ch_map;
}

/*
 * 'copier' is DSP internal module which copies data from Host DMA (HDA host
 * dma) or link (hda link, SSP, PDM)
 * Here we calculate the copier module parameters, like PCM format, output
 * format, gateway settings
 * copier_module_config is sent as input buffer with INIT_INSTANCE IPC msg
 */
static void skl_set_copier_format(struct skl_dev *skl,
			struct skl_module_cfg *mconfig,
			struct skl_cpr_cfg *cpr_mconfig)
{
	struct skl_audio_data_format *out_fmt = &cpr_mconfig->out_fmt;
	struct skl_base_cfg *base_cfg = (struct skl_base_cfg *)cpr_mconfig;

	skl_set_base_module_format(skl, mconfig, base_cfg);

	skl_setup_out_format(skl, mconfig, out_fmt);
	skl_setup_cpr_gateway_cfg(skl, mconfig, cpr_mconfig);
}

/*
 * Algo module are DSP pre processing modules. Algo module take base module
 * configuration and params
 */

static void skl_set_algo_format(struct skl_dev *skl,
			struct skl_module_cfg *mconfig,
			struct skl_algo_cfg *algo_mcfg)
{
	struct skl_base_cfg *base_cfg = (struct skl_base_cfg *)algo_mcfg;

	skl_set_base_module_format(skl, mconfig, base_cfg);

	if (mconfig->formats_config[SKL_PARAM_INIT].caps_size == 0)
		return;

	memcpy(algo_mcfg->params,
			mconfig->formats_config[SKL_PARAM_INIT].caps,
			mconfig->formats_config[SKL_PARAM_INIT].caps_size);

}

/*
 * Mic select module allows selecting one or many input channels, thus
 * acting as a demux.
 *
 * Mic select module take base module configuration and out-format
 * configuration
 */
static void skl_set_base_outfmt_format(struct skl_dev *skl,
			struct skl_module_cfg *mconfig,
			struct skl_base_outfmt_cfg *base_outfmt_mcfg)
{
	struct skl_audio_data_format *out_fmt = &base_outfmt_mcfg->out_fmt;
	struct skl_base_cfg *base_cfg =
				(struct skl_base_cfg *)base_outfmt_mcfg;

	skl_set_base_module_format(skl, mconfig, base_cfg);
	skl_setup_out_format(skl, mconfig, out_fmt);
}

static u16 skl_get_module_param_size(struct skl_dev *skl,
			struct skl_module_cfg *mconfig)
{
	u16 param_size;
	struct skl_module *module = mconfig->module;
	struct skl_module_iface *m_intf = &module->formats[mconfig->fmt_idx];
	struct skl_module_res *m_res = &module->resources[mconfig->res_idx];

	switch (mconfig->m_type) {
	case SKL_MODULE_TYPE_COPIER:
		param_size = sizeof(struct skl_cpr_cfg);
		param_size += mconfig->formats_config[SKL_PARAM_INIT].caps_size;
		return param_size;

	case SKL_MODULE_TYPE_SRCINT:
		return sizeof(struct skl_src_module_cfg);

	case SKL_MODULE_TYPE_ASRC:
		return sizeof(struct skl_asrc_module_cfg);

	case SKL_MODULE_TYPE_UPDWMIX:
		return sizeof(struct skl_up_down_mixer_cfg);

	case SKL_MODULE_TYPE_ALGO:
		param_size = sizeof(struct skl_base_cfg);
		param_size += mconfig->formats_config[SKL_PARAM_INIT].caps_size;
		return param_size;

	case SKL_MODULE_TYPE_BASE_OUTFMT:
	case SKL_MODULE_TYPE_MIC_SELECT:
	case SKL_MODULE_TYPE_KPB:
		return sizeof(struct skl_base_outfmt_cfg);

	case SKL_MODULE_TYPE_GAIN:
		param_size = sizeof(struct skl_base_cfg);
		param_size += sizeof(struct skl_gain_config)
			* m_intf->outputs[0].fmt.channels;
		return param_size;

	case SKL_MODULE_TYPE_BASE_GENEXT:
		param_size = sizeof(struct skl_base_cfg);
		param_size += sizeof(struct skl_base_cfg_ext) +
			(m_res->nr_input_pins + m_res->nr_output_pins)
			* sizeof(struct skl_pin_format);
		return param_size;

	default:
		/*
		 * return only base cfg when no specific module type is
		 * specified
		 */
		return sizeof(struct skl_base_cfg);
	}

	return 0;
}

/*
 * DSP firmware supports various modules like copier, SRC, updown etc.
 * These modules required various parameters to be calculated and sent for
 * the module initialization to DSP. By default a generic module needs only
 * base module format configuration
 */

static int skl_set_module_format(struct skl_dev *skl,
			struct skl_module_cfg *module_config,
			u16 *module_config_size,
			void **param_data)
{
	u16 param_size;

	param_size  = skl_get_module_param_size(skl, module_config);

	*param_data = kzalloc(param_size, GFP_KERNEL);
	if (NULL == *param_data)
		return -ENOMEM;

	*module_config_size = param_size;

	switch (module_config->m_type) {
	case SKL_MODULE_TYPE_COPIER:
		skl_set_copier_format(skl, module_config, *param_data);
		break;

	case SKL_MODULE_TYPE_SRCINT:
		skl_set_src_format(skl, module_config, *param_data);
		break;

	case SKL_MODULE_TYPE_ASRC:
		skl_set_asrc_format(skl, module_config, *param_data);
		break;

	case SKL_MODULE_TYPE_UPDWMIX:
		skl_set_updown_mixer_format(skl, module_config, *param_data);
		break;

	case SKL_MODULE_TYPE_ALGO:
		skl_set_algo_format(skl, module_config, *param_data);
		break;

	case SKL_MODULE_TYPE_BASE_OUTFMT:
	case SKL_MODULE_TYPE_MIC_SELECT:
	case SKL_MODULE_TYPE_KPB:
		skl_set_base_outfmt_format(skl, module_config, *param_data);
		break;

	case SKL_MODULE_TYPE_GAIN:
		skl_set_gain_format(skl, module_config, *param_data);
		break;

	case SKL_MODULE_TYPE_BASE_GENEXT:
		skl_set_base_module_format(skl, module_config, *param_data);
		skl_set_base_ext_module_format(skl, module_config,
			*param_data + sizeof(struct skl_base_cfg));
		break;

	default:
		skl_set_base_module_format(skl, module_config, *param_data);
		break;

	}

	dev_dbg(skl->dev, "Module type=%d id=%d config size: %d bytes\n",
			module_config->m_type, module_config->id.module_id,
			param_size);
	print_hex_dump_debug("Module params:", DUMP_PREFIX_OFFSET, 8, 4,
			*param_data, param_size, false);
	return 0;
}

static int skl_get_queue_index(struct skl_module_pin *mpin,
				struct skl_module_inst_id id, int max)
{
	int i;

	for (i = 0; i < max; i++)  {
		if (mpin[i].id.module_id == id.module_id &&
			mpin[i].id.instance_id == id.instance_id)
			return i;
	}

	return -EINVAL;
}

/*
 * Allocates queue for each module.
 * if dynamic, the pin_index is allocated 0 to max_pin.
 * In static, the pin_index is fixed based on module_id and instance id
 */
static int skl_alloc_queue(struct skl_module_pin *mpin,
			struct skl_module_cfg *tgt_cfg, int max)
{
	int i;
	struct skl_module_inst_id id = tgt_cfg->id;
	/*
	 * if pin in dynamic, find first free pin
	 * otherwise find match module and instance id pin as topology will
	 * ensure a unique pin is assigned to this so no need to
	 * allocate/free
	 */
	for (i = 0; i < max; i++)  {
		if (mpin[i].is_dynamic) {
			if (!mpin[i].in_use &&
				mpin[i].pin_state == SKL_PIN_UNBIND) {

				mpin[i].in_use = true;
				mpin[i].id.module_id = id.module_id;
				mpin[i].id.instance_id = id.instance_id;
				mpin[i].id.pvt_id = id.pvt_id;
				mpin[i].tgt_mcfg = tgt_cfg;
				return i;
			}
		} else {
			if (mpin[i].id.module_id == id.module_id &&
				mpin[i].id.instance_id == id.instance_id &&
				mpin[i].pin_state == SKL_PIN_UNBIND) {

				mpin[i].tgt_mcfg = tgt_cfg;
				return i;
			}
		}
	}

	return -EINVAL;
}

static void skl_free_queue(struct skl_module_pin *mpin, int q_index)
{
	if (mpin[q_index].is_dynamic) {
		mpin[q_index].in_use = false;
		mpin[q_index].id.module_id = 0;
		mpin[q_index].id.instance_id = 0;
		mpin[q_index].id.pvt_id = 0;
	}
	mpin[q_index].pin_state = SKL_PIN_UNBIND;
	mpin[q_index].tgt_mcfg = NULL;
}

/* Module state will be set to unint, if all the out pin state is UNBIND */

static void skl_clear_module_state(struct skl_module_pin *mpin, int max,
						struct skl_module_cfg *mcfg)
{
	int i;
	bool found = false;

	for (i = 0; i < max; i++)  {
		if (mpin[i].pin_state == SKL_PIN_UNBIND)
			continue;
		found = true;
		break;
	}

	if (!found)
		mcfg->m_state = SKL_MODULE_INIT_DONE;
	return;
}

/*
 * A module needs to be instanataited in DSP. A mdoule is present in a
 * collection of module referred as a PIPE.
 * We first calculate the module format, based on module type and then
 * invoke the DSP by sending IPC INIT_INSTANCE using ipc helper
 */
int skl_init_module(struct skl_dev *skl,
			struct skl_module_cfg *mconfig)
{
	u16 module_config_size = 0;
	void *param_data = NULL;
	int ret;
	struct skl_ipc_init_instance_msg msg;

	dev_dbg(skl->dev, "%s: module_id = %d instance=%d\n", __func__,
		 mconfig->id.module_id, mconfig->id.pvt_id);

	if (mconfig->pipe->state != SKL_PIPE_CREATED) {
		dev_err(skl->dev, "Pipe not created state= %d pipe_id= %d\n",
				 mconfig->pipe->state, mconfig->pipe->ppl_id);
		return -EIO;
	}

	ret = skl_set_module_format(skl, mconfig,
			&module_config_size, &param_data);
	if (ret < 0) {
		dev_err(skl->dev, "Failed to set module format ret=%d\n", ret);
		return ret;
	}

	msg.module_id = mconfig->id.module_id;
	msg.instance_id = mconfig->id.pvt_id;
	msg.ppl_instance_id = mconfig->pipe->ppl_id;
	msg.param_data_size = module_config_size;
	msg.core_id = mconfig->core_id;
	msg.domain = mconfig->domain;

	ret = skl_ipc_init_instance(&skl->ipc, &msg, param_data);
	if (ret < 0) {
		dev_err(skl->dev, "Failed to init instance ret=%d\n", ret);
		kfree(param_data);
		return ret;
	}
	mconfig->m_state = SKL_MODULE_INIT_DONE;
	kfree(param_data);
	return ret;
}

static void skl_dump_bind_info(struct skl_dev *skl, struct skl_module_cfg
	*src_module, struct skl_module_cfg *dst_module)
{
	dev_dbg(skl->dev, "%s: src module_id = %d  src_instance=%d\n",
		__func__, src_module->id.module_id, src_module->id.pvt_id);
	dev_dbg(skl->dev, "%s: dst_module=%d dst_instance=%d\n", __func__,
		 dst_module->id.module_id, dst_module->id.pvt_id);

	dev_dbg(skl->dev, "src_module state = %d dst module state = %d\n",
		src_module->m_state, dst_module->m_state);
}

/*
 * On module freeup, we need to unbind the module with modules
 * it is already bind.
 * Find the pin allocated and unbind then using bind_unbind IPC
 */
int skl_unbind_modules(struct skl_dev *skl,
			struct skl_module_cfg *src_mcfg,
			struct skl_module_cfg *dst_mcfg)
{
	int ret;
	struct skl_ipc_bind_unbind_msg msg;
	struct skl_module_inst_id src_id = src_mcfg->id;
	struct skl_module_inst_id dst_id = dst_mcfg->id;
	int in_max = dst_mcfg->module->max_input_pins;
	int out_max = src_mcfg->module->max_output_pins;
	int src_index, dst_index, src_pin_state, dst_pin_state;

	skl_dump_bind_info(skl, src_mcfg, dst_mcfg);

	/* get src queue index */
	src_index = skl_get_queue_index(src_mcfg->m_out_pin, dst_id, out_max);
	if (src_index < 0)
		return 0;

	msg.src_queue = src_index;

	/* get dst queue index */
	dst_index  = skl_get_queue_index(dst_mcfg->m_in_pin, src_id, in_max);
	if (dst_index < 0)
		return 0;

	msg.dst_queue = dst_index;

	src_pin_state = src_mcfg->m_out_pin[src_index].pin_state;
	dst_pin_state = dst_mcfg->m_in_pin[dst_index].pin_state;

	if (src_pin_state != SKL_PIN_BIND_DONE ||
		dst_pin_state != SKL_PIN_BIND_DONE)
		return 0;

	msg.module_id = src_mcfg->id.module_id;
	msg.instance_id = src_mcfg->id.pvt_id;
	msg.dst_module_id = dst_mcfg->id.module_id;
	msg.dst_instance_id = dst_mcfg->id.pvt_id;
	msg.bind = false;

	ret = skl_ipc_bind_unbind(&skl->ipc, &msg);
	if (!ret) {
		/* free queue only if unbind is success */
		skl_free_queue(src_mcfg->m_out_pin, src_index);
		skl_free_queue(dst_mcfg->m_in_pin, dst_index);

		/*
		 * check only if src module bind state, bind is
		 * always from src -> sink
		 */
		skl_clear_module_state(src_mcfg->m_out_pin, out_max, src_mcfg);
	}

	return ret;
}

#define CPR_SINK_FMT_PARAM_ID 2

static struct
skl_module_fmt *skl_get_pin_format(struct skl_module_cfg *mconfig,
				   u8 pin_direction, u8 pin_idx)
{
	struct skl_module *module = mconfig->module;
	int fmt_idx = mconfig->fmt_idx;
	struct skl_module_iface *intf;
	struct skl_module_fmt *pin_fmt;

	intf = &module->formats[fmt_idx];

	if (pin_direction == SKL_INPUT_PIN)
		pin_fmt = &intf->inputs[pin_idx].fmt;
	else
		pin_fmt = &intf->outputs[pin_idx].fmt;

	return pin_fmt;
}

/*
 * This function checks for source module and destination module format
 * mismatch
 */
static void skl_module_format_mismatch_detection(struct skl_dev *skl,
					struct skl_module_cfg *src_mcfg,
					struct skl_module_cfg *dst_mcfg,
					int src_index, int dst_index)
{
	struct skl_module_fmt *src_fmt, *dst_fmt;

	src_fmt = skl_get_pin_format(src_mcfg, SKL_OUTPUT_PIN, src_index);
	dst_fmt = skl_get_pin_format(dst_mcfg, SKL_INPUT_PIN, dst_index);

	if(memcmp(src_fmt, dst_fmt, sizeof(*src_fmt))) {
		dev_warn(skl->dev, "#### src and dst format mismatch: ####\n");
		dev_warn(skl->dev, "pipe=%d src module_id=%d src instance_id=%d\n",
					src_mcfg->pipe->ppl_id,
					src_mcfg->id.module_id,
					src_mcfg->id.pvt_id);

		dev_warn(skl->dev, "pipe=%d dst module_id=%d dst instance_id=%d\n",
					dst_mcfg->pipe->ppl_id,
					dst_mcfg->id.module_id,
					dst_mcfg->id.pvt_id);

		dev_warn(skl->dev, "channels: src=%d dst=%d\n",
				src_fmt->channels, dst_fmt->channels);
		dev_warn(skl->dev, "s_freq: src=%d dst=%d\n",
				src_fmt->s_freq, dst_fmt->s_freq);
		dev_warn(skl->dev, "bit_depth: src=%d dst=%d\n",
				src_fmt->bit_depth, dst_fmt->bit_depth);
		dev_warn(skl->dev, "valid_bit_depth: src=%d dst=%d\n",
				src_fmt->valid_bit_depth, dst_fmt->valid_bit_depth);
		dev_warn(skl->dev, "ch_cfg: src=%d dst=%d\n",
				src_fmt->ch_cfg, dst_fmt->ch_cfg);
		dev_warn(skl->dev, "interleaving_style: src=%d dst=%d\n",
				src_fmt->interleaving_style, dst_fmt->interleaving_style);
		dev_warn(skl->dev, "sample_type: src=%d dst=%d\n",
				src_fmt->sample_type, dst_fmt->sample_type);
		dev_warn(skl->dev, "ch_map: src=0x%08x dst=0x%08x\n",
				src_fmt->ch_map, dst_fmt->ch_map);
	}
}

/*
 * Once a module is instantiated it need to be 'bind' with other modules in
 * the pipeline. For binding we need to find the module pins which are bind
 * together
 * This function finds the pins and then sends bund_unbind IPC message to
 * DSP using IPC helper
 */
int skl_bind_modules(struct skl_dev *skl,
			struct skl_module_cfg *src_mcfg,
			struct skl_module_cfg *dst_mcfg)
{
	int ret = 0;
	struct skl_ipc_bind_unbind_msg msg;
	int in_max = dst_mcfg->module->max_input_pins;
	int out_max = src_mcfg->module->max_output_pins;
	int src_index, dst_index;
	struct skl_module_fmt *format;
	struct skl_cpr_pin_fmt pin_fmt;
	struct skl_module *module;
	struct skl_module_iface *fmt;

	skl_dump_bind_info(skl, src_mcfg, dst_mcfg);

	if (src_mcfg->m_state < SKL_MODULE_INIT_DONE ||
		dst_mcfg->m_state < SKL_MODULE_INIT_DONE)
		return 0;

	src_index = skl_alloc_queue(src_mcfg->m_out_pin, dst_mcfg, out_max);
	if (src_index < 0)
		return -EINVAL;

	msg.src_queue = src_index;
	dst_index = skl_alloc_queue(dst_mcfg->m_in_pin, src_mcfg, in_max);
	if (dst_index < 0) {
		skl_free_queue(src_mcfg->m_out_pin, src_index);
		return -EINVAL;
	}

	/*
	 * Copier module requires the separate large_config_set_ipc to
	 * configure the pins other than 0
	 */
	if (src_mcfg->m_type == SKL_MODULE_TYPE_COPIER && src_index > 0) {
		pin_fmt.sink_id = src_index;
		module = src_mcfg->module;
		fmt = &module->formats[src_mcfg->fmt_idx];

		/* Input fmt is same as that of src module input cfg */
		format = &fmt->inputs[0].fmt;
		fill_pin_params(&(pin_fmt.src_fmt), format);

		format = &fmt->outputs[src_index].fmt;
		fill_pin_params(&(pin_fmt.dst_fmt), format);
		ret = skl_set_module_params(skl, (void *)&pin_fmt,
					sizeof(struct skl_cpr_pin_fmt),
					CPR_SINK_FMT_PARAM_ID, src_mcfg);

		if (ret < 0)
			goto out;
	}

	msg.dst_queue = dst_index;

	dev_dbg(skl->dev, "src queue = %d dst queue =%d\n",
			 msg.src_queue, msg.dst_queue);

	skl_module_format_mismatch_detection(skl, src_mcfg, dst_mcfg,
						src_index, dst_index);

	msg.module_id = src_mcfg->id.module_id;
	msg.instance_id = src_mcfg->id.pvt_id;
	msg.dst_module_id = dst_mcfg->id.module_id;
	msg.dst_instance_id = dst_mcfg->id.pvt_id;
	msg.bind = true;

	ret = skl_ipc_bind_unbind(&skl->ipc, &msg);

	if (!ret) {
		src_mcfg->m_state = SKL_MODULE_BIND_DONE;
		src_mcfg->m_out_pin[src_index].pin_state = SKL_PIN_BIND_DONE;
		dst_mcfg->m_in_pin[dst_index].pin_state = SKL_PIN_BIND_DONE;
		return ret;
	}
out:
	/* error case , if IPC fails, clear the queue index */
	skl_free_queue(src_mcfg->m_out_pin, src_index);
	skl_free_queue(dst_mcfg->m_in_pin, dst_index);

	return ret;
}

static int skl_set_pipe_state(struct skl_dev *skl, struct skl_pipe *pipe,
	enum skl_ipc_pipeline_state state)
{
	dev_dbg(skl->dev, "%s: pipe_state = %d\n", __func__, state);

	return skl_ipc_set_pipeline_state(&skl->ipc, pipe->ppl_id, state);
}

/*
 * A pipeline is a collection of modules. Before a module in instantiated a
 * pipeline needs to be created for it.
 * This function creates pipeline, by sending create pipeline IPC messages
 * to FW
 */
int skl_create_pipeline(struct skl_dev *skl, struct skl_pipe *pipe)
{
	int ret;

	dev_dbg(skl->dev, "%s: pipe_id = %d\n", __func__, pipe->ppl_id);

	ret = skl_ipc_create_pipeline(&skl->ipc, pipe->memory_pages,
				pipe->pipe_priority, pipe->ppl_id,
				pipe->lp_mode);
	if (ret < 0) {
		dev_err(skl->dev, "Failed to create pipeline\n");
		return ret;
	}

	pipe->state = SKL_PIPE_CREATED;
	skl_dbg_event(skl, pipe->state);

	return 0;
}

/*
 * A pipeline needs to be deleted on cleanup. If a pipeline is running,
 * then pause it first. Before actual deletion, pipeline should enter
 * reset state. Finish the procedure by sending delete pipeline IPC.
 * DSP will stop the DMA engines and release resources
 */
int skl_delete_pipe(struct skl_dev *skl, struct skl_pipe *pipe)
{
	int ret;

	dev_dbg(skl->dev, "%s: pipe = %d\n", __func__, pipe->ppl_id);

	/* If pipe was not created in FW, do not try to delete it */
	if (pipe->state < SKL_PIPE_CREATED)
		return 0;

	/* If pipe is started, do stop the pipe in FW. */
	if (pipe->state >= SKL_PIPE_STARTED) {
		ret = skl_set_pipe_state(skl, pipe, PPL_PAUSED);
		if (ret < 0) {
			dev_err(skl->dev, "Failed to stop pipeline\n");
			return ret;
		}

		pipe->state = SKL_PIPE_PAUSED;
	}

	/* reset pipe state before deletion */
	ret = skl_set_pipe_state(skl, pipe, PPL_RESET);
	if (ret < 0) {
		dev_err(skl->dev, "Failed to reset pipe ret=%d\n", ret);
		return ret;
	}

	pipe->state = SKL_PIPE_RESET;

	ret = skl_ipc_delete_pipeline(&skl->ipc, pipe->ppl_id);
	if (ret < 0) {
		dev_err(skl->dev, "Failed to delete pipeline\n");
		return ret;
	}

	pipe->state = SKL_PIPE_INVALID;
	skl_dbg_event(skl, pipe->state);

	return ret;
}

/*
 * A pipeline is also a scheduling entity in DSP which can be run, stopped
 * For processing data the pipe need to be run by sending IPC set pipe state
 * to DSP
 */
int skl_run_pipe(struct skl_dev *skl, struct skl_pipe *pipe)
{
	int ret;

	dev_dbg(skl->dev, "%s: pipe = %d\n", __func__, pipe->ppl_id);

	/* If pipe was not created in FW, do not try to pause or delete */
	if (pipe->state < SKL_PIPE_CREATED)
		return 0;

	/* Pipe has to be paused before it is started */
	ret = skl_set_pipe_state(skl, pipe, PPL_PAUSED);
	if (ret < 0) {
		dev_err(skl->dev, "Failed to pause pipe\n");
		return ret;
	}

	pipe->state = SKL_PIPE_PAUSED;

	ret = skl_set_pipe_state(skl, pipe, PPL_RUNNING);
	if (ret < 0) {
		dev_err(skl->dev, "Failed to start pipe\n");
		return ret;
	}

	pipe->state = SKL_PIPE_STARTED;

	return 0;
}

/*
 * Stop the pipeline by sending set pipe state IPC
 * DSP doesnt implement stop so we always send pause message
 */
int skl_stop_pipe(struct skl_dev *skl, struct skl_pipe *pipe)
{
	int ret;

	dev_dbg(skl->dev, "In %s pipe=%d\n", __func__, pipe->ppl_id);

	/* If pipe was not created in FW, do not try to pause or delete */
	if (pipe->state < SKL_PIPE_PAUSED)
		return 0;

	ret = skl_set_pipe_state(skl, pipe, PPL_PAUSED);
	if (ret < 0) {
		dev_dbg(skl->dev, "Failed to stop pipe\n");
		return ret;
	}

	pipe->state = SKL_PIPE_PAUSED;

	return 0;
}

/*
 * Reset the pipeline by sending set pipe state IPC this will reset the DMA
 * from the DSP side
 */
int skl_reset_pipe(struct skl_dev *skl, struct skl_pipe *pipe)
{
	int ret;

	/* If pipe was not created in FW, do not try to pause or delete */
	if (pipe->state < SKL_PIPE_PAUSED)
		return 0;

	ret = skl_set_pipe_state(skl, pipe, PPL_RESET);
	if (ret < 0) {
		dev_dbg(skl->dev, "Failed to reset pipe ret=%d\n", ret);
		return ret;
	}

	pipe->state = SKL_PIPE_RESET;

	return 0;
}

/* Algo parameter set helper function */
int skl_set_module_params(struct skl_dev *skl, u32 *params, int size,
				u32 param_id, struct skl_module_cfg *mcfg)
{
	struct skl_ipc_large_config_msg msg;

	msg.module_id = mcfg->id.module_id;
	msg.instance_id = mcfg->id.pvt_id;
	msg.param_data_size = size;
	msg.large_param_id = param_id;

	return skl_ipc_set_large_config(&skl->ipc, &msg, params);
}

int skl_get_module_params(struct skl_dev *skl, u32 *params, int size,
			  u32 param_id, struct skl_module_cfg *mcfg)
{
	struct skl_ipc_large_config_msg msg;
	size_t bytes = size;

	msg.module_id = mcfg->id.module_id;
	msg.instance_id = mcfg->id.pvt_id;
	msg.param_data_size = size;
	msg.large_param_id = param_id;

	return skl_ipc_get_large_config(&skl->ipc, &msg, &params, &bytes);
}

int skl_probe_init_module(struct skl_dev *skl, size_t buffer_size)
{
	struct skl_ipc_init_instance_msg msg = {0};
	struct skl_probe_mod_cfg cfg = {{0}};
	union skl_connector_node_id node_id = {0};
	struct skl_module_entry *m = skl_find_module(skl, &skl_probe_mod_uuid);

	if (skl->extractor) {
		node_id.node.vindex = skl->extractor->hstream.stream_tag - 1;
		node_id.node.dma_type = SKL_DMA_HDA_HOST_INPUT_CLASS;
	} else {
		node_id = INVALID_NODE_ID;
	}

	/*
	 * Probe module uses no cycles, audio data format
	 * and input and output frame sizes are unused.
	 */
	cfg.base_cfg.ibs = 1;
	cfg.base_cfg.is_pages = m->segments[2].flags.length;
	cfg.gtw_cfg.node_id = node_id;
	cfg.gtw_cfg.dma_buffer_size = buffer_size;

	msg.module_id = m->module_id;
	msg.instance_id = 0;
	msg.param_data_size = sizeof(cfg);
	msg.ppl_instance_id = INVALID_PIPELINE_ID;
	msg.core_id = 0;
	msg.domain = 0;

	return skl_ipc_init_instance(&skl->ipc, &msg, &cfg);
}

int skl_probe_delete_module(struct skl_dev *skl)
{
	unsigned int module_id = skl_get_module_id(skl, &skl_probe_mod_uuid);

	return skl_ipc_delete_instance(&skl->ipc, module_id, 0);
}

int skl_probe_get_dma(struct skl_dev *skl,
		struct skl_probe_dma **dma, size_t *num_dma)
{
	struct skl_ipc_large_config_msg msg = {0};
	u32 *payload = NULL;
	size_t bytes = 0;
	int ret;

	*dma = NULL;
	*num_dma = 0;
	msg.module_id = skl_get_module_id(skl, &skl_probe_mod_uuid);
	msg.instance_id = 0;
	msg.large_param_id = SKL_PROBE_INJECTION_DMA;

	ret = skl_ipc_get_large_config(&skl->ipc, &msg, &payload, &bytes);
	if (ret < 0 || !bytes)
		return ret;

	*dma = (struct skl_probe_dma *)payload;
	*num_dma = bytes / sizeof(**dma);

	return 0;
}

int skl_probe_dma_attach(struct skl_dev *skl,
		struct skl_probe_dma *dma, size_t num_dma)
{
	struct skl_ipc_large_config_msg msg = {0};

	msg.module_id = skl_get_module_id(skl, &skl_probe_mod_uuid);
	msg.instance_id = 0;
	msg.param_data_size = sizeof(*dma) * num_dma;
	msg.large_param_id = SKL_PROBE_INJECTION_DMA;

	return skl_ipc_set_large_config(&skl->ipc, &msg, (u32 *)dma);
}

int skl_probe_dma_detach(struct skl_dev *skl,
		union skl_connector_node_id *node_id, size_t num_node_id)
{
	struct skl_ipc_large_config_msg msg = {0};

	msg.module_id = skl_get_module_id(skl, &skl_probe_mod_uuid);
	msg.instance_id = 0;
	msg.param_data_size = sizeof(*node_id) * num_node_id;
	msg.large_param_id = SKL_PROBE_INJECTION_DMA_DETACH;

	return skl_ipc_set_large_config(&skl->ipc, &msg, (u32 *)node_id);
}

int skl_probe_get_points(struct skl_dev *skl,
		struct skl_probe_point_desc **desc, size_t *num_desc)
{
	struct skl_ipc_large_config_msg msg = {0};
	u32 *payload = NULL;
	size_t bytes = 0;
	int ret;

	*desc = NULL;
	*num_desc = 0;
	msg.module_id = skl_get_module_id(skl, &skl_probe_mod_uuid);
	msg.instance_id = 0;
	msg.large_param_id = SKL_PROBE_POINTS;

	ret = skl_ipc_get_large_config(&skl->ipc, &msg, &payload, &bytes);
	if (ret < 0 || !bytes)
		return ret;

	*desc = (struct skl_probe_point_desc *)payload;
	*num_desc = bytes / sizeof(**desc);

	return 0;
}

int skl_probe_points_connect(struct skl_dev *skl,
		struct skl_probe_point_desc *desc, size_t num_desc)
{
	struct skl_ipc_large_config_msg msg = {0};

	msg.module_id = skl_get_module_id(skl, &skl_probe_mod_uuid);
	msg.instance_id = 0;
	msg.param_data_size = sizeof(*desc) * num_desc;
	msg.large_param_id = SKL_PROBE_POINTS;

	return skl_ipc_set_large_config(&skl->ipc, &msg, (u32 *)desc);
}

int skl_probe_points_disconnect(struct skl_dev *skl,
		union skl_probe_point_id *id, size_t num_id)
{
	struct skl_ipc_large_config_msg msg = {0};

	msg.module_id = skl_get_module_id(skl, &skl_probe_mod_uuid);
	msg.instance_id = 0;
	msg.param_data_size = sizeof(*id) * num_id;
	msg.large_param_id = SKL_PROBE_POINTS_DISCONNECT;

	return skl_ipc_set_large_config(&skl->ipc, &msg, (u32 *)id);
}

int skl_system_time_set(struct sst_generic_ipc *ipc)
{
	struct skl_ipc_large_config_msg msg = {0};
	struct skl_sys_time sys_time;
	u64 us;

	/* firmware expects UTC time in micro seconds */
	us = ktime_to_us(ktime_get());
	sys_time.val_l = us & UINT_MAX;
	sys_time.val_u = us >> 32;

	msg.large_param_id = SKL_BASEFW_SYSTEM_TIME;
	msg.param_data_size = sizeof(sys_time);

	return skl_ipc_set_large_config(ipc, &msg, (u32 *)&sys_time);
}
EXPORT_SYMBOL_GPL(skl_system_time_set);

int skl_enable_logs_set(struct sst_generic_ipc *ipc, u32 *info, size_t size)
{
	struct skl_ipc_large_config_msg msg = {0};

	msg.param_data_size = size;
	msg.large_param_id = SKL_BASEFW_ENABLE_LOGS;

	return skl_ipc_set_large_config(ipc, &msg, info);
}
EXPORT_SYMBOL_GPL(skl_enable_logs_set);
