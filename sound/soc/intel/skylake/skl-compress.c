// SPDX-License-Identifier: GPL-2.0+
//
// skl-compress.c  --  ASoC Skylake compress operations
//
// Copyright (C) 2018 Intel Corp.
//
// Author: Cezary Rojewski <cezary.rojewski@intel.com>

#include <linux/pci.h>
#include <sound/soc.h>
#include "skl-sst-dsp.h"
#include "skl-sst-ipc.h"
#include "skl-topology.h"
#include "skl-compress.h"

int skl_probe_compr_open(struct snd_compr_stream *cstream,
		struct snd_soc_dai *dai)
{
	struct hdac_bus *bus = dev_get_drvdata(dai->dev);
	struct hdac_ext_stream *stream;
	struct skl_dev *skl = get_skl_ctx(dai->dev);

	if (cstream->direction == SND_COMPRESS_CAPTURE && skl->extractor) {
		dev_err(dai->dev,
			"Cannot open more than one extractor stream\n");
		return -EEXIST;
	}

	stream = hdac_ext_host_stream_compr_assign(bus, cstream);
	if (!stream) {
		dev_err(dai->dev, "Failed to assign probe stream\n");
		return -EBUSY;
	}

	if (cstream->direction == SND_COMPRESS_CAPTURE)
		skl->extractor = stream;
	hdac_stream(stream)->curr_pos = 0;
	cstream->runtime->private_data = stream;

	return 0;
}

int skl_probe_compr_free(struct snd_compr_stream *cstream,
		struct snd_soc_dai *dai)
{
	struct hdac_ext_stream *stream = skl_compr_get_stream(cstream);
	struct skl_dev *skl = get_skl_ctx(dai->dev);
	struct skl_probe_point_desc *desc;
	struct skl_probe_dma *dma;
	size_t num_desc, num_dma;
	unsigned int vindex = INVALID_NODE_ID.node.vindex;
	int i, ret;

	ret = skl_probe_get_points(skl, &desc, &num_desc);
	if (ret < 0) {
		dev_err(dai->dev, "Failed to get probe points, ret: %d\n", ret);
		goto release_resources;
	}

	if (cstream->direction == SND_COMPRESS_PLAYBACK)
		vindex = hdac_stream(stream)->stream_tag - 1;

	for (i = 0; i < num_desc; i++)
		if (desc[i].node_id.node.vindex == vindex)
			skl_probe_points_disconnect(skl, &desc[i].id, 1);
	kfree(desc);

	if (cstream->direction != SND_COMPRESS_PLAYBACK)
		goto release_resources;

	ret = skl_probe_get_dma(skl, &dma, &num_dma);
	if (ret < 0) {
		dev_err(dai->dev, "Failed to get inject dma, ret: %d\n", ret);
		goto release_resources;
	}

	for (i = 0; i < num_dma; i++)
		if (dma[i].node_id.node.vindex == vindex)
			skl_probe_dma_detach(skl, &dma[i].node_id, 1);
	kfree(dma);

release_resources:
	snd_hdac_stream_cleanup(hdac_stream(stream));
	hdac_stream(stream)->prepared = 0;
	snd_compr_free_pages(cstream);

	snd_hdac_ext_stream_release(stream, HDAC_EXT_STREAM_TYPE_HOST);

	if (skl->extractor == stream)
		skl->extractor = NULL;
	if (skl->num_probe_streams) {
		skl->num_probe_streams--;
		if (!skl->num_probe_streams)
			ret = skl_probe_delete_module(skl);
	}

	return ret;
}

int skl_probe_compr_set_params(struct snd_compr_stream *cstream,
		struct snd_compr_params *params, struct snd_soc_dai *dai)
{
	struct hdac_ext_stream *stream = skl_compr_get_stream(cstream);
	struct snd_compr_runtime *rtd = cstream->runtime;
	struct skl_dev *skl = get_skl_ctx(dai->dev);
	struct skl_probe_dma dma;
	unsigned int format_val;
	int bps, ret;
	/* compr params do not store bit depth, default to S32_LE */
	snd_pcm_format_t format = SNDRV_PCM_FORMAT_S32_LE;

	hdac_stream(stream)->bufsize = 0;
	hdac_stream(stream)->period_bytes = 0;
	hdac_stream(stream)->format_val = 0;
	cstream->dma_buffer.dev.type = SNDRV_DMA_TYPE_DEV_SG;
	cstream->dma_buffer.dev.dev = snd_dma_pci_data(skl->pci);

	ret = snd_compr_malloc_pages(cstream, rtd->buffer_size);
	if (ret < 0)
		return ret;
	bps = snd_pcm_format_physical_width(format);
	if (bps < 0)
		return bps;
	format_val = snd_hdac_calc_stream_format(params->codec.sample_rate,
			params->codec.ch_out, format, bps, 0);
	ret = snd_hdac_stream_set_params(hdac_stream(stream), format_val);
	if (ret < 0)
		return ret;
	ret = snd_hdac_stream_setup(hdac_stream(stream));
	if (ret < 0)
		return ret;

	hdac_stream(stream)->prepared = 1;

	if (!skl->num_probe_streams) {
		ret = skl_probe_init_module(skl, rtd->dma_bytes);
		if (ret < 0)
			return ret;
	}

	if (cstream->direction == SND_COMPRESS_PLAYBACK) {
		dma.node_id.node.vindex = hdac_stream(stream)->stream_tag - 1;
		dma.node_id.node.dma_type = SKL_DMA_HDA_HOST_OUTPUT_CLASS;
		dma.node_id.node.rsvd = 0;
		dma.dma_buffer_size = rtd->dma_bytes;

		ret = skl_probe_dma_attach(skl, &dma, 1);
		if (ret < 0)
			return ret;
	}

	skl->num_probe_streams++;
	return 0;
}

int skl_probe_compr_trigger(struct snd_compr_stream *cstream, int cmd,
		struct snd_soc_dai *dai)
{
	struct hdac_bus *bus = dev_get_drvdata(dai->dev);
	struct hdac_ext_stream *stream = skl_compr_get_stream(cstream);
	unsigned long cookie;

	if (!hdac_stream(stream)->prepared)
		return -EPIPE;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		spin_lock_irqsave(&bus->reg_lock, cookie);
		snd_hdac_stream_start(hdac_stream(stream), true);
		spin_unlock_irqrestore(&bus->reg_lock, cookie);
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
		spin_lock_irqsave(&bus->reg_lock, cookie);
		snd_hdac_stream_stop(hdac_stream(stream));
		spin_unlock_irqrestore(&bus->reg_lock, cookie);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

int skl_probe_compr_pointer(struct snd_compr_stream *cstream,
		struct snd_compr_tstamp *tstamp, struct snd_soc_dai *dai)
{
	struct hdac_ext_stream *stream = skl_compr_get_stream(cstream);
	struct snd_soc_pcm_stream *pstream;

	if (cstream->direction == SND_COMPRESS_PLAYBACK)
		pstream = &dai->driver->playback;
	else
		pstream = &dai->driver->capture;

	tstamp->copied_total = hdac_stream(stream)->curr_pos;
	tstamp->sampling_rate = snd_pcm_rate_bit_to_rate(pstream->rates);
	return 0;
}

int skl_probe_compr_copy(struct snd_compr_stream *cstream,
		char __user *buf, size_t count)
{
	struct snd_compr_runtime *rtd = cstream->runtime;
	unsigned int offset, n;
	void *ptr;
	int ret;

	if (count > rtd->buffer_size)
		count = rtd->buffer_size;

	if (cstream->direction == SND_COMPRESS_CAPTURE) {
		div_u64_rem(rtd->total_bytes_transferred,
				rtd->buffer_size, &offset);
		ptr = rtd->dma_area + offset;
		n = rtd->buffer_size - offset;

		if (count < n) {
			ret = copy_to_user(buf, ptr, count);
		} else {
			ret = copy_to_user(buf, ptr, n);
			ret += copy_to_user(buf + n, rtd->dma_area, count - n);
		}
	} else {
		div_u64_rem(rtd->total_bytes_available,
				rtd->buffer_size, &offset);
		ptr = rtd->dma_area + offset;
		n = rtd->buffer_size - offset;

		if (count < n) {
			ret = copy_from_user(ptr, buf, count);
		} else {
			ret = copy_from_user(ptr, buf, n);
			ret += copy_from_user(rtd->dma_area,
					buf + n, count - n);
		}
	}

	if (ret)
		return count - ret;
	return count;
}
