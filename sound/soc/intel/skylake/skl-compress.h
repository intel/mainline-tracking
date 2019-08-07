/* SPDX-License-Identifier: GPL-2.0
 *
 * skl-compress.h -- ASoC Skylake compress header file
 *
 * Copyright (C) 2018 Intel Corp.
 *
 * Author: Cezary Rojewski <cezary.rojewski@intel.com>
 */

#ifndef __SKL_COMPRESS_H__
#define __SKL_COMPRESS_H__

#include <sound/compress_driver.h>

static inline
struct hdac_ext_stream *skl_compr_get_stream(struct snd_compr_stream *cstream)
{
	return cstream->runtime->private_data;
}

int skl_probe_compr_open(struct snd_compr_stream *cstream,
		struct snd_soc_dai *dai);
int skl_probe_compr_free(struct snd_compr_stream *cstream,
		struct snd_soc_dai *dai);
int skl_probe_compr_set_params(struct snd_compr_stream *cstream,
		struct snd_compr_params *params, struct snd_soc_dai *dai);
int skl_probe_compr_trigger(struct snd_compr_stream *cstream, int cmd,
		struct snd_soc_dai *dai);
int skl_probe_compr_pointer(struct snd_compr_stream *cstream,
		struct snd_compr_tstamp *tstamp, struct snd_soc_dai *dai);
int skl_probe_compr_copy(struct snd_compr_stream *cstream,
		char __user *buf, size_t count);

#endif /* __SKL_COMPRESS_H__*/
