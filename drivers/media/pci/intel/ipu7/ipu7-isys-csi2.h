/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2013 - 2024 Intel Corporation
 */

#ifndef IPU7_ISYS_CSI2_H
#define IPU7_ISYS_CSI2_H

#include <linux/container_of.h>
#include <linux/types.h>

#include "ipu7-isys-subdev.h"
#include "ipu7-isys-video.h"

struct ipu7_isys;
struct ipu7_isys_csi2_pdata;
struct ipu7_isys_stream;

#define NR_OF_CSI2_VC		16U
#define INVALID_VC_ID		-1
#define NR_OF_CSI2_SINK_PADS	1U
#define CSI2_PAD_SINK		0U
#define NR_OF_CSI2_SRC_PADS	8U
#define CSI2_PAD_SRC		1U
#define NR_OF_CSI2_PADS		(NR_OF_CSI2_SINK_PADS + NR_OF_CSI2_SRC_PADS)

#define CSI2_CSI_RX_DLY_CNT_TERMEN_CLANE_A		0
#define CSI2_CSI_RX_DLY_CNT_TERMEN_CLANE_B		0
#define CSI2_CSI_RX_DLY_CNT_SETTLE_CLANE_A		95
#define CSI2_CSI_RX_DLY_CNT_SETTLE_CLANE_B		-8

#define CSI2_CSI_RX_DLY_CNT_TERMEN_DLANE_A		0
#define CSI2_CSI_RX_DLY_CNT_TERMEN_DLANE_B		0
#define CSI2_CSI_RX_DLY_CNT_SETTLE_DLANE_A		85
#define CSI2_CSI_RX_DLY_CNT_SETTLE_DLANE_B		-2

/*
 * struct ipu7_isys_csi2
 *
 * @nlanes: number of lanes in the receiver
 */
struct ipu7_isys_csi2 {
	struct ipu7_isys_subdev asd;
	struct ipu7_isys_csi2_pdata *pdata;
	struct ipu7_isys *isys;
	struct ipu7_isys_video av[NR_OF_CSI2_SRC_PADS];

	void __iomem *base;
	u32 receiver_errors;
	u32 legacy_irq_mask;
	unsigned int nlanes;
	unsigned int port;
	unsigned int phy_mode;
	unsigned int stream_count;
};

#define ipu7_isys_subdev_to_csi2(__sd)			\
	container_of(__sd, struct ipu7_isys_csi2, asd)

#define to_ipu7_isys_csi2(__asd) container_of(__asd, struct ipu7_isys_csi2, asd)

s64 ipu7_isys_csi2_get_link_freq(struct ipu7_isys_csi2 *csi2);
int ipu7_isys_csi2_init(struct ipu7_isys_csi2 *csi2, struct ipu7_isys *isys,
			void __iomem *base, unsigned int index);
void ipu7_isys_csi2_cleanup(struct ipu7_isys_csi2 *csi2);
void ipu7_isys_csi2_sof_event_by_stream(struct ipu7_isys_stream *stream);
void ipu7_isys_csi2_eof_event_by_stream(struct ipu7_isys_stream *stream);
int ipu7_isys_csi2_get_remote_desc(u32 source_stream,
				   struct ipu7_isys_csi2 *csi2,
				   struct media_entity *source_entity,
				   struct v4l2_mbus_frame_desc_entry *entry,
				   int *nr_queues);
#endif /* IPU7_ISYS_CSI2_H */
