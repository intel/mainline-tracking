/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2023-2024 Intel Corporation */
#ifndef _ISSEI_HW_HECI_H_
#define _ISSEI_HW_HECI_H_

#include <linux/types.h>
#include <linux/spinlock.h>

#include "issei_dev.h"

struct device;
struct issei_device;

/*
 * hw_heci_cfg - issei heci device configuration
 *
 * @dma_length: DMA area length
 */
struct hw_heci_cfg {
	const struct issei_dma_length dma_length;
};

#define HW_HECI_RPM_TIMEOUT_MSEC 60000
#define HW_HECI_READY_TIMEOUT_MSEC 2000
#define HW_HECI_SETUP_TIMEOUT_MSEC 2000

/**
 * struct issei_heci_hw - issei heci hw specific data
 *
 * @cfg: per device generation config and ops
 * @mem_addr: io memory address
 * @irq: device irq number
 * @access_lock: spinlock to protect hw access
 * @hbuf_depth: depth of hardware host/write buffer in slots
 */
struct issei_heci_hw {
	const struct hw_heci_cfg *cfg;
	void __iomem *mem_addr;
	int irq;
	/* lock to serialize HW access */
	spinlock_t access_lock;
	u8 hbuf_depth;
};

#define to_heci_hw(dev) ((struct issei_heci_hw *)(dev)->hw)

const struct hw_heci_cfg *issei_heci_get_cfg(kernel_ulong_t idx);

struct issei_device *issei_heci_dev_init(struct device *parent, const struct hw_heci_cfg *cfg);

irqreturn_t issei_heci_irq_quick_handler(int irq, void *dev_id);
irqreturn_t issei_heci_irq_thread_handler(int irq, void *dev_id);

#endif /* _ISSEI_HW_HECI_H_ */
