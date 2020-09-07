/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#ifndef XPCIE_EPF_HEADER_
#define XPCIE_EPF_HEADER_

#include "../common/xpcie.h"
#include "../common/util.h"

extern u32 xlink_sw_id;

void intel_xpcie_register_host_irq(struct xpcie *xpcie,
				   irq_handler_t func);

int intel_xpcie_raise_irq(struct xpcie *xpcie,
			  enum xpcie_doorbell_type type);

/*
 * These two functions are for DMA linked list mode.
 *
 * Caller should set the dst/src addresses and length for DMA descriptors in
 * xpcie_epf.dma_ll_tx_descs/dma_ll_rx_descs.
 */
int intel_xpcie_copy_from_host_ll(struct xpcie *xpcie,
				  int chan, int descs_num);
int intel_xpcie_copy_to_host_ll(struct xpcie *xpcie,
				int chan, int descs_num);

#endif /* XPCIE_EPF_HEADER_ */
