/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#ifndef XPCIE_DMA_HEADER_
#define XPCIE_DMA_HEADER_

#include <linux/types.h>
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>

int intel_xpcie_ep_dma_init(struct pci_epf *epf);
int intel_xpcie_ep_dma_uninit(struct pci_epf *epf);
int intel_xpcie_ep_dma_read_ll(struct pci_epf *epf, int chan, int descs_num);
int intel_xpcie_ep_dma_write_ll(struct pci_epf *epf, int chan, int descs_num);
bool intel_xpcie_ep_dma_enabled(struct pci_epf *epf);
int intel_xpcie_ep_dma_reset(struct pci_epf *epf);

#endif // XPCIE_DMA_HEADER_
