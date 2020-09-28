/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#ifndef XPCIE_CORE_HEADER_
#define XPCIE_CORE_HEADER_

#include "xpcie.h"

/* max should be always power of '2' */
#define XPCIE_CIRCULAR_INC(val, max) (((val) + 1) & ((max) - 1))

int intel_xpcie_core_init(struct xpcie *xpcie);
void intel_xpcie_core_cleanup(struct xpcie *xpcie);
int intel_xpcie_core_read(struct xpcie *xpcie, void *buffer, size_t *length,
			  u32 timeout_ms);
int intel_xpcie_core_write(struct xpcie *xpcie, void *buffer, size_t *length,
			   u32 timeout_ms);
u32 intel_xpcie_get_device_num(u32 *id_list);
struct xpcie_dev *intel_xpcie_get_device_by_id(u32 id);
int intel_xpcie_get_device_name_by_id(u32 id, char *device_name,
				      size_t name_size);
int intel_xpcie_get_device_status_by_id(u32 id, u32 *status);
int intel_xpcie_pci_connect_device(u32 id);
int intel_xpcie_pci_read(u32 id, void *data, size_t *size, u32 timeout);
int intel_xpcie_pci_write(u32 id, void *data, size_t *size, u32 timeout);
int intel_xpcie_pci_reset_device(u32 id);
#endif /* XPCIE_CORE_HEADER_ */
