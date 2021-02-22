/* SPDX-License-Identifier: GPL-2.0 only */
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2019 Intel Corporation
 *
 ****************************************************************************/

typedef int (*mxlk_pcie_boot_event)(uint32_t phys_dev_id);


int mxlk_pcie_connect_boot_device(const char*dev_name,
				  uint32_t *phys_dev_id,
				  mxlk_pcie_boot_event notif_fn);
int mxlk_pcie_boot_mmio_write(uint32_t phys_dev_id, uint32_t offset,
			      void *data, size_t size);
int mxlk_pcie_boot_mmio_read(uint32_t phys_dev_id, uint32_t offset,
			     void *status, size_t size);
int mxlk_pcie_disconnect_boot_device(uint32_t phys_dev_id);
