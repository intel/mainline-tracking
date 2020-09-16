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

/*
 * @brief Read buffer from xpcie. Function will block when no data.
 *
 * @param[in] xpcie          - pointer to xpcie instance
 * @param[in] buffer        - pointer to buffer
 * @param[in] length        - max bytes to copy into buffer
 * @param[in] timeout_ms    - timeout in ms for blocking when no data
 *
 * @return:
 *      >=0 - number of bytes read
 *      <0  - linux error code
 *              -ETIME - timeout
 *              -EINTR - interrupted
 */
int intel_xpcie_core_read(struct xpcie *xpcie, void *buffer, size_t *length,
			  u32 timeout_ms);

/*
 * @brief Writes buffer to xpcie. Function will block when no buffer.
 *
 * @param[in] xpcie          - pointer to xpcie instance
 * @param[in] buffer        - pointer to buffer
 * @param[in] length        - length of buffer to copy from
 * @param[in] timeout_ms    - timeout in ms for blocking when no buffer
 *
 * @return:
 *      >=0 - number of bytes write
 *      <0  - linux error code
 *              -ETIME - timeout
 *              -EINTR - interrupted
 */
int intel_xpcie_core_write(struct xpcie *xpcie, void *buffer, size_t *length,
			   u32 timeout_ms);

#ifdef XLINK_PCIE_LOCAL
struct xpcie *intel_xpcie_core_get_by_id(u32 sw_device_id);
#endif

#endif /* XPCIE_CORE_HEADER_ */
