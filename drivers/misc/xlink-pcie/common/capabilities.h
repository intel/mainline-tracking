/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#ifndef XPCIE_CAPABILITIES_HEADER_
#define XPCIE_CAPABILITIES_HEADER_

#include "xpcie.h"
#include "common.h"

#define XPCIE_CAP_TTL (32)

static inline
void *intel_xpcie_cap_find(struct xpcie *xpcie, u32 start, u16 id)
{
	int ttl = XPCIE_CAP_TTL;
	struct xpcie_cap_hdr *hdr;
	struct xpcie_cap_hdr cur_hdr;

	// If user didn't specify start, assume start of mmio
	if (!start)
		start = intel_xpcie_ioread32(&xpcie->mmio->cap_offset);

	// Read header info
#ifdef XLINK_PCIE_REMOTE
	hdr = (struct xpcie_cap_hdr *)((void __iomem *)xpcie->mmio + start);
#else
	hdr = (struct xpcie_cap_hdr *)((void *)xpcie->mmio + start);
#endif
	// Check if we still have time to live
	while (ttl--) {
#ifdef XLINK_PCIE_REMOTE
		memcpy_fromio(&cur_hdr, hdr, sizeof(struct xpcie_cap_hdr));
#else
		cur_hdr = *hdr;
#endif
		// If cap matches, return header
		if (cur_hdr.id == id)
			return hdr;
		// If cap is NULL, we are at the end of the list
		else if (cur_hdr.id == XPCIE_CAP_NULL)
			return NULL;
		// If no match and no end of list, traverse the linked list
		else
#ifdef XLINK_PCIE_REMOTE
			hdr = (struct xpcie_cap_hdr *)
				((void __iomem *)xpcie->mmio + cur_hdr.next);
#else
			hdr = (struct xpcie_cap_hdr *)
				((void *)xpcie->mmio + cur_hdr.next);
#endif
	}

	// If we reached here, the capability list is corrupted
	return NULL;
}

static inline
void intel_xpcie_set_td_address(struct xpcie_transfer_desc *td, u64 address)
{
	intel_xpcie_iowrite64(address, &td->address);
}

static inline
u64 intel_xpcie_get_td_address(struct xpcie_transfer_desc *td)
{
	return intel_xpcie_ioread64(&td->address);
}

static inline
void intel_xpcie_set_td_length(struct xpcie_transfer_desc *td, u32 length)
{
	intel_xpcie_iowrite32(length, &td->length);
}

static inline
u32 intel_xpcie_get_td_length(struct xpcie_transfer_desc *td)
{
	return intel_xpcie_ioread32(&td->length);
}

static inline
void intel_xpcie_set_td_interface(struct xpcie_transfer_desc *td, u16 interface)
{
	intel_xpcie_iowrite16(interface, &td->interface);
}

static inline
u16 intel_xpcie_get_td_interface(struct xpcie_transfer_desc *td)
{
	return intel_xpcie_ioread16(&td->interface);
}

static inline
void intel_xpcie_set_td_status(struct xpcie_transfer_desc *td, u16 status)
{
	intel_xpcie_iowrite16(status, &td->status);
}

static inline
u16 intel_xpcie_get_td_status(struct xpcie_transfer_desc *td)
{
	return intel_xpcie_ioread16(&td->status);
}

static inline
void intel_xpcie_set_tdr_head(struct xpcie_pipe *p, u32 head)
{
	intel_xpcie_iowrite32(head, p->head);
}

static inline
u32 intel_xpcie_get_tdr_head(struct xpcie_pipe *p)
{
	return intel_xpcie_ioread32(p->head);
}

static inline
void intel_xpcie_set_tdr_tail(struct xpcie_pipe *p, u32 tail)
{
	intel_xpcie_iowrite32(tail, p->tail);
}

static inline
u32 intel_xpcie_get_tdr_tail(struct xpcie_pipe *p)
{
	return intel_xpcie_ioread32(p->tail);
}
#endif // XPCIE_CAPABILITIES_HEADER_
