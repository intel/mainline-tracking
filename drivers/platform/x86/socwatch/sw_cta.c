/* SPDX-License-Identifier: GPL-2.0 AND BSD-3-Clause
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2019 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Contact Information:
 * SoC Watch Developer Team <socwatchdevelopers@intel.com>
 * Intel Corporation,
 * 1300 S Mopac Expwy,
 * Austin, TX 78746
 *
 * BSD LICENSE
 *
 * Copyright(c) 2019 Intel Corporation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/compiler.h>     /* Definition of __weak */
#include <linux/kref.h> /* struct kref */
#include <linux/notifier.h> /* struct notifier_block */
#include <linux/pci.h> /* struct pci_dev */
#include <linux/ioport.h> /* struct resource */
#include <linux/kref.h> /* struct kref */

#include "sw_structs.h"      /* sw_driver_io_descriptor */
#include "sw_cta.h"

/* *********************************
 * Begin CTA driver import
 * *********************************
 */

/*
 * Struct definitions taken from CTA driver.
 */

struct telem_header {
        u8      access_type;
        u8      telem_type;
        u16     size;
        u32     guid;
        u32     base_offset;
};

struct telem_endpoint {
        struct pci_dev        *parent;
        struct telem_header   header;
        void __iomem          *base;
        struct resource       res;
        bool                  present;
        struct kref           kref;
};

struct telem_endpoint_info {
        struct pci_dev          *pdev;
        struct telem_header     header;
};

/*
 * Weak linkage of functions from the CTA driver
 */

/**
 * cta_telem_get_next_endpoint() - Get next device id for a telemetry endpoint
 * @start:  starting devid to look from
 *
 * This functions can be used in a while loop predicate to retrieve the devid
 * of all available telemetry endpoints. Functions cta_telem_get_next_endpoint()
 * and cta_telem_register_endpoint() can be used inside of the loop to examine
 * endpoint info and register to receive a pointer to the endpoint. The pointer
 * is then usable in the telemetry read calls to access the telemetry data.
 *
 * Return:
 * * devid       - devid of the next present endpoint from start
 * * 0           - when no more endpoints are present after start
 */
extern int __weak
cta_telem_get_next_endpoint(int start);

/**
 * cta_telem_register_endpoint() - Register a telemetry endpoint
 * @devid: device id/handle of the telemetry endpoint
 *
 * Increments the kref usage counter for the endpoint.
 *
 * Return:
 * * endpoint    - On success returns pointer to the telemetry endpoint
 * * -ENXIO      - telemetry endpoint not found
 */
extern struct telem_endpoint * __weak
cta_telem_register_endpoint(int devid);

/**
 * cta_telem_unregister_endpoint() - Unregister a telemetry endpoint
 * @ep:   ep structure to populate.
 *
 * Decrements the kref usage counter for the endpoint.
 */
extern void __weak
cta_telem_unregister_endpoint(struct telem_endpoint *ep);

/**
 * cta_telem_get_endpoint_info() - Get info for an endpoint from its devid
 * @devid:  device id/handle of the telemetry endpoint
 * @info:   Endpoint info structure to be populated
 *
 * Return:
 * * 0           - Success
 * * -ENXIO      - telemetry endpoint not found for the devid
 * * -EINVAL     - @info is NULL
 */
extern int __weak
cta_telem_get_endpoint_info(int devid,
				struct telem_endpoint_info *info);

/**
 * cta_telem_read32() - Read dwords from telemetry sram
 * @ep:     Telemetry endpoint to be read
 * @offset: Register offset in bytes
 * @data:   Allocated dword buffer
 * @count:  Number of dwords requested
 *
 * Callers must ensure reads are aligned. When the call returns -ENODEV,
 * the device has been removed and callers should unregister the telemetry
 * endpoint.
 *
 * Return:
 * * 0           - Success
 * * -ENODEV	 - The device is not present.
 * * -EINVAL	 - The offset is out out bounds
 * * -EPIPE	 - The device was removed during the read. Data written
 *		   but should be considered invalid.
 */
extern int __weak
cta_telem_read32(struct telem_endpoint *ep, u32 offset, u32 *data,
		     u32 count);

/**
 * cta_telem_read64() - Read qwords from counter sram
 * @ep:     Telemetry endpoint to be read
 * @offset: Register offset in bytes
 * @data:   Allocated qword buffer
 * @count:  Number of qwords requested
 *
 * Callers must ensure reads are aligned. When the call returns -ENODEV,
 * the device has been removed and callers should unregister the telemetry
 * endpoint.
 *
 * Return:
 * * 0           - Success
 * * -ENODEV	 - The device is not present.
 * * -EINVAL	 - The offset is out out bounds
 * * -EPIPE	 - The device was removed during the read. Data written
 *		   but should be considered not valid.
 */
extern int __weak
cta_telem_read64(struct telem_endpoint *ep, u32 offset, u64 *data,
		     u32 count);

/* Notifiers */

#define CTA_TELEM_NOTIFY_ADD	0
#define CTA_TELEM_NOTIFY_REMOVE	1

/**
 * cta_telem_register_notifier() - Receive notification endpoint events
 * @nb:   Notifier block
 *
 * Events:
 *   CTA_TELEM_NOTIFY_ADD   - An endpoint has been added. Notifier data
 *                            is the devid
 *   CTA_TELEM_NOTIF_REMOVE - An endpoint has been removed. Notifier data
 *                            is the devid
 */
extern int __weak
cta_telem_register_notifier(struct notifier_block *nb);

/**
 * cta_telem_unregister_notifier() - Unregister notification of endpoint events
 * @nb:   Notifier block
 *
 */
extern int __weak
cta_telem_unregister_notifier(struct notifier_block *nb);

/* *********************************
 * End CTA driver import
 * *********************************
 */

#define MAX_TELEM_ENDPOINTS MAX_TELEM_AGGR_DEVICES /* For now */
static struct telem_endpoint *s_telem_endpoints[MAX_TELEM_ENDPOINTS]; /* TODO: make this a linked list instead */
size_t s_endpoint_index = 0;

static struct _sw_aggregator_msg s_telem_aggregators;

void sw_read_cta_info(char *dst, int cpu,
		const struct sw_driver_io_descriptor *descriptor,
		u16 counter_size_in_bytes)
{
	u64 *data64 = (u64 *)dst;
	u32 *data32 = (u32 *)dst;
	int retval = 0;
	const struct sw_driver_aggr_telem_io_descriptor *td =
		&(descriptor->aggr_telem_descriptor);
	u32 offset = (u32)td->offset;
	struct telem_endpoint *ep = s_telem_endpoints[0];

	/* We can only support one endpoint as of now */
	if (!ep) {
		return;
	}
	switch (descriptor->counter_size_in_bytes) {
		case 4:
			retval = cta_telem_read32(ep, offset, data32, td->num_entries);
			break;
		case 8:
			retval = cta_telem_read64(ep, offset, data64, td->num_entries);
			break;
		default:
			printk(KERN_ERR "Invalid CTA counter size %u\n", descriptor->counter_size_in_bytes);
			return;
	}
	if (retval) {
		printk(KERN_ERR "Error reading %u byte CTA value from offset 0x%x, val = %d\n", descriptor->counter_size_in_bytes, offset, retval);
	}
}

bool sw_cta_available(void)
{
	/* 1: check if the CTA driver is loaded */
	if (!cta_telem_get_endpoint_info) {
		return false;
	}
	/* 2: TODO: other checks here */
	/*
	 * Note: registering telemetry endpoints done in 'register' since
	 * those endpoints also need to be unregistered (Done in 'fini')
	 */
	return true;
}

bool sw_cta_register(void)
{
	unsigned long index = 0;
	if (!sw_cta_available()) {
		return false;
	}
        s_telem_aggregators.num_entries = 0;
        s_endpoint_index = 0;
	/*
	 * Retrieve list of telemetry endpoints.
	 * TODO: we can only support one endpoint as of now, so should we be
	 * checking the GUID to retrieve only the endpoints of interest?
	 */
	s_endpoint_index = 0;
	while ((index = cta_telem_get_next_endpoint(index)) && s_endpoint_index < (MAX_TELEM_ENDPOINTS-1)) {
		struct telem_endpoint_info ep_info;
		if (cta_telem_get_endpoint_info(index, &ep_info)) {
			printk(KERN_ERR "Could not retrieve telemetry header for CTA endpoint %lu\n", index);
			continue;
		}
		s_telem_endpoints[s_endpoint_index] = cta_telem_register_endpoint(index);
		s_telem_aggregators.info[s_telem_aggregators.num_entries++].globalUniqueID = ep_info.header.guid;
		++s_endpoint_index;
	}
	return s_endpoint_index > 0;
}

bool sw_cta_unregister(void)
{
	size_t i=0;
	if (!sw_cta_available()) {
		return false;
	}
	for (i=0; i<s_endpoint_index; ++i) {
		cta_telem_unregister_endpoint(s_telem_endpoints[i]);
	}
	s_endpoint_index = 0;
	s_telem_aggregators.num_entries = 0;
	return true;
}

struct _sw_aggregator_msg *sw_cta_aggregators(void)
{
	return &s_telem_aggregators;
}
