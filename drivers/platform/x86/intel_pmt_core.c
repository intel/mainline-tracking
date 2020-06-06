// SPDX-License-Identifier: GPL-2.0
/*
 * Intel Platform Monitory Technology Core driver
 *
 * Copyright (c) 2020, Intel Corporation.
 * All Rights Reserved.
 *
 * Author: "David E. Box" <david.e.box@linux.intel.com>
 */

#include <linux/bits.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/types.h>

#include "intel_pmt_core.h"

/* Access types */
#define ACCESS_BARID		2
#define ACCESS_LOCAL		3

#define ACCESS(v)		((v) & GENMASK(3, 0))
#define TYPE(v)			(((v) & GENMASK(11, 4)) >> 4)
/* size is in bytes */
#define SIZE(v)			(((v) & GENMASK(27, 12)) >> 10)
#define IRQ_EN(v)		((v) & BIT(28))
#define BIR(v)			((v) & GENMASK(2, 0))

#define CRASHLOG_VERSION(v)	(((v) & GENMASK(19, 16)) >> 16)

static const struct pci_device_id pmt_early_client_pci_ids[] = {
	{ PCI_VDEVICE(INTEL, 0x9a0d) },
	{ PCI_VDEVICE(INTEL, 0x490e) },
	{ PCI_VDEVICE(INTEL, 0x467d) },
	{ }
};

bool pmt_is_early_client_hw(struct device *dev)
{
	struct pci_dev *parent;

	parent = to_pci_dev(dev->parent);
	if (pci_match_id(pmt_early_client_pci_ids, parent))
		return true;

	return false;
}
EXPORT_SYMBOL_GPL(pmt_is_early_client_hw);

void pmt_populate_header(enum pmt_cap cap, void __iomem *disc_offset,
			 struct pmt_header *header)
{
	u32 discovery_header = readl(disc_offset);

	header->access_type = ACCESS(discovery_header);
	header->type = TYPE(discovery_header);
	header->guid = readl(disc_offset + GUID_OFFSET);
	header->base_offset = readl(disc_offset + BASE_OFFSET);
	/*
	 * For non-local access types the lower 3 bits of base offset
	 * contains the index of the base address register where the
	 * telemetry can be found.
	 */
	header->bir = BIR(header->base_offset);
	header->base_offset ^= header->bir;

	switch (cap) {
	case PMT_CAP_TELEM:
	case PMT_CAP_WATCHER:
		header->size = SIZE(readl(disc_offset));

		if (cap == PMT_CAP_WATCHER)
			header->irq_support = !!(IRQ_EN(readl(disc_offset)));
		break;
	case PMT_CAP_CRASHLOG:
	        /* Size is measured in DWORDs */
		header->crashlog_size = readl(disc_offset +
					      CRASHLOG_SIZE_OFFSET);
		header->crashlog_version = CRASHLOG_VERSION(discovery_header);
	}
}
EXPORT_SYMBOL_GPL(pmt_populate_header);

int pmt_get_base_address(struct device *dev, struct pmt_header *header,
			 struct resource *header_res, unsigned long *address)
{
	struct pci_dev *pci_dev = to_pci_dev(dev->parent);
	/* Local access and BARID only for now */
	switch (header->access_type) {
	case ACCESS_LOCAL:
		if (header->bir) {
			dev_err(dev,
				"Unsupported BAR index %d for access type %d\n",
				header->bir, header->access_type);
			return -EINVAL;
		}

		*address = header_res->start + resource_size(header_res) +
			   header->base_offset;

		/*
		 * XXX: For Intel internal use only to address hardware bug
		 * that will be fixed in production. In the bug, local refers to
		 * an address in the same bar the header but at a fixed instead
		 * of relative offset.
		 */
		if (pmt_is_early_client_hw(dev)) {
			unsigned long pf_addr, mask;

			dev_info(dev, "FW Bug quirk for base offset\n");
			pf_addr = PFN_PHYS(PHYS_PFN(header_res->start)) +
					   header->base_offset;
			mask = ~GENMASK(fls(header->base_offset), 0);
			*address = (pf_addr & mask) + header->base_offset;
		}
		break;

	case ACCESS_BARID:
		*address = pci_resource_start(pci_dev, header->bir) +
			   header->base_offset;
		break;

	default:
		dev_err(dev, "Unsupported access type %d\n",
			header->access_type);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(pmt_get_base_address);

MODULE_LICENSE("GPL v2");
