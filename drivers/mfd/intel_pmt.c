// SPDX-License-Identifier: GPL-2.0
/*
 * Intel Platform Monitoring Technology PMT driver
 *
 * Copyright (c) 2020, Intel Corporation.
 * All Rights Reserved.
 *
 * Author: David E. Box <david.e.box@linux.intel.com>
 */

#include <linux/bits.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/types.h>

/* Intel DVSEC capability vendor space offsets */
#define INTEL_DVSEC_ENTRIES		0xA
#define INTEL_DVSEC_SIZE		0xB
#define INTEL_DVSEC_TABLE		0xC
#define INTEL_DVSEC_TABLE_BAR(x)	((x) & GENMASK(2, 0))
#define INTEL_DVSEC_TABLE_OFFSET(x)	((x) & GENMASK(31, 3))

/* Intel Extended Features */
#define DVSEC_INTEL_ID_TELEMETRY	2
#define DVSEC_INTEL_ID_WATCHER		3
#define DVSEC_INTEL_ID_CRASHLOG		4

#define FEATURE_ID_NAME_LENGTH		16

struct intel_dvsec_header {
	u8	rev;
	u16	length;
	u16	id;
	u8	num_entries;
	u8	entry_size;
	u8	tbir;
	u32	offset;
};

static int intel_ext_feature_allow_list[] = {
	DVSEC_INTEL_ID_TELEMETRY,
	DVSEC_INTEL_ID_WATCHER,
	DVSEC_INTEL_ID_CRASHLOG,
};

enum pmt_quirks {
	/* Watcher capability not supported */
	PMT_QUIRK_NO_WATCHER	= BIT(0),

	/* Crashlog capability not supported */
	PMT_QUIRK_NO_CRASHLOG	= BIT(1),

	/* Use shift instead of mask to read discovery table offset */
	PMT_QUIRK_TABLE_SHIFT	= BIT(2),

	/* DVSEC not present (provided in driver data) */
	PMT_QUIRK_NO_DVSEC	= BIT(3),
};

struct pmt_platform_info {
	unsigned long quirks;
	struct intel_dvsec_header **capabilities;
};

static const struct pmt_platform_info tgl_info = {
	.quirks = PMT_QUIRK_NO_WATCHER | PMT_QUIRK_NO_CRASHLOG |
		  PMT_QUIRK_TABLE_SHIFT,
};

/* DG1 Platform with DVSEC quirk*/
static struct intel_dvsec_header dg1_telemetry = {
	.length = 0x10,
	.id = 2,
	.num_entries = 1,
	.entry_size = 3,
	.tbir = 0,
	.offset = 0x466000,
};

static struct intel_dvsec_header *dg1_capabilities[] = {
	&dg1_telemetry,
	NULL
};

static const struct pmt_platform_info dg1_info = {
	.quirks = PMT_QUIRK_NO_DVSEC,
	.capabilities = dg1_capabilities,
};

static bool pmt_feature_disabled(u16 id, unsigned long quirks)
{
	bool ret;

	switch (id) {
	case DVSEC_INTEL_ID_WATCHER:
		if (quirks & PMT_QUIRK_NO_WATCHER)
			ret = true;
		break;

	case DVSEC_INTEL_ID_CRASHLOG:
		if (quirks & PMT_QUIRK_NO_CRASHLOG)
			ret = true;
		break;

	default:
		ret = false;
		break;
	}

	return ret;
}

static bool intel_ext_feature_allowed(u16 id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(intel_ext_feature_allow_list); i++)
		if (intel_ext_feature_allow_list[i] == id)
			return true;

	return false;
}

static int pmt_add_dev(struct pci_dev *pdev, struct intel_dvsec_header *header,
		       unsigned long quirks)
{
	struct device *dev = &pdev->dev;
	struct resource *res, *tmp;
	struct mfd_cell *cell;
	char feature_id_name[FEATURE_ID_NAME_LENGTH];
	int count = header->num_entries;
	int size = header->entry_size;
	int id = header->id;
	int i;

	if (!intel_ext_feature_allowed(id))
		return -EINVAL;

	if (pmt_feature_disabled(id, quirks))
		return -EINVAL;

	snprintf(feature_id_name, sizeof(feature_id_name), "intel-dvsec-%d", id);

	if (!header->num_entries || !header->entry_size) {
		dev_err(dev, "Invalid count or size for %s header\n", feature_id_name);
		return -EINVAL;
	}

	cell = devm_kzalloc(dev, sizeof(*cell), GFP_KERNEL);
	if (!cell)
		return -ENOMEM;

	res = devm_kcalloc(dev, count, sizeof(*res), GFP_KERNEL);
	if (!res)
		return -ENOMEM;

	if (quirks & PMT_QUIRK_TABLE_SHIFT)
		header->offset >>= 3;

	/*
	 * The PMT DVSEC contains the starting offset and count for a block of
	 * discovery tables, each providing access to monitoring facilities for
	 * a section of the device. Create a resource list of these tables to
	 * provide to the driver.
	 */
	for (i = 0, tmp = res; i < count; i++, tmp++) {
		tmp->start = pdev->resource[header->tbir].start +
			     header->offset + i * (size << 2);
		tmp->end = tmp->start + (size << 2) - 1;
		tmp->flags = IORESOURCE_MEM;
	}

	cell->resources = res;
	cell->num_resources = count;
	cell->name = feature_id_name;

	return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO, cell, 1, NULL, 0,
				    NULL);
}

static int pmt_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct pmt_platform_info *info;
	unsigned long quirks = 0;
	bool found_devices = false;
	int ret, pos = 0;

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	info = (struct pmt_platform_info *)id->driver_data;

	if (info)
		quirks = info->quirks;

	if (info && (info->quirks & PMT_QUIRK_NO_DVSEC)) {
		struct intel_dvsec_header **header;

		header = info->capabilities;
		while (*header) {
			ret = pmt_add_dev(pdev, *header, quirks);
			if (ret)
				dev_warn(&pdev->dev,
					 "Failed to add device for DVSEC id %d\n",
					 (*header)->id);
			else
				found_devices = true;

			++header;
		}
	} else {
		do {
			struct intel_dvsec_header header;
			u32 table, hdr;
			u16 vid;

			pos = pci_find_next_ext_capability(pdev, pos, PCI_EXT_CAP_ID_DVSEC);
			if (!pos)
				break;

			pci_read_config_dword(pdev, pos + PCI_DVSEC_HEADER1, &hdr);
			vid = PCI_DVSEC_HEADER_VID(hdr);
			if (vid != PCI_VENDOR_ID_INTEL)
				continue;

			pci_read_config_word(pdev, pos + PCI_DVSEC_HEADER2, &header.id);

			/* Support only revision 1 */
			header.rev = PCI_VNDR_HEADER_REV(hdr);
			if (header.rev != 1) {
				dev_warn(&pdev->dev, "Unsupported DVSEC revision %d\n",
					 header.rev);
				continue;
			}

			header.length = PCI_DVSEC_HEADER_LEN(hdr);

			pci_read_config_byte(pdev, pos + INTEL_DVSEC_ENTRIES,
					     &header.num_entries);
			pci_read_config_byte(pdev, pos + INTEL_DVSEC_SIZE,
					     &header.entry_size);
			pci_read_config_dword(pdev, pos + INTEL_DVSEC_TABLE,
					      &table);

			header.tbir = INTEL_DVSEC_TABLE_BAR(table);
			header.offset = INTEL_DVSEC_TABLE_OFFSET(table);

			ret = pmt_add_dev(pdev, &header, quirks);
			if (ret)
				continue;

			found_devices = true;
		} while (true);
	}

	if (!found_devices)
		return -ENODEV;

	pm_runtime_put(&pdev->dev);
	pm_runtime_allow(&pdev->dev);

	return 0;
}

static void pmt_pci_remove(struct pci_dev *pdev)
{
	pm_runtime_forbid(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);
}

#define PCI_DEVICE_ID_INTEL_PMT_ADL	0x467d
#define PCI_DEVICE_ID_INTEL_PMT_DG1	0x490e
#define PCI_DEVICE_ID_INTEL_PMT_OOBMSM	0x09a7
#define PCI_DEVICE_ID_INTEL_PMT_TGL	0x9a0d
static const struct pci_device_id pmt_pci_ids[] = {
	{ PCI_DEVICE_DATA(INTEL, PMT_ADL, &tgl_info) },
	{ PCI_DEVICE_DATA(INTEL, PMT_DG1, &dg1_info) },
	{ PCI_DEVICE_DATA(INTEL, PMT_OOBMSM, NULL) },
	{ PCI_DEVICE_DATA(INTEL, PMT_TGL, &tgl_info) },
	{ }
};
MODULE_DEVICE_TABLE(pci, pmt_pci_ids);

static struct pci_driver pmt_pci_driver = {
	.name = "intel-pmt",
	.id_table = pmt_pci_ids,
	.probe = pmt_pci_probe,
	.remove = pmt_pci_remove,
};
module_pci_driver(pmt_pci_driver);

MODULE_AUTHOR("David E. Box <david.e.box@linux.intel.com>");
MODULE_DESCRIPTION("Intel Platform Monitoring Technology PMT driver");
MODULE_LICENSE("GPL v2");
