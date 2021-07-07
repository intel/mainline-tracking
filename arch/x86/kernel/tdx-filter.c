// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Intel Corporation
 */
#define pr_fmt(fmt) "TDX: " fmt

#include <linux/acpi.h>
#include <linux/pci.h>
#include <linux/device/filter.h>
#include <linux/protected_guest.h>

#include <asm/tdx.h>
#include <asm/cmdline.h>

static bool tdg_disable_filter;

/* PCI bus allow-list devices */
static struct pci_filter_node pci_allow_list[] = {
	{ PCI_VENDOR_ID_REDHAT_QUMRANET, 0x1000 }, /* Virtio NET */
	{ PCI_VENDOR_ID_REDHAT_QUMRANET, 0x1001 }, /* Virtio block */
	{ PCI_VENDOR_ID_REDHAT_QUMRANET, 0x1003 }, /* Virtio console */
	{ PCI_VENDOR_ID_REDHAT_QUMRANET, 0x1009 }, /* Virtio FS */

	{ PCI_VENDOR_ID_REDHAT_QUMRANET, 0x1041 }, /* Virtio 1.0 NET */
	{ PCI_VENDOR_ID_REDHAT_QUMRANET, 0x1042 }, /* Virtio 1.0 block */
	{ PCI_VENDOR_ID_REDHAT_QUMRANET, 0x1043 }, /* Virtio 1.0 console */
	{ PCI_VENDOR_ID_REDHAT_QUMRANET, 0x1049 }, /* Virtio 1.0 FS */
};

static struct pci_dev_filter_data pci_data = {
	.allow_list = pci_allow_list,
	.len = ARRAY_SIZE(pci_allow_list)
};

static struct dev_filter_node pci_filter_node = {
	.name = "pci",
	.data = &pci_data,
};

bool tdg_allowed_port(short int port)
{
	if (tdg_debug_enabled() && tdg_disable_filter)
		return true;

	switch (port) {
	/* MC146818 RTC */
	case 0x70 ... 0x71:
	/* PCI */
	case 0xcf8 ... 0xcff:
		return true;
	/* ACPI ports list:
	 * 0600-0603 : ACPI PM1a_EVT_BLK
	 * 0604-0605 : ACPI PM1a_CNT_BLK
	 * 0608-060b : ACPI PM_TMR
	 * 0620-062f : ACPI GPE0_BLK
	 */
	case 0x600 ... 0x62f:
		return true;
	/* COM1 */
	case 0x3f8:
	case 0x3f9:
	case 0x3fa:
	case 0x3fd:
		return tdg_debug_enabled();
	default:
		return false;
	}
}

void __init tdg_filter_init(void)
{
	if (!prot_guest_has(PR_GUEST_TDX))
		return;

	tdg_disable_filter = cmdline_find_option_bool(boot_command_line,
						      "tdx_disable_filter");

	/* Consider tdg_disable_filter only in TDX debug mode */
	if (tdg_debug_enabled() && tdg_disable_filter) {
		pr_info("Disabled TDX guest filter support\n");
		return;
	}

	/* Register TDX PCI device filter list */
	register_dev_filter(&pci_filter_node);

	acpi_tbl_allow_setup("RDSP,XSDT,FACP,DSDT,FACS,APIC,SVKL");

	pr_info("Enabled TDX guest device filter\n");
}
