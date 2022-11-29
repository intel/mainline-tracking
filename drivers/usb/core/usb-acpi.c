// SPDX-License-Identifier: GPL-2.0
/*
 * USB-ACPI glue code
 *
 * Copyright 2012 Red Hat <mjg@redhat.com>
 */
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/acpi.h>
#include <linux/pci.h>
#include <linux/usb/hcd.h>

#include "hub.h"

/**
 * usb_acpi_power_manageable - check whether usb port has
 * acpi power resource.
 * @hdev: USB device belonging to the usb hub
 * @index: port index based zero
 *
 * Return true if the port has acpi power resource and false if no.
 */
bool usb_acpi_power_manageable(struct usb_device *hdev, int index)
{
	acpi_handle port_handle;
	int port1 = index + 1;

	port_handle = usb_get_hub_port_acpi_handle(hdev,
		port1);
	if (port_handle)
		return acpi_bus_power_manageable(port_handle);
	else
		return false;
}
EXPORT_SYMBOL_GPL(usb_acpi_power_manageable);

#define UUID_USB_CONTROLLER_DSM "ce2ee385-00e6-48cb-9f05-2edb927c4899"
#define USB_DSM_DISABLE_U1_U2_FOR_PORT	5

/**
 * usb_acpi_port_lpm_incapable - check if lpm should be disabled for a port.
 * @hdev: USB device belonging to the usb hub
 * @index: zero based port index
 *
 * Some USB3 ports may not support USB3 link power management U1/U2 states
 * due to different retimer setup. ACPI provides _DSM method which returns 0x01
 * if U1 and U2 states should be disabled. Evaluate _DSM with:
 * Arg0: UUID = ce2ee385-00e6-48cb-9f05-2edb927c4899
 * Arg1: Revision ID = 0
 * Arg2: Function Index = 5
 * Arg3: (empty)
 *
 * Return 1 if USB3 port is LPM incapable, negative on error, otherwise 0
 */

int usb_acpi_port_lpm_incapable(struct usb_device *hdev, int index)
{
	union acpi_object *obj;
	acpi_handle port_handle;
	int port1 = index + 1;
	guid_t guid;
	int ret;

	ret = guid_parse(UUID_USB_CONTROLLER_DSM, &guid);
	if (ret)
		return ret;

	port_handle = usb_get_hub_port_acpi_handle(hdev, port1);
	if (!port_handle) {
		dev_dbg(&hdev->dev, "port-%d no acpi handle\n", port1);
		return -ENODEV;
	}

	if (!acpi_check_dsm(port_handle, &guid, 0,
			    BIT(USB_DSM_DISABLE_U1_U2_FOR_PORT))) {
		dev_dbg(&hdev->dev, "port-%d no _DSM function %d\n",
			port1, USB_DSM_DISABLE_U1_U2_FOR_PORT);
		return -ENODEV;
	}

	obj = acpi_evaluate_dsm_typed(port_handle, &guid, 0,
				      USB_DSM_DISABLE_U1_U2_FOR_PORT, NULL,
				      ACPI_TYPE_INTEGER);
	if (!obj) {
		dev_dbg(&hdev->dev, "evaluate port-%d _DSM failed\n", port1);
		return -EINVAL;
	}

	if (obj->integer.value == 0x01)
		ret = 1;

	ACPI_FREE(obj);

	return ret;
}
EXPORT_SYMBOL_GPL(usb_acpi_port_lpm_incapable);

/**
 * usb_acpi_intel_port_lpm_incapable - check if LPM should be disabled for a port.
 * @hdev: USB device belonging to the hub
 * @index: zero based port index
 *
 * Intel Type-C USB4 ports have retimers designed for USB4, which cause too
 * long latency for native USB 3 LPM U1 U2 usage.
 * Normally USB 3 LPM incapable ports are identified in ACPI by their _DSM
 * return value, but some of the platforms lack the _DSM method.
 * Check _UPC values to find USB4 Type-c ports with retimers in case _DSM is
 * missing
 *
 * Return 1 if USB3 port is LPM incapable, negative on error, otherwise 0
 */

int usb_acpi_intel_port_lpm_incapable(struct usb_device *hdev, int index)
{
	struct acpi_upc_info *upc;
	acpi_handle port_handle;
	int port1 = index + 1;
	acpi_status status;
	int ret;

	ret = usb_acpi_port_lpm_incapable(hdev, index);

	if (ret >= 0)
		return ret;
	/*
	 * No _DSM found above, check _UPC instead. USB4 capable type-c ports
	 * on Intel hosts don't support native USB 3 lpm.
	 */

	ret = 0;
	port_handle = usb_get_hub_port_acpi_handle(hdev, port1);
	if (!port_handle) {
		dev_dbg(&hdev->dev, "port-%d has no acpi handle\n", port1);
		return -ENODEV;
	}

	status = acpi_get_usb_port_capabilities(port_handle, &upc);
	if (ACPI_FAILURE(status))
		return -ENODEV;

	if ((upc->type == ACPI_UPC_TYPE_USBC_SS_SWITCH ||
	     upc->type == ACPI_UPC_TYPE_USBC_SS_NO_SWITCH) &&
	    ACPI_UPC_USBC_USB4(upc->usbc_capabilities))
		ret = 1;

	ACPI_FREE(upc);

	return ret;
}
EXPORT_SYMBOL_GPL(usb_acpi_intel_port_lpm_incapable);

/**
 * usb_acpi_set_power_state - control usb port's power via acpi power
 * resource
 * @hdev: USB device belonging to the usb hub
 * @index: port index based zero
 * @enable: power state expected to be set
 *
 * Notice to use usb_acpi_power_manageable() to check whether the usb port
 * has acpi power resource before invoking this function.
 *
 * Returns 0 on success, else negative errno.
 */
int usb_acpi_set_power_state(struct usb_device *hdev, int index, bool enable)
{
	struct usb_hub *hub = usb_hub_to_struct_hub(hdev);
	struct usb_port *port_dev;
	acpi_handle port_handle;
	unsigned char state;
	int port1 = index + 1;
	int error = -EINVAL;

	if (!hub)
		return -ENODEV;
	port_dev = hub->ports[port1 - 1];

	port_handle = (acpi_handle) usb_get_hub_port_acpi_handle(hdev, port1);
	if (!port_handle)
		return error;

	if (enable)
		state = ACPI_STATE_D0;
	else
		state = ACPI_STATE_D3_COLD;

	error = acpi_bus_set_power(port_handle, state);
	if (!error)
		dev_dbg(&port_dev->dev, "acpi: power was set to %d\n", enable);
	else
		dev_dbg(&port_dev->dev, "acpi: power failed to be set\n");

	return error;
}
EXPORT_SYMBOL_GPL(usb_acpi_set_power_state);

#define USB_ACPI_LOCATION_VALID (1 << 31)

static void
usb_acpi_get_connect_type(struct usb_port *port_dev, acpi_handle *handle)
{
	enum usb_port_connect_type connect_type = USB_PORT_CONNECT_TYPE_UNKNOWN;
	struct acpi_pld_info *pld;
	struct acpi_upc_info *upc;
	acpi_status status;

	/*
	 * According to 9.14 in ACPI Spec 6.2. _PLD indicates whether usb port
	 * is user visible and _UPC indicates whether it is connectable. If
	 * the port was visible and connectable, it could be freely connected
	 * and disconnected with USB devices. If no visible and connectable,
	 * a usb device is directly hard-wired to the port. If no visible and
	 * no connectable, the port would be not used.
	 */

	status = acpi_get_physical_device_location(handle, &pld);
	if (ACPI_FAILURE(status) || !pld)
		return;

	port_dev->location = USB_ACPI_LOCATION_VALID |
		pld->group_token << 8 | pld->group_position;

	status = acpi_get_usb_port_capabilities(handle, &upc);
	if (ACPI_FAILURE(status))
		goto out;

	if (upc->connectable)
		if (pld->user_visible)
			connect_type = USB_PORT_CONNECT_TYPE_HOT_PLUG;
		else
			connect_type = USB_PORT_CONNECT_TYPE_HARD_WIRED;
	else if (!pld->user_visible)
		connect_type = USB_PORT_NOT_USED;

	ACPI_FREE(upc);
out:
	port_dev->connect_type = connect_type;

	ACPI_FREE(pld);
}

/*
 * Private to usb-acpi, all the core needs to know is that
 * port_dev->location is non-zero when it has been set by the firmware.
 */

static struct acpi_device *
usb_acpi_find_companion_for_port(struct usb_port *port_dev)
{
	struct usb_device *udev;
	struct acpi_device *adev;
	acpi_handle *parent_handle;
	int port1;

	/* Get the struct usb_device point of port's hub */
	udev = to_usb_device(port_dev->dev.parent->parent);

	/*
	 * The root hub ports' parent is the root hub. The non-root-hub
	 * ports' parent is the parent hub port which the hub is
	 * connected to.
	 */
	if (!udev->parent) {
		adev = ACPI_COMPANION(&udev->dev);
		port1 = usb_hcd_find_raw_port_number(bus_to_hcd(udev->bus),
						     port_dev->portnum);
	} else {
		parent_handle = usb_get_hub_port_acpi_handle(udev->parent,
							     udev->portnum);
		if (!parent_handle)
			return NULL;

		adev = acpi_fetch_acpi_dev(parent_handle);
		port1 = port_dev->portnum;
	}

	return acpi_find_child_by_adr(adev, port1);
}

static struct acpi_device *
usb_acpi_find_companion_for_device(struct usb_device *udev)
{
	struct acpi_device *adev;
	struct usb_port *port_dev;
	struct usb_hub *hub;

	if (!udev->parent) {
		/*
		 * root hub is only child (_ADR=0) under its parent, the HC.
		 * sysdev pointer is the HC as seen from firmware.
		 */
		adev = ACPI_COMPANION(udev->bus->sysdev);
		return acpi_find_child_device(adev, 0, false);
	}

	hub = usb_hub_to_struct_hub(udev->parent);
	if (!hub)
		return NULL;

	/*
	 * This is an embedded USB device connected to a port and such
	 * devices share port's ACPI companion.
	 */
	port_dev = hub->ports[udev->portnum - 1];
	return usb_acpi_find_companion_for_port(port_dev);
}

static struct acpi_device *usb_acpi_find_companion(struct device *dev)
{
	struct acpi_device *adev;

	/*
	 * The USB hierarchy like following:
	 *
	 * Device (EHC1)
	 *	Device (HUBN)
	 *		Device (PR01)
	 *			Device (PR11)
	 *			Device (PR12)
	 *				Device (FN12)
	 *				Device (FN13)
	 *			Device (PR13)
	 *			...
	 * where HUBN is root hub, and PRNN are USB ports and devices
	 * connected to them, and FNNN are individualk functions for
	 * connected composite USB devices. PRNN and FNNN may contain
	 * _CRS and other methods describing sideband resources for
	 * the connected device.
	 *
	 * On the kernel side both root hub and embedded USB devices are
	 * represented as instances of usb_device structure, and ports
	 * are represented as usb_port structures, so the whole process
	 * is split into 2 parts: finding companions for devices and
	 * finding companions for ports.
	 *
	 * Note that we do not handle individual functions of composite
	 * devices yet, for that we would need to assign companions to
	 * devices corresponding to USB interfaces.
	 */
	if (is_usb_device(dev)) {
		return usb_acpi_find_companion_for_device(to_usb_device(dev));
	} else if (is_usb_port(dev)) {
		adev = usb_acpi_find_companion_for_port(to_usb_port(dev));
		if (adev)
			usb_acpi_get_connect_type(to_usb_port(dev), adev->handle);
		return adev;
	}
	return NULL;
}

static bool usb_acpi_bus_match(struct device *dev)
{
	return is_usb_device(dev) || is_usb_port(dev);
}

static struct acpi_bus_type usb_acpi_bus = {
	.name = "USB",
	.match = usb_acpi_bus_match,
	.find_companion = usb_acpi_find_companion,
};

int usb_acpi_register(void)
{
	return register_acpi_bus_type(&usb_acpi_bus);
}

void usb_acpi_unregister(void)
{
	unregister_acpi_bus_type(&usb_acpi_bus);
}
