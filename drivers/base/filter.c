// SPDX-License-Identifier: GPL-2.0
/*
 * filter.c - Add device filter framework.
 *
 * Implements APIs required for registering platform specific
 * device filter.
 *
 * Copyright (c) 2020 Intel Corporation
 *
 * Author: Kuppuswamy Sathyanarayanan <sathyanarayanan.kuppuswamy@linux.intel.com>
 */
#include <linux/init.h>
#include <linux/device/filter.h>
#include <linux/acpi.h>
#include <linux/protected_guest.h>

#include "base.h"

/* List of filter allow list */
static LIST_HEAD(device_filter_list);

/* Protects device_filter_list add/read operations*/
static DEFINE_SPINLOCK(device_filter_lock);

/*
 * Compares the device name with given filter data allow list.
 *
 * If the BUS driver does not register bus specific filter hook
 * default_device_filter() will be used.
 *
 * Return true if device name matches with allow list.
 */
static bool default_device_filter(struct device *dev,
				  struct dev_filter_data *data)
{
	int i;
	char **allow_list;
	const char *name = dev_name(dev);

	if (!name || !data || !data->allow_list)
		return false;

	allow_list = data->allow_list;

	for (i = 0; i < data->len; i++)
		if (!strncmp(allow_list[i], name, strlen(allow_list[i])))
			return true;

	return false;
}

/*
 * is_device_allowed() - Allow or block device based on filter
 *			 list registered by platform.
 *
 * This filter is currently only enabled for protected guests.
 *
 * Return true to allow given device or false to block it.
 */
bool is_device_allowed(struct device *dev)
{
	struct dev_filter_node *node;
	bool status = false, filter_attempted = false;

	/*
	 * Make sure device filter support is enabled, if not
	 * just allow the device and return.
	 */
	if (!prot_guest_has(PR_GUEST_DEVICE_FILTER))
		return true;

	spin_lock(&device_filter_lock);

	/*
	 * If the device is not part of standard bus, skip rest
	 * of the checks and just allow it.
	 */
	if (!dev->bus) {
		status = true;
		goto done;
	}

	/* If platform did not register any allow list, allow all */
	if (list_empty(&device_filter_list)) {
		status = true;
		goto done;
	}

	list_for_each_entry(node, &device_filter_list, list) {
		/*
		 * For every matching filter node, check with device
		 * filter hook and return true if allowed.
		 */
		if (!strcmp(dev->bus->name, node->name)) {
			filter_attempted = true;
			/* If custom bus specific filter exist, use it*/
			if (dev->bus->device_filter) {
				if (dev->bus->device_filter(dev, node->data)) {
					status = true;
					goto done;
				}
			} else {
				if (default_device_filter(dev, node->data)) {
					status = true;
					goto done;
				}
			}
		}
	}

	/*
	 * If platform did not register allow list for given bus
	 * allow all devices.
	 */
	if (!filter_attempted)
		status = true;

done:
	pr_filter_dbg("bus:%s device:%s %s\n",
		      dev->bus ? dev->bus->name : "null",
		      dev_name(dev),
		      status ? "allowed" : "blocked");

	spin_unlock(&device_filter_lock);
	return status;
}

void register_dev_filter(struct dev_filter_node *node)
{
	spin_lock(&device_filter_lock);
	list_add_tail(&node->list, &device_filter_list);
	spin_unlock(&device_filter_lock);
}
