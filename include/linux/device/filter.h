/* SPDX-License-Identifier: GPL-2.0 */
/*
 * filter.h - Device filter specific header
 *
 * Copyright (c) 2020 Intel Corporation
 *
 * Author: Kuppuswamy Sathyanarayanan <sathyanarayanan.kuppuswamy@linux.intel.com>
 */

#ifndef _DEVICE_FILTER_H_
#define _DEVICE_FILTER_H_

#include <linux/device/bus.h>
#include <linux/device/driver.h>
#include <linux/device.h>

#define FILTER_TAG	"filter: "

#define pr_filter_dbg(fmt, ...) \
	pr_debug(FILTER_TAG fmt, ##__VA_ARGS__)
#define pr_filter_info(fmt, ...) \
	pr_info(FILTER_TAG fmt, ##__VA_ARGS__)
#define pr_filter_crit(fmt, ...) \
	pr_crit(FILTER_TAG fmt, ##__VA_ARGS__)

enum filter_policy {
	BLOCK_ALL = 0, /* Block all devices */
	ALLOW_ALL = 1  /* Allow all devices */
};

/* Filter data type for filter types > 0 */
struct dev_filter_data {
	char **allow_list;
	unsigned int len; /* Length of allow list */
};

/**
 * struct dev_filter_node - device filter node
 *
 * @name		: Name of the filter.
 * @data		: Allow list of filter.
 * @default_status	: Default status if allow list is empty.
 */
struct dev_filter_node {
	char *name;
	void *data;
	bool default_status;
	struct list_head list;
};

/* Register platform specific filter allow list */
void register_dev_filter(struct dev_filter_node *node);
#endif
