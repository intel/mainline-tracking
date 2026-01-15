/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2016-2025 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef MEDIA_INTEL_IPU_GET_ACPI_H
#define MEDIA_INTEL_IPU_GET_ACPI_H

int ipu_get_acpi_devices(void *driver_data,
				struct device *dev,
				void **spdata,
				void **built_in_pdata,
				int (*fn)
				(struct device *, void *,
				 void *csi2,
				 bool reprobe));

int ipu_get_acpi_devices_new(void **spdata);

#endif /* MEDIA_INTEL_IPU_GET_ACPI_H */
