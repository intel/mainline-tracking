/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel tsens I2C thermal Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#ifndef _LINUX_INTEL_TSENS_H
#define _LINUX_INTEL_TSENS_H

#include <linux/thermal.h>

struct intel_tsens_plat_data {
	const char *name;
	void __iomem *base_addr;
	int (*get_temp)(struct platform_device *pdev, int type, int *temp);
	void *pdata;
};

__packed __aligned(4)) struct intel_tsens_trip_info {
	enum thermal_trip_type trip_type;
	int temp;
};

struct intel_tsens_plat_info {
	const char *plat_name;
	struct platform_device *pdev;
	void __iomem *base_addr;
};

struct intel_tsens **intel_tsens_hddl_register(int *nsens);

#endif /* _LINUX_INTEL_TSENS_H */
