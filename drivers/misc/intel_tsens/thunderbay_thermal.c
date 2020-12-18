// SPDX-License-Identifier: GPL-2.0-only
/*
 * Thunderbay-thermal.c - ThunderBay Thermal Driver.
 *
 * Copyright (C) 2019-2020 Intel Corporation
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/hddl_device.h>
#include "intel_tsens_thermal.h"
#include "thunderbay_tsens.h"

struct thunderbay_thermal_priv {
	const char *name;
	void __iomem *base_addr;
	spinlock_t lock;		/* Spinlock */
	u32 current_temp[THUNDERBAY_SENSOR_MAX];
	struct intel_tsens_plat_data *plat_data;
};

static int thb_sensor_read_temp(void __iomem *regs_val,
				int offset,
				int *temp)
{
	int reg_val, thb_raw_index;

	reg_val = ioread32(regs_val + offset);
	reg_val = (reg_val & 0xff);
	thb_raw_index = reg_val - THUNDERBAY_SENSOR_BASE_TEMP;
	if (thb_raw_index < 0)
		reg_val = raw_thb[0];
	else if	(thb_raw_index > (raw_thb_size - 1))
		reg_val = raw_thb[raw_thb_size - 1];
	else
		reg_val = raw_thb[thb_raw_index];

	*temp = reg_val;

	return 0;
}

static int thunderbay_get_temp(struct platform_device *pdev, int type, int *temp)
{
	struct thunderbay_thermal_priv *priv = platform_get_drvdata(pdev);

	spin_lock(&priv->lock);
	switch (type) {
	case CPUSS_SOUTH_NOC:
		thb_sensor_read_temp(priv->base_addr, SAR_TEMP_DTS0_OFFSET, temp);
		priv->current_temp[CPUSS_SOUTH_NOC] = *temp;
		break;

	case CPUSS_NORTH_NOC:
		thb_sensor_read_temp(priv->base_addr, SAR_TEMP_DTS1_OFFSET, temp);
		priv->current_temp[CPUSS_NORTH_NOC] = *temp;
		break;

	case PAR_VPU_0:
		thb_sensor_read_temp(priv->base_addr, SAR_TEMP_DTS0_OFFSET, temp);
		priv->current_temp[PAR_VPU_0] = *temp;
		break;

	case PAR_VPU_1:
		thb_sensor_read_temp(priv->base_addr, SAR_TEMP_DTS0_OFFSET, temp);
		priv->current_temp[PAR_VPU_1] = *temp;
		break;

	case PAR_VPU_2:
		thb_sensor_read_temp(priv->base_addr, SAR_TEMP_DTS0_OFFSET, temp);
		priv->current_temp[PAR_VPU_2] = *temp;
		break;

	case PAR_VPU_3:
		thb_sensor_read_temp(priv->base_addr, SAR_TEMP_DTS0_OFFSET, temp);
		priv->current_temp[PAR_VPU_3] = *temp;
		break;

	case PAR_MEDIA_0:
		thb_sensor_read_temp(priv->base_addr, SAR_TEMP_DTS1_OFFSET, temp);
		priv->current_temp[PAR_MEDIA_0] = *temp;
		break;

	case PAR_MEDIA_1:
		thb_sensor_read_temp(priv->base_addr, SAR_TEMP_DTS1_OFFSET, temp);
		priv->current_temp[PAR_MEDIA_1] = *temp;
		break;

	case PAR_MEDIA_2:
		thb_sensor_read_temp(priv->base_addr, SAR_TEMP_DTS1_OFFSET, temp);
		priv->current_temp[PAR_MEDIA_2] = *temp;
		break;

	case PAR_MEDIA_3:
		thb_sensor_read_temp(priv->base_addr, SAR_TEMP_DTS1_OFFSET, temp);
		priv->current_temp[PAR_MEDIA_3] = *temp;
		break;

	case NOC_VPU_DDR_0:
		thb_sensor_read_temp(priv->base_addr, SAR_TEMP_DTS2_OFFSET, temp);
		priv->current_temp[NOC_VPU_DDR_0] = *temp;
		break;

	case NOC_VPU_DDR_1:
		thb_sensor_read_temp(priv->base_addr, SAR_TEMP_DTS2_OFFSET, temp);
		priv->current_temp[NOC_VPU_DDR_1] = *temp;
		break;

	case NOC_VPU_DDR_2:
		thb_sensor_read_temp(priv->base_addr, SAR_TEMP_DTS2_OFFSET, temp);
		priv->current_temp[NOC_VPU_DDR_2] = *temp;
		break;

	case NOC_VPU_DDR_3:
		thb_sensor_read_temp(priv->base_addr, SAR_TEMP_DTS2_OFFSET, temp);
		priv->current_temp[NOC_VPU_DDR_3] = *temp;
		break;

	default:
		break;
	}
	spin_unlock(&priv->lock);
	return 0;
}

static int thunderbay_thermal_probe(struct platform_device *pdev)
{
	struct intel_tsens_plat_data *plat_data = NULL;
	struct thunderbay_thermal_priv *priv;

	plat_data = pdev->dev.platform_data;
	if (!plat_data) {
		dev_err(&pdev->dev, "Platform data not found\n");
		return -EINVAL;
	}
	if (!plat_data->base_addr)
		return -EINVAL;
	priv = devm_kzalloc(&pdev->dev, sizeof(struct thunderbay_thermal_priv),	GFP_KERNEL);
	if (!priv) {
		dev_err(&pdev->dev, "No memory");
		return -ENOMEM;
	}
	iowrite32(0x1c0, plat_data->base_addr + DTS_THD_SAR_PID_EN_ABORT);
	priv->name = plat_data->name;
	priv->base_addr = plat_data->base_addr;
	priv->plat_data = plat_data;
	plat_data->get_temp = thunderbay_get_temp;
	spin_lock_init(&priv->lock);
	platform_set_drvdata(pdev, priv);

	dev_info(&pdev->dev, "Thermal driver loaded for %s\n", plat_data->name);
	return 0;
}

static int thunderbay_thermal_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver thunderbay_thermal_driver = {
	.probe = thunderbay_thermal_probe,
	.remove = thunderbay_thermal_remove,
	.driver = {
		.name = "intel,thunderbay_thermal",
	},
};

module_platform_driver(thunderbay_thermal_driver);

MODULE_DESCRIPTION("Thunderbay Thermal Driver");
MODULE_AUTHOR("Sandeep Singh <sandeep1.singh@intel.com>");
MODULE_AUTHOR("Raja Subramanian, Lakshmi Bai <lakshmi.bai.raja.subramanian@intel.com>");
MODULE_AUTHOR("Udhayakumar C <udhayakumar.c@intel.com>");
MODULE_LICENSE("GPL v2");
