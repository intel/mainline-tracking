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

struct intel_tsens_pid {
u32 hw_throt_mode_ccu0;
u32 hw_throt_mode_ccu1;
u32 thres_throt_cnt_cfg;
u32 therm_scaling_cnt;
u32 div_throt_cfg_ccu0;
u32 div_throt_cfg_ccu1;
u32 div_throt_cfg_ccu2;
u32 div_throt_cfg_ccu3;
u32 cg_throt_cfg1_ccu0;
u32 cg_throt_cfg1_ccu1;
u32 cg_throt_cfg1_ccu2;
u32 cg_throt_cfg1_ccu3;
u32 dts_thd_sar_pid_en_abort;
u32 dts_sar_mode_cfg;
u32 dts_pid_throt_cfg;
u32 dts_pid_ctrl_dts0;
u32 dts_pid_ctrl_dts1;
u32 dts_pid_ctrl_dts2;
u32 dts_pid_kp_dts0;
u32 dts_pid_kp_dts1;
u32 dts_pid_kp_dts2;
u32 dts_pid_kd_dts0;
u32 dts_pid_kd_dts1;
u32 dts_pid_kd_dts2;
u32 dts_pid_ki_dts0;
u32 dts_pid_ki_dts1;
u32 dts_pid_ki_dts2;
u32 dts_pid_rise_thres_dts0;
u32 dts_pid_rise_thres_dts1;
u32 dts_pid_rise_thres_dts2;
u32 dts_pid_fall_thres_dts0;
u32 dts_pid_fall_thres_dts1;
u32 dts_pid_fall_thres_dts2;
u32 dts_pid_max_integ_limit;
u32 dts_pid_min_integ_limit;
u32 dts_pid_max_accum_limit;
u32 dts_pid_min_accum_limit;
};

struct thunderbay_thermal_priv {
	const char *name;
	void __iomem *base_addr;
	spinlock_t lock;		/* Spinlock */
	u32 current_temp[THUNDERBAY_SENSOR_MAX];
	struct intel_tsens_plat_data *plat_data;
	struct device_node *s_node;
	struct intel_tsens_pid *pid_info;
};

static int thb_dt_parse;
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

static int intel_tsens_pid_config_reg(struct thunderbay_thermal_priv *priv)
{
		if (priv->pid_info->hw_throt_mode_ccu0 != 0)
			iowrite32(priv->pid_info->hw_throt_mode_ccu0,
				  priv->base_addr + HW_THROT_MODE_CCU0);

		if (priv->pid_info->hw_throt_mode_ccu1 != 0)
			iowrite32(priv->pid_info->hw_throt_mode_ccu1,
				  priv->base_addr + HW_THROT_MODE_CCU1);

		if (priv->pid_info->thres_throt_cnt_cfg != 0)
			iowrite32(priv->pid_info->thres_throt_cnt_cfg,
				  priv->base_addr + THRES_THROT_CNT_CFG);

		if (priv->pid_info->therm_scaling_cnt != 0)
			iowrite32(priv->pid_info->therm_scaling_cnt,
				  priv->base_addr + THERM_SCALING_CNT);

		if (priv->pid_info->div_throt_cfg_ccu0 != 0)
			iowrite32(priv->pid_info->div_throt_cfg_ccu0,
				  priv->base_addr + DIV_THROT_CFG_CCU0);

		if (priv->pid_info->div_throt_cfg_ccu1 != 0)
			iowrite32(priv->pid_info->div_throt_cfg_ccu1,
				  priv->base_addr + DIV_THROT_CFG_CCU1);

		if (priv->pid_info->div_throt_cfg_ccu2 != 0)
			iowrite32(priv->pid_info->div_throt_cfg_ccu2,
				  priv->base_addr + DIV_THROT_CFG_CCU2);

		if (priv->pid_info->div_throt_cfg_ccu3 != 0)
			iowrite32(priv->pid_info->div_throt_cfg_ccu3,
				  priv->base_addr + DIV_THROT_CFG_CCU3);

		if (priv->pid_info->cg_throt_cfg1_ccu0 != 0)
			iowrite32(priv->pid_info->cg_throt_cfg1_ccu0,
				  priv->base_addr + CG_THROT_CFG1_CCU0);

		if (priv->pid_info->cg_throt_cfg1_ccu1 != 0)
			iowrite32(priv->pid_info->cg_throt_cfg1_ccu1,
				  priv->base_addr + CG_THROT_CFG1_CCU1);

		if (priv->pid_info->cg_throt_cfg1_ccu2 != 0)
			iowrite32(priv->pid_info->cg_throt_cfg1_ccu2,
				  priv->base_addr + CG_THROT_CFG1_CCU2);

		if (priv->pid_info->cg_throt_cfg1_ccu3 != 0)
			iowrite32(priv->pid_info->cg_throt_cfg1_ccu3,
				  priv->base_addr + CG_THROT_CFG1_CCU3);

		if (priv->pid_info->dts_thd_sar_pid_en_abort != 0)
			iowrite32(priv->pid_info->dts_thd_sar_pid_en_abort,
				  priv->base_addr + DTS_THD_SAR_PID_EN_ABORT);

		if (priv->pid_info->dts_sar_mode_cfg != 0)
			iowrite32(priv->pid_info->dts_sar_mode_cfg,
				  priv->base_addr + DTS_SAR_MODE_CFG);

		if (priv->pid_info->dts_pid_throt_cfg != 0)
			iowrite32(priv->pid_info->dts_pid_throt_cfg,
				  priv->base_addr + DTS_PID_THROT_CFG);

		if (priv->pid_info->dts_pid_ctrl_dts0 != 0)
			iowrite32(priv->pid_info->dts_pid_ctrl_dts0,
				  priv->base_addr + DTS_PID_CTRL_DTS0);

		if (priv->pid_info->dts_pid_ctrl_dts1 != 0)
			iowrite32(priv->pid_info->dts_pid_ctrl_dts1,
				  priv->base_addr + DTS_PID_CTRL_DTS1);

		if (priv->pid_info->dts_pid_ctrl_dts2 != 0)
			iowrite32(priv->pid_info->dts_pid_ctrl_dts2,
				  priv->base_addr + DTS_PID_CTRL_DTS2);

		if (priv->pid_info->dts_pid_kp_dts0 != 0)
			iowrite32(priv->pid_info->dts_pid_kp_dts0,
				  priv->base_addr + DTS_PID_KP_DTS0);

		if (priv->pid_info->dts_pid_kp_dts1 != 0)
			iowrite32(priv->pid_info->dts_pid_kp_dts1,
				  priv->base_addr + DTS_PID_KP_DTS1);

		if (priv->pid_info->dts_pid_kp_dts2 != 0)
			iowrite32(priv->pid_info->dts_pid_kp_dts2,
				  priv->base_addr + DTS_PID_KP_DTS2);

		if (priv->pid_info->dts_pid_kd_dts0 != 0)
			iowrite32(priv->pid_info->dts_pid_kd_dts0,
				  priv->base_addr + DTS_PID_KD_DTS0);

		if (priv->pid_info->dts_pid_kd_dts1 != 0)
			iowrite32(priv->pid_info->dts_pid_kd_dts1,
				  priv->base_addr + DTS_PID_KD_DTS1);

		if (priv->pid_info->dts_pid_kd_dts2 != 0)
			iowrite32(priv->pid_info->dts_pid_kd_dts2,
				  priv->base_addr + DTS_PID_KD_DTS2);

		if (priv->pid_info->dts_pid_ki_dts0 != 0)
			iowrite32(priv->pid_info->dts_pid_ki_dts0,
				  priv->base_addr + DTS_PID_KI_DTS0);

		if (priv->pid_info->dts_pid_ki_dts1 != 0)
			iowrite32(priv->pid_info->dts_pid_ki_dts1,
				  priv->base_addr + DTS_PID_KI_DTS1);

		if (priv->pid_info->dts_pid_ki_dts2 != 0)
			iowrite32(priv->pid_info->dts_pid_ki_dts2,
				  priv->base_addr + DTS_PID_KI_DTS2);

		if (priv->pid_info->dts_pid_rise_thres_dts0 != 0)
			iowrite32(priv->pid_info->dts_pid_rise_thres_dts0,
				  priv->base_addr + DTS_PID_RISE_THRES_DTS0);

		if (priv->pid_info->dts_pid_rise_thres_dts1 != 0)
			iowrite32(priv->pid_info->dts_pid_rise_thres_dts1,
				  priv->base_addr + DTS_PID_RISE_THRES_DTS1);

		if (priv->pid_info->dts_pid_rise_thres_dts2 != 0)
			iowrite32(priv->pid_info->dts_pid_rise_thres_dts2,
				  priv->base_addr + DTS_PID_RISE_THRES_DTS2);

		if (priv->pid_info->dts_pid_fall_thres_dts0 != 0)
			iowrite32(priv->pid_info->dts_pid_fall_thres_dts0,
				  priv->base_addr + DTS_PID_FALL_THRES_DTS0);

		if (priv->pid_info->dts_pid_fall_thres_dts1 != 0)
			iowrite32(priv->pid_info->dts_pid_fall_thres_dts1,
				  priv->base_addr + DTS_PID_FALL_THRES_DTS1);

		if (priv->pid_info->dts_pid_fall_thres_dts2 != 0)
			iowrite32(priv->pid_info->dts_pid_fall_thres_dts2,
				  priv->base_addr + DTS_PID_FALL_THRES_DTS2);

		if (priv->pid_info->dts_pid_max_integ_limit != 0)
			iowrite32(priv->pid_info->dts_pid_max_integ_limit,
				  priv->base_addr + DTS_PID_MAX_INTEG_LIMIT);

		if (priv->pid_info->dts_pid_min_integ_limit != 0)
			iowrite32(priv->pid_info->dts_pid_min_integ_limit,
				  priv->base_addr + DTS_PID_MIN_INTEG_LIMIT);

		if (priv->pid_info->dts_pid_max_accum_limit != 0)
			iowrite32(priv->pid_info->dts_pid_max_accum_limit,
				  priv->base_addr + DTS_PID_MAX_ACCUM_LIMIT);

		if (priv->pid_info->dts_pid_min_accum_limit != 0)
			iowrite32(priv->pid_info->dts_pid_min_accum_limit,
				  priv->base_addr + DTS_PID_MIN_ACCUM_LIMIT);
return 0;
}

int intel_tsens_pid_config_dt(struct thunderbay_thermal_priv *priv)
{
	struct device_node *t_node = priv->s_node;
	struct device_node *np, *s_node;

	for_each_child_of_node(t_node, s_node) {
		int ret;
		struct intel_tsens_pid pid = {0};

		priv->pid_info = &pid;
		np = of_parse_phandle(s_node, "pid_cfg", 0);
		if (!np)
			return NULL;

		ret = of_property_read_u32(np, "hw_throt_mode_ccu0",
					   &priv->pid_info->hw_throt_mode_ccu0);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "hw_throt_mode_ccu1",
					   &priv->pid_info->hw_throt_mode_ccu1);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "thres_throt_cnt_cfg",
					   &priv->pid_info->thres_throt_cnt_cfg);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "therm_scaling_cnt",
					   &priv->pid_info->therm_scaling_cnt);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "div_throt_cfg_ccu0",
					   &priv->pid_info->div_throt_cfg_ccu0);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "div_throt_cfg_ccu1",
					   &priv->pid_info->div_throt_cfg_ccu1);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "div_throt_cfg_ccu2",
					   &priv->pid_info->div_throt_cfg_ccu2);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "div_throt_cfg_ccu3",
					   &priv->pid_info->div_throt_cfg_ccu3);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "cg_throt_cfg1_ccu0",
					   &priv->pid_info->cg_throt_cfg1_ccu0);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "cg_throt_cfg1_ccu1",
					   &priv->pid_info->cg_throt_cfg1_ccu1);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "cg_throt_cfg1_ccu2",
					   &priv->pid_info->cg_throt_cfg1_ccu2);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "cg_throt_cfg1_ccu3",
					   &priv->pid_info->cg_throt_cfg1_ccu3);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_thd_sar_pid_en_abort",
					   &priv->pid_info->dts_thd_sar_pid_en_abort);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_sar_mode_cfg",
					   &priv->pid_info->dts_sar_mode_cfg);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_throt_cfg",
					   &priv->pid_info->dts_pid_throt_cfg);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_ctrl_dts0",
					   &priv->pid_info->dts_pid_ctrl_dts0);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_ctrl_dts1",
					   &priv->pid_info->dts_pid_ctrl_dts1);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_ctrl_dts2",
					   &priv->pid_info->dts_pid_ctrl_dts2);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_kp_dts0",
					   &priv->pid_info->dts_pid_kp_dts0);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_kp_dts1",
					   &priv->pid_info->dts_pid_kp_dts1);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_kp_dts2",
					   &priv->pid_info->dts_pid_kp_dts2);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_kd_dts0",
					   &priv->pid_info->dts_pid_kd_dts0);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_kd_dts1",
					   &priv->pid_info->dts_pid_kd_dts1);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_kd_dts2",
					   &priv->pid_info->dts_pid_kd_dts2);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_ki_dts0",
					   &priv->pid_info->dts_pid_ki_dts0);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_ki_dts1",
					   &priv->pid_info->dts_pid_ki_dts1);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_ki_dts2",
					   &priv->pid_info->dts_pid_ki_dts2);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_rise_thres_dts0",
					   &priv->pid_info->dts_pid_rise_thres_dts0);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_rise_thres_dts1",
					   &priv->pid_info->dts_pid_rise_thres_dts1);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_rise_thres_dts2",
					   &priv->pid_info->dts_pid_rise_thres_dts2);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_fall_thres_dts0",
					   &priv->pid_info->dts_pid_fall_thres_dts0);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_fall_thres_dts1",
					   &priv->pid_info->dts_pid_fall_thres_dts1);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_fall_thres_dts2",
					   &priv->pid_info->dts_pid_fall_thres_dts2);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_max_integ_limit",
					   &priv->pid_info->dts_pid_max_integ_limit);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_min_integ_limit",
					   &priv->pid_info->dts_pid_min_integ_limit);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_max_accum_limit",
					   &priv->pid_info->dts_pid_max_accum_limit);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = of_property_read_u32(np, "dts_pid_min_accum_limit",
					   &priv->pid_info->dts_pid_min_accum_limit);
		if (ret && ret != -EINVAL)
			goto invalid;

		ret = intel_tsens_pid_config_reg(priv);
		if (ret)
			goto invalid;

invalid:
	return ret;
	}
return 0;
}
EXPORT_SYMBOL_GPL(intel_tsens_pid_config_dt);

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
	//iowrite32(0x1c0, plat_data->base_addr+DTS_THD_SAR_PID_EN_ABORT);
	priv->name = plat_data->name;
	priv->base_addr = plat_data->base_addr;
	priv->plat_data = plat_data;
	priv->s_node = plat_data->s_node;
	if (priv->s_node != 0 && thb_dt_parse < 1) {
		if (intel_tsens_pid_config_dt(priv))
			dev_info(&pdev->dev, "PID dt_parsing failed");
		thb_dt_parse++;
	}
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
