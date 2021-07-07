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
#include <linux/interrupt.h>
#include <linux/debugfs.h> /* this is for DebugFS libraries */

struct intel_tsens_pid {
u32 hw_throt_mode_ccu0;
u32 hw_throt_mode_ccu1;
u32 hw_throt_mode_ccu2;
u32 hw_throt_mode_ccu3;
u32 thres_mask_dts0;
u32 thres_mask_dts1;
u32 thres_mask_dts2;
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
	void __iomem *gpio_addr;
	spinlock_t lock;		/* Spinlock */
	u32 current_temp[THUNDERBAY_SENSOR_MAX];
	struct intel_tsens_plat_data *plat_data;
	struct device_node *s_node;
	struct intel_tsens_pid *pid_info;
	bool irq_available;
	int calib_off;
	int board_type;
	struct platform_device *pdev;
};

static const mode_t THERMAL_MODE = 0600;

struct thb_thermal {
	char *name;
	int offset;
	void __iomem *base_addr;
};

static int thb_dt_parse;

static int thb_sensor_read_temp(void __iomem *regs_val,
				int fuse_val,
				int board_type,
				int offset,
				int *temp)
{
	int reg_val, thb_raw_index;
	int T0 = 273, T1 = 378, C0, C1,  A, B, Const = 124;
	reg_val = ioread32(regs_val + offset);
	reg_val = (reg_val & 0xff);
	if ( board_type == 11 && fuse_val !=0) {
	/* new formula  for EVT2 board*/
	C0 = (fuse_val & 0xff);
	C1 = (fuse_val >> 8);
	A = ((Const)*(T0-T1)*1000000)/((C1-C0)*T0*T1);
	B = ((Const*10000)-(A*T0*C0))/273;
	*temp = ((((Const*10000)/((A*reg_val)+B)) - 273)*1000);
	/* new-formula end */
	} else {
	thb_raw_index = reg_val - THUNDERBAY_SENSOR_BASE_TEMP;
	if (thb_raw_index < 0)
		reg_val = raw_thb[0];
	else if	(thb_raw_index > (raw_thb_size - 1))
		reg_val = raw_thb[raw_thb_size - 1];
	else
		reg_val = raw_thb[thb_raw_index];

	*temp = reg_val;
	}
	return 0;
}

static int thunderbay_get_temp(struct platform_device *pdev, int type, int *temp)
{
	struct thunderbay_thermal_priv *priv = platform_get_drvdata(pdev);

	spin_lock(&priv->lock);
	switch (type) {
	case CPUSS_SOUTH_NOC:
		thb_sensor_read_temp(priv->base_addr,
				priv->calib_off,
				priv->board_type,
				SAR_TEMP_DTS0_OFFSET, temp);
		priv->current_temp[CPUSS_SOUTH_NOC] = *temp;
		break;

	case CPUSS_NORTH_NOC:
		thb_sensor_read_temp(priv->base_addr,
				priv->calib_off,
				priv->board_type,
				SAR_TEMP_DTS1_OFFSET, temp);
		priv->current_temp[CPUSS_NORTH_NOC] = *temp;
		break;

	case PAR_VPU_0:
		thb_sensor_read_temp(priv->base_addr,
				priv->calib_off,
				priv->board_type,
				SAR_TEMP_DTS0_OFFSET, temp);
		priv->current_temp[PAR_VPU_0] = *temp;
		break;

	case PAR_VPU_1:
		thb_sensor_read_temp(priv->base_addr,
				priv->calib_off,
				priv->board_type,
				SAR_TEMP_DTS0_OFFSET, temp);
		priv->current_temp[PAR_VPU_1] = *temp;
		break;

	case PAR_VPU_2:
		thb_sensor_read_temp(priv->base_addr,
				priv->calib_off,
				priv->board_type,
				SAR_TEMP_DTS0_OFFSET, temp);
		priv->current_temp[PAR_VPU_2] = *temp;
		break;

	case PAR_VPU_3:
		thb_sensor_read_temp(priv->base_addr,
				priv->calib_off,
				priv->board_type,
				SAR_TEMP_DTS0_OFFSET, temp);
		priv->current_temp[PAR_VPU_3] = *temp;
		break;

	case PAR_MEDIA_0:
		thb_sensor_read_temp(priv->base_addr,
				priv->calib_off,
				priv->board_type,
				SAR_TEMP_DTS1_OFFSET, temp);
		priv->current_temp[PAR_MEDIA_0] = *temp;
		break;

	case PAR_MEDIA_1:
		thb_sensor_read_temp(priv->base_addr,
				priv->calib_off,
				priv->board_type,
				SAR_TEMP_DTS1_OFFSET, temp);
		priv->current_temp[PAR_MEDIA_1] = *temp;
		break;

	case PAR_MEDIA_2:
		thb_sensor_read_temp(priv->base_addr,
				priv->calib_off,
				priv->board_type,
				SAR_TEMP_DTS1_OFFSET, temp);
		priv->current_temp[PAR_MEDIA_2] = *temp;
		break;

	case PAR_MEDIA_3:
		thb_sensor_read_temp(priv->base_addr,
				priv->calib_off,
				priv->board_type,
				SAR_TEMP_DTS1_OFFSET, temp);
		priv->current_temp[PAR_MEDIA_3] = *temp;
		break;

	case NOC_VPU_DDR_0:
		thb_sensor_read_temp(priv->base_addr,
				priv->calib_off,
				priv->board_type,
				SAR_TEMP_DTS2_OFFSET, temp);
		priv->current_temp[NOC_VPU_DDR_0] = *temp;
		break;

	case NOC_VPU_DDR_1:
		thb_sensor_read_temp(priv->base_addr,
				priv->calib_off,
				priv->board_type,
				SAR_TEMP_DTS2_OFFSET, temp);
		priv->current_temp[NOC_VPU_DDR_1] = *temp;
		break;

	case NOC_VPU_DDR_2:
		thb_sensor_read_temp(priv->base_addr,
				priv->calib_off,
				priv->board_type,
				SAR_TEMP_DTS2_OFFSET, temp);
		priv->current_temp[NOC_VPU_DDR_2] = *temp;
		break;

	case NOC_VPU_DDR_3:
		thb_sensor_read_temp(priv->base_addr,
				priv->calib_off,
				priv->board_type,
				SAR_TEMP_DTS2_OFFSET, temp);
		priv->current_temp[NOC_VPU_DDR_3] = *temp;
		break;

	default:
		break;
	}
	spin_unlock(&priv->lock);
	return 0;
}

//static irqreturn_t irq_handler(int irq, void *args)
//{
//	struct thunderbay_thermal_priv *priv = args;
//	u32 reg_val;
//
//	reg_val = ioread32(priv->base_addr + SW_THERM_INTR_STAT);
//	iowrite32(reg_val, priv->base_addr + SW_THERM_INTR_STAT);
//
//return IRQ_HANDLED;
//}

static ssize_t offset_read_op(struct file *file, char __user *buf,
			      size_t count, loff_t *ppos)
{
	struct thb_thermal *thb_t = file->private_data;
	int output;
	char *data;
	ssize_t ret;

	data = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	output = ioread32(thb_t->base_addr + thb_t->offset);
	ret = snprintf(data, PAGE_SIZE, "Register_base_address : %p\n"
		       "Register_offset : %x\n"
		       "Register_Data : %x\n",
		       thb_t->base_addr,
		       thb_t->offset,
		       output);

	ret = simple_read_from_buffer(buf, count, ppos, data, ret);
	kfree(data);
return ret;
}

static ssize_t offset_write_op(struct file *file, const char __user *buf,
			       size_t count, loff_t *ppos)
{
	struct thb_thermal *thb_t = file->private_data;
	char *data;
	int ret, val, offset;
	data = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	ret = simple_write_to_buffer(data, 32, ppos, buf, count);
	//ret = kstrtou32(data, 0, &output);
	ret = sscanf(data, "%x %x", &val, &offset);
	if (ret > 2) {
		kfree(data);
		return -EINVAL;
	}
	iowrite32(val, thb_t->base_addr + thb_t->offset);
	kfree(data);
return ret;
}

static const struct file_operations thermal_offset_fops = {
	.open           = simple_open,
	.read		= offset_read_op,
	.write		= offset_write_op,
};

static int thermal_throttle_debugfs(struct platform_device *pdev,
				    struct dentry *thermal_debug,
				    struct thb_thermal *thb_t)
{
	struct thb_thermal *thb_debug = NULL;
	struct dentry *thermal_debug_value;

	thb_debug = devm_kzalloc(&pdev->dev,
				 sizeof(struct thb_thermal),
				 GFP_KERNEL);
	if (!thb_debug) {
		dev_err(&pdev->dev, "No memory");
		return -ENOMEM;
	}

	thb_debug->base_addr = thb_t->base_addr;
	thb_debug->name = thb_t->name;
	thb_debug->offset = thb_t->offset;

	thermal_debug_value = debugfs_create_file(thb_t->name,
						  THERMAL_MODE,
						  thermal_debug,
						  thb_debug,
						  &thermal_offset_fops);
	if (!thermal_debug_value) {
		pr_info(" failed to create file\n");
		return -1;
	}
return 0;
}

static int intel_tbh_thermal_config(struct thunderbay_thermal_priv *priv, int type)
{
	struct platform_device *pdev = priv->pdev;
	int ret, t_reg;
	//int irq_line;
	static struct dentry *dir;
	struct dentry *thermal_debug;
	struct thb_thermal *thb_t;
	static int common_dir;
	struct device_node *np = pdev->dev.of_node;

	thb_t = devm_kzalloc(&pdev->dev,
			     sizeof(struct thb_thermal),
			     GFP_KERNEL);
	if (!thb_t) {
		dev_err(&pdev->dev, "No memory");
		return -ENOMEM;
	}
	ret = of_property_read_u32(np, "calib_off",
				   &priv->calib_off);
	if (ret && ret != -EINVAL)
		pr_info("calib_off failed \n");
	pr_info("hddl: calibration value for %s is %x",
				   np->name,  priv->calib_off);
	ret = of_property_read_u32(np, "board_type",
				   &priv->board_type);
	if (ret && ret != -EINVAL)
		pr_info("board_id failed \n");
	pr_info("hddl_thb: board_type value for %s is %x",
		np->name, priv->board_type);

	if (common_dir == 0) {
		dir = debugfs_create_dir("thb_thermal", 0);
		if (!dir) {
			pr_info("debugfs_dir: failed to create /sys/kernel/debug/thb_thermal\n");
			return -1;
		}
	common_dir++;
 	}
		switch (type) {
		case CPUSS_SOUTH_NOC:
			//irq_line = platform_get_irq_byname(pdev, "cpuss");
			//if (irq_line > 0) {
				//ret = request_irq(irq_line, irq_handler,
				//		  0, "Thermal_A53SS_IRQ",
				//		  priv);
				//if (ret) {
				//	pr_info("%d: request_irq failed, errno=%d\n",
				//		irq_line, -ret);
				//	free_irq(irq_line, NULL);
				//}
			//}
			// PID CONFIG
			iowrite32(DTS_THD_SAR_PID_EN_ABORT_VALUE,
				  priv->base_addr + DTS_THD_SAR_PID_EN_ABORT);
			iowrite32(DTS_PID_RISE_THRES_DTS0_VALUE,
				  priv->base_addr + DTS_PID_RISE_THRES_DTS0);
			iowrite32(DTS_PID_RISE_THRES_DTS1_VALUE,
				  priv->base_addr + DTS_PID_RISE_THRES_DTS1);
			iowrite32(DTS_PID_FALL_THRES_DTS0_VALUE,
				  priv->base_addr + DTS_PID_FALL_THRES_DTS0);
			iowrite32(DTS_PID_FALL_THRES_DTS1_VALUE,
				  priv->base_addr + DTS_PID_FALL_THRES_DTS1);
			iowrite32(DTS_PID_KP_DTS0_VALUE,
				  priv->base_addr + DTS_PID_KP_DTS0);
			iowrite32(DTS_PID_KP_DTS1_VALUE,
				  priv->base_addr + DTS_PID_KP_DTS1);
			iowrite32(DTS_PID_KD_DTS0_VALUE,
				  priv->base_addr + DTS_PID_KD_DTS0);
			iowrite32(DTS_PID_KD_DTS1_VALUE,
				  priv->base_addr + DTS_PID_KD_DTS1);
			iowrite32(DTS_PID_KI_DTS0_VALUE,
				  priv->base_addr + DTS_PID_KI_DTS0);
			iowrite32(DTS_PID_KI_DTS1_VALUE,
				  priv->base_addr + DTS_PID_KI_DTS1);
			iowrite32(DTS_PID_CTRL_DTS0_VALUE,
				  priv->base_addr + DTS_PID_CTRL_DTS0);
			iowrite32(DTS_PID_CTRL_DTS1_VALUE,
				  priv->base_addr + DTS_PID_CTRL_DTS1);
			iowrite32(DIV_THROT_CFG_CCU0_VALUE,
				  priv->base_addr + DIV_THROT_CFG_CCU0);
			iowrite32(DIV_THROT_CFG_CCU1_VALUE,
				  priv->base_addr + DIV_THROT_CFG_CCU1);
			iowrite32(DIV_THROT_CFG_CCU2_VALUE,
				  priv->base_addr + DIV_THROT_CFG_CCU2);
			iowrite32(DIV_THROT_CFG_CCU3_VALUE,
				  priv->base_addr + DIV_THROT_CFG_CCU3);
			iowrite32(HW_THROT_MODE_CCU0_VALUE,
				  priv->base_addr + HW_THROT_MODE_CCU0);
			iowrite32(HW_THROT_MODE_CCU1_VALUE,
				  priv->base_addr + HW_THROT_MODE_CCU1);
			iowrite32(HW_THROT_MODE_CCU2_VALUE,
				  priv->base_addr + HW_THROT_MODE_CCU2);
			iowrite32(HW_THROT_MODE_CCU3_VALUE,
				  priv->base_addr + HW_THROT_MODE_CCU3);
			//debugfs
			thermal_debug = debugfs_create_dir("CPUSS", dir);
			if (!thermal_debug)
				pr_info("failed to create thb_thermal/CPUSS dir\n");
			for (t_reg = 0; t_reg < thb_thermal_size; t_reg++) {
				thb_t->name = thermal_throt[t_reg];
				thb_t->offset = thermal_throt_offset[t_reg];
				thb_t->base_addr = priv->base_addr;
				ret = thermal_throttle_debugfs(pdev,
							       thermal_debug,
							       thb_t);
				if (ret)
					pr_info("Thermal Throttling DebugFS Failed");
			}

		break;

		case PAR_VPU_0:
			//irq_line = platform_get_irq_byname(pdev, "slice0");
			//if (irq_line > 0) {
			//	ret = request_irq(irq_line, irq_handler, 0,
			//			  "Thermal_COM_CPR_0_IRQ",
			//			  priv);
			//if (ret) {
			//	pr_info("%d: request_irq failed, errno=%d\n",
			//		irq_line, -ret);
			//	free_irq(irq_line, NULL);
			//}
			//}
			//PID Config
			iowrite32(DTS_THD_SAR_PID_EN_ABORT_VALUE,
				  priv->base_addr + DTS_THD_SAR_PID_EN_ABORT);
			iowrite32(DTS_PID_RISE_THRES_DTS0_VALUE,
				  priv->base_addr + DTS_PID_RISE_THRES_DTS0);
			iowrite32(DTS_PID_RISE_THRES_DTS1_VALUE,
				  priv->base_addr + DTS_PID_RISE_THRES_DTS1);
			iowrite32(DTS_PID_RISE_THRES_DTS2_VALUE,
				  priv->base_addr + DTS_PID_RISE_THRES_DTS2);
			iowrite32(DTS_PID_FALL_THRES_DTS0_VALUE,
				  priv->base_addr + DTS_PID_FALL_THRES_DTS0);
			iowrite32(DTS_PID_FALL_THRES_DTS1_VALUE,
				  priv->base_addr + DTS_PID_FALL_THRES_DTS1);
			iowrite32(DTS_PID_FALL_THRES_DTS2_VALUE,
				  priv->base_addr + DTS_PID_FALL_THRES_DTS2);
			iowrite32(DTS_PID_KP_DTS0_VALUE,
				  priv->base_addr + DTS_PID_KP_DTS0);
			iowrite32(DTS_PID_KP_DTS1_VALUE,
				  priv->base_addr + DTS_PID_KP_DTS1);
			iowrite32(DTS_PID_KP_DTS2_VALUE,
				  priv->base_addr + DTS_PID_KP_DTS2);
			iowrite32(DTS_PID_KD_DTS0_VALUE,
				  priv->base_addr + DTS_PID_KD_DTS0);
			iowrite32(DTS_PID_KD_DTS1_VALUE,
				  priv->base_addr + DTS_PID_KD_DTS1);
			iowrite32(DTS_PID_KD_DTS2_VALUE,
				  priv->base_addr + DTS_PID_KD_DTS2);
			iowrite32(DTS_PID_KI_DTS0_VALUE,
				  priv->base_addr + DTS_PID_KI_DTS0);
			iowrite32(DTS_PID_KI_DTS1_VALUE,
				  priv->base_addr + DTS_PID_KI_DTS1);
			iowrite32(DTS_PID_KI_DTS2_VALUE,
				  priv->base_addr + DTS_PID_KI_DTS2);
			iowrite32(DTS_PID_CTRL_DTS0_VALUE,
				  priv->base_addr + DTS_PID_CTRL_DTS0);
			iowrite32(DTS_PID_CTRL_DTS1_VALUE,
				  priv->base_addr + DTS_PID_CTRL_DTS1);
			iowrite32(DTS_PID_CTRL_DTS2_VALUE,
				  priv->base_addr + DTS_PID_CTRL_DTS2);
			iowrite32(CG_THROT_CFG1_CCU0_VALUE,
				  priv->base_addr + CG_THROT_CFG1_CCU0);
			iowrite32(CG_THROT_CFG1_CCU1_VALUE,
				  priv->base_addr + CG_THROT_CFG1_CCU1);
			iowrite32(COMPSS_HW_THROT_MODE_CCU0_VALUE,
				  priv->base_addr + HW_THROT_MODE_CCU0);
			iowrite32(COMPSS_HW_THROT_MODE_CCU1_VALUE,
				  priv->base_addr + HW_THROT_MODE_CCU1);
			//debugfs
			thermal_debug = debugfs_create_dir("COMSS1", dir);
			if (!thermal_debug)
				pr_info("failed to create thb_thermal/COMSS1 dir\n");
			for (t_reg = 0; t_reg < thb_thermal_size; t_reg++) {
				thb_t->name = thermal_throt[t_reg];
				thb_t->offset = thermal_throt_offset[t_reg];
				thb_t->base_addr = priv->base_addr;
				ret = thermal_throttle_debugfs(pdev,
							       thermal_debug,
							       thb_t);
				if (ret)
					pr_info("Thermal Throttling DebugFS Failed");
			}
		break;

		case PAR_VPU_1:
			//irq_line = platform_get_irq_byname(pdev, "slice1");
			//if (irq_line > 0) {
			//	ret = request_irq(irq_line, irq_handler,
			//			  0,
			//			  "Thermal_COM_CPR_1_IRQ",
			//			  priv);
			//	if (ret) {
			//		pr_info("%d: request_irq failed, errno=%d\n",
			//			irq_line, -ret);
			//		free_irq(irq_line, NULL);
			//	}
			//}
			//PID Config
			iowrite32(DTS_THD_SAR_PID_EN_ABORT_VALUE,
				  priv->base_addr + DTS_THD_SAR_PID_EN_ABORT);
			iowrite32(DTS_PID_RISE_THRES_DTS0_VALUE,
				  priv->base_addr + DTS_PID_RISE_THRES_DTS0);
			iowrite32(DTS_PID_RISE_THRES_DTS1_VALUE,
				  priv->base_addr + DTS_PID_RISE_THRES_DTS1);
			iowrite32(DTS_PID_RISE_THRES_DTS2_VALUE,
				  priv->base_addr + DTS_PID_RISE_THRES_DTS2);
			iowrite32(DTS_PID_FALL_THRES_DTS0_VALUE,
				  priv->base_addr + DTS_PID_FALL_THRES_DTS0);
			iowrite32(DTS_PID_FALL_THRES_DTS1_VALUE,
				  priv->base_addr + DTS_PID_FALL_THRES_DTS1);
			iowrite32(DTS_PID_FALL_THRES_DTS2_VALUE,
				  priv->base_addr + DTS_PID_FALL_THRES_DTS2);
			iowrite32(DTS_PID_KP_DTS0_VALUE,
				  priv->base_addr + DTS_PID_KP_DTS0);
			iowrite32(DTS_PID_KP_DTS1_VALUE,
				  priv->base_addr + DTS_PID_KP_DTS1);
			iowrite32(DTS_PID_KP_DTS2_VALUE,
				  priv->base_addr + DTS_PID_KP_DTS2);
			iowrite32(DTS_PID_KD_DTS0_VALUE,
				  priv->base_addr + DTS_PID_KD_DTS0);
			iowrite32(DTS_PID_KD_DTS1_VALUE,
				  priv->base_addr + DTS_PID_KD_DTS1);
			iowrite32(DTS_PID_KD_DTS2_VALUE,
				  priv->base_addr + DTS_PID_KD_DTS2);
			iowrite32(DTS_PID_KI_DTS0_VALUE,
				  priv->base_addr + DTS_PID_KI_DTS0);
			iowrite32(DTS_PID_KI_DTS1_VALUE,
				  priv->base_addr + DTS_PID_KI_DTS1);
			iowrite32(DTS_PID_KI_DTS2_VALUE,
				  priv->base_addr + DTS_PID_KI_DTS2);
			iowrite32(DTS_PID_CTRL_DTS0_VALUE,
				  priv->base_addr + DTS_PID_CTRL_DTS0);
			iowrite32(DTS_PID_CTRL_DTS1_VALUE,
				  priv->base_addr + DTS_PID_CTRL_DTS1);
			iowrite32(DTS_PID_CTRL_DTS2_VALUE,
				  priv->base_addr + DTS_PID_CTRL_DTS2);
			iowrite32(CG_THROT_CFG1_CCU0_VALUE,
				  priv->base_addr + CG_THROT_CFG1_CCU0);
			iowrite32(CG_THROT_CFG1_CCU1_VALUE,
				  priv->base_addr + CG_THROT_CFG1_CCU1);
			iowrite32(COMPSS_HW_THROT_MODE_CCU0_VALUE,
				  priv->base_addr + HW_THROT_MODE_CCU0);
			iowrite32(COMPSS_HW_THROT_MODE_CCU1_VALUE,
				  priv->base_addr + HW_THROT_MODE_CCU1);
			//debugfs
			thermal_debug = debugfs_create_dir("COMSS2", dir);
			if (!thermal_debug)
				pr_info("failed to create thb_thermal/COMSS2 dir\n");
			for (t_reg = 0; t_reg < thb_thermal_size; t_reg++) {
				thb_t->name = thermal_throt[t_reg];
				thb_t->offset = thermal_throt_offset[t_reg];
				thb_t->base_addr = priv->base_addr;
				ret = thermal_throttle_debugfs(pdev,
							       thermal_debug,
							       thb_t);
				if (ret)
					pr_info("Thermal Throttling DebugFS Failed");
			}
		break;

		case PAR_VPU_2:
			//irq_line = platform_get_irq_byname(pdev, "slice2");
			//if (irq_line > 0) {
			//	ret = request_irq(irq_line, irq_handler,
			//			  0,
			//			  "Thermal_COM_CPR_2_IRQ",
			//			  priv);
			//if (ret) {
			//	pr_info("%d: request_irq failed, errno=%d\n",
			//		irq_line, -ret);
			//	free_irq(irq_line, NULL);
			//}
			//}
			//PID Config
			iowrite32(DTS_THD_SAR_PID_EN_ABORT_VALUE,
				  priv->base_addr + DTS_THD_SAR_PID_EN_ABORT);
			iowrite32(DTS_PID_RISE_THRES_DTS0_VALUE,
				  priv->base_addr + DTS_PID_RISE_THRES_DTS0);
			iowrite32(DTS_PID_RISE_THRES_DTS1_VALUE,
				  priv->base_addr + DTS_PID_RISE_THRES_DTS1);
			iowrite32(DTS_PID_RISE_THRES_DTS2_VALUE,
				  priv->base_addr + DTS_PID_RISE_THRES_DTS2);
			iowrite32(DTS_PID_FALL_THRES_DTS0_VALUE,
				  priv->base_addr + DTS_PID_FALL_THRES_DTS0);
			iowrite32(DTS_PID_FALL_THRES_DTS1_VALUE,
				  priv->base_addr + DTS_PID_FALL_THRES_DTS1);
			iowrite32(DTS_PID_FALL_THRES_DTS2_VALUE,
				  priv->base_addr + DTS_PID_FALL_THRES_DTS2);
			iowrite32(DTS_PID_KP_DTS0_VALUE,
				  priv->base_addr + DTS_PID_KP_DTS0);
			iowrite32(DTS_PID_KP_DTS1_VALUE,
				  priv->base_addr + DTS_PID_KP_DTS1);
			iowrite32(DTS_PID_KP_DTS2_VALUE,
				  priv->base_addr + DTS_PID_KP_DTS2);
			iowrite32(DTS_PID_KD_DTS0_VALUE,
				  priv->base_addr + DTS_PID_KD_DTS0);
			iowrite32(DTS_PID_KD_DTS1_VALUE,
				  priv->base_addr + DTS_PID_KD_DTS1);
			iowrite32(DTS_PID_KD_DTS2_VALUE,
				  priv->base_addr + DTS_PID_KD_DTS2);
			iowrite32(DTS_PID_KI_DTS0_VALUE,
				  priv->base_addr + DTS_PID_KI_DTS0);
			iowrite32(DTS_PID_KI_DTS1_VALUE,
				  priv->base_addr + DTS_PID_KI_DTS1);
			iowrite32(DTS_PID_KI_DTS2_VALUE,
				  priv->base_addr + DTS_PID_KI_DTS2);
			iowrite32(DTS_PID_CTRL_DTS0_VALUE,
				  priv->base_addr + DTS_PID_CTRL_DTS0);
			iowrite32(DTS_PID_CTRL_DTS1_VALUE,
				  priv->base_addr + DTS_PID_CTRL_DTS1);
			iowrite32(DTS_PID_CTRL_DTS2_VALUE,
				  priv->base_addr + DTS_PID_CTRL_DTS2);
			iowrite32(CG_THROT_CFG1_CCU0_VALUE,
				  priv->base_addr + CG_THROT_CFG1_CCU0);
			iowrite32(CG_THROT_CFG1_CCU1_VALUE,
				  priv->base_addr + CG_THROT_CFG1_CCU1);
			iowrite32(COMPSS_HW_THROT_MODE_CCU0_VALUE,
				  priv->base_addr + HW_THROT_MODE_CCU0);
			iowrite32(COMPSS_HW_THROT_MODE_CCU1_VALUE,
				  priv->base_addr + HW_THROT_MODE_CCU1);
			//debugfs
			thermal_debug = debugfs_create_dir("COMSS3", dir);
			if (!thermal_debug)
				pr_info("failed to create thb_thermal/COMSS3 dir\n");
			for (t_reg = 0; t_reg < thb_thermal_size; t_reg++) {
				thb_t->name = thermal_throt[t_reg];
				thb_t->offset = thermal_throt_offset[t_reg];
				thb_t->base_addr = priv->base_addr;
				ret = thermal_throttle_debugfs(pdev,
							       thermal_debug,
							       thb_t);
				if (ret)
					pr_info("Thermal Throttling DebugFS Failed");
			}
			break;

		case PAR_VPU_3:
			//irq_line = platform_get_irq_byname(pdev, "slice3");
			//if (irq_line > 0) {
				//ret = request_irq(irq_line, irq_handler,
						//  0,
						//  "Thermal_COM_CPR_3_IRQ",
						//  priv);
			//if (ret) {
				//pr_info("%d: request_irq failed, errno=%d\n",
					//irq_line, -ret);
				//free_irq(irq_line, NULL);
			//}
			//}
			//PID Config
			iowrite32(DTS_THD_SAR_PID_EN_ABORT_VALUE,
				  priv->base_addr + DTS_THD_SAR_PID_EN_ABORT);
			iowrite32(DTS_PID_RISE_THRES_DTS0_VALUE,
				  priv->base_addr + DTS_PID_RISE_THRES_DTS0);
			iowrite32(DTS_PID_RISE_THRES_DTS1_VALUE,
				  priv->base_addr + DTS_PID_RISE_THRES_DTS1);
			iowrite32(DTS_PID_RISE_THRES_DTS2_VALUE,
				  priv->base_addr + DTS_PID_RISE_THRES_DTS2);
			iowrite32(DTS_PID_FALL_THRES_DTS0_VALUE,
				  priv->base_addr + DTS_PID_FALL_THRES_DTS0);
			iowrite32(DTS_PID_FALL_THRES_DTS1_VALUE,
				  priv->base_addr + DTS_PID_FALL_THRES_DTS1);
			iowrite32(DTS_PID_FALL_THRES_DTS2_VALUE,
				  priv->base_addr + DTS_PID_FALL_THRES_DTS2);
			iowrite32(DTS_PID_KP_DTS0_VALUE,
				  priv->base_addr + DTS_PID_KP_DTS0);
			iowrite32(DTS_PID_KP_DTS1_VALUE,
				  priv->base_addr + DTS_PID_KP_DTS1);
			iowrite32(DTS_PID_KP_DTS2_VALUE,
				  priv->base_addr + DTS_PID_KP_DTS2);
			iowrite32(DTS_PID_KD_DTS0_VALUE,
				  priv->base_addr + DTS_PID_KD_DTS0);
			iowrite32(DTS_PID_KD_DTS1_VALUE,
				  priv->base_addr + DTS_PID_KD_DTS1);
			iowrite32(DTS_PID_KD_DTS2_VALUE,
				  priv->base_addr + DTS_PID_KD_DTS2);
			iowrite32(DTS_PID_KI_DTS0_VALUE,
				  priv->base_addr + DTS_PID_KI_DTS0);
			iowrite32(DTS_PID_KI_DTS1_VALUE,
				  priv->base_addr + DTS_PID_KI_DTS1);
			iowrite32(DTS_PID_KI_DTS2_VALUE,
				  priv->base_addr + DTS_PID_KI_DTS2);
			iowrite32(DTS_PID_CTRL_DTS0_VALUE,
				  priv->base_addr + DTS_PID_CTRL_DTS0);
			iowrite32(DTS_PID_CTRL_DTS1_VALUE,
				  priv->base_addr + DTS_PID_CTRL_DTS1);
			iowrite32(DTS_PID_CTRL_DTS2_VALUE,
				  priv->base_addr + DTS_PID_CTRL_DTS2);
			iowrite32(CG_THROT_CFG1_CCU0_VALUE,
				  priv->base_addr + CG_THROT_CFG1_CCU0);
			iowrite32(CG_THROT_CFG1_CCU1_VALUE,
				  priv->base_addr + CG_THROT_CFG1_CCU1);
			iowrite32(COMPSS_HW_THROT_MODE_CCU0_VALUE,
				  priv->base_addr + HW_THROT_MODE_CCU0);
			iowrite32(COMPSS_HW_THROT_MODE_CCU1_VALUE,
				  priv->base_addr + HW_THROT_MODE_CCU1);
			//debugfs
			thermal_debug = debugfs_create_dir("COMSS4", dir);
			if (!thermal_debug)
				pr_info("failed to create thb_thermal/COMSS4 dir\n");
			for (t_reg = 0; t_reg < thb_thermal_size; t_reg++) {
				thb_t->name = thermal_throt[t_reg];
				thb_t->offset = thermal_throt_offset[t_reg];
				thb_t->base_addr = priv->base_addr;
				ret = thermal_throttle_debugfs(pdev, thermal_debug, thb_t);
				if (ret)
					pr_info("Thermal Throttling DebugFS Failed");
			}
		break;

		default:
			break;
	}
return 0;
}

static int thunderbay_thermal_trip(void __iomem *base_addr, int type)
{
	if (type == CPUSS_SOUTH_NOC)
		iowrite32(0x1, base_addr + A53SS_THERM_CTRL);
return 0;
}

static int thunderbay_iccmax_default_config(void __iomem *base_addr,
					    void __iomem *gpio_addr,
    					    int board_type, int type)
{
	if (board_type == 9 || board_type == 11) {
		iowrite32(GPIO_POLARITY_EVT2, gpio_addr + GPIO_POWER_OFFSET);
		switch (type) {
		case PAR_VPU_0:
			iowrite32(COMSS_THERM_SCALING_CNT,
				  base_addr + COMSS_THERM_SCALING_CNT_OFFSET);
			iowrite32(COMSS_0_ICCMAX_GPIO_EN_CCU0,
				  base_addr + ICCMAX_GPIO_EN_CCU0_OFFSET);
			iowrite32(COMSS_ICCMAX_THROT_CFG_CCU0,
				  base_addr + ICCMAX_THROT_CFG_CCU0_OFFSET);
			break;

		case PAR_VPU_1:
			iowrite32(COMSS_THERM_SCALING_CNT,
				  base_addr + COMSS_THERM_SCALING_CNT_OFFSET);
			iowrite32(COMSS_1_ICCMAX_GPIO_EN_CCU0,
				  base_addr + ICCMAX_GPIO_EN_CCU0_OFFSET);
			iowrite32(COMSS_ICCMAX_THROT_CFG_CCU0,
				  base_addr + ICCMAX_THROT_CFG_CCU0_OFFSET);
			break;

		case PAR_VPU_2:
			iowrite32(COMSS_THERM_SCALING_CNT,
				  base_addr + COMSS_THERM_SCALING_CNT_OFFSET);
			iowrite32(COMSS_2_ICCMAX_GPIO_EN_CCU0,
				  base_addr + ICCMAX_GPIO_EN_CCU0_OFFSET);
			iowrite32(COMSS_ICCMAX_THROT_CFG_CCU0,
				  base_addr + ICCMAX_THROT_CFG_CCU0_OFFSET);
			break;

		case PAR_VPU_3:
			iowrite32(COMSS_THERM_SCALING_CNT,
				  base_addr + COMSS_THERM_SCALING_CNT_OFFSET);
			iowrite32(COMSS_3_ICCMAX_GPIO_EN_CCU0,
				  base_addr + ICCMAX_GPIO_EN_CCU0_OFFSET);
			iowrite32(COMSS_ICCMAX_THROT_CFG_CCU0,
				  base_addr + ICCMAX_THROT_CFG_CCU0_OFFSET);
			break;

		default:
			break;
			}
		}
	if (board_type == 8 || board_type == 10 || board_type == 12) {
		iowrite32(GPIO_PMAX_POLARITY_EVT1, gpio_addr + GPIO_POWER_OFFSET);
		iowrite32(COMSS_THERM_SCALING_CNT,
			  base_addr + COMSS_THERM_SCALING_CNT_OFFSET);
		iowrite32(COMSS_ONLY_PMAX_ENABLED,
			  base_addr + ICCMAX_GPIO_EN_CCU0_OFFSET);
		iowrite32(COMSS_ICCMAX_THROT_CFG_CCU0,
			  base_addr + ICCMAX_THROT_CFG_CCU0_OFFSET);
	}
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
	priv->gpio_addr = devm_ioremap(&pdev->dev, GPIO_REG, GPIO_POWER_OFFSET);	
	//iowrite32(0x1c0, plat_data->base_addr+DTS_THD_SAR_PID_EN_ABORT);
	priv->name = plat_data->name;
	priv->base_addr = plat_data->base_addr;
	priv->plat_data = plat_data;
	priv->s_node = plat_data->s_node;
	priv->pdev = plat_data->pdev;
	//thermal register config
	if (intel_tbh_thermal_config(priv, plat_data->sensor_type))
		dev_info(&pdev->dev, "THENDERBAY_THERMAL_CONFIGURATION_FAILED");
	//thermal framework get_temp
	plat_data->get_temp = thunderbay_get_temp;
	//setting thermal_trip
	thunderbay_thermal_trip(priv->base_addr, plat_data->sensor_type);
	//seting iccmax and pmax default values
	thunderbay_iccmax_default_config(priv->base_addr,priv->gpio_addr,
					 priv->board_type,
					 plat_data->sensor_type);
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
