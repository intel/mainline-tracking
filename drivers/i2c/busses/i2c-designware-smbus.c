// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Synopsys DesignWare SMBus driver.
 *
 * Copyright (C) 2024 Intel Corporation.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/i2c-smbus.h>
#include <linux/irqreturn.h>
#include <linux/regmap.h>
#include <linux/types.h>

#include "i2c-designware-core.h"

static struct i2c_smbus_alert_setup i2c_dw_smbus_setup;

/**
 * i2c_dw_smbus_isr - Interrupt service routine for SMBus interrupts
 * @dev: handle to the controller
 *
 * This function currently only handles the SMBus Alert signal.
 *
 * Return: IRQ_HANDLED if the interrupt was caused by the SMBUS Alert,
 * otherwise IRQ_NONE.
 */
irqreturn_t i2c_dw_smbus_isr(struct dw_i2c_dev *dev)
{
	u32 stat;

	if (!dev->smbus_alert)
		return IRQ_NONE;

	regmap_read(dev->map, DW_IC_SMBUS_INTR_STAT, &stat);
	if (!stat)
		return IRQ_NONE;

	regmap_write(dev->map, DW_IC_CLR_SMBUS_INTR, stat);

	if (stat & DW_IC_SMBUS_INTR_ALERT)
		i2c_handle_smbus_alert(dev->smbus_alert);

	return IRQ_HANDLED;
}

/**
 * i2c_dw_smbus_host_register - Register the SMBus alert device for the host
 * @dev: handle to the controller
 *
 * This function checks is the SMBus feature available, and then registers the
 * alert device if it is. If the SMBus feature is not available the function
 * returns silently with a success.
 *
 * The SMBus alert device needs to be unregistered by calling
 * i2c_dw_smbus_unregister().
 *
 * Return: 0 on success, errno on error.
 */
int i2c_dw_smbus_host_register(struct dw_i2c_dev *dev)
{
	struct i2c_client *alert;
	u32 ic_version;
	int ret;

	ret = regmap_read(dev->map, DW_IC_COMP_VERSION, &ic_version);
	if (ret)
		return ret;

	if (ic_version < DW_IC_SMBUS_MIN_VER)
		return 0;

	alert = i2c_new_smbus_alert_device(&dev->adapter, &i2c_dw_smbus_setup);
	if (IS_ERR(alert))
		return PTR_ERR(alert);

	dev->smbus_alert = alert;

	ret = regmap_clear_bits(dev->map, DW_IC_SMBUS_INTR_MASK, DW_IC_SMBUS_INTR_ALERT);
	if (ret)
		i2c_dw_smbus_unregister(dev);

	return ret;
}
