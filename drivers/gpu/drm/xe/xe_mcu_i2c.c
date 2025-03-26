// SPDX-License-Identifier: GPL-2.0
/*
 * Intel Xe I2C attached Microcontroller Units (MCU)
 *
 * Copyright (C) 2025 Intel Corporation.
 */

#include <linux/array_size.h>
#include <linux/container_of.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/ioport.h>
#include <linux/notifier.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/sprintf.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "regs/xe_i2c_regs.h"

#include "xe_device.h"
#include "xe_device_types.h"
#include "xe_mcu_i2c.h"
#include "xe_mmio.h"
#include "xe_platform_types.h"

/* Synopsys DesignWare I2C Host Adapter */
static const char adapter_name[] = "i2c_designware";

static const struct property_entry xe_i2c_adapter_properties[] = {
	PROPERTY_ENTRY_STRING("compatible", "intel,xe-mcu-i2c"),
	PROPERTY_ENTRY_U32("clock-frequency", I2C_MAX_FAST_MODE_PLUS_FREQ),
	{ }
};

static inline void xe_i2c_read_endpoint(struct xe_mmio *mmio, void *ep)
{
	u32 *val = ep;

	val[0] = xe_mmio_read32(mmio, CLIENT_DISC_COOKIE);
	val[1] = xe_mmio_read32(mmio, CLIENT_DISC_ADDRESS);
}

static void xe_i2c_client_work(struct work_struct *work)
{
	struct xe_i2c *i2c = container_of(work, struct xe_i2c, work);
	struct i2c_board_info info = {
		.type	= "amc",
		.flags	= I2C_CLIENT_HOST_NOTIFY,
		.addr	= i2c->ep.addr[1],
	};

	i2c->client[0] = i2c_new_client_device(i2c->adapter, &info);
}

static int xe_i2c_notifier(struct notifier_block *nb, unsigned long action, void *data)
{
	struct xe_i2c *i2c = container_of(nb, struct xe_i2c, bus_notifier);
	struct i2c_adapter *adapter = i2c_verify_adapter(data);
	struct device *dev = data;

	if (action == BUS_NOTIFY_ADD_DEVICE &&
	    adapter && dev->parent == &i2c->pdev->dev) {
		i2c->adapter = adapter;
		schedule_work(&i2c->work);
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

static int xe_i2c_register_adapter(struct xe_i2c *i2c)
{
	struct pci_dev *pci = to_pci_dev(i2c->drm_dev);
	struct platform_device *pdev;
	struct fwnode_handle *fwnode;
	int ret;

	fwnode = fwnode_create_software_node(xe_i2c_adapter_properties, NULL);
	if (!fwnode)
		return -ENOMEM;

	/*
	 * Not using platform_device_register_full() here because we don't have
	 * a handle to the platform_device before it returns. xe_i2c_notifier()
	 * uses that handle, but it may be called before
	 * platform_device_register_full() is done.
	 *
	 * The problem could also be handled for example by always
	 * unconditionally scheduling the i2c->work in xe_i2c_notifier(), and
	 * handling all the checks in that work, but it would still require an
	 * additional lock to be used. That would complicate the code to the
	 * extent that it is more clear to just "manually" allocate the platform
	 * device. This way xe_i2c_notifier() can be guaranteed to be called
	 * only after i2c->pdev is assigned.
	 */
	pdev = platform_device_alloc(adapter_name, pci_dev_id(pci));
	if (!pdev) {
		ret = -ENOMEM;
		goto err_fwnode_remove;
	}

	pdev->dev.parent = i2c->drm_dev;
	pdev->dev.fwnode = fwnode;
	i2c->pdev = pdev;

	ret = platform_device_add(pdev);
	if (ret)
		goto err_pdev_put;

	return 0;

err_pdev_put:
	platform_device_put(pdev);
err_fwnode_remove:
	fwnode_remove_software_node(fwnode);

	return ret;
}

static void xe_i2c_unregister_adapter(struct xe_i2c *i2c)
{
	fwnode_remove_software_node(dev_fwnode(&i2c->pdev->dev));
	platform_device_unregister(i2c->pdev);
}

#define IC_ENABLE                      0x6c
#define IC_ENABLE_STATUS               0x9c

static int xe_i2c_read(void *context, unsigned int reg, unsigned int *val)
{
	struct xe_i2c *i2c = context;

	if (reg == IC_ENABLE)
		*val = i2c->ic_enable;
	else if (reg == IC_ENABLE_STATUS)
		*val = i2c->ic_enable & 1; /* NOTE: Checking only the enable bit */
	else
		*val = xe_mmio_read32(i2c->mmio, XE_REG(reg + I2C_MEM_SPACE_OFFSET));

	return 0;
}

static int xe_i2c_write(void *context, unsigned int reg, unsigned int val)
{
	struct xe_i2c *i2c = context;

	if (reg == IC_ENABLE)
		i2c->ic_enable = val;
	else
		xe_mmio_write32(i2c->mmio, XE_REG(reg + I2C_MEM_SPACE_OFFSET), val);

	return 0;
}

static const struct regmap_config i2c_regmap_config = {
	.name = "xe_i2c",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_read = xe_i2c_read,
	.reg_write = xe_i2c_write,
	.fast_io = true,
};

void xe_i2c_pm_suspend(struct xe_device *xe)
{
	struct xe_mmio *mmio = xe_root_tile_mmio(xe);
	struct xe_reg pmcsr = XE_REG(I2C_CONFIG_SPACE_OFFSET + 0x84);

	if (!xe->i2c || xe->i2c->ep.cookie != XE_I2C_EP_COOKIE_DEVICE)
		return;

	xe_mmio_rmw32(mmio, pmcsr, PCI_PM_CTRL_STATE_MASK, PCI_D3hot);
	drm_dbg(&xe->drm, "pmcsr: 0x%08x\n", xe_mmio_read32(mmio, pmcsr));
}

void xe_i2c_pm_resume(struct xe_device *xe, bool d3cold)
{
	struct xe_mmio *mmio = xe_root_tile_mmio(xe);
	struct xe_reg pmcsr = XE_REG(I2C_CONFIG_SPACE_OFFSET + 0x84);

	if (!xe->i2c || xe->i2c->ep.cookie != XE_I2C_EP_COOKIE_DEVICE)
		return;

	if (d3cold) {
		xe_mmio_rmw32(mmio, XE_REG(I2C_CONFIG_SPACE_OFFSET + PCI_COMMAND), 0, PCI_COMMAND_MEMORY);

		drm_dbg(&xe->drm, "vid: 0x%04x\n", xe_mmio_read16(mmio, XE_REG(I2C_CONFIG_SPACE_OFFSET + PCI_VENDOR_ID)));
		drm_dbg(&xe->drm, "did: 0x%04x\n", xe_mmio_read16(mmio, XE_REG(I2C_CONFIG_SPACE_OFFSET + PCI_DEVICE_ID)));
		drm_dbg(&xe->drm, "com: 0x%04x\n", xe_mmio_read16(mmio, XE_REG(I2C_CONFIG_SPACE_OFFSET + PCI_COMMAND)));
		drm_dbg(&xe->drm, "stat: 0x%04x\n", xe_mmio_read16(mmio, XE_REG(I2C_CONFIG_SPACE_OFFSET + PCI_STATUS)));
		drm_dbg(&xe->drm, "cookie: 0x%08x\n", xe_mmio_read32(mmio, CLIENT_DISC_COOKIE));
		drm_dbg(&xe->drm, "addr: 0x%08x\n", xe_mmio_read32(mmio, CLIENT_DISC_ADDRESS));
	}

	xe_mmio_rmw32(mmio, pmcsr, PCI_PM_CTRL_STATE_MASK, PCI_D0);
	drm_dbg(&xe->drm, "pmcsr: 0x%08x\n", xe_mmio_read32(mmio, pmcsr));
	drm_dbg(&xe->drm, "dw: 0x%08x\n", xe_mmio_read32(mmio, XE_REG(I2C_MEM_SPACE_OFFSET + 0xfc)));
}

static void xe_i2c_remove(void *data)
{
	struct xe_i2c *i2c = data;
	int i;

	for (i = 0; i < XE_I2C_MAX_CLIENTS; i++)
		i2c_unregister_device(i2c->client[i]);

	xe_i2c_unregister_adapter(i2c);
	bus_unregister_notifier(&i2c_bus_type, &i2c->bus_notifier);
}

int xe_i2c_probe(struct xe_device *xe)
{
	struct xe_i2c_endpoint ep;
	struct regmap *regmap;
	struct xe_i2c *i2c;
	int ret;

	if (xe->info.platform != XE_BATTLEMAGE)
		return 0;

	xe_i2c_read_endpoint(xe_root_tile_mmio(xe), &ep);
	if (ep.cookie != XE_I2C_EP_COOKIE_DEVICE)
		return 0;

	i2c = devm_kzalloc(xe->drm.dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	INIT_WORK(&i2c->work, xe_i2c_client_work);
	i2c->mmio = xe_root_tile_mmio(xe);
	i2c->drm_dev = xe->drm.dev;
	i2c->ep = ep;
	xe->i2c = i2c;

	/* PCI PM isn't aware of this device, bring it up and match it with SGUnit state */
	xe_i2c_pm_resume(xe, true);

	regmap = devm_regmap_init(i2c->drm_dev, NULL, i2c, &i2c_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	i2c->bus_notifier.notifier_call = xe_i2c_notifier;
	ret = bus_register_notifier(&i2c_bus_type, &i2c->bus_notifier);
	if (ret)
		return ret;

	ret = xe_i2c_register_adapter(i2c);
	if (ret) {
		bus_unregister_notifier(&i2c_bus_type, &i2c->bus_notifier);
		return ret;
	}

	return devm_add_action_or_reset(i2c->drm_dev, xe_i2c_remove, i2c);
}
