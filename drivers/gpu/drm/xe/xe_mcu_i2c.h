/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _XE_I2C_H_
#define _XE_I2C_H_

#include <linux/bits.h>
#include <linux/notifier.h>
#include <linux/types.h>
#include <linux/workqueue.h>

struct device;
struct i2c_adapter;
struct i2c_client;
struct platform_device;
struct xe_device;
struct xe_mmio;

#define XE_I2C_MAX_CLIENTS		3

#define XE_I2C_EP_COOKIE_DEVICE		0xde

/* Endpoint Capabilities */
#define XE_I2C_EP_CAP_IRQ		BIT(0)
#define XE_I2C_EP_CAP_MULTI_MCU		BIT(1)
#define XE_I2C_EP_CAP_SMBUS		BIT(2)

struct xe_i2c_endpoint {
	u8 cookie;
	u8 capabilities;
	u16 addr[XE_I2C_MAX_CLIENTS];
};

struct xe_i2c {
	struct notifier_block bus_notifier;
	struct work_struct work;

	struct platform_device *pdev;
	struct i2c_adapter *adapter;
	struct i2c_client *client[XE_I2C_MAX_CLIENTS];

	struct xe_i2c_endpoint ep;
	struct device *drm_dev;

	struct xe_mmio *mmio;
	u32 ic_enable;
};

#if IS_ENABLED(CONFIG_I2C)
int xe_i2c_probe(struct xe_device *xe);
#else
static inline int xe_i2c_probe(struct xe_device *xe) { return 0; }
#endif

#endif
