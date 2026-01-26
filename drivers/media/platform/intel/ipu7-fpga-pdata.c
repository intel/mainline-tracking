// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2020 Intel Corporation
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <media/v4l2-mediabus.h>

#include "ipu7.h"
#include "ipu7-isys.h"

#define OV13B10_LANES		4
#define OV13B10_2LANES		2
#define OV13B10_I2C_ADDRESS	0x10

static struct ipu7_isys_csi2_config ov13b10_csi2_cfg_0 = {
	.nlanes = OV13B10_LANES,
	.port = 0,
	.bus_type = V4L2_MBUS_CSI2_DPHY,
};

static struct ipu7_isys_subdev_info ov13b10_sd_0 = {
	.csi2 = &ov13b10_csi2_cfg_0,
	.i2c = {
	.board_info = {
		I2C_BOARD_INFO("ov13b10", OV13B10_I2C_ADDRESS),
	},
	.i2c_adapter_id = 0,
	}
};

static struct ipu7_isys_csi2_config ov13b10_csi2_cfg_1 = {
	.nlanes = OV13B10_2LANES,
	.port = 2,
	.bus_type = V4L2_MBUS_CSI2_DPHY,
};

static struct ipu7_isys_subdev_info ov13b10_sd_1 = {
	.csi2 = &ov13b10_csi2_cfg_1,
	.i2c = {
	.board_info = {
		I2C_BOARD_INFO("ov13b10", OV13B10_I2C_ADDRESS),
	},
	.i2c_adapter_id = 1,
	}
};

static struct ipu7_isys_subdev_pdata pdata = {
	.subdevs = (struct ipu7_isys_subdev_info *[]) {
		&ov13b10_sd_0,
		&ov13b10_sd_1,
		NULL,
	},
};

static void ipu7_quirk(struct pci_dev *pci_dev)
{
	dev_info(&pci_dev->dev, "%s() attach the platform data", __func__);
	pci_dev->dev.platform_data = &pdata;
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, IPU7_PCI_ID, ipu7_quirk);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, IPU7P5_PCI_ID, ipu7_quirk);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, IPU8_PCI_ID, ipu7_quirk);
