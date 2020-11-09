/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020, Intel Corporation. All rights reserved
 * Intel Management Engine Interface (Intel MEI) Linux driver
 */
#include <linux/module.h>
#include <linux/mei_cl_bus.h>

#define HBM_DMA_BUF_ID_WLAN 1

static bool mei_bus_dma_test_mapped;

static int mei_bus_dma_test_probe(struct mei_cl_device *device,
				  const struct mei_cl_device_id *id)
{
	int ret;
	void *vaddr = NULL;

	ret = mei_cldev_enable(device);
	if (ret) {
		dev_err(&device->dev, "probe failed = %d\n", ret);
		return ret;
	}
	dev_err(&device->dev, "probe succeeded\n");

	vaddr = mei_cldev_dma_map(device, HBM_DMA_BUF_ID_WLAN, 16 * 4096);
	if (IS_ERR(vaddr)) {
		mei_cldev_disable(device);
		ret = PTR_ERR(vaddr);
		dev_err(&device->dev, "mei_cldev_dma_map ret=%d\n", ret);
	} else {
		mei_bus_dma_test_mapped = true;
		dev_err(&device->dev, "mei_cldev_dma_map succeeded\n");
		ret = 0;
	}

	return ret;
}

static void mei_bus_dma_test_remove(struct mei_cl_device *device)
{
	int ret;
	if (mei_bus_dma_test_mapped) {
		ret = mei_cldev_dma_unmap(device);
		dev_err(&device->dev, "mei_cldev_dma_unmap ret=%d\n", ret);
		mei_bus_dma_test_mapped = false;
	}

	ret = mei_cldev_disable(device);
	if (ret)
		dev_err(&device->dev, "remove failed = %d\n", ret);

	dev_err(&device->dev, "remove succeeded\n");
}

#define MEI_WLAN UUID_LE(0x13280904, 0x7792, 0x4fcb, \
			 0xa1, 0xaa, 0x5e, 0x70, 0xcb, 0xb1, 0xe8, 0x65)

static struct mei_cl_device_id mei_bus_dma_test_tbl[] = {
	{ .uuid = MEI_WLAN, .version = MEI_CL_VERSION_ANY},
	/* required last entry */
	{ }
};

MODULE_DEVICE_TABLE(mei, mei_bus_dma_test_tbl);

static struct mei_cl_driver mei_bus_dma_test_driver = {
	.id_table = mei_bus_dma_test_tbl,
	.name = "mei_bus_dma_test",
	.probe = mei_bus_dma_test_probe,
	.remove = mei_bus_dma_test_remove,
};

static int mei_bus_dma_test_init(void)
{
	int ret;

	ret = mei_cldev_driver_register(&mei_bus_dma_test_driver);
	if (ret) {
		pr_err(KBUILD_MODNAME ": driver registration failed\n");
		return ret;
	}

	return 0;
}

static void mei_bus_dma_test_exit(void)
{
	mei_cldev_driver_unregister(&mei_bus_dma_test_driver);
}

module_init(mei_bus_dma_test_init);
module_exit(mei_bus_dma_test_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("mei bus dma test");
