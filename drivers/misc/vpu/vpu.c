// SPDX-License-Identifier: GPL-2.0-only
/*
 * Kernel module for:
 *    1. Memory mapping DDR address space to Intel Thunder Bay SOC's VPU
 *       address space
 *
 * Copyright (C) 2019 Intel Corporation
 *
 */
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/iommu.h>
#include <linux/of.h>

static int vpu_platform_driver_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct device *dev = &pdev->dev;
	struct iommu_domain *vpu_domain = NULL;

	int params_length = 0;
	int i;
	u32 va, size, attr, pa_l, pa_h;
	u64 pa;

	if (iommu_present(dev->bus)) {
		if (!vpu_domain)
			vpu_domain = iommu_domain_alloc(dev->bus);

		iommu_attach_device(vpu_domain, dev);
		params_length =
			of_property_count_elems_of_size(dev->of_node, "intel,smmu_params",
							sizeof(u32));

		if (params_length == 0) {
			dev_err(dev, "No memory region to map\n");
			goto error_exit;
		}

		pr_info("vpu-mm: found %d entries for mapping\n", params_length);

		for (i = 0; i < params_length; i += 5) {
			char attr_s[] = "      ";

			rc = of_property_read_u32_index(dev->of_node, "intel,smmu_params",
							i + 0, &va);

			if (rc) {
				dev_err(dev, "property not set\n");
				goto error_exit;
			}

			rc = of_property_read_u32_index(dev->of_node, "intel,smmu_params",
							i + 1, &pa_h);

			rc = of_property_read_u32_index(dev->of_node, "intel,smmu_params",
							i + 2, &pa_l);

			rc = of_property_read_u32_index(dev->of_node, "intel,smmu_params",
							i + 3, &size);

			rc = of_property_read_u32_index(dev->of_node, "intel,smmu_params",
							i + 4, &attr);

			if (rc) {
				dev_err(dev, "No memory region to map\n");
				goto error_exit;
			}

			pa = (((u64)pa_h) << 32) | pa_l;

			if (attr & IOMMU_READ)
				attr_s[0] = 'R';
			if (attr & IOMMU_WRITE)
				attr_s[1] = 'W';
			if (attr & IOMMU_CACHE)
				attr_s[2] = 'C';
			if (!(attr & IOMMU_NOEXEC))
				attr_s[3] = 'X';
			if (attr & IOMMU_MMIO)
				attr_s[4] = 'I';

			if (attr & IOMMU_PRIV)
				attr_s[5] = 'P';
			else
				attr_s[5] = 'U';

			dev_info(dev, "vpu-mm: iommu mapping done for vaddr: 0x%0X--0x%0X, paddr: 0x%0llX++0x%0X, attr=0x%0x[%s]\n",
				 va,
				 va + size - 1,
				 pa,
				 size,
				 attr, attr_s);

			iommu_map(vpu_domain, (unsigned int)va, (unsigned long long)pa,
				  size, attr);
		}
	}

	return 0;

error_exit:
	return -1;
}

static void vpu_platform_driver_shutdown(struct platform_device *pdev)
{
}

static const struct of_device_id vpu_of_match[] = {
	{ .compatible = "intel,vpu", },
	{ /* end of table */}
};

static struct platform_driver vpu_platform_driver = {
	.probe  = vpu_platform_driver_probe,
	.shutdown = vpu_platform_driver_shutdown,
	.driver = {
		.owner = THIS_MODULE,
		.name  = "intel,vpu",
		.of_match_table = vpu_of_match,
	},
};

builtin_platform_driver(vpu_platform_driver);

MODULE_DESCRIPTION("VPU memory mapping driver");
MODULE_AUTHOR("Demakkanavar, Kenchappa <kenchappa.demakkanavar@intel.com>");
MODULE_AUTHOR("Raja Subramanian, Lakshmi Bai <lakshmi.bai.raja.subramanian@intel.com>");
MODULE_AUTHOR("S, Kiran Kumar1 <kiran.kumar1.s@intel.com>");
MODULE_LICENSE("GPL v2");
