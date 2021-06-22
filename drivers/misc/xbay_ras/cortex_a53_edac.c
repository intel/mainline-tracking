// SPDX-License-Identifier: GPL-2.0 only
/*
 * Intel xBay RAS: CPU EDAC SW
 *
 * Copyright (C) 2021 Intel Corporation
 *
 * Code used from:
 *  https://lwn.net/Articles/662208/
 */

#include <linux/cpu.h>

#include "ras_core.h"

#define CPUMERRSR_EL1_INDEX(x, y)	((x) & (y))
#define CPUMERRSR_EL1_BANK_WAY(x, y)	(((x) >> 18) & (y))
#define CPUMERRSR_EL1_RAMID(x)		(((x) >> 24) & 0x7f)
#define CPUMERRSR_EL1_VALID(x)		((x) & (1 << 31))
#define CPUMERRSR_EL1_REPEAT(x)		(((x) >> 32) & 0x7f)
#define CPUMERRSR_EL1_OTHER(x)		(((x) >> 40) & 0xff)
#define CPUMERRSR_EL1_FATAL(x)		((x) & (1UL << 63))
#define L1_I_TAG_RAM			0x00
#define L1_I_DATA_RAM			0x01
#define L1_D_TAG_RAM			0x08
#define L1_D_DATA_RAM			0x09
#define L1_D_DIRTY_RAM			0x14
#define TLB_RAM				0x18

#define L2MERRSR_EL1_CPUID_WAY(x)	(((x) >> 18) & 0xf)
#define L2MERRSR_EL1_RAMID(x)		(((x) >> 24) & 0x7f)
#define L2MERRSR_EL1_VALID(x)		((x) & (1 << 31))
#define L2MERRSR_EL1_REPEAT(x)		(((x) >> 32) & 0xff)
#define L2MERRSR_EL1_OTHER(x)		(((x) >> 40) & 0xff)
#define L2MERRSR_EL1_FATAL(x)		((x) & (1UL << 63))
#define L2_TAG_RAM			0x10
#define L2_DATA_RAM			0x11
#define L2_SNOOP_RAM			0x12
#define L2_DIRTY_RAM			0x14
#define L2_INCLUSION_PF_RAM		0x18

#define L1_CACHE			0
#define L2_CACHE			1

#define EDAC_MOD_STR			DRV_NAME

/* Error injectin macros*/
#define L1_DCACHE_ERRINJ_ENABLE		BIT(6)
#define L1_DCACHE_ERRINJ_DISABLE	(~(1 << 6))
#define L2_DCACHE_ERRINJ_ENABLE		BIT(29)
#define L2_DCACHE_ERRINJ_DISABLE	(~(1 << 29))
#define L2_ECC_PROTECTION		BIT(22)

static struct cpu_edac_info_t
	cpu_edac_info[NUM_CACHES];

static inline u64 read_cpumerrsr_el1(void)
{
	u64 val;

	asm volatile("mrs %0, s3_1_c15_c2_2" : "=r" (val));
	return val;
}

static inline void write_cpumerrsr_el1(u64 val)
{
	asm volatile("msr s3_1_c15_c2_2, %0" :: "r" (val));
}

static inline u64 read_l2merrsr_el1(void)
{
	u64 val;

	asm volatile("mrs %0, s3_1_c15_c2_3" : "=r" (val));
	return val;
}

static inline void write_l2merrsr_el1(u64 val)
{
	asm volatile("msr s3_1_c15_c2_3, %0" :: "r" (val));
}

static inline void cortex_a53_edac_busy_on_inst(void)
{
	asm volatile("isb sy");
}

static inline void cortex_a53_edac_busy_on_data(void)
{
	asm volatile("dsb sy");
}

static inline void write_l2actrl_el1(u64 val)
{
	asm volatile("msr s3_1_c15_c0_0, %0" :: "r" (val));
	cortex_a53_edac_busy_on_inst();
}

static inline u64 read_l2actrl_el1(void)
{
	u64 val;

	asm volatile("mrs %0, s3_1_c15_c0_0" : "=r" (val));
	return val;
}

static inline u64 read_l2ctlr_el1(void)
{
	u64 rval;

	asm volatile("mrs %0,  S3_1_C11_C0_2" : "=r" (rval));
	return rval;
}

static inline u64 read_l1actrl_el1(void)
{
	u64 rval;

	asm volatile("mrs %0,  S3_1_C15_C2_0" : "=r" (rval));
	return rval;
}

static inline void write_l1actrl_el1(u64 val)
{
	asm volatile("msr S3_1_C15_C2_0, %0" :: "r" (val));
}

static void parse_cpumerrsr(void *arg)
{
	struct cpu_edac_info_t *e_sts = &cpu_edac_info[0];
	u64 val = read_cpumerrsr_el1();

	/* we do not support fatal error handling so far */
	if (CPUMERRSR_EL1_FATAL(val))
		return;

	/* check if we have valid error before continuing */
	if (!CPUMERRSR_EL1_VALID(val))
		return;

	e_sts->cpu = smp_processor_id();
	e_sts->repeat_err = CPUMERRSR_EL1_REPEAT(val);
	e_sts->other_err = CPUMERRSR_EL1_OTHER(val);
	e_sts->index = CPUMERRSR_EL1_INDEX(val, 0xfff);
	e_sts->way = CPUMERRSR_EL1_BANK_WAY(val, 0x7);
	e_sts->ram_id = CPUMERRSR_EL1_RAMID(val);

#ifdef DEBUG_RAS_CPU
	switch (e_sts->ram_id) {
	case L1_I_TAG_RAM:
		pr_debug("'L1-I Tag RAM' (way %d)", e_sts->way);
		break;
	case L1_I_DATA_RAM:
		pr_debug("'L1-I Data RAM' (bank %d)", e_sts->way);
		break;
	case L1_D_TAG_RAM:
		pr_debug("'L1-D Tag RAM' (way %d)", e_sts->way);
		break;
	case L1_D_DATA_RAM:
		pr_debug("'L1-D Data RAM' (bank %d)", e_sts->way);
		break;
	case L1_D_DIRTY_RAM:
		pr_debug("'L1 Dirty RAM'");
		break;
	case TLB_RAM:
		pr_debug("'TLB RAM'");
		break;
	default:
		pr_debug("'unknown'");
		break;
	}
#endif

	write_cpumerrsr_el1(0);
}

#ifdef DEBUG_RAS_CPU
static void cortex_a53_parse_l2merrsr_way(u8 ramid, u8 val)
{
	switch (ramid) {
	case L2_TAG_RAM:
		pr_debug("(way %d)", val);
		break;
	case L2_DATA_RAM:
		pr_debug("(bank %d)", val);
		break;
	case L2_SNOOP_RAM:
		pr_debug("(cpu%d tag, way %d)", val / 2, val % 4);
		break;
	}
}
#endif

static void parse_l2merrsr(void *arg)
{
	struct cpu_edac_info_t *e_sts = &cpu_edac_info[1];
	u64 val = read_l2merrsr_el1();

	/* we do not support fatal error handling so far */
	if (L2MERRSR_EL1_FATAL(val))
		return;

	/* check if we have valid error before continuing */
	if (!L2MERRSR_EL1_VALID(val))
		return;

	e_sts->cpu = smp_processor_id();
	e_sts->repeat_err = L2MERRSR_EL1_REPEAT(val);
	e_sts->other_err = L2MERRSR_EL1_OTHER(val);
	e_sts->index = (val >> 3) & 0x3fff;
	e_sts->ram_id = L2MERRSR_EL1_RAMID(val);
	e_sts->way = L2MERRSR_EL1_CPUID_WAY(val);

#ifdef DEBUG_RAS_CPU
	switch (e_sts->ram_id) {
	case L2_TAG_RAM:
		pr_debug("'L2 Tag RAM'");
		break;
	case L2_DATA_RAM:
		pr_debug("'L2 Data RAM'");
		break;
	case L2_SNOOP_RAM:
		pr_debug("'L2 Snoop tag RAM'");
		break;
	case L2_DIRTY_RAM:
		pr_debug("'L2 Dirty RAM'");
		break;
	case L2_INCLUSION_PF_RAM:
		pr_debug("'L2 inclusion PF RAM'");
		break;
	default:
		pr_debug("unknown");
		break;
	}

	/* cpuid/way bit description is different between A57 and A53 */
	cortex_a53_parse_l2merrsr_way(e_sts->ram_id, e_sts->way);
#endif

	write_l2merrsr_el1(0);
}

static void cortex_a53_edac_check(void)
{
	int cpu;
	struct cpumask cluster_mask, old_mask;

	cpumask_clear(&cluster_mask);
	cpumask_clear(&old_mask);

	get_online_cpus();
	for_each_online_cpu(cpu) {
		/* Check CPU L1 error */
		smp_call_function_single(cpu, parse_cpumerrsr, NULL, 0);
		cpumask_copy(&cluster_mask, topology_core_cpumask(cpu));
		if (cpumask_equal(&cluster_mask, &old_mask))
			continue;
		cpumask_copy(&old_mask, &cluster_mask);

		/* Check CPU L2 error */
		smp_call_function_any(&cluster_mask, parse_l2merrsr, NULL, 0);
	}
	put_online_cpus();
}

#if defined(DEBUG_RAS_CPU) && defined(DEBUG_CPU_HW_INJ)
static void cortex_a53_edac_inject_L2(void)
{
	u64 l2actrl, l2ecc;

	l2ecc = read_l2ctlr_el1();
	if ((l2ecc & L2_ECC_PROTECTION)) {
		l2actrl = read_l2actrl_el1();
		l2actrl = l2actrl | L2_DCACHE_ERRINJ_ENABLE;
		write_l2actrl_el1(l2actrl);
		cortex_a53_edac_busy_on_inst();
	}
}

static void cortex_a53_edac_inject_L1(void)
{
	u64 l1actrl;

	l1actrl = read_l1actrl_el1();
	l1actrl |= L1_DCACHE_ERRINJ_ENABLE;
	write_l1actrl_el1(l1actrl);
	cortex_a53_edac_busy_on_inst();
}

static ssize_t cortex_a53_inj_L2_err_store(struct device *dev,
					   struct device_attribute *mattr,
					   const char *data, size_t count)
{
	struct platform_device *pdev = to_pdev(dev);

	dev_info(&pdev->dev, "Injecting L2 Cache Error ..\n");
	cortex_a53_edac_inject_L2();

	return count;
}

static ssize_t cortex_a53_inj_L1_err_store(struct device *dev,
					   struct device_attribute *mattr,
					   const char *data, size_t count)
{
	struct platform_device *pdev = to_pdev(dev);

	dev_info(&pdev->dev, "Injecting L1 Cache Error ..\n");
	cortex_a53_edac_inject_L1();

	return count;
}

static DEVICE_ATTR_WO(cortex_a53_inj_L1_err);
static DEVICE_ATTR_WO(cortex_a53_inj_L2_err);
#endif

#if defined(DEBUG_RAS_CPU) && defined(DEBUG_CPU_SW_INJ)
static ssize_t tst_cpu_edac_sw_inj_store(struct device *dev,
					 struct device_attribute *mattr,
					 const char *data, size_t count)
{
	struct platform_device *pdev = to_pdev(dev);
	struct cpu_edac_info_t *e_sts = NULL;
	int cache_id;

	if (strncmp(data, "L1", 2) == 0) {
		cache_id = 0;
	} else if (strncmp(data, "L2", 2) == 0) {
		cache_id = 1;
	} else {
		dev_err(&pdev->dev, "Invalid options chosen\n");
		dev_err(&pdev->dev, "L1, L2 are available\n");
		return -EINVAL;
	}

	dev_info(&pdev->dev, "Injecting SW Error on CPU Cache L1/L2\n");
	e_sts = &cpu_edac_info[cache_id];

	e_sts->cpu = 0x01 + cache_id;
	e_sts->repeat_err = 0x02 + cache_id;
	e_sts->other_err = 0x03 + cache_id;
	e_sts->index = 0x02 + cache_id;
	e_sts->way = 0x03 + cache_id;
	e_sts->ram_id = 0x08 + (cache_id << 3);

	return count;
}
static DEVICE_ATTR_WO(tst_cpu_edac_sw_inj);
#endif
static const struct attribute *cortex_a53_edac_attrs[] = {
#ifdef DEBUG_CPU_HW_INJ
	&dev_attr_cortex_a53_inj_L1_err.attr,
	&dev_attr_cortex_a53_inj_L2_err.attr,
#endif
#ifdef DEBUG_CPU_SW_INJ
	&dev_attr_tst_cpu_edac_sw_inj.attr,
#endif
	NULL,
};

#ifdef DEBUG_RAS_CPU
static const struct attribute_group cortex_a53_edac_attributes = {
	.attrs = (struct attribute **)cortex_a53_edac_attrs,
};
#endif

void xbay_cpu_edac_get_all_info(struct platform_device *pdev,
				char *buf, int count)
{
	cortex_a53_edac_check();
	memcpy(buf, (char *)cpu_edac_info, count);
}

void xbay_cpu_edac_clear_all_info(struct platform_device *pdev, int count)
{
	memset((char *)cpu_edac_info, 0, count);
}

int xbay_cortex_a53_edac_probe(struct platform_device *pdev,
			       struct device_node *np)
{
	int err;

	err = sysfs_create_group(&pdev->dev.kobj,
				 &cortex_a53_edac_attributes);
	if (err < 0) {
		dev_err(&pdev->dev,
			"Failed to create Cortex a53 EDAC sysfs entries\n");
		return err;
	}

	dev_info(&pdev->dev,
		 "ARM Cortex a53 EDAC module probed Successfully\n");

	return 0;
}
