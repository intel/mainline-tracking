// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright(c) 2020 Intel Corporation. All rights reserved.
 *
 * Implement PKS testing
 * Access to run this test can be with a command line parameter
 * ("pks-test-on-boot") or more detailed tests can be triggered through:
 *
 *    /sys/kernel/debug/x86/run_pks
 *
 *  debugfs controls are:
 *
 *  '0' -- Run access tests with a single pkey
 *
 *  '1' -- Set up the pkey register with no access for the pkey allocated to
 *         this fd
 *  '2' -- Check that the pkey register updated in '1' is still the same.  (To
 *         be used after a forced context switch.)
 *
 *  '3' -- Allocate all pkeys possible and run tests on each pkey allocated.
 *         DEFAULT when run at boot.
 *
 *  Closing the fd will cleanup and release the pkey.
 *
 *  A companion user space program is provided in:
 *
 *          .../tools/testing/selftests/x86/test_pks.c
 *
 *  which will better test the context switching.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/entry-common.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/mman.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/percpu-defs.h>
#include <linux/pgtable.h>
#include <linux/pkeys.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include <asm/ptrace.h>       /* for struct pt_regs */
#include <asm/pkeys_common.h>
#include <asm/processor.h>
#include <asm/pks.h>

#define PKS_TEST_MEM_SIZE (PAGE_SIZE)

#define RUN_ALLOCATE            "0"
#define ARM_CTX_SWITCH          "1"
#define CHECK_CTX_SWITCH        "2"
#define RUN_ALLOCATE_ALL        "3"
#define RUN_ALLOCATE_DEBUG      "4"
#define RUN_ALLOCATE_ALL_DEBUG  "5"
#define RUN_CRASH_TEST          "9"

DECLARE_PER_CPU(u32, pkrs_cache);

/*
 * run_on_boot default '= false' which checkpatch complains about initializing;
 * so we don't
 */
static bool run_on_boot;
static struct dentry *pks_test_dentry;
static bool run_9;

/*
 * We must lock the following globals for brief periods while the fault handler
 * checks/updates them.
 */
static DEFINE_SPINLOCK(test_lock);
static int test_armed_key;
static unsigned long prev_cnt;
static unsigned long fault_cnt;

struct pks_test_ctx {
	bool pass;
	bool pks_cpu_enabled;
	bool debug;
	int pkey;
	char data[64];
};
static struct pks_test_ctx *test_exception_ctx;

static bool check_pkey_val(u32 pk_reg, int pkey, u32 expected)
{
	pk_reg = (pk_reg & PKR_PKEY_MASK(pkey)) >> PKR_PKEY_SHIFT(pkey);
	return (pk_reg == expected);
}

/*
 * Check if the register @pkey value matches @expected value
 *
 * Both the cached and actual MSR must match.
 */
static bool check_pkrs(int pkey, u32 expected)
{
	bool ret = true;
	u64 pkrs;
	u32 *tmp_cache;

	tmp_cache = get_cpu_ptr(&pkrs_cache);
	if (!check_pkey_val(*tmp_cache, pkey, expected))
		ret = false;
	put_cpu_ptr(tmp_cache);

	rdmsrl(MSR_IA32_PKRS, pkrs);
	if (!check_pkey_val(pkrs, pkey, expected))
		ret = false;

	return ret;
}

static void check_exception(u32 thread_pkrs)
{
	/* Check the thread saved state */
	if (!check_pkey_val(thread_pkrs, test_armed_key, PKEY_DISABLE_WRITE)) {
		pr_err("     FAIL: checking ept_regs->thread_pkrs\n");
		test_exception_ctx->pass = false;
	}

	/* Check the exception state */
	if (!check_pkrs(test_armed_key, PKEY_DISABLE_ACCESS)) {
		pr_err("     FAIL: PKRS cache and MSR\n");
		test_exception_ctx->pass = false;
	}

	/*
	 * Check we can update the value during exception without affecting the
	 * calling thread.  The calling thread is checked after exception...
	 */
	pks_mk_readwrite(test_armed_key);
	if (!check_pkrs(test_armed_key, 0)) {
		pr_err("     FAIL: exception did not change register to 0\n");
		test_exception_ctx->pass = false;
	}
	pks_mk_noaccess(test_armed_key);
	if (!check_pkrs(test_armed_key, PKEY_DISABLE_ACCESS)) {
		pr_err("     FAIL: exception did not change register to 0x%x\n",
			PKEY_DISABLE_ACCESS);
		test_exception_ctx->pass = false;
	}
}

/**
 * pks_test_callback() is exported so that the fault handler can detect
 * and report back status of intentional faults.
 *
 * NOTE: It clears the protection key from the page such that the fault handler
 * will not re-trigger.
 */
bool pks_test_callback(struct pt_regs *regs)
{
	struct extended_pt_regs *ept_regs = extended_pt_regs(regs);
	bool armed = (test_armed_key != 0);

	if (test_exception_ctx) {
		check_exception(ept_regs->thread_pkrs);
		/*
		 * We stop this check within the exception because the
		 * fault handler clean up code will call us 2x while checking
		 * the PMD entry and we don't need to check this again
		 */
		test_exception_ctx = NULL;
	}

	if (armed) {
		/* Enable read and write to stop faults */
		ept_regs->thread_pkrs = update_pkey_val(ept_regs->thread_pkrs,
							test_armed_key, 0);
		fault_cnt++;
	}

	return armed;
}

static bool exception_caught(void)
{
	bool ret = (fault_cnt != prev_cnt);

	prev_cnt = fault_cnt;
	return ret;
}

static void report_pkey_settings(void *info)
{
	u8 pkey;
	unsigned long long msr = 0;
	unsigned int cpu = smp_processor_id();
	struct pks_test_ctx *ctx = info;

	rdmsrl(MSR_IA32_PKRS, msr);

	pr_info("for CPU %d : 0x%llx\n", cpu, msr);

	if (ctx->debug) {
		for (pkey = 0; pkey < PKS_NUM_KEYS; pkey++) {
			int ad, wd;

			ad = (msr >> PKR_PKEY_SHIFT(pkey)) & PKEY_DISABLE_ACCESS;
			wd = (msr >> PKR_PKEY_SHIFT(pkey)) & PKEY_DISABLE_WRITE;
			pr_info("   %u: A:%d W:%d\n", pkey, ad, wd);
		}
	}
}

enum pks_access_mode {
	PKS_TEST_NO_ACCESS,
	PKS_TEST_RDWR,
	PKS_TEST_RDONLY
};

static char *get_mode_str(enum pks_access_mode mode)
{
	switch (mode) {
	case PKS_TEST_NO_ACCESS:
		return "No Access";
	case PKS_TEST_RDWR:
		return "Read Write";
	case PKS_TEST_RDONLY:
		return "Read Only";
	default:
		pr_err("BUG in test invalid mode\n");
		break;
	}

	return "";
}

struct pks_access_test {
	enum pks_access_mode mode;
	bool write;
	bool exception;
};

static struct pks_access_test pkey_test_ary[] = {
	/*  disable both */
	{ PKS_TEST_NO_ACCESS, true,  true },
	{ PKS_TEST_NO_ACCESS, false, true },

	/*  enable both */
	{ PKS_TEST_RDWR, true,  false },
	{ PKS_TEST_RDWR, false, false },

	/*  enable read only */
	{ PKS_TEST_RDONLY, true,  true },
	{ PKS_TEST_RDONLY, false, false },
};

static int test_it(struct pks_test_ctx *ctx, struct pks_access_test *test, void *ptr)
{
	bool exception;
	int ret = 0;

	spin_lock(&test_lock);
	WRITE_ONCE(test_armed_key, ctx->pkey);

	if (test->write)
		memcpy(ptr, ctx->data, 8);
	else
		memcpy(ctx->data, ptr, 8);

	exception = exception_caught();

	WRITE_ONCE(test_armed_key, 0);
	spin_unlock(&test_lock);

	if (test->exception != exception) {
		pr_err("pkey test FAILED: mode %s; write %s; exception %s != %s\n",
			get_mode_str(test->mode),
			test->write ? "TRUE" : "FALSE",
			test->exception ? "TRUE" : "FALSE",
			exception ? "TRUE" : "FALSE");
		ret = -EFAULT;
	}

	return ret;
}

static int run_access_test(struct pks_test_ctx *ctx,
			   struct pks_access_test *test,
			   void *ptr)
{
	switch (test->mode) {
	case PKS_TEST_NO_ACCESS:
		pks_mk_noaccess(ctx->pkey);
		break;
	case PKS_TEST_RDWR:
		pks_mk_readwrite(ctx->pkey);
		break;
	case PKS_TEST_RDONLY:
		pks_mk_readonly(ctx->pkey);
		break;
	default:
		pr_err("BUG in test invalid mode\n");
		break;
	}

	return test_it(ctx, test, ptr);
}

static void *alloc_test_page(int pkey)
{
	return __vmalloc_node_range(PKS_TEST_MEM_SIZE, 1, VMALLOC_START, VMALLOC_END,
				    GFP_KERNEL, PAGE_KERNEL_PKEY(pkey), 0,
				    NUMA_NO_NODE, __builtin_return_address(0));
}

static void test_mem_access(struct pks_test_ctx *ctx)
{
	int i, rc;
	u8 pkey;
	void *ptr = NULL;
	pte_t *ptep = NULL;
	spinlock_t *ptl;

	ptr = alloc_test_page(ctx->pkey);
	if (!ptr) {
		pr_err("Failed to vmalloc page???\n");
		ctx->pass = false;
		return;
	}

	if (follow_pte(&init_mm, (unsigned long)ptr, &ptep, &ptl)) {
		pr_err("Failed to walk table???\n");
		ctx->pass = false;
		goto done;
	}

	pkey = pte_flags_pkey(ptep->pte);
	pr_info("ptep flags 0x%lx pkey %u\n",
		(unsigned long)ptep->pte, pkey);
	pte_unmap_unlock(ptep, ptl);

	if (pkey != ctx->pkey) {
		pr_err("invalid pkey found: %u, test_pkey: %u\n",
			pkey, ctx->pkey);
		ctx->pass = false;
		goto done;
	}

	if (!ctx->pks_cpu_enabled) {
		pr_err("not CPU enabled; skipping access tests...\n");
		ctx->pass = true;
		goto done;
	}

	for (i = 0; i < ARRAY_SIZE(pkey_test_ary); i++) {
		rc = run_access_test(ctx, &pkey_test_ary[i], ptr);

		/*  only save last error is fine */
		if (rc)
			ctx->pass = false;
	}

done:
	vfree(ptr);
}

static void pks_run_test(struct pks_test_ctx *ctx)
{
	ctx->pass = true;

	pr_info("\n");
	pr_info("\n");
	pr_info("     ***** BEGIN: Testing (CPU enabled : %s) *****\n",
		ctx->pks_cpu_enabled ? "TRUE" : "FALSE");

	if (ctx->pks_cpu_enabled)
		on_each_cpu(report_pkey_settings, ctx, 1);

	pr_info("           BEGIN: pkey %d Testing\n", ctx->pkey);
	test_mem_access(ctx);
	pr_info("           END: PAGE_KERNEL_PKEY Testing : %s\n",
		ctx->pass ? "PASS" : "FAIL");

	pr_info("     ***** END: Testing *****\n");
	pr_info("\n");
	pr_info("\n");
}

static ssize_t pks_read_file(struct file *file, char __user *user_buf,
			     size_t count, loff_t *ppos)
{
	struct pks_test_ctx *ctx = file->private_data;
	char buf[32];
	unsigned int len;

	if (!ctx)
		len = sprintf(buf, "not run\n");
	else
		len = sprintf(buf, "%s\n", ctx->pass ? "PASS" : "FAIL");

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static struct pks_test_ctx *alloc_ctx(const char *name)
{
	struct pks_test_ctx *ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);

	if (!ctx) {
		pr_err("Failed to allocate memory for test context\n");
		return ERR_PTR(-ENOMEM);
	}

	ctx->pkey = pks_key_alloc(name);
	if (ctx->pkey <= 0) {
		pr_err("Failed to allocate memory for test context\n");
		kfree(ctx);
		return ERR_PTR(-ENOMEM);
	}

	ctx->pks_cpu_enabled = cpu_feature_enabled(X86_FEATURE_PKS);
	sprintf(ctx->data, "%s", "DEADBEEF");
	return ctx;
}

static void free_ctx(struct pks_test_ctx *ctx)
{
	pks_key_free(ctx->pkey);
	kfree(ctx);
}

static void run_exception_test(void)
{
	void *ptr = NULL;
	bool pass = true;
	struct pks_test_ctx *ctx;

	pr_info("     ***** BEGIN: exception checking\n");

	ctx = alloc_ctx("Exception test");
	if (IS_ERR(ctx)) {
		pr_err("     FAIL: no context\n");
		pass = false;
		goto result;
	}
	ctx->pass = true;

	ptr = alloc_test_page(ctx->pkey);
	if (!ptr) {
		pr_err("     FAIL: no vmalloc page\n");
		pass = false;
		goto free_context;
	}

	pks_mk_readonly(ctx->pkey);

	spin_lock(&test_lock);
	WRITE_ONCE(test_exception_ctx, ctx);
	WRITE_ONCE(test_armed_key, ctx->pkey);

	memcpy(ptr, ctx->data, 8);

	if (!exception_caught()) {
		pr_err("     FAIL: did not get an exception\n");
		pass = false;
	}

	/*
	 * NOTE The exception code has to enable access (b00) to keep the
	 * fault from looping forever.  So we don't see the write disabled
	 * restored but rather full access restored.  Also note that as part
	 * of this test the exception callback attempted to disable access
	 * completely (b11) and so we ensure that we are seeing the proper
	 * thread value restored here.
	 */
	if (!check_pkrs(test_armed_key, 0)) {
		pr_err("     FAIL: PKRS not restored\n");
		pass = false;
	}

	if (!ctx->pass)
		pass = false;

	WRITE_ONCE(test_armed_key, 0);
	spin_unlock(&test_lock);

	vfree(ptr);
free_context:
	free_ctx(ctx);
result:
	pr_info("     ***** END: exception checking : %s\n",
		 pass ? "PASS" : "FAIL");
}

static void run_all(bool debug)
{
	struct pks_test_ctx *ctx[PKS_NUM_KEYS];
	static char name[PKS_NUM_KEYS][64];
	int i;

	for (i = 1; i < PKS_NUM_KEYS; i++) {
		sprintf(name[i], "pks ctx %d", i);
		ctx[i] = alloc_ctx((const char *)name[i]);
		if (!IS_ERR(ctx[i]))
			ctx[i]->debug = debug;
	}

	for (i = 1; i < PKS_NUM_KEYS; i++) {
		if (!IS_ERR(ctx[i]))
			pks_run_test(ctx[i]);
	}

	for (i = 1; i < PKS_NUM_KEYS; i++) {
		if (!IS_ERR(ctx[i]))
			free_ctx(ctx[i]);
	}

	run_exception_test();
}

static void crash_it(void)
{
	struct pks_test_ctx *ctx;
	void *ptr;

	pr_warn("     ***** BEGIN: Unhandled fault test *****\n");

	ctx = alloc_ctx("crashing kernel\n");
	if (IS_ERR(ctx)) {
		pr_err("Failed to allocate context???\n");
		return;
	}

	ptr = alloc_test_page(ctx->pkey);
	if (!ptr) {
		pr_err("Failed to vmalloc page???\n");
		ctx->pass = false;
		return;
	}

	pks_mk_noaccess(ctx->pkey);

	spin_lock(&test_lock);
	WRITE_ONCE(test_armed_key, 0);
	/* This purposely faults */
	memcpy(ptr, ctx->data, 8);
	spin_unlock(&test_lock);

	vfree(ptr);
	free_ctx(ctx);
}

static ssize_t pks_write_file(struct file *file, const char __user *user_buf,
			      size_t count, loff_t *ppos)
{
	char buf[2];
	struct pks_test_ctx *ctx = file->private_data;

	if (copy_from_user(buf, user_buf, 1))
		return -EFAULT;
	buf[1] = '\0';

	/*
	 * WARNING: Test "9" will crash the kernel.
	 *
	 * So we arm the test and print a warning.  A second "9" will run the
	 * test.
	 */
	if (!strcmp(buf, RUN_CRASH_TEST)) {
		if (run_9) {
			crash_it();
			run_9 = false;
		} else {
			pr_warn("CAUTION: Test 9 will crash in the kernel.\n");
			pr_warn("         Specify 9 a second time to run\n");
			pr_warn("         run any other test to clear\n");
			run_9 = true;
		}
	} else {
		run_9 = false;
	}

	/*
	 * Test "3" will test allocating all keys. Do it first without
	 * using "ctx".
	 */
	if (!strcmp(buf, RUN_ALLOCATE_ALL))
		run_all(false);
	if (!strcmp(buf, RUN_ALLOCATE_ALL_DEBUG))
		run_all(true);

	/*
	 * This context is only required if the file is held open for the below
	 * tests.  Otherwise the context just get's freed in pks_release_file.
	 */
	if (!ctx) {
		ctx = alloc_ctx("pks test");
		if (IS_ERR(ctx))
			return -ENOMEM;
		file->private_data = ctx;
	}

	if (!strcmp(buf, RUN_ALLOCATE)) {
		ctx->debug = false;
		pks_run_test(ctx);
	}
	if (!strcmp(buf, RUN_ALLOCATE_DEBUG)) {
		ctx->debug = true;
		pks_run_test(ctx);
	}

	/* start of context switch test */
	if (!strcmp(buf, ARM_CTX_SWITCH)) {
		unsigned long reg_pkrs;
		int access;

		/* Ensure a known state to test context switch */
		pks_mk_readwrite(ctx->pkey);

		rdmsrl(MSR_IA32_PKRS, reg_pkrs);

		access = (reg_pkrs >> PKR_PKEY_SHIFT(ctx->pkey)) &
			  PKEY_ACCESS_MASK;
		pr_info("Context switch armed : pkey %d: 0x%x reg: 0x%lx\n",
			ctx->pkey, access, reg_pkrs);
	}

	/* After context switch msr should be restored */
	if (!strcmp(buf, CHECK_CTX_SWITCH) && ctx->pks_cpu_enabled) {
		unsigned long reg_pkrs;
		int access;

		rdmsrl(MSR_IA32_PKRS, reg_pkrs);

		access = (reg_pkrs >> PKR_PKEY_SHIFT(ctx->pkey)) &
			  PKEY_ACCESS_MASK;
		if (access != 0) {
			ctx->pass = false;
			pr_err("Context switch check failed: pkey %d: 0x%x reg: 0x%lx\n",
				ctx->pkey, access, reg_pkrs);
		} else {
			pr_err("Context switch check passed: pkey %d: 0x%x reg: 0x%lx\n",
				ctx->pkey, access, reg_pkrs);
		}
	}

	return count;
}

static int pks_release_file(struct inode *inode, struct file *file)
{
	struct pks_test_ctx *ctx = file->private_data;

	if (!ctx)
		return 0;

	free_ctx(ctx);
	return 0;
}

static const struct file_operations fops_init_pks = {
	.read = pks_read_file,
	.write = pks_write_file,
	.llseek = default_llseek,
	.release = pks_release_file,
};

static int __init parse_pks_test_options(char *str)
{
	run_on_boot = true;

	return 0;
}
early_param("pks-test-on-boot", parse_pks_test_options);

static int __init pks_test_init(void)
{
	if (cpu_feature_enabled(X86_FEATURE_PKS)) {
		if (run_on_boot)
			run_all(true);

		pks_test_dentry = debugfs_create_file("run_pks", 0600, arch_debugfs_dir,
						      NULL, &fops_init_pks);
	}

	return 0;
}
late_initcall(pks_test_init);

static void __exit pks_test_exit(void)
{
	debugfs_remove(pks_test_dentry);
	pr_info("test exit\n");
}
