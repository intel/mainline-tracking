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

#define PKS_TEST_MEM_SIZE (PAGE_SIZE)

/*
 * run_on_boot default '= false' which checkpatch complains about initializing;
 * so we don't
 */
static bool run_on_boot;
static struct dentry *pks_test_dentry;

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
	int pkey;
	char data[64];
};
static struct pks_test_ctx *test_exception_ctx;

static pte_t *walk_table(void *ptr)
{
	struct page *page = NULL;
	pgd_t *pgdp;
	p4d_t *p4dp;
	pud_t *pudp;
	pmd_t *pmdp;
	pte_t *ret = NULL;

	pgdp = pgd_offset_k((unsigned long)ptr);
	if (pgd_none(*pgdp) || pgd_bad(*pgdp))
		goto error;

	p4dp = p4d_offset(pgdp, (unsigned long)ptr);
	if (p4d_none(*p4dp) || p4d_bad(*p4dp))
		goto error;

	pudp = pud_offset(p4dp, (unsigned long)ptr);
	if (pud_none(*pudp) || pud_bad(*pudp))
		goto error;

	pmdp = pmd_offset(pudp, (unsigned long)ptr);
	if (pmd_none(*pmdp) || pmd_bad(*pmdp))
		goto error;

	ret = pte_offset_map(pmdp, (unsigned long)ptr);
	if (pte_present(*ret)) {
		page = pte_page(*ret);
		if (!page) {
			pte_unmap(ret);
			goto error;
		}
		pr_info("page 0x%lx; flags 0x%lx\n",
		       (unsigned long)page, page->flags);
	}

error:
	return ret;
}

static bool check_pkey_val(u32 pk_reg, int pkey, u32 expected)
{
	u32 pkey_shift = pkey * PKR_BITS_PER_PKEY;
	u32 pkey_mask = ((1 << PKR_BITS_PER_PKEY) - 1) << pkey_shift;

	pk_reg = (pk_reg & pkey_mask) >> pkey_shift;
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

static void check_exception(irqentry_state_t *irq_state)
{
	/* Check the thread saved state */
	if (!check_pkey_val(irq_state->pkrs, test_armed_key, PKEY_DISABLE_WRITE)) {
		pr_err("     FAIL: checking irq_state->pkrs\n");
		test_exception_ctx->pass = false;
	}

	/* Check the exception state */
	if (!check_pkrs(test_armed_key,
			PKEY_DISABLE_ACCESS | PKEY_DISABLE_WRITE)) {
		pr_err("     FAIL: PKRS cache and MSR\n");
		test_exception_ctx->pass = false;
	}

	/*
	 * Check we can update the value during exception without affecting the
	 * calling thread.  The calling thread is checked after exception...
	 */
	pks_update_protection(test_armed_key, 0, false);
	if (!check_pkrs(test_armed_key, 0)) {
		pr_err("     FAIL: exception did not change register to 0\n");
		test_exception_ctx->pass = false;
	}
	pks_update_protection(test_armed_key,
			      PKEY_DISABLE_ACCESS | PKEY_DISABLE_WRITE, false);
	if (!check_pkrs(test_armed_key, PKEY_DISABLE_ACCESS | PKEY_DISABLE_WRITE)) {
		pr_err("     FAIL: exception did not change register to 0x3\n");
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
bool pks_test_callback(void *irq_state)
{
	irqentry_state_t *state = (irqentry_state_t *)irq_state;
	bool armed = (test_armed_key != 0);

	if (test_exception_ctx) {
		check_exception(state);
		/*
		 * We stop this check within the exception because the
		 * fault handler clean up code will call us 2x while checking
		 * the PMD entry and we don't need to check this again
		 */
		test_exception_ctx = NULL;
	}

	if (armed) {
		/* Enable read and write to stop faults */
		state->pkrs = update_pkey_val(state->pkrs, test_armed_key, 0);
		fault_cnt++;
	}

	return armed;
}
EXPORT_SYMBOL(pks_test_callback);

static bool exception_caught(void)
{
	bool ret = (fault_cnt != prev_cnt);

	prev_cnt = fault_cnt;
	return ret;
}

static void report_pkey_settings(void *unused)
{
	u8 pkey;
	unsigned long long msr = 0;
	unsigned int cpu = smp_processor_id();

	rdmsrl(MSR_IA32_PKRS, msr);

	pr_info("for CPU %d : 0x%llx\n", cpu, msr);
	for (pkey = 0; pkey < PKS_NUM_KEYS; pkey++) {
		int ad, wd;

		ad = (msr >> (pkey * PKR_BITS_PER_PKEY)) & PKEY_DISABLE_ACCESS;
		wd = (msr >> (pkey * PKR_BITS_PER_PKEY)) & PKEY_DISABLE_WRITE;
		pr_info("   %u: A:%d W:%d\n", pkey, ad, wd);
	}
}

struct pks_access_test {
	int ad;
	int wd;
	bool write;
	bool exception;
};

static struct pks_access_test pkey_test_ary[] = {
	/*  disable both */
	{ PKEY_DISABLE_ACCESS, PKEY_DISABLE_WRITE,   true,  true },
	{ PKEY_DISABLE_ACCESS, PKEY_DISABLE_WRITE,   false, true },

	/*  enable both */
	{ 0, 0, true,  false },
	{ 0, 0, false, false },

	/*  enable read only */
	{ 0, PKEY_DISABLE_WRITE,  true,  true },
	{ 0, PKEY_DISABLE_WRITE,  false, false },
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
		pr_err("pkey test FAILED: ad %d; wd %d; write %s; exception %s != %s\n",
			test->ad, test->wd,
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
	pks_update_protection(ctx->pkey, test->ad | test->wd, false);

	return test_it(ctx, test, ptr);
}

static void *alloc_test_page(int pkey)
{
	return vmalloc_pks(PKS_TEST_MEM_SIZE, pkey);
}

static void test_mem_access(struct pks_test_ctx *ctx)
{
	int i, rc;
	u8 pkey;
	void *ptr = NULL;
	pte_t *ptep;

	ptr = alloc_test_page(ctx->pkey);
	if (!ptr) {
		pr_err("Failed to vmalloc page???\n");
		ctx->pass = false;
		return;
	}

	ptep = walk_table(ptr);
	if (!ptep) {
		pr_err("Failed to walk table???\n");
		ctx->pass = false;
		goto done;
	}

	pkey = pte_flags_pkey(ptep->pte);
	pr_info("ptep flags 0x%lx pkey %u\n",
	       (unsigned long)ptep->pte, pkey);

	if (pkey != ctx->pkey) {
		pr_err("invalid pkey found: %u, test_pkey: %u\n",
			pkey, ctx->pkey);
		ctx->pass = false;
		goto unmap;
	}

	if (!ctx->pks_cpu_enabled) {
		pr_err("not CPU enabled; skipping access tests...\n");
		ctx->pass = true;
		goto unmap;
	}

	for (i = 0; i < ARRAY_SIZE(pkey_test_ary); i++) {
		rc = run_access_test(ctx, &pkey_test_ary[i], ptr);

		/*  only save last error is fine */
		if (rc)
			ctx->pass = false;
	}

unmap:
	pte_unmap(ptep);
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
		on_each_cpu(report_pkey_settings, NULL, 1);

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

	pks_update_protection(ctx->pkey, PKEY_DISABLE_WRITE, false);

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

struct shared_data {
	struct mutex lock;
	struct pks_test_ctx *ctx;
	void *kmap_addr;
	struct pks_access_test *test;
};

static int thread_main(void *d)
{
	struct shared_data *data = d;
	struct pks_test_ctx *ctx = data->ctx;

	while (!kthread_should_stop()) {
		mutex_lock(&data->lock);
		/*
		 * wait for the main thread to hand us the page
		 * We should be spinning so hopefully we will not have gotten
		 * the global value from a schedule in.
		 */
		if (data->kmap_addr) {
			if (test_it(ctx, data->test, data->kmap_addr))
				ctx->pass = false;
			data->kmap_addr = NULL;
		}
		mutex_unlock(&data->lock);
	}

	return 0;
}

static void run_thread_access_test(struct shared_data *data,
				   struct pks_test_ctx *ctx,
				   struct pks_access_test *test,
				   void *ptr)
{
	pks_update_protection(ctx->pkey, test->ad | test->wd, true);

	pr_info("checking... ad %d; wd %d; write %s\n",
			test->ad, test->wd, test->write ? "TRUE" : "FALSE");

	mutex_lock(&data->lock);
	data->test = test;
	data->kmap_addr = ptr;
	mutex_unlock(&data->lock);

	while (data->kmap_addr) {
		msleep(10);
	}
}

static void run_global_test(void)
{
	struct task_struct *other_task;
	struct pks_test_ctx *ctx;
	struct shared_data data;
	bool pass = true;
	void *ptr;
	int i;

	pr_info("     ***** BEGIN: global pkey checking\n");

	/* Set up context, data pgae, and thread */
	ctx = alloc_ctx("global pkey test");
	if (IS_ERR(ctx)) {
		pr_err("     FAIL: no context\n");
		pass = false;
		goto result;
	}
	ptr = alloc_test_page(ctx->pkey);
	if (!ptr) {
		pr_err("     FAIL: no vmalloc page\n");
		pass = false;
		goto free_context;
	}
	other_task = kthread_run(thread_main, &data, "PKRS global test");
	if (IS_ERR(other_task)) {
		pr_err("     FAIL: Failed to start thread\n");
		pass = false;
		goto free_page;
	}

	memset(&data, 0, sizeof(data));
	mutex_init(&data.lock);
	data.ctx = ctx;

	/* Start testing */
	ctx->pass = true;

	for (i = 0; i < ARRAY_SIZE(pkey_test_ary); i++) {
		run_thread_access_test(&data, ctx, &pkey_test_ary[i], ptr);
	}

	kthread_stop(other_task);
	pass = ctx->pass;

free_page:
	vfree(ptr);
free_context:
	free_ctx(ctx);
result:
	pr_info("     ***** END: global pkey checking : %s\n",
		 pass ? "PASS" : "FAIL");
}

static void run_all(void)
{
	struct pks_test_ctx *ctx[PKS_NUM_KEYS];
	static char name[PKS_NUM_KEYS][64];
	int i;

	for (i = 1; i < PKS_NUM_KEYS; i++) {
		sprintf(name[i], "pks ctx %d", i);
		ctx[i] = alloc_ctx((const char *)name[i]);
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

	run_global_test();
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
	 * Test "3" will test allocating all keys. Do it first without
	 * using "ctx".
	 */
	if (!strcmp(buf, "3"))
		run_all();

	if (!ctx) {
		ctx = alloc_ctx("pks test");
		if (IS_ERR(ctx))
			return -ENOMEM;
		file->private_data = ctx;
	}

	if (!strcmp(buf, "0"))
		pks_run_test(ctx);

	/* start of context switch test */
	if (!strcmp(buf, "1")) {
		/* Ensure a known state to test context switch */
		pks_update_protection(ctx->pkey,
				      PKEY_DISABLE_ACCESS | PKEY_DISABLE_WRITE,
				      false);
	}

	/* After context switch msr should be restored */
	if (!strcmp(buf, "2") && ctx->pks_cpu_enabled) {
		unsigned long reg_pkrs;
		int access;

		rdmsrl(MSR_IA32_PKRS, reg_pkrs);

		access = (reg_pkrs >> (ctx->pkey * PKR_BITS_PER_PKEY)) &
			  PKEY_ACCESS_MASK;
		if (access != (PKEY_DISABLE_ACCESS | PKEY_DISABLE_WRITE)) {
			ctx->pass = false;
			pr_err("Context switch check failed\n");
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
			run_all();

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
module_exit(pks_test_exit);

MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("GPL v2");
