// SPDX-License-Identifier: GPL-2.0
/*  Copyright(c) 2016-20 Intel Corporation. */

#include <elf.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/auxv.h>
#include "defines.h"
#include "../kselftest_harness.h"
#include "main.h"

static const uint64_t MAGIC = 0x1122334455667788ULL;
vdso_sgx_enter_enclave_t vdso_sgx_enter_enclave;

/*
 * Security Information (SECINFO) data structure needed by a few SGX
 * instructions (eg. ENCLU[EACCEPT] and ENCLU[EMODPE]) holds meta-data
 * about an enclave page. &enum sgx_secinfo_page_state specifies the
 * secinfo flags used for page state.
 */
enum sgx_secinfo_page_state {
	SGX_SECINFO_PENDING = (1 << 3),
	SGX_SECINFO_MODIFIED = (1 << 4),
	SGX_SECINFO_PR = (1 << 5),
};

struct vdso_symtab {
	Elf64_Sym *elf_symtab;
	const char *elf_symstrtab;
	Elf64_Word *elf_hashtab;
};

static Elf64_Dyn *vdso_get_dyntab(void *addr)
{
	Elf64_Ehdr *ehdr = addr;
	Elf64_Phdr *phdrtab = addr + ehdr->e_phoff;
	int i;

	for (i = 0; i < ehdr->e_phnum; i++)
		if (phdrtab[i].p_type == PT_DYNAMIC)
			return addr + phdrtab[i].p_offset;

	return NULL;
}

static void *vdso_get_dyn(void *addr, Elf64_Dyn *dyntab, Elf64_Sxword tag)
{
	int i;

	for (i = 0; dyntab[i].d_tag != DT_NULL; i++)
		if (dyntab[i].d_tag == tag)
			return addr + dyntab[i].d_un.d_ptr;

	return NULL;
}

static bool vdso_get_symtab(void *addr, struct vdso_symtab *symtab)
{
	Elf64_Dyn *dyntab = vdso_get_dyntab(addr);

	symtab->elf_symtab = vdso_get_dyn(addr, dyntab, DT_SYMTAB);
	if (!symtab->elf_symtab)
		return false;

	symtab->elf_symstrtab = vdso_get_dyn(addr, dyntab, DT_STRTAB);
	if (!symtab->elf_symstrtab)
		return false;

	symtab->elf_hashtab = vdso_get_dyn(addr, dyntab, DT_HASH);
	if (!symtab->elf_hashtab)
		return false;

	return true;
}

static inline void __cpuid(unsigned int *eax, unsigned int *ebx,
			   unsigned int *ecx, unsigned int *edx)
{
	asm volatile("cpuid"
	    : "=a" (*eax),
	      "=b" (*ebx),
	      "=c" (*ecx),
	      "=d" (*edx)
	    : "0" (*eax), "2" (*ecx)
	    : "memory");
}

static inline int sgx2_supported(void)
{
	unsigned int eax, ebx, ecx, edx;

	eax = SGX_CPUID;
	ecx = 0x0;

	__cpuid(&eax, &ebx, &ecx, &edx);

	return eax & 0x2;
}

static unsigned long elf_sym_hash(const char *name)
{
	unsigned long h = 0, high;

	while (*name) {
		h = (h << 4) + *name++;
		high = h & 0xf0000000;

		if (high)
			h ^= high >> 24;

		h &= ~high;
	}

	return h;
}

static Elf64_Sym *vdso_symtab_get(struct vdso_symtab *symtab, const char *name)
{
	Elf64_Word bucketnum = symtab->elf_hashtab[0];
	Elf64_Word *buckettab = &symtab->elf_hashtab[2];
	Elf64_Word *chaintab = &symtab->elf_hashtab[2 + bucketnum];
	Elf64_Sym *sym;
	Elf64_Word i;

	for (i = buckettab[elf_sym_hash(name) % bucketnum]; i != STN_UNDEF;
	     i = chaintab[i]) {
		sym = &symtab->elf_symtab[i];
		if (!strcmp(name, &symtab->elf_symstrtab[sym->st_name]))
			return sym;
	}

	return NULL;
}

/*
 * Return the offset in the enclave where the TCS segment can be found.
 * The first RW segment loaded is the TCS.
 */
static off_t encl_get_tcs_offset(struct encl *encl)
{
	int i;

	for (i = 0; i < encl->nr_segments; i++) {
		struct encl_segment *seg = &encl->segment_tbl[i];

		if (i == 0 && seg->prot == (PROT_READ | PROT_WRITE))
			return seg->offset;
	}

	return -1;
}

/*
 * Return the offset in the enclave where the data segment can be found.
 * The first RW segment loaded is the TCS, skip that to get info on the
 * data segment.
 */
static off_t encl_get_data_offset(struct encl *encl)
{
	int i;

	for (i = 1; i < encl->nr_segments; i++) {
		struct encl_segment *seg = &encl->segment_tbl[i];

		if (seg->prot == (PROT_READ | PROT_WRITE))
			return seg->offset;
	}

	return -1;
}

FIXTURE(enclave) {
	struct encl encl;
	struct sgx_enclave_run run;
};

FIXTURE_SETUP(enclave)
{
	Elf64_Sym *sgx_enter_enclave_sym = NULL;
	struct vdso_symtab symtab;
	struct encl_segment *seg;
	char maps_line[256];
	FILE *maps_file;
	unsigned int i;
	void *addr;

	if (!encl_load("test_encl.elf", &self->encl)) {
		encl_delete(&self->encl);
		ksft_exit_skip("cannot load enclaves\n");
	}

	for (i = 0; i < self->encl.nr_segments; i++) {
		seg = &self->encl.segment_tbl[i];

		TH_LOG("0x%016lx 0x%016lx 0x%02x", seg->offset, seg->size, seg->prot);
	}

	if (!encl_measure(&self->encl))
		goto err;

	if (!encl_build(&self->encl))
		goto err;

	/*
	 * An enclave consumer only must do this.
	 */
	for (i = 0; i < self->encl.nr_segments; i++) {
		struct encl_segment *seg = &self->encl.segment_tbl[i];

		addr = mmap((void *)self->encl.encl_base + seg->offset, seg->size,
			    seg->prot, MAP_SHARED | MAP_FIXED, self->encl.fd, 0);
		EXPECT_NE(addr, MAP_FAILED);
		if (addr == MAP_FAILED)
			goto err;
	}

	/* Get vDSO base address */
	addr = (void *)getauxval(AT_SYSINFO_EHDR);
	if (!addr)
		goto err;

	if (!vdso_get_symtab(addr, &symtab))
		goto err;

	sgx_enter_enclave_sym = vdso_symtab_get(&symtab, "__vdso_sgx_enter_enclave");
	if (!sgx_enter_enclave_sym)
		goto err;

	vdso_sgx_enter_enclave = addr + sgx_enter_enclave_sym->st_value;

	memset(&self->run, 0, sizeof(self->run));
	self->run.tcs = self->encl.encl_base;

	maps_file = fopen("/proc/self/maps", "r");
	if (maps_file != NULL)  {
		while (fgets(maps_line, sizeof(maps_line), maps_file) != NULL) {
			maps_line[strlen(maps_line) - 1] = '\0';

			if (strstr(maps_line, "/dev/sgx_enclave"))
				TH_LOG("%s", maps_line);
		}

		fclose(maps_file);
	}

err:
	if (!sgx_enter_enclave_sym)
		encl_delete(&self->encl);

	ASSERT_NE(sgx_enter_enclave_sym, NULL);
}

FIXTURE_TEARDOWN(enclave)
{
	encl_delete(&self->encl);
}

#define ENCL_CALL(op, run, clobbered) \
	({ \
		int ret; \
		if ((clobbered)) \
			ret = vdso_sgx_enter_enclave((unsigned long)(op), 0, 0, \
						     EENTER, 0, 0, (run)); \
		else \
			ret = sgx_enter_enclave((void *)(op), NULL, 0, EENTER, NULL, NULL, \
						(run)); \
		ret; \
	})

#define EXPECT_EEXIT(run) \
	do { \
		EXPECT_EQ((run)->function, EEXIT); \
		if ((run)->function != EEXIT) \
			TH_LOG("0x%02x 0x%02x 0x%016llx", (run)->exception_vector, \
			       (run)->exception_error_code, (run)->exception_addr); \
	} while (0)

TEST_F(enclave, unclobbered_vdso)
{
	struct encl_op op;

	op.type = ENCL_OP_PUT;
	op.buffer = MAGIC;

	EXPECT_EQ(ENCL_CALL(&op, &self->run, false), 0);

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.user_data, 0);

	op.type = ENCL_OP_GET;
	op.buffer = 0;

	EXPECT_EQ(ENCL_CALL(&op, &self->run, false), 0);

	EXPECT_EQ(op.buffer, MAGIC);
	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.user_data, 0);
}

TEST_F(enclave, clobbered_vdso)
{
	struct encl_op op;

	op.type = ENCL_OP_PUT;
	op.buffer = MAGIC;

	EXPECT_EQ(ENCL_CALL(&op, &self->run, true), 0);

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.user_data, 0);

	op.type = ENCL_OP_GET;
	op.buffer = 0;

	EXPECT_EQ(ENCL_CALL(&op, &self->run, true), 0);

	EXPECT_EQ(op.buffer, MAGIC);
	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.user_data, 0);
}

static int test_handler(long rdi, long rsi, long rdx, long ursp, long r8, long r9,
			struct sgx_enclave_run *run)
{
	run->user_data = 0;

	return 0;
}

TEST_F(enclave, clobbered_vdso_and_user_function)
{
	struct encl_op op;

	self->run.user_handler = (__u64)test_handler;
	self->run.user_data = 0xdeadbeef;

	op.type = ENCL_OP_PUT;
	op.buffer = MAGIC;

	EXPECT_EQ(ENCL_CALL(&op, &self->run, true), 0);

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.user_data, 0);

	op.type = ENCL_OP_GET;
	op.buffer = 0;

	EXPECT_EQ(ENCL_CALL(&op, &self->run, true), 0);

	EXPECT_EQ(op.buffer, MAGIC);
	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.user_data, 0);
}

/*
 * Modifying permissions of TCS page should not be possible.
 */
TEST_F(enclave, tcs_permissions)
{
	struct sgx_page_modp ioc;
	int ret, errno_save;

	ASSERT_TRUE(setup_test_encl(ENCL_HEAP_SIZE_DEFAULT, &self->encl, _metadata));

	memset(&self->run, 0, sizeof(self->run));
	self->run.tcs = self->encl.encl_base;

	memset(&ioc, 0, sizeof(ioc));

	/*
	 * Ensure kernel supports needed ioctl and system supports needed
	 * commands.
	 */

	ret = ioctl(self->encl.fd, SGX_IOC_PAGE_MODP, &ioc);

	if (ret == -1) {
		if (errno == ENOTTY)
			SKIP(return, "Kernel does not support test SGX_IOC_PAGE_MODP ioctl");
		else if (errno == ENODEV)
			SKIP(return, "System does not support SGX2");
	}

	/*
	 * Invalid parameters were provided during sanity check,
	 * expect command to fail.
	 */
	EXPECT_EQ(ret, -1);

	/*
	 * Attempt to make TCS page read-only. This is not allowed and
	 * should be prevented by OS.
	 */
	ioc.offset = encl_get_tcs_offset(&self->encl);
	ioc.length = PAGE_SIZE;
	ioc.prot = PROT_READ;

	ret = ioctl(self->encl.fd, SGX_IOC_PAGE_MODP, &ioc);
	errno_save = ret == -1 ? errno : 0;

	EXPECT_EQ(ret, -1);
	EXPECT_EQ(errno_save, EINVAL);
	EXPECT_EQ(ioc.result, 0);
	EXPECT_EQ(ioc.count, 0);
}

/*
 * Enclave page permission test.
 *
 * Modify and restore enclave page's EPCM (enclave) permissions from
 * outside enclave (ENCLS[EMODPR] via OS) as well as from within enclave (via
 * ENCLU[EMODPE]). Kernel should ensure PTE permissions are the same as
 * the EPCM permissions so check for page fault if VMA allows access but
 * EPCM and PTE does not.
 */
TEST_F(enclave, epcm_permissions)
{
	struct encl_op_get_from_addr get_addr_op;
	struct encl_op_put_to_addr put_addr_op;
	struct encl_op_eaccept eaccept_op;
	struct encl_op_emodpe emodpe_op;
	unsigned long data_start;
	struct sgx_page_modp ioc;
	int ret, errno_save;

	ASSERT_TRUE(setup_test_encl(ENCL_HEAP_SIZE_DEFAULT, &self->encl, _metadata));

	memset(&self->run, 0, sizeof(self->run));
	self->run.tcs = self->encl.encl_base;

	/*
	 * Ensure kernel supports needed ioctl and system supports needed
	 * commands.
	 */
	memset(&ioc, 0, sizeof(ioc));

	ret = ioctl(self->encl.fd, SGX_IOC_PAGE_MODP, &ioc);

	if (ret == -1) {
		if (errno == ENOTTY)
			SKIP(return, "Kernel does not support test SGX_IOC_PAGE_MODP ioctl");
		else if (errno == ENODEV)
			SKIP(return, "System does not support SGX2");
	}

	/*
	 * Invalid parameters were provided during sanity check,
	 * expect command to fail.
	 */
	EXPECT_EQ(ret, -1);

	/*
	 * Page that will have its permissions changed is the second data
	 * page in the .data segment. This forms part of the local encl_buffer
	 * within the enclave.
	 *
	 * At start of test @data_start should have EPCM as well as PTE
	 * permissions of RW.
	 */

	data_start = self->encl.encl_base +
		     encl_get_data_offset(&self->encl) + PAGE_SIZE;

	/*
	 * Sanity check that page at @data_start is writable before making
	 * any changes to page permissions.
	 *
	 * Start by writing MAGIC to test page.
	 */
	put_addr_op.value = MAGIC;
	put_addr_op.addr = data_start;
	put_addr_op.header.type = ENCL_OP_PUT_TO_ADDRESS;

	EXPECT_EQ(ENCL_CALL(&put_addr_op, &self->run, true), 0);

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);

	/*
	 * Read memory that was just written to, confirming that
	 * page is writable.
	 */
	get_addr_op.value = 0;
	get_addr_op.addr = data_start;
	get_addr_op.header.type = ENCL_OP_GET_FROM_ADDRESS;

	EXPECT_EQ(ENCL_CALL(&get_addr_op, &self->run, true), 0);

	EXPECT_EQ(get_addr_op.value, MAGIC);
	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);

	/*
	 * Change EPCM permissions to read-only, PTE entry flushed by OS in
	 * the process.
	 */
	memset(&ioc, 0, sizeof(ioc));

	ioc.offset = encl_get_data_offset(&self->encl) + PAGE_SIZE;
	ioc.length = PAGE_SIZE;
	ioc.prot = PROT_READ;

	ret = ioctl(self->encl.fd, SGX_IOC_PAGE_MODP, &ioc);
	errno_save = ret == -1 ? errno : 0;

	EXPECT_EQ(ret, 0);
	EXPECT_EQ(errno_save, 0);
	EXPECT_EQ(ioc.result, 0);
	EXPECT_EQ(ioc.count, 4096);

	/*
	 * EPCM permissions changed from OS, need to EACCEPT from enclave.
	 */
	eaccept_op.epc_addr = data_start;
	eaccept_op.flags = PROT_READ | SGX_SECINFO_REG | SGX_SECINFO_PR;
	eaccept_op.ret = 0;
	eaccept_op.header.type = ENCL_OP_EACCEPT;

	EXPECT_EQ(ENCL_CALL(&eaccept_op, &self->run, true), 0);

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);
	EXPECT_EQ(eaccept_op.ret, 0);

	/*
	 * EPCM permissions of page is now read-only, expect #PF
	 * on PTE (not EPCM) when attempting to write to page from
	 * within enclave.
	 */
	put_addr_op.value = MAGIC2;

	EXPECT_EQ(ENCL_CALL(&put_addr_op, &self->run, true), 0);

	EXPECT_EQ(self->run.function, ERESUME);
	EXPECT_EQ(self->run.exception_vector, 14);
	EXPECT_EQ(self->run.exception_error_code, 0x7);
	EXPECT_EQ(self->run.exception_addr, data_start);

	self->run.exception_vector = 0;
	self->run.exception_error_code = 0;
	self->run.exception_addr = 0;

	/*
	 * Received AEX but cannot return to enclave at same entrypoint,
	 * need different TCS from where EPCM permission can be made writable
	 * again.
	 */
	self->run.tcs = self->encl.encl_base + PAGE_SIZE;

	/*
	 * Enter enclave at new TCS to change EPCM permissions to be
	 * writable again and thus fix the page fault that triggered the
	 * AEX.
	 */

	emodpe_op.epc_addr = data_start;
	emodpe_op.flags = PROT_READ | PROT_WRITE;
	emodpe_op.header.type = ENCL_OP_EMODPE;

	EXPECT_EQ(ENCL_CALL(&emodpe_op, &self->run, true), 0);

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);

	/*
	 * Attempt to return to main TCS to resume execution at faulting
	 * instruction, but PTE should still prevent writing to the page.
	 */
	self->run.tcs = self->encl.encl_base;

	EXPECT_EQ(vdso_sgx_enter_enclave((unsigned long)&put_addr_op, 0, 0,
					 ERESUME, 0, 0,
					 &self->run),
		  0);

	EXPECT_EQ(self->run.function, ERESUME);
	EXPECT_EQ(self->run.exception_vector, 14);
	EXPECT_EQ(self->run.exception_error_code, 0x7);
	EXPECT_EQ(self->run.exception_addr, data_start);

	self->run.exception_vector = 0;
	self->run.exception_error_code = 0;
	self->run.exception_addr = 0;
	/*
	 * Inform OS about new permissions to have PTEs match EPCM.
	 */
	memset(&ioc, 0, sizeof(ioc));

	ioc.offset = encl_get_data_offset(&self->encl) + PAGE_SIZE;
	ioc.length = PAGE_SIZE;
	ioc.prot = PROT_READ | PROT_WRITE;

	ret = ioctl(self->encl.fd, SGX_IOC_PAGE_MODP, &ioc);
	errno_save = ret == -1 ? errno : 0;

	EXPECT_EQ(ret, 0);
	EXPECT_EQ(errno_save, 0);
	EXPECT_EQ(ioc.result, 0);
	EXPECT_EQ(ioc.count, 4096);

	/*
	 * Wrong page permissions that caused original fault has
	 * now been fixed via EPCM permissions as well as PTE.
	 * Resume execution in main TCS to re-attempt the memory access.
	 */
	self->run.tcs = self->encl.encl_base;

	EXPECT_EQ(vdso_sgx_enter_enclave((unsigned long)&put_addr_op, 0, 0,
					 ERESUME, 0, 0,
					 &self->run),
		  0);

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);

	get_addr_op.value = 0;

	EXPECT_EQ(ENCL_CALL(&get_addr_op, &self->run, true), 0);

	EXPECT_EQ(get_addr_op.value, MAGIC2);
	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.user_data, 0);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);

	/*
	 * The SGX_IOC_PAGE_MODP runs ENCLS[EMODPR] that sets EPCM.PR even
	 * if permissions are not actually restricted. The previous memory
	 * access succeeding shows that the PR flag does not prevent
	 * access. Even so, include an ENCLU[EACCEPT] as reference
	 * implementation to ensure EPCM does not have a dangling PR bit set.
	 */

	eaccept_op.epc_addr = data_start;
	eaccept_op.flags = PROT_READ | PROT_WRITE | SGX_SECINFO_REG | SGX_SECINFO_PR;
	eaccept_op.ret = 0;
	eaccept_op.header.type = ENCL_OP_EACCEPT;

	EXPECT_EQ(ENCL_CALL(&eaccept_op, &self->run, true), 0);

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);
	EXPECT_EQ(eaccept_op.ret, 0);
}

/*
 * Test the addition of pages to an initialized enclave via writing to
 * a page belonging to the enclave's address space but was not added
 * during enclave creation.
 */
TEST_F(enclave, augment)
{
	struct encl_op_get_from_addr get_addr_op;
	struct encl_op_put_to_addr put_addr_op;
	struct encl_op_eaccept eaccept_op;
	size_t total_size = 0;
	void *addr;
	int i;

	if (!sgx2_supported())
		SKIP(return, "SGX2 not supported");

	ASSERT_TRUE(setup_test_encl(ENCL_HEAP_SIZE_DEFAULT, &self->encl, _metadata));

	memset(&self->run, 0, sizeof(self->run));
	self->run.tcs = self->encl.encl_base;

	for (i = 0; i < self->encl.nr_segments; i++) {
		struct encl_segment *seg = &self->encl.segment_tbl[i];

		total_size += seg->size;
	}

	/*
	 * Actual enclave size is expected to be larger than the loaded
	 * test enclave since enclave size must be a power of 2 in bytes
	 * and test_encl does not consume it all.
	 */
	EXPECT_LT(total_size + PAGE_SIZE, self->encl.encl_size);

	/*
	 * Create memory mapping for the page that will be added. New
	 * memory mapping is for one page right after all existing
	 * mappings.
	 */
	addr = mmap((void *)self->encl.encl_base + total_size, PAGE_SIZE,
		    PROT_READ | PROT_WRITE | PROT_EXEC,
		    MAP_SHARED | MAP_FIXED, self->encl.fd, 0);
	EXPECT_NE(addr, MAP_FAILED);

	self->run.exception_vector = 0;
	self->run.exception_error_code = 0;
	self->run.exception_addr = 0;

	/*
	 * Attempt to write to the new page from within enclave.
	 * Expected to fail since page is not (yet) part of the enclave.
	 * The first #PF will trigger the addition of the page to the
	 * enclave, but since the new page needs an EACCEPT from within the
	 * enclave before it can be used it would not be possible
	 * to successfully return to the failing instruction. This is the
	 * cause of the second #PF captured here having the SGX bit set,
	 * it is from hardware preventing the page from being used.
	 */
	put_addr_op.value = MAGIC;
	put_addr_op.addr = (unsigned long)addr;
	put_addr_op.header.type = ENCL_OP_PUT_TO_ADDRESS;

	EXPECT_EQ(ENCL_CALL(&put_addr_op, &self->run, true), 0);

	EXPECT_EQ(self->run.function, ERESUME);
	EXPECT_EQ(self->run.exception_vector, 14);
	EXPECT_EQ(self->run.exception_addr, (unsigned long)addr);

	if (self->run.exception_error_code == 0x6) {
		munmap(addr, PAGE_SIZE);
		SKIP(return, "Kernel does not support adding pages to initialized enclave");
	}

	EXPECT_EQ(self->run.exception_error_code, 0x8007);

	self->run.exception_vector = 0;
	self->run.exception_error_code = 0;
	self->run.exception_addr = 0;

	/* Handle AEX by running EACCEPT from new entry point. */
	self->run.tcs = self->encl.encl_base + PAGE_SIZE;

	eaccept_op.epc_addr = self->encl.encl_base + total_size;
	eaccept_op.flags = SGX_SECINFO_R | SGX_SECINFO_W | SGX_SECINFO_REG | SGX_SECINFO_PENDING;
	eaccept_op.ret = 0;
	eaccept_op.header.type = ENCL_OP_EACCEPT;

	EXPECT_EQ(ENCL_CALL(&eaccept_op, &self->run, true), 0);

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);
	EXPECT_EQ(eaccept_op.ret, 0);

	/* Can now return to main TCS to resume execution. */
	self->run.tcs = self->encl.encl_base;

	EXPECT_EQ(vdso_sgx_enter_enclave((unsigned long)&put_addr_op, 0, 0,
					 ERESUME, 0, 0,
					 &self->run),
		  0);

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);

	/*
	 * Read memory that was just written to, confirming that data
	 * previously written (MAGIC) is present. Only change two test
	 * parameters, rest are same as previous test.
	 */
	get_addr_op.value = 0;
	get_addr_op.addr = (unsigned long)addr;
	get_addr_op.header.type = ENCL_OP_GET_FROM_ADDRESS;

	EXPECT_EQ(ENCL_CALL(&get_addr_op, &self->run, true), 0);

	EXPECT_EQ(get_addr_op.value, MAGIC);
	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);

	munmap(addr, PAGE_SIZE);
}

/*
 * Test for the addition of pages to an initialized enclave via a
 * pre-emptive run of EACCEPT on page to be added.
 */
TEST_F(enclave, augment_via_eaccept)
{
	struct encl_op_get_from_addr get_addr_op;
	struct encl_op_put_to_addr put_addr_op;
	struct encl_op_eaccept eaccept_op;
	size_t total_size = 0;
	void *addr;
	int i;

	if (!sgx2_supported())
		SKIP(return, "SGX2 not supported");

	ASSERT_TRUE(setup_test_encl(ENCL_HEAP_SIZE_DEFAULT, &self->encl, _metadata));

	memset(&self->run, 0, sizeof(self->run));
	self->run.tcs = self->encl.encl_base;

	for (i = 0; i < self->encl.nr_segments; i++) {
		struct encl_segment *seg = &self->encl.segment_tbl[i];

		total_size += seg->size;
	}

	/*
	 * Actual enclave size is expected to be larger than the loaded
	 * test enclave since enclave size must be a power of 2 in bytes while
	 * test_encl does not consume it all.
	 */
	EXPECT_LT(total_size + PAGE_SIZE, self->encl.encl_size);

	/*
	 * mmap() a page at end of existing enclave to be used for dynamic
	 * EPC page.
	 */

	addr = mmap((void *)self->encl.encl_base + total_size, PAGE_SIZE,
		    PROT_READ | PROT_WRITE | PROT_EXEC, MAP_SHARED | MAP_FIXED,
		    self->encl.fd, 0);
	EXPECT_NE(addr, MAP_FAILED);

	self->run.exception_vector = 0;
	self->run.exception_error_code = 0;
	self->run.exception_addr = 0;

	/*
	 * Run EACCEPT on new page to trigger the #PF->EAUG->EACCEPT(again
	 * without a #PF). All should be transparent to userspace.
	 */
	eaccept_op.epc_addr = self->encl.encl_base + total_size;
	eaccept_op.flags = SGX_SECINFO_R | SGX_SECINFO_W | SGX_SECINFO_REG | SGX_SECINFO_PENDING;
	eaccept_op.ret = 0;
	eaccept_op.header.type = ENCL_OP_EACCEPT;

	EXPECT_EQ(ENCL_CALL(&eaccept_op, &self->run, true), 0);

	if (self->run.exception_vector == 14 &&
	    self->run.exception_error_code == 4 &&
	    self->run.exception_addr == self->encl.encl_base + total_size) {
		munmap(addr, PAGE_SIZE);
		SKIP(return, "Kernel does not support adding pages to initialized enclave");
	}

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);
	EXPECT_EQ(eaccept_op.ret, 0);

	/*
	 * New page should be accessible from within enclave - attempt to
	 * write to it.
	 */
	put_addr_op.value = MAGIC;
	put_addr_op.addr = (unsigned long)addr;
	put_addr_op.header.type = ENCL_OP_PUT_TO_ADDRESS;

	EXPECT_EQ(ENCL_CALL(&put_addr_op, &self->run, true), 0);

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);

	/*
	 * Read memory that was just written to, confirming that data
	 * previously written (MAGIC) is present. Only change two test
	 * parameters, rest are same as previous test.
	 */
	get_addr_op.value = 0;
	get_addr_op.addr = (unsigned long)addr;
	get_addr_op.header.type = ENCL_OP_GET_FROM_ADDRESS;

	EXPECT_EQ(ENCL_CALL(&get_addr_op, &self->run, true), 0);

	EXPECT_EQ(get_addr_op.value, MAGIC);
	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);

	munmap(addr, PAGE_SIZE);
}

/*
 * SGX2 page type modification test in two phases:
 * Phase 1:
 * Create a new TCS, consisting out of three new pages (stack page with regular
 * page type, SSA page with regular page type, and TCS page with TCS page
 * type) in an initialized enclave and run a simple workload within it.
 * Phase 2:
 * Remove the three pages added in phase 1, add a new regular page at the
 * same address that previously hosted the TCS page and verify that it can
 * be modified.
 */
TEST_F(enclave, tcs_create)
{
	struct encl_op_init_tcs_page init_tcs_page_op;
	struct encl_op_get_from_addr get_addr_op;
	struct encl_op_put_to_addr put_addr_op;
	struct encl_op_get_from_buf get_buf_op;
	struct encl_op_put_to_buf put_buf_op;
	void *addr, *tcs, *stack_end, *ssa;
	struct encl_op_eaccept eaccept_op;
	struct sgx_page_remove remove_ioc;
	struct sgx_page_modt modt_ioc;
	size_t total_size = 0;
	uint64_t val_64;
	int errno_save;
	int ret, i;

	ASSERT_TRUE(setup_test_encl(ENCL_HEAP_SIZE_DEFAULT, &self->encl,
				    _metadata));

	memset(&self->run, 0, sizeof(self->run));
	self->run.tcs = self->encl.encl_base;

	/*
	 * Hardware (SGX2) and OS support is needed for this test. Start
	 * with check that test has a chance of succeeding.
	 */
	memset(&modt_ioc, 0, sizeof(modt_ioc));
	ret = ioctl(self->encl.fd, SGX_IOC_PAGE_MODT, &modt_ioc);

	if (ret == -1) {
		if (errno == ENOTTY)
			SKIP(return, "Kernel does not support test SGX_IOC_PAGE_MODT ioctl");
		else if (errno == ENODEV)
			SKIP(return, "System does not support SGX2");
	}

	/*
	 * Invalid parameters were provided during sanity check,
	 * expect command to fail.
	 */
	EXPECT_EQ(ret, -1);

	/*
	 * Add three regular pages via EAUG: one will be the TCS stack, one
	 * will be the TCS SSA, and one will be the new TCS. The stack and
	 * SSA will remain as regular pages, the TCS page will need its
	 * type changed after populated with needed data.
	 */
	for (i = 0; i < self->encl.nr_segments; i++) {
		struct encl_segment *seg = &self->encl.segment_tbl[i];

		total_size += seg->size;
	}

	/*
	 * Actual enclave size is expected to be larger than the loaded
	 * test enclave since enclave size must be a power of 2 in bytes while
	 * test_encl does not consume it all.
	 */
	EXPECT_LT(total_size + 3 * PAGE_SIZE, self->encl.encl_size);

	/*
	 * mmap() three pages at end of existing enclave to be used for the
	 * three new pages.
	 */
	addr = mmap((void *)self->encl.encl_base + total_size, 3 * PAGE_SIZE,
		    PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED,
		    self->encl.fd, 0);
	EXPECT_NE(addr, MAP_FAILED);

	self->run.exception_vector = 0;
	self->run.exception_error_code = 0;
	self->run.exception_addr = 0;

	stack_end = (void *)self->encl.encl_base + total_size;
	tcs = (void *)self->encl.encl_base + total_size + PAGE_SIZE;
	ssa = (void *)self->encl.encl_base + total_size + 2 * PAGE_SIZE;

	/*
	 * Run EACCEPT on each new page to trigger the
	 * EACCEPT->(#PF)->EAUG->EACCEPT(again without a #PF) flow.
	 */

	eaccept_op.epc_addr = (unsigned long)stack_end;
	eaccept_op.flags = SGX_SECINFO_R | SGX_SECINFO_W | SGX_SECINFO_REG | SGX_SECINFO_PENDING;
	eaccept_op.ret = 0;
	eaccept_op.header.type = ENCL_OP_EACCEPT;

	EXPECT_EQ(ENCL_CALL(&eaccept_op, &self->run, true), 0);

	if (self->run.exception_vector == 14 &&
	    self->run.exception_error_code == 4 &&
	    self->run.exception_addr == (unsigned long)stack_end) {
		munmap(addr, 3 * PAGE_SIZE);
		SKIP(return, "Kernel does not support adding pages to initialized enclave");
	}

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);
	EXPECT_EQ(eaccept_op.ret, 0);

	eaccept_op.epc_addr = (unsigned long)ssa;

	EXPECT_EQ(ENCL_CALL(&eaccept_op, &self->run, true), 0);

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);
	EXPECT_EQ(eaccept_op.ret, 0);

	eaccept_op.epc_addr = (unsigned long)tcs;

	EXPECT_EQ(ENCL_CALL(&eaccept_op, &self->run, true), 0);

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);
	EXPECT_EQ(eaccept_op.ret, 0);

	/*
	 * Three new pages added to enclave. Now populate the TCS page with
	 * needed data. This should be done from within enclave. Provide
	 * the function that will do the actual data population with needed
	 * data.
	 */

	/*
	 * New TCS will use the "encl_dyn_entry" entrypoint that expects
	 * stack to begin in page before TCS page.
	 */
	val_64 = encl_get_entry(&self->encl, "encl_dyn_entry");
	EXPECT_NE(val_64, 0);

	init_tcs_page_op.tcs_page = (unsigned long)tcs;
	init_tcs_page_op.ssa = (unsigned long)total_size + 2 * PAGE_SIZE;
	init_tcs_page_op.entry = val_64;
	init_tcs_page_op.header.type = ENCL_OP_INIT_TCS_PAGE;

	EXPECT_EQ(ENCL_CALL(&init_tcs_page_op, &self->run, true), 0);

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);

	/* Change TCS page type to TCS. */
	memset(&modt_ioc, 0, sizeof(modt_ioc));

	modt_ioc.offset = total_size + PAGE_SIZE;
	modt_ioc.length = PAGE_SIZE;
	modt_ioc.type = SGX_PAGE_TYPE_TCS;

	ret = ioctl(self->encl.fd, SGX_IOC_PAGE_MODT, &modt_ioc);
	errno_save = ret == -1 ? errno : 0;

	EXPECT_EQ(ret, 0);
	EXPECT_EQ(errno_save, 0);
	EXPECT_EQ(modt_ioc.result, 0);
	EXPECT_EQ(modt_ioc.count, 4096);

	/* EACCEPT new TCS page from enclave. */
	eaccept_op.epc_addr = (unsigned long)tcs;
	eaccept_op.flags = SGX_SECINFO_TCS | SGX_SECINFO_MODIFIED;
	eaccept_op.ret = 0;
	eaccept_op.header.type = ENCL_OP_EACCEPT;

	EXPECT_EQ(ENCL_CALL(&eaccept_op, &self->run, true), 0);

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);
	EXPECT_EQ(eaccept_op.ret, 0);

	/* Run workload from new TCS. */
	self->run.tcs = (unsigned long)tcs;

	/*
	 * Simple workload to write to data buffer and read value back.
	 */
	put_buf_op.header.type = ENCL_OP_PUT_TO_BUFFER;
	put_buf_op.value = MAGIC;

	EXPECT_EQ(ENCL_CALL(&put_buf_op, &self->run, true), 0);

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);

	get_buf_op.header.type = ENCL_OP_GET_FROM_BUFFER;
	get_buf_op.value = 0;

	EXPECT_EQ(ENCL_CALL(&get_buf_op, &self->run, true), 0);

	EXPECT_EQ(get_buf_op.value, MAGIC);
	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);

	/*
	 * Phase 2 of test:
	 * Remove pages associated with new TCS, create a regular page
	 * where TCS page used to be and verify it can be used as a regular
	 * page.
	 */

	/* Start page removal by requesting change of page type to PT_TRIM. */
	memset(&modt_ioc, 0, sizeof(modt_ioc));

	modt_ioc.offset = total_size;
	modt_ioc.length = 3 * PAGE_SIZE;
	modt_ioc.type = SGX_PAGE_TYPE_TRIM;

	ret = ioctl(self->encl.fd, SGX_IOC_PAGE_MODT, &modt_ioc);
	errno_save = ret == -1 ? errno : 0;

	EXPECT_EQ(ret, 0);
	EXPECT_EQ(errno_save, 0);
	EXPECT_EQ(modt_ioc.result, 0);
	EXPECT_EQ(modt_ioc.count, 3 * PAGE_SIZE);

	/*
	 * Enter enclave via TCS #1 and approve page removal by sending
	 * EACCEPT for each of three removed pages.
	 */
	self->run.tcs = self->encl.encl_base;

	eaccept_op.epc_addr = (unsigned long)stack_end;
	eaccept_op.flags = SGX_SECINFO_TRIM | SGX_SECINFO_MODIFIED;
	eaccept_op.ret = 0;
	eaccept_op.header.type = ENCL_OP_EACCEPT;

	EXPECT_EQ(ENCL_CALL(&eaccept_op, &self->run, true), 0);

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);
	EXPECT_EQ(eaccept_op.ret, 0);

	eaccept_op.epc_addr = (unsigned long)tcs;
	eaccept_op.ret = 0;

	EXPECT_EQ(ENCL_CALL(&eaccept_op, &self->run, true), 0);

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);
	EXPECT_EQ(eaccept_op.ret, 0);

	eaccept_op.epc_addr = (unsigned long)ssa;
	eaccept_op.ret = 0;

	EXPECT_EQ(ENCL_CALL(&eaccept_op, &self->run, true), 0);

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);
	EXPECT_EQ(eaccept_op.ret, 0);

	/* Send final ioctl to complete page removal. */
	memset(&remove_ioc, 0, sizeof(remove_ioc));

	remove_ioc.offset = total_size;
	remove_ioc.length = 3 * PAGE_SIZE;

	ret = ioctl(self->encl.fd, SGX_IOC_PAGE_REMOVE, &remove_ioc);
	errno_save = ret == -1 ? errno : 0;

	EXPECT_EQ(ret, 0);
	EXPECT_EQ(errno_save, 0);
	EXPECT_EQ(remove_ioc.count, 3 * PAGE_SIZE);

	/*
	 * Enter enclave via TCS #1 and access location where TCS #3 was to
	 * trigger dynamic add of regular page at that location.
	 */
	eaccept_op.epc_addr = (unsigned long)tcs;
	eaccept_op.flags = SGX_SECINFO_R | SGX_SECINFO_W | SGX_SECINFO_REG | SGX_SECINFO_PENDING;
	eaccept_op.ret = 0;
	eaccept_op.header.type = ENCL_OP_EACCEPT;

	EXPECT_EQ(ENCL_CALL(&eaccept_op, &self->run, true), 0);

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);
	EXPECT_EQ(eaccept_op.ret, 0);

	/*
	 * New page should be accessible from within enclave - write to it.
	 */
	put_addr_op.value = MAGIC;
	put_addr_op.addr = (unsigned long)tcs;
	put_addr_op.header.type = ENCL_OP_PUT_TO_ADDRESS;

	EXPECT_EQ(ENCL_CALL(&put_addr_op, &self->run, true), 0);

	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);

	/*
	 * Read memory that was just written to, confirming that data
	 * previously written (MAGIC) is present. Only change two test
	 * parameters, rest are same as previous test.
	 */
	get_addr_op.value = 0;
	get_addr_op.addr = (unsigned long)tcs;
	get_addr_op.header.type = ENCL_OP_GET_FROM_ADDRESS;

	EXPECT_EQ(ENCL_CALL(&get_addr_op, &self->run, true), 0);

	EXPECT_EQ(get_addr_op.value, MAGIC);
	EXPECT_EEXIT(&self->run);
	EXPECT_EQ(self->run.exception_vector, 0);
	EXPECT_EQ(self->run.exception_error_code, 0);
	EXPECT_EQ(self->run.exception_addr, 0);

	munmap(addr, 3 * PAGE_SIZE);
}

TEST_HARNESS_MAIN
