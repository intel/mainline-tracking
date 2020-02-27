.. SPDX-License-Identifier: GPL-2.0

======================
Memory Protection Keys
======================

Memory Protection Keys provide a mechanism for enforcing page-based
protections, but without requiring modification of the page tables
when an application changes protection domains.

PKeys Userspace (PKU) is a feature which is found on Intel's Skylake "Scalable
Processor" Server CPUs and later.  And it will be available in future
non-server Intel parts and future AMD processors.

Protection Keys for Supervisor pages (PKS) is available in the SDM since May
2020.

pkeys work by dedicating 4 previously Reserved bits in each page table entry to
a "protection key", giving 16 possible keys.  User and Supervisor pages are
treated separately.

Protections for each page are controlled with per-CPU registers for each type
of page User and Supervisor.  Each of these 32-bit register stores two separate
bits (Access Disable and Write Disable) for each key.

For Userspace the register is user-accessible (rdpkru/wrpkru).  For
Supervisor, the register (MSR_IA32_PKRS) is accessible only to the kernel.

Being a CPU register, pkeys are inherently thread-local, potentially giving
each thread an independent set of protections from every other thread.

There are two new instructions (RDPKRU/WRPKRU) for reading and writing
to the new register.  The feature is only available in 64-bit mode,
even though there is theoretically space in the PAE PTEs.  These
permissions are enforced on data access only and have no effect on
instruction fetches.

For kernel space rdmsr/wrmsr are used to access the kernel MSRs.


Syscalls for user space keys
============================

There are 3 system calls which directly interact with pkeys::

	int pkey_alloc(unsigned long flags, unsigned long init_access_rights)
	int pkey_free(int pkey);
	int pkey_mprotect(unsigned long start, size_t len,
			  unsigned long prot, int pkey);

Before a pkey can be used, it must first be allocated with
pkey_alloc().  An application calls the WRPKRU instruction
directly in order to change access permissions to memory covered
with a key.  In this example WRPKRU is wrapped by a C function
called pkey_set().
::

	int real_prot = PROT_READ|PROT_WRITE;
	pkey = pkey_alloc(0, PKEY_DISABLE_WRITE);
	ptr = mmap(NULL, PAGE_SIZE, PROT_NONE, MAP_ANONYMOUS|MAP_PRIVATE, -1, 0);
	ret = pkey_mprotect(ptr, PAGE_SIZE, real_prot, pkey);
	... application runs here

Now, if the application needs to update the data at 'ptr', it can
gain access, do the update, then remove its write access::

	pkey_set(pkey, 0); // clear PKEY_DISABLE_WRITE
	*ptr = foo; // assign something
	pkey_set(pkey, PKEY_DISABLE_WRITE); // set PKEY_DISABLE_WRITE again

Now when it frees the memory, it will also free the pkey since it
is no longer in use::

	munmap(ptr, PAGE_SIZE);
	pkey_free(pkey);

.. note:: pkey_set() is a wrapper for the RDPKRU and WRPKRU instructions.
          An example implementation can be found in
          tools/testing/selftests/x86/protection_keys.c.

Behavior
========

The kernel attempts to make protection keys consistent with the
behavior of a plain mprotect().  For instance if you do this::

	mprotect(ptr, size, PROT_NONE);
	something(ptr);

you can expect the same effects with protection keys when doing this::

	pkey = pkey_alloc(0, PKEY_DISABLE_WRITE | PKEY_DISABLE_READ);
	pkey_mprotect(ptr, size, PROT_READ|PROT_WRITE, pkey);
	something(ptr);

That should be true whether something() is a direct access to 'ptr'
like::

	*ptr = foo;

or when the kernel does the access on the application's behalf like
with a read()::

	read(fd, ptr, 1);

The kernel will send a SIGSEGV in both cases, but si_code will be set
to SEGV_PKERR when violating protection keys versus SEGV_ACCERR when
the plain mprotect() permissions are violated.


Kernel API for PKS support
==========================

Similar to user space pkeys, supervisor pkeys allow additional protections to
be defined for a supervisor mappings.

The following interface is used to allocate, use, and free a pkey which defines
a 'protection domain' within the kernel.  Setting a pkey value in a supervisor
PTE adds this additional protection to the page.

Kernel users intending to use PKS support should check (depend on)
ARCH_HAS_SUPERVISOR_PKEYS and add their config to ARCH_ENABLE_SUPERVISOR_PKEYS
to turn on this support within the core.  See the test configuration option
'PKS_TEST' for an example.

        int pks_key_alloc(const char * const pkey_user);
        #define PAGE_KERNEL_PKEY(pkey)
        #define _PAGE_KEY(pkey)
        void pks_mk_noaccess(int pkey);
        void pks_mk_readonly(int pkey);
        void pks_mk_readwrite(int pkey);
        void pks_key_free(int pkey);

pks_key_alloc() allocates keys dynamically to allow better use of the limited
key space.

Callers of pks_key_alloc() _must_ be prepared for it to fail and take
appropriate action.  This is due mainly to the fact that PKS may not be
available on all arch's.  Failure to check the return of pks_key_alloc() and
using any of the rest of the API is undefined.

Keys are allocated with 'No Access' permissions.  If other permissions are
required before the pkey is used, the pks_mk*() family of calls, documented
below, can be used prior to setting the pkey within the page table entries.

Kernel users must set the pkey in the page table entries for the mappings they
want to protect.  This can be done with PAGE_KERNEL_PKEY() or _PAGE_KEY().

The pks_mk*() family of calls allows kernel users to change the protections for
the domain identified by the pkey parameter.  3 states are available:
pks_mk_noaccess(), pks_mk_readonly(), and pks_mk_readwrite() which set the
access to none, read, and read/write respectively.

Finally, pks_key_free() allows a user to return the key to the allocator for
use by others.

The interface maintains pks_mk_noaccess() (Access Disabled (AD=1)) for all keys
not currently allocated.  Therefore, the user can depend on access being
disabled when pks_key_alloc() returns a key and the user should remove mappings
from the domain (remove the pkey from the PTE) prior to calling pks_key_free().

It should be noted that the underlying WRMSR(MSR_IA32_PKRS) is not serializing
but still maintains ordering properties similar to WRPKRU.  Thus it is safe to
immediately use a mapping when the pks_mk*() functions return.

Older versions of the SDM on PKRS may be wrong with regard to this
serialization.  The text should be the same as that of WRPKRU.  From the WRPKRU
text:

	WRPKRU will never execute transiently. Memory accesses
	affected by PKRU register will not execute (even transiently)
	until all prior executions of WRPKRU have completed execution
	and updated the PKRU register.

Example code can be found in lib/pks/pks_test.c
