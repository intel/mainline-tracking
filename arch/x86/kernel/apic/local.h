/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Historical copyright notices:
 *
 * Copyright 2004 James Cleverdon, IBM.
 * (c) 1995 Alan Cox, Building #3 <alan@redhat.com>
 * (c) 1998-99, 2000 Ingo Molnar <mingo@redhat.com>
 * (c) 2002,2003 Andi Kleen, SuSE Labs.
 */

#include <linux/jump_label.h>

#include <asm/irq_vectors.h>
#include <asm/apic.h>

/* X2APIC */
void __x2apic_send_IPI_dest(unsigned int apicid, int vector, unsigned int dest);
u32 x2apic_get_apic_id(u32 id);

void x2apic_send_IPI_all(int vector);
void x2apic_send_IPI_allbutself(int vector);
void x2apic_send_IPI_self(int vector);
extern u32 x2apic_max_apicid;

u16 __prepare_ICR_dm_and_vector(u16 dm, u8 vector);

/* IPI */

DECLARE_STATIC_KEY_FALSE(apic_use_ipi_shorthand);

static inline bool is_nmi(u16 dm_vector)
{
	if ((dm_vector & APIC_DM_MASK) == APIC_DM_NMI)
		return true;
	if ((dm_vector & APIC_VECTOR_MASK) == NMI_VECTOR)
		return true;
	return false;
}

static inline u32 __prepare_ICR(u32 shortcut, u16 dm_vector, u32 dest)
{
	return shortcut | dest |
	       __prepare_ICR_dm_and_vector(dm_vector & APIC_DM_MASK, dm_vector);
}

void default_init_apic_ldr(void);

void apic_mem_wait_icr_idle(void);
u32 apic_mem_wait_icr_idle_timeout(void);

/*
 * This is used to send an IPI with no shorthand notation (the destination is
 * specified in bits 56 to 63 of the ICR).
 */
void __default_send_IPI_dest_field(unsigned int mask, int vector, unsigned int dest);

void default_send_IPI_single(int cpu, int vector);
void default_send_IPI_single_phys(int cpu, int vector);
void default_send_IPI_mask_sequence_phys(const struct cpumask *mask, int vector);
void default_send_IPI_mask_allbutself_phys(const struct cpumask *mask, int vector);
void default_send_IPI_allbutself(int vector);
void default_send_IPI_all(int vector);
void default_send_IPI_self(int vector);

#ifdef CONFIG_X86_32
void default_send_IPI_mask_sequence_logical(const struct cpumask *mask, int vector);
void default_send_IPI_mask_allbutself_logical(const struct cpumask *mask, int vector);
void default_send_IPI_mask_logical(const struct cpumask *mask, int vector);
void x86_32_probe_bigsmp_early(void);
void x86_32_install_bigsmp(void);
#else
static inline void x86_32_probe_bigsmp_early(void) { }
static inline void x86_32_install_bigsmp(void) { }
#endif

#ifdef CONFIG_X86_BIGSMP
bool apic_bigsmp_possible(bool cmdline_selected);
void apic_bigsmp_force(void);
#else
static inline bool apic_bigsmp_possible(bool cmdline_selected) { return false; };
static inline void apic_bigsmp_force(void) { }
#endif
