// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Intel client SoC with integrated memory controller using IBECC
 *
 * Copyright (C) 2019 Intel Corporation
 *
 * The In-Band ECC (IBECC) IP provides ECC protection to all or specific
 * regions of the physical memory space. It's used for memory controllers
 * that don't support the out-of-band ECC which often needs an additional
 * storage device to each channel for storing ECC data. The first supported
 * platform is Ice Lake Neural Network Processor for Inference (ICL-NNPI).
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/irq_work.h>
#include <linux/llist.h>
#include <linux/genalloc.h>
#include <linux/edac.h>
#include <linux/processor.h>
#include <linux/sched/clock.h>
#include <linux/nmi.h>
#include <asm/cpu_device_id.h>
#include <asm/intel-family.h>
#include <asm/mach_traps.h>

#include "edac_mc.h"
#include "edac_module.h"

#define IGEN6_REVISION	"v2.0"

#define EDAC_MOD_STR	"igen6_edac"
#define IGEN6_NMI_NAME	"igen6_ibecc"

/* Debug macros */
#define igen6_printk(level, fmt, arg...)		\
	edac_printk(level, "igen6", fmt, ##arg)

#define igen6_mc_printk(mci, level, fmt, arg...)	\
	edac_mc_chipset_printk(mci, level, "igen6", fmt, ##arg)

#define GET_BITFIELD(v, lo, hi) (((v) & GENMASK_ULL(hi, lo)) >> (lo))

#define NUM_IMC				2 /* Max memory controllers */
#define NUM_CHANNELS			2 /* Max channels */
#define NUM_DIMMS			2 /* Max DIMMs per channel */

#define IGEN6_TOM_OFF			0xa0
#define IGEN6_TOUUD_OFF			0xa8
#define IGEN6_TOLUD_OFF			0xbc
#define IGEN6_CAPID_C_OFF		0xec
#define IGEN6_CAPID_C_IBECC		BIT(15)
#define _4GB				BIT_ULL(32)

#define IGEN6_ERRSTS_OFF		0xc8
#define IGEN6_ERRSTS_CE			BIT_ULL(6)
#define IGEN6_ERRSTS_UE			BIT_ULL(7)

#define IGEN6_ECC_BASE			(res_cfg->ibecc_base)
#define IGEN6_ECCACTIVATE_OFF		IGEN6_ECC_BASE
#define IGEN6_ECCACTIVATE_EN		BIT(0)

#define IGEN6_ECCERRLOG_OFF		(IGEN6_ECC_BASE + 0x170)
#define IGEN6_ECCERRLOG_CE		BIT_ULL(62)
#define IGEN6_ECCERRLOG_UE		BIT_ULL(63)
#define IGEN6_ECCERRLOG_ADDR_SHIFT	5
#define IGEN6_ECCERRLOG_ADDR(v)		GET_BITFIELD(v, 5, 38)
#define IGEN6_ECCERRLOG_SYND(v)		GET_BITFIELD(v, 46, 61)

#define IGEN6_MCHBAR_HI_OFF		0x4c
#define IGEN6_MCHBAR_LO_OFF		0x48
#define IGEN6_MCHBAR_EN			BIT_ULL(0)
#define IGEN6_MCHBAR_BASE(v)		(GET_BITFIELD(v, 16, 38) << 16)
#define IGEN6_MCHBAR_SIZE		0x10000

#define IGEN6_MAD_INTER_OFF		0x5000
#define IGEN6_MAD_INTRA_OFF		0x5004
#define IGEN6_MAD_DIMM_OFF		0x500c
#define IGEN6_HASH_OFF			0X5024
#define IGEN6_EHASH_OFF			0X5028

#define IGEN6_MAD_INTER_DDR_TYPE(v)	GET_BITFIELD(v, 0, 2)
#define IGEN6_MAD_INTER_ECHM(v)		GET_BITFIELD(v, 3, 3)
#define IGEN6_MAD_INTER_CH_L_MAP(v)	GET_BITFIELD(v, 4, 4)
#define IGEN6_MAD_INTER_CH_S_SIZE(v)	((u64)GET_BITFIELD(v, 12, 19) << 29)
#define IGEN6_MAD_INTRA_DIMM_L_MAP(v)	GET_BITFIELD(v, 0, 0)
#define IGEN6_MAD_INTRA_RI(v)		GET_BITFIELD(v, 4, 4)
#define IGEN6_MAD_INTRA_EIM(v)		GET_BITFIELD(v, 8, 8)
#define IGEN6_MAD_INTRA_ECC(v)		(GET_BITFIELD(v, 12, 13) == 0x3)

#define IGEN6_DIMM_CH_DIMM_L_SIZE(v)	((u64)GET_BITFIELD(v, 0, 6) << 29)
#define IGEN6_DIMM_CH_DLW(v)		GET_BITFIELD(v, 7, 8)
#define IGEN6_DIMM_CH_DLNOR(v)		(GET_BITFIELD(v, 9, 10) + 1)
#define IGEN6_DIMM_CH_DIMM_S_SIZE(v)	((u64)GET_BITFIELD(v, 16, 22) << 29)
#define IGEN6_DIMM_CH_DSW(v)		GET_BITFIELD(v, 24, 25)
#define IGEN6_DIMM_CH_DSNOR(v)		(GET_BITFIELD(v, 26, 27) + 1)
#define IGEN6_DIMM_CH_DLS_BG0(v)	GET_BITFIELD(v, 29, 29)

#define IGEN6_HASH_MASK(v)		(GET_BITFIELD(v, 6, 19) << 6)
#define IGEN6_HASH_LSB_MASK_BIT(v)	GET_BITFIELD(v, 24, 26)
#define IGEN6_HASH_MODE(v)		GET_BITFIELD(v, 28, 28)

#define igen6_getreg(imc, type, off)		\
	(*(type *)((imc)->window + (off)))
#define igen6_setreg(imc, type, off, v)		\
	(*(type *)((imc)->window + (off)) = (v))

static struct res_config {
	int num_imc;
	u32 ibecc_base;
	bool (*ibecc_available)(struct pci_dev *pdev);
	u64 (*eaddr_to_saddr)(u64 eaddr);
	u64 (*eaddr_to_iaddr)(u64 eaddr);
} *res_cfg;

struct igen6_imc {
	int mc;
	struct mem_ctl_info *mci;
	struct pci_dev *pdev;
	void __iomem *window;
	u64 ch_s_size;
	int ch_l_map;
	u64 dimm_s_size[NUM_CHANNELS];
	u64 dimm_l_size[NUM_CHANNELS];
	int dimm_l_map[NUM_CHANNELS];
};

static struct igen6_pvt {
	struct igen6_imc imc[NUM_IMC];
} *igen6_pvt;

/* The top of upper usable DRAM */
static u64 igen6_touud;
/* The top of low usable DRAM */
static u32 igen6_tolud;
/* The size of physical memory */
static u64 igen6_tom;

struct decoded_addr {
	u64 imc_addr;
	u64 sys_addr;
	int mc;
	u64 chan_addr;
	int chan;
	u64 sub_chan_addr;
	int sub_chan;
};

struct ecclog_node {
	struct llist_node llnode;
	int mc;
	u64 ecclog;
};

/*
 * printk() is not safe in NMI context. So in NMI handler, the driver uses
 * the lock-less memory alocator to allocate memory for ECC error log and
 * saves it to a lock-less list. Delay the printk() and the work of error
 * reporting to EDAC core in a worker.
 */
#define ECCLOG_POOLSZ	PAGE_SIZE
LLIST_HEAD(ecclog_llist);
static struct gen_pool *ecclog_pool;
static char ecclog_buf[ECCLOG_POOLSZ];
static struct irq_work ecclog_irq_work;
static struct work_struct ecclog_work;

/* Compute die IDs for ICL-NNPI with IBECC */
#define DID_ICL_SKU8	0x4581
#define DID_ICL_SKU10	0x4585
#define DID_ICL_SKU11	0x4589
#define DID_ICL_SKU12	0x458d

static bool icl_ibecc_available(struct pci_dev *pdev)
{
	u32 v;

	if (pci_read_config_dword(pdev, IGEN6_CAPID_C_OFF, &v))
		return false;

	return !(IGEN6_CAPID_C_IBECC & v) &&
		(boot_cpu_data.x86_stepping >= 1);
}

static u64 icl_eaddr_to_saddr(u64 eaddr)
{
	return eaddr;
}

static u64 icl_eaddr_to_iaddr(u64 eaddr)
{
	if (eaddr < igen6_tolud)
		return eaddr;

	if (igen6_tom <= _4GB)
		return eaddr + igen6_tolud - _4GB;

	if (eaddr < _4GB)
		return eaddr + igen6_tolud - igen6_tom;

	return eaddr;
}

static struct res_config icl_cfg = {
	.num_imc	 = 1,
	.ibecc_base	 = 0xd800,
	.ibecc_available = icl_ibecc_available,
	.eaddr_to_saddr  = icl_eaddr_to_saddr,
	.eaddr_to_iaddr  = icl_eaddr_to_iaddr,
};

static const struct pci_device_id igen6_pci_tbl[] = {
	{ PCI_VDEVICE(INTEL, DID_ICL_SKU8), (kernel_ulong_t)&icl_cfg },
	{ PCI_VDEVICE(INTEL, DID_ICL_SKU10), (kernel_ulong_t)&icl_cfg },
	{ PCI_VDEVICE(INTEL, DID_ICL_SKU11), (kernel_ulong_t)&icl_cfg },
	{ PCI_VDEVICE(INTEL, DID_ICL_SKU12), (kernel_ulong_t)&icl_cfg },
	{ },
};
MODULE_DEVICE_TABLE(pci, igen6_pci_tbl);

static enum dev_type get_width(int dimm_l, u32 mad_dimm)
{
	u32 w = dimm_l ? IGEN6_DIMM_CH_DLW(mad_dimm) :
			 IGEN6_DIMM_CH_DSW(mad_dimm);

	switch (w) {
	case 0:
		return DEV_X8;
	case 1:
		return DEV_X16;
	case 2:
		return DEV_X32;
	default:
		return DEV_UNKNOWN;
	}
}

static enum mem_type get_memory_type(u32 mad_inter)
{
	u32 t = IGEN6_MAD_INTER_DDR_TYPE(mad_inter);

	switch (t) {
	case 0:
		return MEM_DDR4;
	case 1:
		return MEM_DDR3;
	case 2:
		return MEM_LPDDR3;
	case 3:
		return MEM_LPDDR4;
	case 4:
		return MEM_WIO2;
	default:
		return MEM_UNKNOWN;
	}
}

static int decode_chan_idx(u64 addr, u64 mask, int intlv_bit)
{
	u64 hash_addr = addr & mask, hash = 0;
	u64 intlv = (addr >> intlv_bit) & 1;
	int i;

	for (i = 6; i < 20; i++)
		hash ^= (hash_addr >> i) & 1;

	return (int)hash ^ intlv;
}

static u64 decode_chan_addr(u64 addr, int intlv_bit)
{
	u64 chan_addr;

	/* Remove the interleave bit and shift upper part down to fill gap */
	chan_addr  = GET_BITFIELD(addr, intlv_bit + 1, 63) << intlv_bit;
	chan_addr |= GET_BITFIELD(addr, 0, intlv_bit - 1);

	return chan_addr;
}

static void decode_addr(u64 addr, u32 hash, u64 s_size, int l_map,
			int *sel, u64 *sub_addr)
{
	int intlv_bit = IGEN6_HASH_LSB_MASK_BIT(hash) + 6;

	if (addr > 2 * s_size) {
		*sub_addr = addr - s_size;
		*sel = l_map;
		return;
	}

	if (IGEN6_HASH_MODE(hash)) {
		*sub_addr = decode_chan_addr(addr, intlv_bit);
		*sel = decode_chan_idx(addr, IGEN6_HASH_MASK(hash), intlv_bit);
	} else {
		*sub_addr = decode_chan_addr(addr, 6);
		*sel = GET_BITFIELD(addr, 6, 6);
	}
}

static int igen6_decode(struct decoded_addr *res)
{
	struct igen6_imc *imc = &igen6_pvt->imc[res->mc];
	u64 addr = res->imc_addr, sub_addr, s_size;
	int sel, l_map;
	u32 hash;

	if (addr >= igen6_tom) {
		edac_dbg(0, "Address 0x%llx out of range\n", addr);
		return -EINVAL;
	}

	/* Decode channel */
	hash	= igen6_getreg(imc, u32, IGEN6_HASH_OFF);
	s_size	= imc->ch_s_size;
	l_map	= imc->ch_l_map;
	decode_addr(addr, hash, s_size, l_map, &sel, &sub_addr);
	res->chan	= sel;
	res->chan_addr	= sub_addr;

	/* Decode sub-channel/DIMM */
	hash	= igen6_getreg(imc, u32, IGEN6_EHASH_OFF);
	s_size	= imc->dimm_s_size[sel];
	l_map	= imc->dimm_l_map[sel];
	decode_addr(res->chan_addr, hash, s_size, l_map, &sel, &sub_addr);
	res->sub_chan	   = sel;
	res->sub_chan_addr = sub_addr;

	return 0;
}

static void igen6_output_error(struct decoded_addr *res,
			       struct mem_ctl_info *mci, u64 ecclog)
{
	enum hw_event_mc_err_type type = ecclog & IGEN6_ECCERRLOG_UE ?
					 HW_EVENT_ERR_UNCORRECTED :
					 HW_EVENT_ERR_CORRECTED;

	edac_mc_handle_error(type, mci, 1,
			     res->sys_addr >> PAGE_SHIFT,
			     res->sys_addr & ~PAGE_MASK,
			     IGEN6_ECCERRLOG_SYND(ecclog),
			     res->chan, res->sub_chan,
			     -1, "", "");
}

static struct gen_pool *ecclog_gen_pool_create(void)
{
	struct gen_pool *pool;

	pool = gen_pool_create(ilog2(sizeof(struct ecclog_node)), -1);
	if (!pool)
		return NULL;

	if (gen_pool_add(pool, (unsigned long)ecclog_buf, ECCLOG_POOLSZ, -1)) {
		gen_pool_destroy(pool);
		return NULL;
	}

	return pool;
}

static int ecclog_gen_pool_add(int mc, u64 ecclog)
{
	struct ecclog_node *node;

	node = (void *)gen_pool_alloc(ecclog_pool, sizeof(*node));
	if (!node)
		return -ENOMEM;

	node->mc = mc;
	node->ecclog = ecclog;
	llist_add(&node->llnode, &ecclog_llist);

	return 0;
}

static u64 ecclog_read(struct igen6_imc *imc)
{
	u64 ecclog = igen6_getreg(imc, u64, IGEN6_ECCERRLOG_OFF);

	if (ecclog & (IGEN6_ECCERRLOG_CE | IGEN6_ECCERRLOG_UE))
		return ecclog;

	return 0;
}

static void ecclog_clear(struct igen6_imc *imc, u64 ecclog)
{
	/* Clear CE/UE bits in IBECC register by writing 1 to it */
	ecclog |= IGEN6_ECCERRLOG_CE | IGEN6_ECCERRLOG_UE;
	igen6_setreg(imc, u64, IGEN6_ECCERRLOG_OFF, ecclog);
}

static void errsts_clear(void)
{
	struct igen6_imc *imc = &igen6_pvt->imc[0];
	u16 errsts;

	if (pci_read_config_word(imc->pdev, IGEN6_ERRSTS_OFF, &errsts)) {
		igen6_printk(KERN_ERR, "Failed to read ERRSTS\n");
		return;
	}

	if (!(errsts & (IGEN6_ERRSTS_CE | IGEN6_ERRSTS_UE)))
		return;

	/* Clear CE/UE bits in PCI ERRSTS register by writing 1 to it */
	errsts |= IGEN6_ERRSTS_CE | IGEN6_ERRSTS_UE;
	pci_write_config_word(imc->pdev, IGEN6_ERRSTS_OFF, errsts);
}

static int ecclog_handler(void)
{
	struct igen6_imc *imc;
	int i, n = 0;
	u64 ecclog;

	for (i = 0; i < res_cfg->num_imc; i++) {
		imc = &igen6_pvt->imc[i];

		ecclog = ecclog_read(imc);
		if (!ecclog)
			continue;

		ecclog_clear(imc, ecclog);
		/* errsts_clear() is not NMI safe, delay it in irq_work */

		if (!ecclog_gen_pool_add(i, ecclog))
			irq_work_queue(&ecclog_irq_work);

		n++;
	}

	return n;
}

static void ecclog_work_cb(struct work_struct *work)
{
	struct ecclog_node *node, *tmp;
	struct mem_ctl_info *mci;
	struct llist_node *head;
	struct decoded_addr res;
	u64 eaddr;

	head = llist_del_all(&ecclog_llist);
	if (!head)
		return;

	llist_for_each_entry_safe(node, tmp, head, llnode) {
		memset(&res, 0, sizeof(res));
		eaddr = IGEN6_ECCERRLOG_ADDR(node->ecclog) <<
			IGEN6_ECCERRLOG_ADDR_SHIFT;
		res.mc	     = node->mc;
		res.sys_addr = res_cfg->eaddr_to_saddr(eaddr);
		res.imc_addr = res_cfg->eaddr_to_iaddr(eaddr);

		mci = igen6_pvt->imc[res.mc].mci;

		edac_dbg(2, "MC %d, ecclog = 0x%llx\n", node->mc, node->ecclog);
		igen6_mc_printk(mci, KERN_DEBUG, "HANDLING IBECC MEMORY ERROR\n");
		igen6_mc_printk(mci, KERN_DEBUG, "ADDR 0x%llx ", res.sys_addr);

		if (!igen6_decode(&res))
			igen6_output_error(&res, mci, node->ecclog);

		gen_pool_free(ecclog_pool, (unsigned long)node, sizeof(*node));
	}
}

static void ecclog_irq_work_cb(struct irq_work *irq_work)
{
	errsts_clear();

	if (!llist_empty(&ecclog_llist))
		schedule_work(&ecclog_work);
}

static int ecclog_nmi_handler(unsigned int cmd, struct pt_regs *regs)
{
	unsigned char reason;

	if (!ecclog_handler())
		return NMI_DONE;

	/* Re-enable the PCI SERR error line */
	reason = x86_platform.get_nmi_reason() & NMI_REASON_CLEAR_MASK;
	reason |= NMI_REASON_CLEAR_SERR;
	outb(reason, NMI_REASON_PORT);
	reason &= ~NMI_REASON_CLEAR_SERR;
	outb(reason, NMI_REASON_PORT);

	return NMI_HANDLED;
}

static bool igen6_check_ecc(struct igen6_imc *imc)
{
	u32 activate = igen6_getreg(imc, u32, IGEN6_ECCACTIVATE_OFF);

	return !!(activate & IGEN6_ECCACTIVATE_EN);
}

static int igen6_get_dimm_config(struct mem_ctl_info *mci)
{
	struct igen6_imc *imc = mci->pvt_info;
	u32 mad_inter, mad_intra, mad_dimm;
	int i, j, ndimms, mc = imc->mc;
	struct dimm_info *dimm;
	enum mem_type mtype;
	enum dev_type dtype;
	u64 dsize;
	bool ecc;

	edac_dbg(2, "\n");

	mad_inter = igen6_getreg(imc, u32, IGEN6_MAD_INTER_OFF);
	mtype = get_memory_type(mad_inter);
	ecc = igen6_check_ecc(imc);
	imc->ch_s_size = IGEN6_MAD_INTER_CH_S_SIZE(mad_inter);
	imc->ch_l_map  = IGEN6_MAD_INTER_CH_L_MAP(mad_inter);

	for (i = 0; i < NUM_CHANNELS; i++) {
		mad_intra = igen6_getreg(imc, u32, IGEN6_MAD_INTRA_OFF + i * 4);
		mad_dimm  = igen6_getreg(imc, u32, IGEN6_MAD_DIMM_OFF + i * 4);

		imc->dimm_l_size[i] = IGEN6_DIMM_CH_DIMM_L_SIZE(mad_dimm);
		imc->dimm_s_size[i] = IGEN6_DIMM_CH_DIMM_S_SIZE(mad_dimm);
		imc->dimm_l_map[i]  = IGEN6_MAD_INTRA_DIMM_L_MAP(mad_intra);
		ndimms = 0;

		for (j = 0; j < NUM_DIMMS; j++) {
			dimm = edac_get_dimm(mci, i, j, 0);

			if (j ^ imc->dimm_l_map[i]) {
				dtype = get_width(0, mad_dimm);
				dsize = imc->dimm_s_size[i];
			} else {
				dtype = get_width(1, mad_dimm);
				dsize = imc->dimm_l_size[i];
			}

			if (!dsize)
				continue;

			dimm->grain = 32;
			dimm->mtype = mtype;
			dimm->dtype = dtype;
			dimm->nr_pages  = MiB_TO_PAGES(dsize >> 20);
			dimm->edac_mode = EDAC_SECDED;
			snprintf(dimm->label, sizeof(dimm->label),
				 "MC#%d_Chan#%d_DIMM#%d", mc, i, j);
			edac_dbg(0, "MC %d, Channel %d, DIMM %d, Size %llu MiB (%u pages)\n",
				 mc, i, j, dsize >> 20, dimm->nr_pages);

			ndimms++;
		}

		if (ndimms && !ecc) {
			igen6_printk(KERN_ERR, "MC%d ECC is disabled\n", mc);
			return -ENODEV;
		}
	}

	return 0;
}

#define PCI_RD_U32(pdev, w, v)						    \
	do {								    \
		if (pci_read_config_dword(pdev, w, v)) {		    \
			igen6_printk(KERN_ERR, "Failed to read 0x%x\n", w); \
			goto fail;					    \
		}							    \
	} while (0)

static int igen6_pci_setup(struct pci_dev *pdev, struct res_config *cfg,
			   u64 *mchbar)
{
	union  {
		u64 v;
		struct {
			u32 v_lo;
			u32 v_hi;
		};
	} u;

	edac_dbg(2, "\n");

	if (!cfg->ibecc_available(pdev)) {
		edac_dbg(2, "No In-Band ECC IP\n");
		return -ENODEV;
	}

	PCI_RD_U32(pdev, IGEN6_TOUUD_OFF, &u.v_lo);
	PCI_RD_U32(pdev, IGEN6_TOUUD_OFF + 4, &u.v_hi);
	igen6_touud = u.v & GENMASK_ULL(38, 20);

	PCI_RD_U32(pdev, IGEN6_TOLUD_OFF, &igen6_tolud);
	igen6_tolud &= GENMASK(31, 20);

	PCI_RD_U32(pdev, IGEN6_TOM_OFF, &u.v_lo);
	PCI_RD_U32(pdev, IGEN6_TOM_OFF + 4, &u.v_hi);
	igen6_tom = u.v & GENMASK_ULL(38, 20);

	PCI_RD_U32(pdev, IGEN6_MCHBAR_LO_OFF, &u.v_lo);
	PCI_RD_U32(pdev, IGEN6_MCHBAR_HI_OFF, &u.v_hi);
	if (!(u.v & IGEN6_MCHBAR_EN)) {
		igen6_printk(KERN_ERR, "MCHBAR is disabled\n");
		return -ENODEV;
	}
	*mchbar = IGEN6_MCHBAR_BASE(u.v);

	return 0;
fail:
	return -ENODEV;
}

#ifdef CONFIG_EDAC_DEBUG
static void igen6_reg_dump(struct igen6_imc *imc)
{
	int i;

	edac_dbg(2, "Hash	: 0x%x\n",
		 igen6_getreg(imc, u32, IGEN6_HASH_OFF));
	edac_dbg(2, "Ehash	: 0x%x\n",
		 igen6_getreg(imc, u32, IGEN6_EHASH_OFF));
	edac_dbg(2, "Mad_inter	: 0x%x\n",
		 igen6_getreg(imc, u32, IGEN6_MAD_INTER_OFF));
	edac_dbg(2, "Eccerrlog	: 0x%llx\n",
		 igen6_getreg(imc, u64, IGEN6_ECCERRLOG_OFF));

	for (i = 0; i < NUM_CHANNELS; i++) {
		edac_dbg(2, "Mad_intra_%d : 0x%x\n", i,
			 igen6_getreg(imc, u32, IGEN6_MAD_INTRA_OFF + i * 4));
		edac_dbg(2, "Mad_dimm_%d  : 0x%x\n", i,
			 igen6_getreg(imc, u32, IGEN6_MAD_DIMM_OFF + i * 4));
	}
	edac_dbg(2, "Touud	: 0x%llx", igen6_touud);
	edac_dbg(2, "Tolud	: 0x%x", igen6_tolud);
	edac_dbg(2, "Tom	: 0x%llx", igen6_tom);
}
#else
static void igen6_reg_dump(struct igen6_imc *imc) {}
#endif

static int igen6_register_mci(int mc, u64 mchbar, struct pci_dev *pdev)
{
	struct edac_mc_layer layers[2];
	struct mem_ctl_info *mci;
	struct igen6_imc *imc;
	void __iomem *window;
	int rc = -ENODEV;

	edac_dbg(2, "\n");

	mchbar += mc * 0x10000;
	window = ioremap(mchbar, IGEN6_MCHBAR_SIZE);
	if (!window) {
		igen6_printk(KERN_ERR, "Failed to ioremap 0x%llx\n", mchbar);
		return -ENODEV;
	}

	layers[0].type = EDAC_MC_LAYER_CHANNEL;
	layers[0].size = NUM_CHANNELS;
	layers[0].is_virt_csrow = false;
	layers[1].type = EDAC_MC_LAYER_SLOT;
	layers[1].size = NUM_DIMMS;
	layers[1].is_virt_csrow = true;

	mci = edac_mc_alloc(mc, ARRAY_SIZE(layers), layers, 0);
	if (!mci) {
		rc = -ENOMEM;
		goto fail;
	}

	mci->ctl_name = kasprintf(GFP_KERNEL, "Intel_client_SoC MC#%d", mc);
	if (!mci->ctl_name) {
		rc = -ENOMEM;
		goto fail2;
	}

	mci->mtype_cap = MEM_FLAG_LPDDR4 | MEM_FLAG_DDR4;
	mci->edac_ctl_cap = EDAC_FLAG_SECDED;
	mci->edac_cap = EDAC_FLAG_SECDED;
	mci->mod_name = EDAC_MOD_STR;
	mci->dev_name = pci_name(pdev);
	mci->pdev = &pdev->dev;
	mci->pvt_info = &igen6_pvt->imc[mc];

	imc = mci->pvt_info;
	imc->mc	= mc;
	imc->pdev = pdev;
	imc->window = window;

	igen6_reg_dump(imc);

	rc = igen6_get_dimm_config(mci);
	if (rc)
		goto fail3;

	rc = edac_mc_add_mc(mci);
	if (rc) {
		igen6_printk(KERN_ERR, "Failed to register mci#%d\n", mc);
		goto fail3;
	}

	imc->mci = mci;
	return 0;

fail3:
	kfree(mci->ctl_name);
fail2:
	edac_mc_free(mci);
fail:
	iounmap(window);
	return rc;
}

static void igen6_unregister_mcis(void)
{
	struct igen6_imc *imc = igen6_pvt->imc;
	struct mem_ctl_info *mci;
	int i;

	edac_dbg(2, "\n");

	for (i = 0; i < NUM_IMC; i++, imc++) {
		mci = imc->mci;
		if (!mci)
			continue;

		edac_mc_del_mc(mci->pdev);
		kfree(mci->ctl_name);
		edac_mc_free(mci);
		iounmap(imc->window);
	}
}

static int igen6_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	u64 mchbar;
	int i, rc;

	edac_dbg(2, "\n");

	igen6_pvt = kzalloc(sizeof(*igen6_pvt), GFP_KERNEL);
	if (!igen6_pvt)
		return -ENOMEM;

	res_cfg = (struct res_config *)ent->driver_data;

	rc = igen6_pci_setup(pdev, res_cfg, &mchbar);
	if (rc)
		goto fail;

	for (i = 0; i < res_cfg->num_imc; i++) {
		rc = igen6_register_mci(i, mchbar, pdev);
		if (rc)
			goto fail2;
	}

	ecclog_pool = ecclog_gen_pool_create();
	if (!ecclog_pool) {
		rc = -ENOMEM;
		goto fail2;
	}

	INIT_WORK(&ecclog_work, ecclog_work_cb);
	init_irq_work(&ecclog_irq_work, ecclog_irq_work_cb);

	/* Check if any pending error before registering the NMI handler */
	ecclog_handler();

	rc = register_nmi_handler(NMI_SERR, ecclog_nmi_handler,
				  0, IGEN6_NMI_NAME);
	if (rc) {
		igen6_printk(KERN_ERR, "Failed to register nmi handler\n");
		goto fail3;
	}

	return 0;

fail3:
	gen_pool_destroy(ecclog_pool);
fail2:
	igen6_unregister_mcis();
fail:
	kfree(igen6_pvt);
	return rc;
}

static void igen6_remove(struct pci_dev *pdev)
{
	edac_dbg(2, "\n");

	unregister_nmi_handler(NMI_SERR, IGEN6_NMI_NAME);
	irq_work_sync(&ecclog_irq_work);
	flush_work(&ecclog_work);
	gen_pool_destroy(ecclog_pool);
	igen6_unregister_mcis();
	kfree(igen6_pvt);
}

static struct pci_driver igen6_driver = {
	.name     = EDAC_MOD_STR,
	.probe    = igen6_probe,
	.remove   = igen6_remove,
	.id_table = igen6_pci_tbl,
};

static int __init igen6_init(void)
{
	const char *owner;
	int rc;

	edac_dbg(2, "\n");

	owner = edac_get_owner();
	if (owner && strncmp(owner, EDAC_MOD_STR, sizeof(EDAC_MOD_STR)))
		return -ENODEV;

	edac_op_state = EDAC_OPSTATE_NMI;

	rc = pci_register_driver(&igen6_driver);
	if (rc)
		return rc;

	igen6_printk(KERN_INFO, "%s\n", IGEN6_REVISION);

	return 0;
}

static void __exit igen6_exit(void)
{
	edac_dbg(2, "\n");

	pci_unregister_driver(&igen6_driver);
}

module_init(igen6_init);
module_exit(igen6_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Qiuxu Zhuo");
MODULE_DESCRIPTION("MC Driver for Intel client SoC using In-Band ECC");
