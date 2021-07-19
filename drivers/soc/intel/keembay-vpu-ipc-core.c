// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keem Bay VPU IPC Driver.
 *
 * Copyright (c) 2018-2021 Intel Corporation.
 *
 * The purpose of this driver is to facilitate booting, control and
 * communication with the VPU IP on the Keem Bay SoC.
 *
 * Specifically the driver provides the following functionality to other kernel
 * components:
 * - Loading the VPU firmware into DDR for the VPU to execute.
 * - Starting / Stopping / Rebooting the VPU.
 * - Getting notifications of VPU events (e.g., WDT events).
 * - Communicating with the VPU using the Keem Bay IPC mechanism.
 *
 * VPU Firmware loading
 * --------------------
 *
 * The VPU Firmware consists of both the RTOS and the application code meant to
 * be run by the VPU.
 *
 * The VPU Firmware is loaded into DDR using the Linux Firmware API. The
 * firmware is loaded into a specific reserved memory region in DDR and
 * executed by the VPU directly from there.
 *
 * The VPU Firmware binary is expected to have the following format:
 *
 * +------------------+ 0x0
 * | Header           |
 * +------------------+ 0x1000
 * | FW Version Area  |
 * +------------------+ 0x2000
 * | FW Image         |
 * +------------------+ 0x2000 + FW image size
 * | x509 Certificate |
 * +------------------+
 *
 * As part of the firmware loading process, the driver performs the following
 * operations:
 * - It parses the VPU firmware binary.
 * - It loads the FW version area to the DDR location expected by the VPU
 *   firmware and specified in the FW header.
 * - It loads the FW image to the location specified in the FW header.
 * - It copies the x509 certificate to the specific reserved memory region.
 * - It prepares the boot parameters to be passed to the VPU firmware and loads
 *   them at the location specified in the FW header.
 *
 * VPU Start / Stop / Reboot
 * -------------------------
 *
 * The VPU is started / stopped by the SoC firmware, not by this driver
 * directly. Specifically, the driver uses an OP-TEE Trusted Application (TA)
 * to start / stop the VPU.
 *
 * Reboot is performed by stopping and re-starting the VPU, including
 * re-loading the VPU firmware (this is because the VPU firmware .data and .bss
 * sections need to be re-initialized).
 */

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/dma-direct.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/tee_drv.h>

#include <linux/soc/intel/keembay-vpu-ipc.h>

#include "keembay-ipc.h"

/* The function IDs implemented in VPU AUTH TA */
#define KMB_TA_VPU_BOOT			0
#define KMB_TA_VPU_STOP			1

/* The number of parameters passed to VPU AUTH TA */
#define NUM_TEE_PARAMS			1

/* Device tree "memory-region" for VPU firmware area */
#define VPU_IPC_FW_AREA			"fw-area"

/* Device tree region for VPU driver to store X509 region */
#define VPU_IPC_X509_AREA		"x509-area"

/*
 * These are the parameters of the ready message to be received
 * from the VPU when it is booted correctly.
 */
#define READY_MESSAGE_IPC_CHANNEL	0xA

/* Ready message timeout, in ms */
#define READY_MESSAGE_TIMEOUT_MS	2000

/* Ready message 'physical data address', which is actually a command. */
#define READY_MESSAGE_EXPECTED_PADDR	0x424f4f54

/* Ready message size */
#define READY_MESSAGE_EXPECTED_SIZE	0

/* Size of version information in the header */
#define VPU_VERSION_SIZE		32

/* Version of header that this driver supports. */
#define HEADER_VERSION_SUPPORTED	0x1

/* Maximum size allowed for firmware version region */
#define MAX_FIRMWARE_VERSION_SIZE	0x1000

/* Size allowed for header region */
#define MAX_HEADER_SIZE			0x1000

/* Maximum size of X509 certificate */
#define MAX_X509_SIZE			0x1000

/* VPU reset vector must be aligned to 4kB. */
#define VPU_RESET_VECTOR_ALIGNMENT	0x1000

/* Watchdog timer reset trigger */
#define TIM_WATCHDOG			0x0

/* Watchdog counter enable register */
#define TIM_WDOG_EN			0x8

/* Write access to protected registers */
#define TIM_SAFE			0xC

/* Watchdog timer count value */
#define TIM_WATCHDOG_RESET_VALUE	0xFFFFFFFF

/* Watchdog timer safe value */
#define TIM_SAFE_ENABLE			0xf1d0dead

/* Watchdog timeout interrupt clear bit */
#define TIM_GEN_CONFIG_WDOG_TO_INT_CLR	BIT(9)

/* Magic number for the boot parameters. */
#define BOOT_PARAMS_MAGIC_NUMBER	0x00010000

/* Maximum size of string of form "pll_i_out_j" */
#define PLL_STRING_SIZE			128

/* Number of PLLs */
#define NUM_PLLS			3

/* Every PLL has this many outputs. */
#define NUM_PLL_OUTPUTS			4

/* SoC SKU length, in bytes. */
#define SOC_INFO_SKU_BYTES		6

/* SoC stepping length, in bytes. */
#define SOC_INFO_STEPPING_BYTES		2

/*
 * The offset to convert the HW IRQ reported by irqd_to_hwirq() to the GIC_SPI
 * IRQ number specified in the device tree:
 * GIC_SPI x = irqd_to_hwirq() - 32
 */
#define GIC_SPI_HWIRQ_OFFSET		32

/**
 * struct boot_parameters - Boot parameters passed to the VPU.
 * @magic_number:		Magic number to indicate structure populated
 * @vpu_id:			ID to be passed to the VPU firmware.
 * @reserved_0:			Reserved memory for other 'header' information
 * @cpu_frequency_hz:		Frequency that the CPU is running at
 * @pll0_out:			Frequency of each of the outputs of PLL 0
 * @pll1_out:			Frequency of each of the outputs of PLL 1
 * @pll2_out:			Frequency of each of the outputs of PLL 2
 * @reserved_1:			Reserved memory for other clock frequencies
 * @mss_ipc_header_address:	Base address of MSS IPC header region
 * @mss_ipc_header_area_size:	Size of MSS IPC header region
 * @mmio_address:		MMIO region for VPU communication
 * @mmio_area_size:		Size of MMIO region for VPU communication
 * @reserved_2:			Reserved memory for other memory regions
 * @mss_wdt_to_irq_a53_redir:	MSS redirects WDT TO IRQ to this ARM IRQ number
 * @nce_wdt_to_irq_a53_redir:	NCE redirects WDT TO IRQ to this ARM IRQ number
 * @vpu_to_host_irq:		VPU to host notification IRQ
 * @reserved_3:			Reserved memory for other configurations
 * @a53ss_version_id:		SoC A53SS_VERSION_ID register value
 * @si_stepping:		Silicon stepping, 2 characters
 * @device_id:			64 bits of device ID info from fuses
 * @feature_exclusion:		64 bits of feature exclusion info from fuses
 * @sku:			64-bit SKU identifier.
 * @reserved_4:			Reserved memory for other information
 * @reserved_5:			Unused/reserved memory
 */
struct boot_parameters {
	/* Header: 0x0 - 0x1F */
	u32 magic_number;
	u32 vpu_id;
	u32 reserved_0[6];
	/* Clock frequencies: 0x20 - 0xFF */
	u32 cpu_frequency_hz;
	u32 pll0_out[NUM_PLL_OUTPUTS];
	u32 pll1_out[NUM_PLL_OUTPUTS];
	u32 pll2_out[NUM_PLL_OUTPUTS];
	u32 reserved_1[43];
	/* Memory regions: 0x100 - 0x1FF */
	u64 mss_ipc_header_address;
	u32 mss_ipc_header_area_size;
	u64 mmio_address;
	u32 mmio_area_size;
	u32 reserved_2[58];
	/* IRQ re-direct numbers: 0x200 - 0x2FF */
	u32 mss_wdt_to_irq_a53_redir;
	u32 nce_wdt_to_irq_a53_redir;
	u32 vpu_to_host_irq;
	u32 reserved_3[61];
	/* Silicon information: 0x300 - 0x3FF */
	u32 a53ss_version_id;
	u32 si_stepping;
	u64 device_id;
	u64 feature_exclusion;
	u64 sku;
	u32 reserved_4[56];
	/* Unused/reserved: 0x400 - 0xFFF */
	u32 reserved_5[0x300];
} __packed;

/**
 * struct firmware_header - Firmware header information
 * @header_ver:		This header version dictates content structure of
 *			remainder of firmware image, including the header
 *			itself.
 * @image_format:	Image format defines how the loader will handle the
 *			'firmware image'.
 * @image_load_addr:	VPU address where the firmware image must be loaded to.
 * @image_size:		Size of the image.
 * @entry_point:	Entry point for the VPU firmware (this is a VPU
 *			address).
 * @vpu_ver:		Version of the VPU firmware.
 * @compression_type:	Type of compression used for the VPU firmware.
 * @fw_ver_load_addr:	VPU address where to load the data in the FW version
 *			area of the binary.
 * @fw_ver_size:	Size of the FW version.
 * @config_load_addr:	VPU IPC driver will populate the 4kB of configuration
 *			data to this address.
 */
struct firmware_header {
	u32 header_ver;
	u32 image_format;
	u64 image_load_addr;
	u32 image_size;
	u64 entry_point;
	u8  vpu_ver[VPU_VERSION_SIZE];
	u32 compression_type;
	u64 fw_ver_load_addr;
	u32 fw_ver_size;
	u64 config_load_addr;
} __packed;

/**
 * struct vpu_mem - Information about reserved memory shared with VPU.
 * @vaddr:	The virtual address of the memory region.
 * @paddr:	The (CPU) physical address of the memory region.
 * @vpu_addr:	The VPU address of the memory region.
 * @size:	The size of the memory region.
 */
struct vpu_mem {
	void		*vaddr;
	phys_addr_t	paddr;
	dma_addr_t	vpu_addr;
	size_t		size;
};

/**
 * struct atf_mem - Information about reserved memory shared with ATF.
 * @vaddr:	The virtual address of the memory region.
 * @paddr:	The physical address of the memory region.
 * @size:	The size of the memory region.
 */
struct atf_mem {
	void __iomem	*vaddr;
	phys_addr_t	paddr;
	size_t		size;
};

/**
 * struct vpu_ipc_dev - The VPU IPC device structure.
 * @pdev:		Platform device associated with this VPU IPC device.
 * @state:		The current state of the device's finite state machine.
 * @reserved_mem:	VPU firmware reserved memory region. The VPU firmware,
 *			VPU firmware version and the boot parameters are loaded
 *			inside this region.
 * @x509_mem:		x509 reserved memory region.
 * @x509_size:		The size of the x509 certificate in the firmware binary.
 * @mss_ipc_mem:	The reserved memory from which the VPU is expected to
 *			allocate its own IPC buffers.
 * @boot_vec_paddr:	The VPU entry point (as specified in the VPU FW binary).
 * @boot_params:	Pointer to the VPU boot parameters.
 * @fw_res:		The memory region where the VPU FW was loaded.
 * @ready_message_task: The ktrhead instantiated to handle the reception of the
 *			VPU ready message.
 * @lock:		Spinlock protecting @state.
 * @cpu_clock:		The main clock powering the VPU IP.
 * @pll:		Array of PLL clocks.
 * @nce_irq:		IRQ number of the A53 re-direct IRQ used for receiving
 *			the NCE WDT timeout interrupt.
 * @mss_irq:		IRQ number of the A53 re-direct IRQ which will be used
 *			for receiving the MSS WDT timeout interrupt.
 * @vpu_id:		The ID of the VPU associated with this device.
 * @nce_wdt_reg:	NCE WDT registers.
 * @nce_tim_cfg_reg:	NCE TIM registers.
 * @mss_wdt_reg:	MSS WDT registers.
 * @mss_tim_cfg_reg:	MSS TIM registers.
 * @nce_wdt_count:	Number of NCE WDT timeout event occurred since device
 *			probing.
 * @mss_wdt_count:	Number of MSS WDT timeout event occurred since device
 *			probing.
 * @ready_queue:	Wait queue for waiting on VPU to be ready.
 * @kmb_ipc:		The IPC device to use for IPC communication.
 * @firmware_name:	The name of the firmware binary to be loaded.
 * @callback:		The callback executed when CONNECT / DISCONNECT events
 *			happen.
 * @tee_ctx:		TEE context for communication with VPU TA.
 * @shm:		TEE shared memory for exchanging data with VPU TA.
 * @tee_session:	TEE session with VPU TA.
 */
struct vpu_ipc_dev {
	struct platform_device		*pdev;
	enum intel_keembay_vpu_state	state;
	struct vpu_mem			reserved_mem;
	struct atf_mem			x509_mem;
	size_t				x509_size;
	struct vpu_mem			mss_ipc_mem;
	u64				boot_vec_paddr;
	struct boot_parameters		*boot_params;
	struct resource			fw_res;
	struct task_struct		*ready_message_task;
	spinlock_t			lock; /* Protects @state. */
	struct clk			*cpu_clock;
	struct clk			*pll[NUM_PLLS][NUM_PLL_OUTPUTS];
	int				nce_irq;
	int				mss_irq;
	u32 nce_wdt_redirect;
	u32 mss_wdt_redirect;
	u32 imr;
	u32				vpu_id;
	void __iomem			*nce_wdt_reg;
	void __iomem			*nce_tim_cfg_reg;
	void __iomem			*mss_wdt_reg;
	void __iomem			*mss_tim_cfg_reg;
	unsigned int			nce_wdt_count;
	unsigned int			mss_wdt_count;
	wait_queue_head_t		ready_queue;
	struct keembay_ipc		*kmb_ipc;
	char				*firmware_name;
	void				(*callback)(struct device *dev,
						    enum intel_keembay_vpu_event event);
	struct tee_context		*tee_ctx;
	struct tee_shm			*shm;
	u32				tee_session;
};

/**
 * struct vpu_ipc_soc_info - VPU SKU information.
 * @device_id:		Value of device ID e-fuse.
 * @feature_exclusion:	Value of feature exclusion e-fuse.
 * @hardware_id:	Hardware identifier.
 * @stepping:		Silicon stepping.
 * @sku:		SKU identifier.
 *
 * SoC information read from the device-tree and exported via sysfs.
 */
struct vpu_ipc_soc_info {
	u64 device_id;
	u64 feature_exclusion;
	u32 hardware_id;
	u8  stepping[SOC_INFO_STEPPING_BYTES];
	u8  sku[SOC_INFO_SKU_BYTES];
};

/**
 * enum keembay_vpu_event - Internal events handled by the driver state machine.
 * @KEEMBAY_VPU_EVENT_BOOT:		VPU booted.
 * @KEEMBAY_VPU_EVENT_BOOT_FAILED:	VPU boot failed.
 * @KEEMBAY_VPU_EVENT_STOP:		VPU stop initiated.
 * @KEEMBAY_VPU_EVENT_STOP_COMPLETE:	VPU stop completed.
 * @KEEMBAY_VPU_EVENT_NCE_WDT_TIMEOUT:	NCE watchdog triggered.
 * @KEEMBAY_VPU_EVENT_MSS_WDT_TIMEOUT:	MSS watchdog triggered.
 * @KEEMBAY_VPU_EVENT_MSS_READY_OK:	VPU ready message successfully received.
 * @KEEMBAY_VPU_EVENT_MSS_READY_FAIL:	Failed to receive VPU ready message.
 */
enum keembay_vpu_event {
	KEEMBAY_VPU_EVENT_BOOT = 0,
	KEEMBAY_VPU_EVENT_BOOT_FAILED,
	KEEMBAY_VPU_EVENT_STOP,
	KEEMBAY_VPU_EVENT_STOP_COMPLETE,
	KEEMBAY_VPU_EVENT_NCE_WDT_TIMEOUT,
	KEEMBAY_VPU_EVENT_MSS_WDT_TIMEOUT,
	KEEMBAY_VPU_EVENT_MSS_READY_OK,
	KEEMBAY_VPU_EVENT_MSS_READY_FAIL
};

/**
 * struct vpu_boot_ta_shmem
 * @vpu_mem_addr:	Start address of VPU allocated memory
 * @vpu_mem_size:	Size of VPU allocated memory
 * @x509_mem_addr:	Start address of memory allocated for X.509 certificate
 * @x509_mem_size:	Size of memory allocated for X.509 certificate
 * @fw_header_addr:	Memory address of VPU firmware header data
 * @fw_header_size:	Size of VPU header data
 * @fw_version_addr:	Memory address of VPU firmware version data
 * @fw_version_size:	Size of VPU firmware version data
 * @fw_load_addr:	Memory address of VPU firmware image
 * @fw_load_size:	Size of VPU firmware image
 * @x509_addr:		Memory address of VPU binary X.509 certificate
 * @x509_size:		Size of X.509 certificate
 * @fw_entry_addr:	VPU boot entry address
 * @vpu_id:		VPU ID to be used
 */
struct vpu_boot_ta_shmem {
	u64 fw_header_addr;
	u64 fw_header_size;
	u64 fw_version_addr;
	u64 fw_version_size;
	u64 fw_load_addr;
	u64 fw_load_size;
	u64 x509_addr;
	u64 x509_size;
	u64 fw_entry_addr;
	u32 vpu_id;
} __packed;

static struct vpu_ipc_dev *to_vpu_dev(struct device *dev);

/* Variable containing SoC information. */
static struct vpu_ipc_soc_info *vpu_ipc_soc_info;

/**
 * VPU_AUTH_TA_UUID: 7671AC66-1B46-4B34-B929-18BB15D398EE
 *
 * Randomly generated, and must correspond to the GUID on the TA side.
 */
static const uuid_t vpu_auth_ta_uuid =
	UUID_INIT(0x7671AC66, 0x1B46, 0x4B34,
		  0xB9, 0x29, 0x18, 0xBB, 0x15, 0xD3, 0x98, 0xEE);

/**
 * vpu_auth_ta_match() - Determine if GlobalPlatform compliant
 * @ver:		Version data of TEE device
 * @data:		Optional additional data for match function
 *
 * Returns: 1 if compliant, 0 if not compliant
 */
static int vpu_auth_ta_match(struct tee_ioctl_version_data *ver,
			     const void *data)
{
	return ver->gen_caps & TEE_GEN_CAP_GP;
}

/**
 * vpu_ipc_notify_event() - Trigger callback
 * @vpu_dev:		Private data
 * @event:		Event to notify
 *
 * This function is called when an event has occurred. If a callback has
 * been registered it is called with the device and event as arguments.
 *
 * This function can be called from interrupt context.
 */
static void vpu_ipc_notify_event(struct vpu_ipc_dev *vpu_dev,
				 enum intel_keembay_vpu_event event)
{
	struct device *dev = &vpu_dev->pdev->dev;

	if (vpu_dev->callback)
		vpu_dev->callback(dev, event);
}

/**
 * vpu_ipc_handle_event() - Handle events and optionally update state
 *
 * @vpu_dev:		Private data
 * @event:		Event that has occurred
 *
 * This function is called in the case that an event has occurred. This
 * function tells the calling code if the event is valid for the current state
 * and also updates the internal state accordingly to the event.
 *
 * This function can be called from interrupt context.
 *
 * Returns: 0 for success otherwise negative error value
 */
static int vpu_ipc_handle_event(struct vpu_ipc_dev *vpu_dev,
				enum keembay_vpu_event event)
{
	struct device *dev = &vpu_dev->pdev->dev;
	unsigned long flags;
	int rc = -EINVAL;

	/*
	 * Atomic update of state.
	 * Note: this function is called by the WDT IRQ handlers; therefore
	 * we must use the spin_lock_irqsave().
	 */
	spin_lock_irqsave(&vpu_dev->lock, flags);

	switch (vpu_dev->state) {
	case KEEMBAY_VPU_OFF:
		if (event == KEEMBAY_VPU_EVENT_BOOT) {
			vpu_dev->state = KEEMBAY_VPU_BUSY;
			rc = 0;
		}
		break;
	case KEEMBAY_VPU_BUSY:
		if (event == KEEMBAY_VPU_EVENT_MSS_READY_OK) {
			vpu_dev->state = KEEMBAY_VPU_READY;
			vpu_ipc_notify_event(vpu_dev,
					     KEEMBAY_VPU_NOTIFY_CONNECT);
			rc = 0;
			break;
		}
		if (event == KEEMBAY_VPU_EVENT_MSS_READY_FAIL ||
		    event == KEEMBAY_VPU_EVENT_BOOT_FAILED) {
			vpu_dev->state = KEEMBAY_VPU_ERROR;
			rc = 0;
		}
		break;
	case KEEMBAY_VPU_READY:
		if (event != KEEMBAY_VPU_EVENT_MSS_READY_OK)
			vpu_ipc_notify_event(vpu_dev,
					     KEEMBAY_VPU_NOTIFY_DISCONNECT);

		if (event == KEEMBAY_VPU_EVENT_MSS_READY_FAIL ||
		    event == KEEMBAY_VPU_EVENT_BOOT_FAILED) {
			vpu_dev->state = KEEMBAY_VPU_ERROR;
			rc = 0;
			break;
		}
		if (event == KEEMBAY_VPU_EVENT_NCE_WDT_TIMEOUT ||
		    event == KEEMBAY_VPU_EVENT_MSS_WDT_TIMEOUT) {
			vpu_dev->state = KEEMBAY_VPU_ERROR;
			rc = 0;
			break;
		}
		fallthrough;
	case KEEMBAY_VPU_ERROR:
		if (event == KEEMBAY_VPU_EVENT_BOOT) {
			vpu_dev->state = KEEMBAY_VPU_BUSY;
			rc = 0;
			break;
		}
		if (event == KEEMBAY_VPU_EVENT_STOP) {
			vpu_dev->state = KEEMBAY_VPU_STOPPING;
			rc = 0;
		}
		break;
	case KEEMBAY_VPU_STOPPING:
		if (event == KEEMBAY_VPU_EVENT_STOP_COMPLETE) {
			vpu_dev->state = KEEMBAY_VPU_OFF;
			rc = 0;
		}
		break;
	default:
		break;
	}

	spin_unlock_irqrestore(&vpu_dev->lock, flags);

	if (rc)
		dev_err(dev, "Can't handle event %d in state %d\n",
			event, vpu_dev->state);

	return rc;
}

/**
 * clear_and_disable_vpu_wdt() - Clear and disable VPU WDT.
 * @wdt_base:		Base address of the WDT register.
 * @tim_cfg_base:	Base address of the associated TIM configuration
 *			register.
 */
static void clear_and_disable_vpu_wdt(void __iomem *wdt_base,
				      void __iomem *tim_cfg_base)
{
	u32 tim_gen_config;

	/* Enable writing and set non-zero WDT value */
	iowrite32(TIM_SAFE_ENABLE, wdt_base + TIM_SAFE);
	iowrite32(TIM_WATCHDOG_RESET_VALUE, wdt_base + TIM_WATCHDOG);

	/* Enable writing and disable watchdog timer */
	iowrite32(TIM_SAFE_ENABLE, wdt_base + TIM_SAFE);
	iowrite32(0, wdt_base + TIM_WDOG_EN);

	/* Now clear the timeout interrupt */
	tim_gen_config = ioread32(tim_cfg_base);
	tim_gen_config &= ~(TIM_GEN_CONFIG_WDOG_TO_INT_CLR);
	iowrite32(tim_gen_config, tim_cfg_base);
}

static irqreturn_t nce_wdt_irq_handler(int irq, void *ptr)
{
	struct vpu_ipc_dev *vpu_dev = ptr;
	struct device *dev = &vpu_dev->pdev->dev;
	int rc;

	vpu_ipc_notify_event(vpu_dev, KEEMBAY_VPU_NOTIFY_NCE_WDT);
	dev_dbg_ratelimited(dev, "NCE WDT IRQ occurred.\n");

	clear_and_disable_vpu_wdt(vpu_dev->nce_wdt_reg,
				  vpu_dev->nce_tim_cfg_reg);
	/* Update driver state machine. */
	rc = vpu_ipc_handle_event(vpu_dev, KEEMBAY_VPU_EVENT_NCE_WDT_TIMEOUT);
	if (rc < 0)
		dev_warn_ratelimited(dev, "Unexpected NCE WDT event.\n");

	vpu_dev->nce_wdt_count++;

	return IRQ_HANDLED;
}

static irqreturn_t mss_wdt_irq_handler(int irq, void *ptr)
{
	struct vpu_ipc_dev *vpu_dev = ptr;
	struct device *dev = &vpu_dev->pdev->dev;
	int rc;

	vpu_ipc_notify_event(vpu_dev, KEEMBAY_VPU_NOTIFY_MSS_WDT);
	dev_dbg_ratelimited(dev, "MSS WDT IRQ occurred.\n");

	clear_and_disable_vpu_wdt(vpu_dev->mss_wdt_reg,
				  vpu_dev->mss_tim_cfg_reg);
	/* Update driver state machine. */
	rc = vpu_ipc_handle_event(vpu_dev, KEEMBAY_VPU_EVENT_MSS_WDT_TIMEOUT);
	if (rc < 0)
		dev_warn_ratelimited(dev, "Unexpected MSS WDT event.\n");

	vpu_dev->mss_wdt_count++;

	return IRQ_HANDLED;
}

static resource_size_t get_reserved_mem_size(struct device *dev,
					     unsigned int idx)
{
	struct resource mem;
	struct device_node *np;
	int rc;

	np = of_parse_phandle(dev->of_node, "memory-region", idx);
	if (!np) {
		pr_err("Couldn't find memory-region %d\n", idx);
		return 0;
	}

	rc = of_address_to_resource(np, 0, &mem);
	if (rc) {
		pr_err("Couldn't map address to resource\n");
		return 0;
	}

	return resource_size(&mem);
}

static int setup_vpu_fw_region(struct vpu_ipc_dev *vpu_dev)
{
	struct device *dev = &vpu_dev->pdev->dev;
	struct vpu_mem *rsvd_mem = &vpu_dev->reserved_mem;
	int mem_idx;
	int rc;

	rc = of_reserved_mem_device_init_by_name(dev, dev->of_node,
						 VPU_IPC_FW_AREA);
	if (rc) {
		dev_err(dev, "Failed to initialise device reserved memory.\n");
		return rc;
	}

	/* Get memory region ID from "memory-region-names". */
	mem_idx = of_property_match_string(dev->of_node, "memory-region-names",
					   VPU_IPC_FW_AREA);

	rsvd_mem->size = get_reserved_mem_size(dev, mem_idx);
	if (rsvd_mem->size == 0) {
		dev_err(dev, "Couldn't get size of reserved memory region.\n");
		rc = -ENODEV;
		goto setup_vpu_fw_fail;
	}

	rsvd_mem->vaddr = dmam_alloc_coherent(dev, rsvd_mem->size,
					      &rsvd_mem->vpu_addr, GFP_KERNEL);
	/* Get the physical address of the reserved memory region. */
	rsvd_mem->paddr = dma_to_phys(dev, vpu_dev->reserved_mem.vpu_addr);

	if (!rsvd_mem->vaddr) {
		dev_err(dev, "Failed to allocate memory for firmware.\n");
		rc = -ENOMEM;
		goto setup_vpu_fw_fail;
	}

	return 0;

setup_vpu_fw_fail:
	of_reserved_mem_device_release(dev);

	return rc;
}

static int setup_x509_region(struct vpu_ipc_dev *vpu_dev)
{
	struct device *dev = &vpu_dev->pdev->dev;
	struct device_node *node;
	struct resource res;
	int mem_idx;
	int rc;

	/* Get memory region ID from "memory-region-names". */
	mem_idx = of_property_match_string(dev->of_node, "memory-region-names",
					   VPU_IPC_X509_AREA);

	node = of_parse_phandle(dev->of_node, "memory-region", mem_idx);
	if (!node) {
		dev_err(dev, "Couldn't find X509 region.\n");
		return -EINVAL;
	}

	rc = of_address_to_resource(node, 0, &res);

	/* Release node first as we will not use it anymore */
	of_node_put(node);

	if (rc) {
		dev_err(dev, "Couldn't resolve X509 region.\n");
		return rc;
	}

	vpu_dev->x509_mem.vaddr =
		devm_ioremap(dev, res.start, resource_size(&res));
	if (!vpu_dev->x509_mem.vaddr) {
		dev_err(dev, "Couldn't ioremap x509 region.\n");
		return -EADDRNOTAVAIL;
	}

	vpu_dev->x509_mem.paddr = res.start;
	vpu_dev->x509_mem.size = resource_size(&res);

	return 0;
}

static int setup_reserved_memory(struct vpu_ipc_dev *vpu_dev)
{
	struct device *dev = &vpu_dev->pdev->dev;
	int rc;

	/*
	 * Find the VPU firmware area described in the device tree,
	 * and allocate it for our usage.
	 */
	rc = setup_vpu_fw_region(vpu_dev);
	if (rc) {
		dev_err(dev, "Failed to init FW memory.\n");
		return rc;
	}

	/*
	 * Find the X509 area described in the device tree,
	 * and allocate it for our usage.
	 */
	rc = setup_x509_region(vpu_dev);
	if (rc) {
		dev_err(dev, "Failed to setup X509 region.\n");
		goto res_mem_setup_fail;
	}

	return 0;

res_mem_setup_fail:
	/*
	 * Explicitly free memory allocated by setup_reserved_memory(); this is
	 * needed in order to safely call of_reserved_mem_device_release()
	 */
	dmam_free_coherent(dev, vpu_dev->reserved_mem.size,
			   vpu_dev->reserved_mem.vaddr,
			   vpu_dev->reserved_mem.vpu_addr);
	of_reserved_mem_device_release(dev);

	return rc;
}

static int retrieve_clocks(struct vpu_ipc_dev *vpu_dev)
{
	struct device *dev = &vpu_dev->pdev->dev;
	char pll_string[PLL_STRING_SIZE];
	struct clk *clk;
	int pll;
	int out;

	clk = devm_clk_get(dev, "cpu_clock");
	if (IS_ERR(clk)) {
		dev_err(dev, "cpu_clock not found.\n");
		return PTR_ERR(clk);
	}
	vpu_dev->cpu_clock = clk;

	for (pll = 0; pll < NUM_PLLS; pll++) {
		for (out = 0; out < NUM_PLL_OUTPUTS; out++) {
			sprintf(pll_string, "pll_%d_out_%d", pll, out);
			clk = devm_clk_get(dev, pll_string);
			if (IS_ERR(clk)) {
				dev_err(dev, "%s not found.\n", pll_string);
				return PTR_ERR(clk);
			}
			vpu_dev->pll[pll][out] = clk;
		}
	}

	return 0;
}

/* Get register resource from device tree and re-map as I/O memory. */
static int get_pdev_res_and_ioremap(struct platform_device *pdev,
				    const char *reg_name,
				    void __iomem **target_reg)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	void __iomem *reg;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, reg_name);
	if (!res) {
		dev_err(dev, "Couldn't get resource for %s\n", reg_name);
		return -EINVAL;
	}

	reg = devm_ioremap_resource(dev, res);
	if (IS_ERR(reg)) {
		dev_err(dev, "Couldn't ioremap resource for %s\n", reg_name);
		return PTR_ERR(reg);
	}

	*target_reg = reg;

	return 0;
}

static int setup_watchdog_resources(struct vpu_ipc_dev *vpu_dev)
{
	struct platform_device *pdev = vpu_dev->pdev;
	struct device *dev = &vpu_dev->pdev->dev;
	int rc;

	/* Get registers */
	rc = get_pdev_res_and_ioremap(pdev, "nce_wdt", &vpu_dev->nce_wdt_reg);
	if (rc) {
		dev_err(dev, "Failed to get NCE WDT registers\n");
		return rc;
	}
	rc = get_pdev_res_and_ioremap(pdev, "nce_tim_cfg",
				      &vpu_dev->nce_tim_cfg_reg);
	if (rc) {
		dev_err(dev, "Failed to get NCE TIM_GEN_CONFIG register\n");
		return rc;
	}
	rc = get_pdev_res_and_ioremap(pdev, "mss_wdt", &vpu_dev->mss_wdt_reg);
	if (rc) {
		dev_err(dev, "Failed to get MSS WDT registers\n");
		return rc;
	}
	rc = get_pdev_res_and_ioremap(pdev, "mss_tim_cfg",
				      &vpu_dev->mss_tim_cfg_reg);
	if (rc) {
		dev_err(dev, "Failed to get MSS TIM_GEN_CONFIG register\n");
		return rc;
	}

	/* Request interrupts */
	vpu_dev->nce_irq = platform_get_irq_byname(pdev, "nce_wdt");
	if (vpu_dev->nce_irq < 0)
		return vpu_dev->nce_irq;
	vpu_dev->mss_irq = platform_get_irq_byname(pdev, "mss_wdt");
	if (vpu_dev->mss_irq < 0)
		return vpu_dev->mss_irq;
	rc = devm_request_irq(dev, vpu_dev->nce_irq, nce_wdt_irq_handler, 0,
			      "keembay-vpu-ipc", vpu_dev);
	if (rc) {
		dev_err(dev, "failed to request NCE IRQ.\n");
		return rc;
	}
	rc = devm_request_irq(dev, vpu_dev->mss_irq, mss_wdt_irq_handler, 0,
			      "keembay-vpu-ipc", vpu_dev);
	if (rc) {
		dev_err(dev, "failed to request MSS IRQ.\n");
		return rc;
	}

	/* Request interrupt re-direct numbers */
	rc = of_property_read_u32(dev->of_node,
				  "intel,keembay-vpu-ipc-nce-wdt-redirect",
				  &vpu_dev->nce_wdt_redirect);
	if (rc) {
		dev_err(dev, "failed to get NCE WDT redirect number.\n");
		return rc;
	}
	rc = of_property_read_u32(dev->of_node,
				  "intel,keembay-vpu-ipc-mss-wdt-redirect",
				  &vpu_dev->mss_wdt_redirect);
	if (rc) {
		dev_err(dev, "failed to get MSS WDT redirect number.\n");
		return rc;
	}

	return 0;
}

/* Populate the boot parameters to be passed to the VPU. */
static int setup_boot_parameters(struct vpu_ipc_dev *vpu_dev)
{
	int i;

	/* Set all values to zero. This will disable most clocks/devices */
	memset(vpu_dev->boot_params, 0, sizeof(*vpu_dev->boot_params));

	/*
	 * Set magic number, so VPU knows that the parameters are
	 * populated correctly
	 */
	vpu_dev->boot_params->magic_number = BOOT_PARAMS_MAGIC_NUMBER;

	/* Set VPU ID. */
	vpu_dev->boot_params->vpu_id = vpu_dev->vpu_id;

	/* Inform VPU of clock frequencies */
	vpu_dev->boot_params->cpu_frequency_hz =
		clk_get_rate(vpu_dev->cpu_clock);
	for (i = 0; i < NUM_PLL_OUTPUTS; i++) {
		vpu_dev->boot_params->pll0_out[i] =
			clk_get_rate(vpu_dev->pll[0][i]);
		vpu_dev->boot_params->pll1_out[i] =
			clk_get_rate(vpu_dev->pll[1][i]);
		vpu_dev->boot_params->pll2_out[i] =
			clk_get_rate(vpu_dev->pll[2][i]);
	}

	/*
	 * Fill in IPC buffer information: the VPU needs to know where it
	 * should allocate IPC buffer from.
	 */
	vpu_dev->boot_params->mss_ipc_header_address =
		vpu_dev->mss_ipc_mem.vpu_addr;
	vpu_dev->boot_params->mss_ipc_header_area_size =
		vpu_dev->mss_ipc_mem.size;

	/* Fill in IRQ re-direct request information */
	vpu_dev->boot_params->mss_wdt_to_irq_a53_redir =
			vpu_dev->mss_wdt_redirect;
	vpu_dev->boot_params->nce_wdt_to_irq_a53_redir =
			vpu_dev->nce_wdt_redirect;

	/* Setup A53SS_VERSION_ID */
	vpu_dev->boot_params->a53ss_version_id = vpu_ipc_soc_info->hardware_id;

	/* Setup Silicon stepping */
	vpu_dev->boot_params->si_stepping = vpu_ipc_soc_info->stepping[0] |
					    vpu_ipc_soc_info->stepping[1] << 8;

	/* Set feature exclude and device id information. */
	vpu_dev->boot_params->device_id = vpu_ipc_soc_info->device_id;
	vpu_dev->boot_params->feature_exclusion =
					vpu_ipc_soc_info->feature_exclusion;

	/* Set SKU information */
	vpu_dev->boot_params->sku = (u64)vpu_ipc_soc_info->sku[0] |
				    (u64)vpu_ipc_soc_info->sku[1] << 8 |
				    (u64)vpu_ipc_soc_info->sku[2] << 16 |
				    (u64)vpu_ipc_soc_info->sku[3] << 24 |
				    (u64)vpu_ipc_soc_info->sku[4] << 32 |
				    (u64)vpu_ipc_soc_info->sku[5] << 40;
	return 0;
}

static void init_ta_args(struct vpu_ipc_dev *vpu_dev,
			 struct tee_ioctl_invoke_arg *inv_arg,
			 struct tee_param *param)
{
	memset(inv_arg, 0, sizeof(*inv_arg));
	memset(param, 0, sizeof(*param) * NUM_TEE_PARAMS);

	inv_arg->session = vpu_dev->tee_session;
	inv_arg->num_params = NUM_TEE_PARAMS;

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
	param[0].u.memref.shm_offs = 0x0;
	param[0].u.memref.size = vpu_dev->shm->size;
	param[0].u.memref.shm = vpu_dev->shm;
}

/* Request SoC firmware to boot the VPU. */
static int request_vpu_boot(struct vpu_ipc_dev *vpu_dev)
{
	struct vpu_boot_ta_shmem *vpu_boot_ta_args;
	struct device *dev = &vpu_dev->pdev->dev;
	struct tee_param param[NUM_TEE_PARAMS];
	struct tee_ioctl_invoke_arg inv_arg;
	int ret;

	init_ta_args(vpu_dev, &inv_arg, param);
	inv_arg.func = KMB_TA_VPU_BOOT;

	vpu_boot_ta_args = tee_shm_get_va(vpu_dev->shm, 0);
	if (IS_ERR(vpu_boot_ta_args)) {
		dev_err(dev, "Failed to get address of TEE shared memory\n");
		return PTR_ERR(vpu_boot_ta_args);
	}

	memset(vpu_boot_ta_args, 0, sizeof(*vpu_boot_ta_args));

	vpu_boot_ta_args->fw_header_addr = vpu_dev->x509_mem.paddr +
					   vpu_dev->x509_size;
	vpu_boot_ta_args->fw_header_size = MAX_HEADER_SIZE;
	vpu_boot_ta_args->fw_load_addr = dma_to_phys(dev, vpu_dev->fw_res.start);
	vpu_boot_ta_args->fw_load_size = resource_size(&vpu_dev->fw_res);
	vpu_boot_ta_args->fw_version_addr = vpu_boot_ta_args->fw_load_addr -
					    MAX_FIRMWARE_VERSION_SIZE;
	vpu_boot_ta_args->fw_version_size = MAX_FIRMWARE_VERSION_SIZE;
	vpu_boot_ta_args->x509_addr = vpu_dev->x509_mem.paddr;
	vpu_boot_ta_args->x509_size = vpu_dev->x509_size;
	vpu_boot_ta_args->fw_entry_addr = dma_to_phys(dev, vpu_dev->boot_vec_paddr);
	vpu_boot_ta_args->vpu_id = vpu_dev->vpu_id;

	ret = tee_client_invoke_func(vpu_dev->tee_ctx, &inv_arg, param);
	if (ret < 0 || inv_arg.ret != 0) {
		dev_err(dev, "KMB_TA_VPU_BOOT invoke err: %x\n",
			inv_arg.ret);
		if (ret)
			return ret;
		return -EINVAL;
	}

	dev_info(&vpu_dev->pdev->dev,
		 "VPU Boot successfully requested to secure monitor.\n");

	return 0;
}

/* Request SoC firmware to stop the VPU. */
static int request_vpu_stop(struct vpu_ipc_dev *vpu_dev)
{
	struct vpu_boot_ta_shmem *vpu_boot_ta_args;
	struct device *dev = &vpu_dev->pdev->dev;
	struct tee_param param[NUM_TEE_PARAMS];
	struct tee_ioctl_invoke_arg inv_arg;
	int ret;

	init_ta_args(vpu_dev, &inv_arg, param);
	inv_arg.func = KMB_TA_VPU_STOP;

	vpu_boot_ta_args = tee_shm_get_va(vpu_dev->shm, 0);
	if (IS_ERR(vpu_boot_ta_args)) {
		dev_err(dev, "Failed to get address of TEE shared memory\n");
		return PTR_ERR(vpu_boot_ta_args);
	}

	memset(vpu_boot_ta_args, 0, sizeof(*vpu_boot_ta_args));

	vpu_boot_ta_args->vpu_id = vpu_dev->vpu_id;

	ret = tee_client_invoke_func(vpu_dev->tee_ctx, &inv_arg, param);
	if (ret < 0 || inv_arg.ret != 0) {
		dev_err(dev, "KMB_TA_VPU_STOP invoke err: %x\n",
			inv_arg.ret);
		if (ret)
			return ret;
		return -EINVAL;
	}

	dev_info(&vpu_dev->pdev->dev,
		 "VPU Stop successfully requested to secure monitor.\n");

	return 0;
}

/*
 * Get kernel virtual address of resource inside the VPU reserved memory
 * region.
 */
static void *get_vpu_dev_vaddr(struct vpu_ipc_dev *vpu_dev,
			       struct resource *res)
{
	unsigned long offset;

	/* Given the calculation below, this must not be true. */
	if (res->start < vpu_dev->reserved_mem.vpu_addr)
		return NULL;

	offset = res->start - vpu_dev->reserved_mem.vpu_addr;

	return vpu_dev->reserved_mem.vaddr + offset;
}

static int parse_fw_header(struct vpu_ipc_dev *vpu_dev,
			   const struct firmware *fw)
{
	struct resource config_res, version_res, total_reserved_res;
	struct device *dev = &vpu_dev->pdev->dev;
	struct firmware_header *fw_header;
	void *version_region;
	void *config_region;
	void __iomem *x509_region;
	void *fw_region;

	/* Is the fw size big enough to read the header? */
	if (fw->size < sizeof(struct firmware_header)) {
		dev_err(dev, "Firmware was too small for header.\n");
		return -EINVAL;
	}

	fw_header = (struct firmware_header *)fw->data;

	/* Check header version */
	if (fw_header->header_ver != HEADER_VERSION_SUPPORTED) {
		dev_err(dev, "Header version check expected 0x%x, got 0x%x\n",
			HEADER_VERSION_SUPPORTED, fw_header->header_ver);
		return -EINVAL;
	}

	/* Check firmware version size is allowed */
	if (fw_header->fw_ver_size > MAX_FIRMWARE_VERSION_SIZE) {
		dev_err(dev, "Firmware version area larger than allowed: %d\n",
			fw_header->fw_ver_size);
		return -EINVAL;
	}

	/*
	 * Check the firmware binary is at least large enough for the
	 * firmware image size described in the header.
	 */
	if (fw->size < (MAX_HEADER_SIZE + MAX_FIRMWARE_VERSION_SIZE +
			fw_header->image_size)) {
		dev_err(dev, "Real firmware size is not large enough.\n");
		return -EINVAL;
	}

	/*
	 * Make sure that the final address is aligned correctly. If not, the
	 * boot will never work.
	 */
	if (!IS_ALIGNED(fw_header->entry_point, VPU_RESET_VECTOR_ALIGNMENT)) {
		dev_err(dev,
			"Entry point for firmware (0x%llX) is not correctly aligned.\n",
			fw_header->entry_point);
		return -EINVAL;
	}

	/*
	 * Generate the resource describing the region containing the actual
	 * firmware data.
	 */
	vpu_dev->fw_res.start = fw_header->image_load_addr;
	vpu_dev->fw_res.end =
		fw_header->image_size + fw_header->image_load_addr - 1;
	vpu_dev->fw_res.flags = IORESOURCE_MEM;

	/*
	 * Generate the resource describing the region containing the
	 * configuration data for the VPU.
	 */
	config_res.start = fw_header->config_load_addr;
	config_res.end = sizeof(struct boot_parameters) +
			 fw_header->config_load_addr - 1;
	config_res.flags = IORESOURCE_MEM;

	/*
	 * Generate the resource describing the region containing the
	 * version information for the VPU.
	 */
	version_res.start = fw_header->fw_ver_load_addr;
	version_res.end = fw_header->fw_ver_size +
			  fw_header->fw_ver_load_addr - 1;
	version_res.flags = IORESOURCE_MEM;

	/*
	 * Generate the resource describing the region of memory
	 * completely dedicated to the VPU.
	 */
	total_reserved_res.start = vpu_dev->reserved_mem.vpu_addr;
	total_reserved_res.end = vpu_dev->reserved_mem.vpu_addr +
		vpu_dev->reserved_mem.size - 1;
	total_reserved_res.flags = IORESOURCE_MEM;

	/*
	 * Check all pieces to be copied reside completely in the reserved
	 * region
	 */
	if (!resource_contains(&total_reserved_res, &vpu_dev->fw_res)) {
		dev_err(dev, "Can't fit firmware in reserved region.\n");
		return -EINVAL;
	}
	if (!resource_contains(&total_reserved_res, &version_res)) {
		dev_err(dev,
			"Can't fit firmware version data in reserved region.\n");
		return -EINVAL;
	}
	if (!resource_contains(&total_reserved_res, &config_res)) {
		dev_err(dev,
			"Can't fit configuration information in reserved region.\n");
		return -EINVAL;
	}

	/* Check for overlapping regions */
	if (resource_overlaps(&vpu_dev->fw_res, &version_res)) {
		dev_err(dev, "FW and version regions overlap.\n");
		return -EINVAL;
	}
	if (resource_overlaps(&vpu_dev->fw_res, &config_res)) {
		dev_err(dev, "FW and config regions overlap.\n");
		return -EINVAL;
	}
	if (resource_overlaps(&config_res, &version_res)) {
		dev_err(dev, "Version and config regions overlap.\n");
		return -EINVAL;
	}

	/* Setup boot parameter region */
	config_region = get_vpu_dev_vaddr(vpu_dev, &config_res);
	if (!config_region) {
		dev_err(dev,
			"Couldn't map boot configuration area to CPU virtual address.\n");
		return -EINVAL;
	}
	version_region = get_vpu_dev_vaddr(vpu_dev, &version_res);
	if (!version_region) {
		dev_err(dev,
			"Couldn't map version area to CPU virtual address.\n");
		return -EINVAL;
	}
	fw_region = get_vpu_dev_vaddr(vpu_dev, &vpu_dev->fw_res);
	if (!fw_region) {
		dev_err(dev,
			"Couldn't map firmware area to CPU virtual address.\n");
		return -EINVAL;
	}

	/*
	 * Copy version region: the region is located in the file @ offset of
	 * MAX_HEADER_SIZE, size was specified in the header and has been
	 * checked to not be larger than that allowed.
	 */
	memcpy(version_region, &fw->data[MAX_HEADER_SIZE],
	       fw_header->fw_ver_size);

	/*
	 * Copy firmware region: the region is located in the file @ offset of
	 * MAX_HEADER_SIZE + MAX_FIRMWARE_VERSION_SIZE, size was specified in
	 * the header and has been checked to not be larger than that allowed.
	 */
	memcpy(fw_region,
	       &fw->data[MAX_HEADER_SIZE + MAX_FIRMWARE_VERSION_SIZE],
	       fw_header->image_size);

	/* Save off boot parameters region vaddr */
	vpu_dev->boot_params = (struct boot_parameters *)config_region;

	/* Save off boot vector physical address */
	vpu_dev->boot_vec_paddr = fw_header->entry_point;

	/* Handle X.509 region */
	x509_region = vpu_dev->x509_mem.vaddr;
	if (!x509_region) {
		dev_err(dev,
			"Couldn't get CPU virtual address for X.509 certificate area.\n");
		return -EINVAL;
	}

	/*
	 * fw->size already checked to be >= (MAX_HEADER_SIZE +
	 * MAX_FIRMWARE_VERSION_SIZE + fw_header->image_size) above.
	 */
	vpu_dev->x509_size = fw->size - (MAX_HEADER_SIZE +
					 MAX_FIRMWARE_VERSION_SIZE +
					 fw_header->image_size);

	if (vpu_dev->x509_size + MAX_HEADER_SIZE > vpu_dev->x509_mem.size) {
		dev_err(dev,
			"X.509 cert + fw header exceeds allowable size.\n");
		return -EINVAL;
	}

	/*
	 * An X.509 certificate is optional on a non-secure system.
	 * On a secure system the TEE will not boot the VPU if a
	 * valid X.509 certificate is not present.
	 */
	if (vpu_dev->x509_size) {
		if (vpu_dev->x509_size > MAX_X509_SIZE) {
			dev_err(dev,
				"X.509 cert size exceeds maximum allowable.\n");
			return -EINVAL;
		}

		/*
		 * Copy X.509 certificate: the cert is located in the file
		 * @ offset of MAX_HEADER_SIZE + MAX_FIRMWARE_VERSION_SIZE +
		 * fw_header->image_size.
		 * Size has been checked to not be larger than that allowed.
		 */
		memcpy_toio(x509_region,
			    &fw->data[MAX_HEADER_SIZE +
				      MAX_FIRMWARE_VERSION_SIZE +
				      fw_header->image_size],
			    vpu_dev->x509_size);
	}

	/*
	 * Copy header to X.509 region directly after certificate
	 * Certificate size + MAX_HEADER_SIZE has already been checked
	 * to confirm both will fit in the allocated memory.
	 */
	memcpy_toio(x509_region + vpu_dev->x509_size, fw->data,
		    MAX_HEADER_SIZE);

	return 0;
}

static int ready_message_wait_thread(void *arg)
{
	struct vpu_ipc_dev *vpu_dev = arg;
	struct device *dev = &vpu_dev->pdev->dev;
	size_t size = 0;
	u32 paddr = 0;
	int close_rc;
	int rc;

	/*
	 * We will wait a few seconds for the message. We will complete earlier
	 * if the message is received earlier.
	 * NOTE: this is not a busy wait, we sleep until message is received.
	 */
	rc = keembay_ipc_recv(vpu_dev->kmb_ipc, READY_MESSAGE_IPC_CHANNEL,
			      &paddr, &size, READY_MESSAGE_TIMEOUT_MS);
	/*
	 * IPC channel must be closed regardless of 'rc' value, so close the
	 * channel now and then process 'rc' value.
	 */
	close_rc = keembay_ipc_close_channel(vpu_dev->kmb_ipc,
					     READY_MESSAGE_IPC_CHANNEL);
	if (close_rc < 0) {
		dev_warn(dev, "Couldn't close IPC channel.\n");
		/* Continue, as this is not a critical issue. */
	}

	/* Now process recv() return code. */
	if (rc < 0) {
		dev_err(dev,
			"Failed to receive ready message within %d ms: %d.\n",
			READY_MESSAGE_TIMEOUT_MS, rc);
		goto ready_message_thread_failure;
	}

	if (paddr != READY_MESSAGE_EXPECTED_PADDR ||
	    size != READY_MESSAGE_EXPECTED_SIZE) {
		dev_err(dev, "Bad ready message: (paddr, size) = (0x%x, %zu)\n",
			paddr, size);
		goto ready_message_thread_failure;
	}

	dev_info(dev, "VPU ready message received successfully!\n");

	rc = vpu_ipc_handle_event(vpu_dev, KEEMBAY_VPU_EVENT_MSS_READY_OK);
	if (rc < 0)
		dev_err(dev, "Fatal error: failed to set state (ready ok).\n");

	/* Wake up anyone waiting for READY. */
	wake_up_all(&vpu_dev->ready_queue);

	return 0;

ready_message_thread_failure:
	rc = vpu_ipc_handle_event(vpu_dev, KEEMBAY_VPU_EVENT_MSS_READY_FAIL);
	if (rc < 0)
		dev_err(dev,
			"Fatal error: failed to set state (ready timeout).\n");

	return 0;
}

static int create_ready_message_thread(struct vpu_ipc_dev *vpu_dev)
{
	struct device *dev = &vpu_dev->pdev->dev;
	struct task_struct *task;

	task = kthread_run(&ready_message_wait_thread, (void *)vpu_dev,
			   "keembay-vpu-ipc-ready");
	if (IS_ERR(task)) {
		dev_err(dev, "Couldn't start thread to receive message.\n");
		return -EIO;
	}

	vpu_dev->ready_message_task = task;

	return 0;
}

static int kickoff_vpu_sequence(struct vpu_ipc_dev *vpu_dev)
{
	struct device *dev = &vpu_dev->pdev->dev;
	int err_rc;
	int rc;

	/*
	 * Open the IPC channel. If we don't do it before booting
	 * the VPU, we may miss the message, as the IPC driver will
	 * discard messages for unopened channels.
	 */
	rc = keembay_ipc_open_channel(vpu_dev->kmb_ipc,
				      READY_MESSAGE_IPC_CHANNEL);
	if (rc < 0) {
		dev_err(dev,
			"Couldn't open IPC channel to receive ready message.\n");
		goto kickoff_failed;
	}

	/* Request boot */
	rc = request_vpu_boot(vpu_dev);
	if (rc < 0) {
		dev_err(dev, "Failed to do request to boot.\n");
		goto close_and_kickoff_failed;
	}

	/*
	 * Start thread waiting for message, and update state
	 * if the request was successful.
	 */
	rc = create_ready_message_thread(vpu_dev);
	if (rc < 0) {
		dev_err(dev,
			"Failed to create thread to wait for ready message.\n");
		goto close_and_kickoff_failed;
	}

	return 0;

close_and_kickoff_failed:
	/* Close the channel. */
	err_rc = keembay_ipc_close_channel(vpu_dev->kmb_ipc,
					   READY_MESSAGE_IPC_CHANNEL);
	if (err_rc < 0) {
		dev_err(dev, "Couldn't close IPC channel: %d\n", err_rc);
		/*
		 * We have had a more serious failure - don't update the
		 * original 'rc' and continue.
		 */
	}

kickoff_failed:
	return rc;
}

/*
 * Try to boot the VPU using the firmware name stored in
 * vpu_dev->firmware_name (which when this function is called is expected to be
 * not NULL).
 */
static int do_boot_sequence(struct vpu_ipc_dev *vpu_dev)
{
	struct device *dev = &vpu_dev->pdev->dev;
	const struct firmware *fw;
	int event_rc;
	int rc;

	/* Update state machine. */
	rc = vpu_ipc_handle_event(vpu_dev, KEEMBAY_VPU_EVENT_BOOT);
	if (rc < 0) {
		dev_err(dev, "Can't start in this state.\n");
		return rc;
	}

	/* Stop the VPU running */
	rc = request_vpu_stop(vpu_dev);
	if (rc < 0)
		dev_err(dev, "Failed stop - continue sequence anyway.\n");

	dev_info(dev, "Keem Bay VPU IPC start with %s.\n",
		 vpu_dev->firmware_name);

	/* Request firmware and wait for it. */
	rc = request_firmware(&fw, vpu_dev->firmware_name, &vpu_dev->pdev->dev);
	if (rc < 0) {
		dev_err(dev, "Couldn't find firmware: %d\n", rc);
		goto boot_failed_no_fw;
	}

	/* Do checks on the firmware header. */
	rc = parse_fw_header(vpu_dev, fw);
	if (rc < 0) {
		dev_err(dev, "Firmware checks failed.\n");
		goto boot_failed;
	}

	/* Write configuration data. */
	rc = setup_boot_parameters(vpu_dev);
	if (rc < 0) {
		dev_err(dev, "Failed to set up boot parameters.\n");
		goto boot_failed;
	}

	/* Try 'boot' sequence */
	rc = kickoff_vpu_sequence(vpu_dev);
	if (rc < 0) {
		dev_err(dev, "Failed to boot VPU.\n");
		goto boot_failed;
	}

	release_firmware(fw);
	return 0;

boot_failed:
	release_firmware(fw);

boot_failed_no_fw:
	/* Update state machine after failure. */
	event_rc = vpu_ipc_handle_event(vpu_dev,
					KEEMBAY_VPU_EVENT_BOOT_FAILED);
	if (event_rc < 0) {
		dev_err(dev,
			"Unexpected error: failed to handle fail event: %d.\n",
			event_rc);
		/* Continue: prefer original 'rc' to 'event_rc'. */
	}

	return rc;
}

static int validate_api_args(struct vpu_ipc_dev *vpu_dev, u8 node_id)
{
	if (IS_ERR(vpu_dev))
		return PTR_ERR(vpu_dev);

	if (node_id != KMB_VPU_IPC_NODE_LEON_MSS) {
		dev_warn(&vpu_dev->pdev->dev, "Invalid Link ID\n");
		return -EINVAL;
	}

	return 0;
}

/**
 * intel_keembay_vpu_ipc_open_channel() - Open an IPC channel.
 * @dev:	The VPU IPC device to use.
 * @node_id:	The node ID of the remote node (used to identify the link the
 *		channel must be added to). KMB_IPC_NODE_LEON_MSS is the only
 *		allowed value for now.
 * @chan_id:	The ID of the channel to be opened.
 *
 * Return:	0 on success, negative error code otherwise.
 */
int intel_keembay_vpu_ipc_open_channel(struct device *dev, u8 node_id,
				       u16 chan_id)
{
	struct vpu_ipc_dev *vpu_dev = to_vpu_dev(dev);
	int rc;

	rc = validate_api_args(vpu_dev, node_id);
	if (rc)
		return rc;
	return keembay_ipc_open_channel(vpu_dev->kmb_ipc, chan_id);
}
EXPORT_SYMBOL(intel_keembay_vpu_ipc_open_channel);

/**
 * intel_keembay_vpu_ipc_close_channel() - Close an IPC channel.
 * @dev:       The VPU IPC device to use.
 * @node_id:   The node ID of the remote node (used to identify the link the
 *             channel must be added to). KMB_IPC_NODE_LEON_MSS is the only
 *             allowed value for now.
 * @chan_id:   The ID of the channel to be closed.
 *
 * Return:     0 on success, negative error code otherwise.
 */

int intel_keembay_vpu_ipc_close_channel(struct device *dev, u8 node_id,
					u16 chan_id)
{
	struct vpu_ipc_dev *vpu_dev = to_vpu_dev(dev);
	int rc;

	rc = validate_api_args(vpu_dev, node_id);
	if (rc)
		return rc;

	return keembay_ipc_close_channel(vpu_dev->kmb_ipc, chan_id);
}
EXPORT_SYMBOL(intel_keembay_vpu_ipc_close_channel);

/**
 * intel_keembay_vpu_ipc_send() - Send data via IPC.
 * @dev:	The VPU IPC device to use.
 * @node_id:	The node ID of the remote node (used to identify the link the
 *		channel must be added to). KMB_IPC_NODE_LEON_MSS is the only
 *		allowed value for now.
 * @chan_id:	The IPC channel to be used to send the message.
 * @vpu_addr:	The VPU address of the data to be transferred.
 * @size:	The size of the data to be transferred.
 *
 * Return:	0 on success, negative error code otherwise.
 */
int intel_keembay_vpu_ipc_send(struct device *dev, u8 node_id, u16 chan_id,
			       u32 vpu_addr, size_t size)
{
	struct vpu_ipc_dev *vpu_dev = to_vpu_dev(dev);
	int rc;

	rc = validate_api_args(vpu_dev, node_id);
	if (rc)
		return rc;

	return keembay_ipc_send(vpu_dev->kmb_ipc, chan_id, vpu_addr, size);
}
EXPORT_SYMBOL(intel_keembay_vpu_ipc_send);

/**
 * intel_keembay_vpu_ipc_recv() - Read data via IPC
 * @dev:       The VPU IPC device to use.
 * @node_id:   The node ID of the remote node (used to identify the link the
 *             channel must be added to). KMB_IPC_NODE_LEON_MSS is the only
 *             allowed value for now.
 * @chan_id:   The IPC channel to read from.
 * @vpu_addr:  [out] The VPU address of the received data.
 * @size:      [out] Where to store the size of the received data.
 * @timeout:   How long (in ms) the function will block waiting for an IPC
 *             message; if UINT32_MAX it will block indefinitely; if 0 it
 *             will not block.
 *
 * Return:     0 on success, negative error code otherwise
 */
int intel_keembay_vpu_ipc_recv(struct device *dev, u8 node_id, u16 chan_id,
			       u32 *vpu_addr, size_t *size, u32 timeout)
{
	struct vpu_ipc_dev *vpu_dev = to_vpu_dev(dev);
	int rc;

	rc = validate_api_args(vpu_dev, node_id);
	if (rc)
		return rc;

	return keembay_ipc_recv(vpu_dev->kmb_ipc, chan_id, vpu_addr, size,
				timeout);
}
EXPORT_SYMBOL(intel_keembay_vpu_ipc_recv);

/**
 * intel_keembay_vpu_startup() - Boot the VPU
 * @dev:	   The VPU device to boot.
 * @firmware_name: Name of firmware file
 *
 * This API is only valid while the VPU is OFF.
 *
 * The firmware called "firmware_name" will be searched for using the
 * kernel firmware API. The firmware header will then be parsed. This driver
 * will load requested information to the reserved memory region, including
 * initialisation data. Lastly, we will request the secure world to do the
 * boot sequence. If the boot sequence is successful, the
 * VPU state will become BUSY. The caller should then wait for the status to
 * become READY before starting to communicate with the VPU. If the boot
 * sequence failed, this function will fail and the caller may try again,
 * the VPU status will still be OFF.
 *
 * If we fail to get to READY, because the VPU did not send us the 'ready'
 * message, the VPU state will go to ERROR.
 *
 * Return: 0 on success, negative error code otherwise
 */
int intel_keembay_vpu_startup(struct device *dev, const char *firmware_name)
{
	struct vpu_ipc_dev *vpu_dev = to_vpu_dev(dev);

	if (IS_ERR(vpu_dev))
		return PTR_ERR(vpu_dev);

	if (!firmware_name)
		return -EINVAL;

	/* Free old vpu_dev->firmware_name value (if any). */
	kfree(vpu_dev->firmware_name);

	/* Set new value. */
	vpu_dev->firmware_name = kstrdup(firmware_name, GFP_KERNEL);

	return do_boot_sequence(vpu_dev);
}
EXPORT_SYMBOL(intel_keembay_vpu_startup);

/**
 * intel_keembay_vpu_reset() - Reset the VPU
 * @dev:	The VPU device to reset.
 *
 * Resets the VPU. Only valid when the VPU is in the READY or ERROR state.
 * The state of the VPU will become BUSY.
 *
 * Return: 0 on success, negative error code otherwise
 */
int intel_keembay_vpu_reset(struct device *dev)
{
	struct vpu_ipc_dev *vpu_dev = to_vpu_dev(dev);

	if (IS_ERR(vpu_dev))
		return PTR_ERR(vpu_dev);

	/*
	 * If vpu_dev->firmware_name == NULL then the VPU is not running
	 * (either it was never booted or vpu_stop() was called). So, calling
	 * reset is not allowed.
	 */
	if (!vpu_dev->firmware_name)
		return -EINVAL;

	return do_boot_sequence(vpu_dev);
}
EXPORT_SYMBOL(intel_keembay_vpu_reset);

/**
 * intel_keembay_vpu_stop() - Stop the VPU
 * @dev:	The VPU device to stop.
 *
 * Stops the VPU and restores to the OFF state. Only valid when the VPU is in
 * the READY or ERROR state.
 *
 * Return: 0 on success, negative error code otherwise
 */
int intel_keembay_vpu_stop(struct device *dev)
{
	struct vpu_ipc_dev *vpu_dev = to_vpu_dev(dev);
	int event_rc;
	int rc;

	if (IS_ERR(vpu_dev))
		return -EINVAL;

	rc = vpu_ipc_handle_event(vpu_dev, KEEMBAY_VPU_EVENT_STOP);
	if (rc < 0) {
		dev_err(dev, "Can't stop in this state.\n");
		return rc;
	}

	dev_info(dev, "Keem Bay VPU IPC stop.\n");

	/* Request stop */
	rc = request_vpu_stop(vpu_dev);
	if (rc < 0) {
		dev_err(dev,
			"Failed to do request to stop - resetting state to OFF anyway.\n");
	}

	/* Remove any saved-off name */
	kfree(vpu_dev->firmware_name);
	vpu_dev->firmware_name = NULL;

	event_rc = vpu_ipc_handle_event(vpu_dev,
					KEEMBAY_VPU_EVENT_STOP_COMPLETE);
	if (event_rc < 0) {
		dev_err(dev,
			"Failed to handle 'stop complete' event, probably fatal.\n");
		return event_rc;
	}

	return rc;
}
EXPORT_SYMBOL(intel_keembay_vpu_stop);

/**
 * intel_keembay_vpu_status() - Get the VPU state.
 * @dev:	The VPU device to retrieve the status for.
 *
 * Returns the state of the VPU as tracked by this driver.
 *
 * Return: Relevant value of enum intel_keembay_vpu_state
 */
enum intel_keembay_vpu_state intel_keembay_vpu_status(struct device *dev)
{
	struct vpu_ipc_dev *vpu_dev = to_vpu_dev(dev);

	if (IS_ERR(vpu_dev))
		return -EINVAL;

	return vpu_dev->state;
}
EXPORT_SYMBOL(intel_keembay_vpu_status);

/**
 * intel_keembay_vpu_get_wdt_count() - Get the WDT count
 * @dev:	The VPU device to get the WDT count for.
 * @id:		ID of WDT events we wish to get
 *
 * Returns: Number of WDT timeout occurrences for given ID, or negative
 *	    error value for invalid ID.
 */
int intel_keembay_vpu_get_wdt_count(struct device *dev,
				    enum intel_keembay_wdt_cpu_id id)
{
	struct vpu_ipc_dev *vpu_dev = to_vpu_dev(dev);
	int rc = -EINVAL;

	if (IS_ERR(vpu_dev))
		return -EINVAL;

	switch (id) {
	case KEEMBAY_VPU_NCE:
		rc = vpu_dev->nce_wdt_count;
		break;
	case KEEMBAY_VPU_MSS:
		rc = vpu_dev->mss_wdt_count;
		break;
	default:
		break;
	}
	return rc;
}
EXPORT_SYMBOL(intel_keembay_vpu_get_wdt_count);

/**
 * intel_keembay_vpu_wait_for_ready() - Sleep until VPU is READY
 * @dev:	The VPU device for which we are waiting the ready message.
 * @timeout:	How long (in ms) the function will block waiting for the VPU
 *		to become ready.
 *
 * The caller may ask the VPU IPC driver to notify it when the VPU
 * is READY. The driver performs no checks on the current state, so it
 * is up to the caller to confirm that the state is correct before starting
 * the wait.
 *
 * Returns: 0 on success negative error code otherwise
 */
int intel_keembay_vpu_wait_for_ready(struct device *dev, u32 timeout)
{
	struct vpu_ipc_dev *vpu_dev = to_vpu_dev(dev);
	int rc;

	if (IS_ERR(vpu_dev))
		return -EINVAL;

	/*
	 * If we are in ERROR state, we will not get to READY
	 * state without some other transitions, so return
	 * error immediately for caller to handle.
	 */
	if (vpu_dev->state == KEEMBAY_VPU_ERROR)
		return -EIO;

	rc = wait_event_interruptible_timeout(vpu_dev->ready_queue,
					      vpu_dev->state == KEEMBAY_VPU_READY,
					      msecs_to_jiffies(timeout));

	/* Condition was false after timeout elapsed */
	if (!rc)
		rc = -ETIME;

	/* Condition was true, so rc == 1 */
	if (rc > 0)
		rc = 0;

	return rc;
}
EXPORT_SYMBOL(intel_keembay_vpu_wait_for_ready);

/**
 * intel_keembay_vpu_register_for_events() - Register callback for event notification
 * @dev:	The VPU device.
 * @callback: Callback function pointer
 *
 * Only a single callback can be registered at a time.
 *
 * Callback can be triggered from any context, so needs to be able to be run
 * from IRQ context.
 *
 * Return: 0 on success, negative error code otherwise
 */
int intel_keembay_vpu_register_for_events(struct device *dev,
					  void (*callback)(struct device *dev,
							   enum intel_keembay_vpu_event))
{
	struct vpu_ipc_dev *vpu_dev = to_vpu_dev(dev);

	if (IS_ERR(vpu_dev))
		return PTR_ERR(vpu_dev);

	if (vpu_dev->callback)
		return -EEXIST;

	vpu_dev->callback = callback;

	return 0;
}
EXPORT_SYMBOL(intel_keembay_vpu_register_for_events);

/**
 * intel_keembay_vpu_unregister_for_events() - Unregister callback for event notification
 * @dev:	The VPU device.
 *
 * Return: 0 on success, negative error code otherwise
 */
int intel_keembay_vpu_unregister_for_events(struct device *dev)
{
	struct vpu_ipc_dev *vpu_dev = to_vpu_dev(dev);

	if (IS_ERR(vpu_dev))
		return PTR_ERR(vpu_dev);

	vpu_dev->callback = NULL;

	return 0;
}
EXPORT_SYMBOL(intel_keembay_vpu_unregister_for_events);

/* Probe() function for the VPU IPC platform driver. */
static int keembay_vpu_ipc_probe(struct platform_device *pdev)
{
	struct tee_ioctl_open_session_arg sess_arg;
	struct device *dev = &pdev->dev;
	struct vpu_ipc_dev *vpu_dev;
	int rc;

	vpu_dev = devm_kzalloc(dev, sizeof(*vpu_dev), GFP_KERNEL);
	if (!vpu_dev)
		return -ENOMEM;

	vpu_dev->pdev = pdev;
	vpu_dev->state = KEEMBAY_VPU_OFF;
	vpu_dev->ready_message_task = NULL;
	vpu_dev->firmware_name = NULL;
	vpu_dev->nce_wdt_count = 0;
	vpu_dev->mss_wdt_count = 0;
	spin_lock_init(&vpu_dev->lock);
	init_waitqueue_head(&vpu_dev->ready_queue);

	/* Retrieve clocks */
	rc = retrieve_clocks(vpu_dev);
	if (rc) {
		dev_err(dev, "Failed to retrieve clocks %d\n", rc);
		return rc;
	}

	/* Retrieve memory regions, allocate memory */
	rc = setup_reserved_memory(vpu_dev);
	if (rc) {
		dev_err(dev,
			"Failed to set up reserved memory regions: %d\n", rc);
		return rc;
	}

	/* Request watchdog timer resources */
	rc = setup_watchdog_resources(vpu_dev);
	if (rc) {
		dev_err(dev, "Failed to setup watchdog resources %d\n", rc);
		goto probe_fail_post_resmem_setup;
	}

	/* Request the IMR number to be used */
	rc = of_property_read_u32(dev->of_node, "intel,keembay-vpu-ipc-imr",
				  &vpu_dev->imr);
	if (rc) {
		dev_err(dev, "failed to get IMR number.\n");
		goto probe_fail_post_resmem_setup;
	}

	/* Get VPU ID. */
	rc = of_property_read_u32(dev->of_node, "intel,keembay-vpu-ipc-id",
				  &vpu_dev->vpu_id);
	if (rc) {
		/* Only warn for now; we will enforce this in the future. */
		dev_err(dev, "VPU ID not defined in Device Tree\n");
		goto probe_fail_post_resmem_setup;
	}

	/* Open context with TEE driver */
	vpu_dev->tee_ctx = tee_client_open_context(NULL, vpu_auth_ta_match,
						   NULL, NULL);

	if (IS_ERR(vpu_dev->tee_ctx)) {
		if (PTR_ERR(vpu_dev->tee_ctx) == -ENOENT) {
			rc = -EPROBE_DEFER;
			goto probe_fail_post_resmem_setup;
		}
		dev_err(dev, "%s: tee_client_open_context failed\n", __func__);
		rc = PTR_ERR(vpu_dev->tee_ctx);
		goto probe_fail_post_resmem_setup;
	}

	/* Open a session with VPU auth TA */
	memset(&sess_arg, 0, sizeof(sess_arg));
	memcpy(sess_arg.uuid, vpu_auth_ta_uuid.b, TEE_IOCTL_UUID_LEN);
	sess_arg.clnt_login = TEE_IOCTL_LOGIN_PUBLIC;
	sess_arg.num_params = 0;

	rc = tee_client_open_session(vpu_dev->tee_ctx, &sess_arg, NULL);
	if (rc < 0 || sess_arg.ret != 0) {
		dev_err(dev, "%s: tee_client_open_session failed, err=%x\n",
			__func__, sess_arg.ret);
		if (!rc)
			rc = -EINVAL;
		goto probe_fail_post_tee_ctx_setup;
	}
	vpu_dev->tee_session = sess_arg.session;

	/* Allocate dynamic shared memory for VPU boot params */
	vpu_dev->shm = tee_shm_alloc(vpu_dev->tee_ctx,
				     (sizeof(struct vpu_boot_ta_shmem) +
				     MAX_HEADER_SIZE),
				     (TEE_SHM_MAPPED | TEE_SHM_DMA_BUF));

	if (IS_ERR(vpu_dev->shm)) {
		dev_err(dev, "%s: tee_shm_alloc failed\n", __func__);
		rc = -ENOMEM;
		goto probe_fail_post_tee_session_setup;
	}

	/* Get IPC instance to be used for IPC communication. */
	vpu_dev->kmb_ipc = keembay_ipc_init(dev);
	if (IS_ERR(vpu_dev->kmb_ipc)) {
		rc = PTR_ERR(vpu_dev->kmb_ipc);
		dev_err(dev, "Failed to initialize IPC: %d\n", rc);
		goto probe_fail_post_tee_session_setup;
	}

	/* Get address and size of IPC memory reserved to VPU. */
	keembay_ipc_get_vpu_mem_info(vpu_dev->kmb_ipc,
				     &vpu_dev->mss_ipc_mem.vpu_addr,
				     &vpu_dev->mss_ipc_mem.size);

	/* Set platform data reference. */
	platform_set_drvdata(pdev, vpu_dev);

	return 0;

probe_fail_post_tee_session_setup:
	tee_client_close_session(vpu_dev->tee_ctx, vpu_dev->tee_session);

probe_fail_post_tee_ctx_setup:
	tee_client_close_context(vpu_dev->tee_ctx);

probe_fail_post_resmem_setup:
	/*
	 * Explicitly free memory allocated by setup_reserved_memory(); this is
	 * needed in order to safely call of_reserved_mem_device_release()
	 */
	dmam_free_coherent(dev, vpu_dev->reserved_mem.size,
			   vpu_dev->reserved_mem.vaddr,
			   vpu_dev->reserved_mem.vpu_addr);
	of_reserved_mem_device_release(dev);

	return rc;
}

/* Remove() function for the VPU IPC platform driver. */
static int keembay_vpu_ipc_remove(struct platform_device *pdev)
{
	struct vpu_ipc_dev *vpu_dev = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	if (vpu_dev->ready_message_task) {
		kthread_stop(vpu_dev->ready_message_task);
		vpu_dev->ready_message_task = NULL;
	}

	of_reserved_mem_device_release(dev);

	tee_shm_free(vpu_dev->shm);

	tee_client_close_session(vpu_dev->tee_ctx, vpu_dev->tee_session);

	tee_client_close_context(vpu_dev->tee_ctx);

	keembay_ipc_deinit(vpu_dev->kmb_ipc);

	return 0;
}

/* Compatible string for the VPU/IPC driver. */
static const struct of_device_id keembay_vpu_ipc_of_match[] = {
	{
		.compatible = "intel,keembay-vpu-ipc",
	},
	{}
};

/* The VPU IPC platform driver. */
static struct platform_driver keembay_vpu_ipc_driver = {
	.driver = {
			.name = "keembay-vpu-ipc",
			.of_match_table = keembay_vpu_ipc_of_match,
		},
	.probe = keembay_vpu_ipc_probe,
	.remove = keembay_vpu_ipc_remove,
};

/* Helper function to get a vpu_dev struct from a generic device pointer. */
static struct vpu_ipc_dev *to_vpu_dev(struct device *dev)
{
	struct platform_device *pdev;

	if (!dev || dev->driver != &keembay_vpu_ipc_driver.driver)
		return ERR_PTR(-EINVAL);
	pdev = to_platform_device(dev);

	return platform_get_drvdata(pdev);
}

/*
 * Retrieve SoC information from the '/soc/version-info' device tree node and
 * store it into 'vpu_ipc_soc_info' global variable.
 */
static int retrieve_dt_soc_information(void)
{
	struct device_node *soc_info_dn;
	int ret;

	soc_info_dn = of_find_node_by_path("/soc/version-info");
	if (!soc_info_dn)
		return -ENOENT;

	ret = of_property_read_u64(soc_info_dn, "feature-exclusion",
				   &vpu_ipc_soc_info->feature_exclusion);
	if (ret) {
		pr_err("Property 'feature-exclusion' can't be read.\n");
		return ret;
	}
	ret = of_property_read_u64(soc_info_dn, "device-id",
				   &vpu_ipc_soc_info->device_id);
	if (ret) {
		pr_err("Property 'device-id' can't be read.\n");
		return ret;
	}
	ret = of_property_read_u32(soc_info_dn, "hardware-id",
				   &vpu_ipc_soc_info->hardware_id);
	if (ret) {
		pr_err("Property 'hardware-id' can't be read.\n");
		return ret;
	}
	/*
	 * Note: the SKU and stepping information from the device tree is
	 * not a string, but an array of u8/chars. Therefore, we cannot
	 * parse it as a string.
	 */
	ret = of_property_read_u8_array(soc_info_dn, "sku",
					vpu_ipc_soc_info->sku,
					sizeof(vpu_ipc_soc_info->sku));
	if (ret) {
		pr_err("Property 'sku' can't be read.\n");
		return ret;
	}
	ret = of_property_read_u8_array(soc_info_dn, "stepping",
					vpu_ipc_soc_info->stepping,
					sizeof(vpu_ipc_soc_info->stepping));
	if (ret) {
		pr_err("Property 'stepping' can't be read.\n");
		return ret;
	}

	return 0;
}

/*
 * Init VPU IPC module:
 * - Retrieve SoC information from device tree.
 * - Register the VPU IPC platform driver.
 */
static int __init vpu_ipc_init(void)
{
	int rc;

	vpu_ipc_soc_info = kzalloc(sizeof(*vpu_ipc_soc_info), GFP_KERNEL);
	if (!vpu_ipc_soc_info)
		return -ENOMEM;

	rc = retrieve_dt_soc_information();
	if (rc < 0)
		pr_warn("VPU IPC failed to find SoC info, using defaults.\n");

	rc = platform_driver_register(&keembay_vpu_ipc_driver);
	if (rc < 0) {
		pr_err("Failed to register platform driver for VPU IPC.\n");
		goto cleanup_soc_info;
	}

	return 0;

cleanup_soc_info:
	kfree(vpu_ipc_soc_info);

	return rc;
}

/*
 * Remove VPU IPC module.
 * - Un-register the VPU IPC platform driver.
 * - Remove sysfs exposing SoC information.
 * - Free allocated memory.
 */
static void __exit vpu_ipc_exit(void)
{
	platform_driver_unregister(&keembay_vpu_ipc_driver);
	kfree(vpu_ipc_soc_info);
}

module_init(vpu_ipc_init);
module_exit(vpu_ipc_exit);

MODULE_DESCRIPTION("Keem Bay VPU IPC Driver");
MODULE_AUTHOR("Daniele Alessandrelli <daniele.alessandrelli@intel.com>");
MODULE_AUTHOR("Paul Murphy <paul.j.murphy@intel.com>");
MODULE_AUTHOR("Darren Roche <darren.b.roche@intel.com>");
MODULE_LICENSE("GPL");
