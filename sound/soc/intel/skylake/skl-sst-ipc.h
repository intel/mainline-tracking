/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Intel SKL IPC Support
 *
 * Copyright (C) 2014-15, Intel Corporation.
 */

#ifndef __SKL_IPC_H
#define __SKL_IPC_H

#include <linux/irqreturn.h>
#include "../common/sst-ipc.h"
#include "skl-sst-dsp.h"
#include <uapi/sound/skl-tplg-interface.h>

#define	SKL_EVENT_GLB_MODULE_NOTIFICATION	12

struct kfifo;
struct sst_dsp;
struct sst_generic_ipc;

union skl_connector_node_id {
	u32 val;
	struct {
		u32 vindex:8;
		u32 dma_type:5;
		u32 rsvd:19;
	} node;
};

#define INVALID_NODE_ID \
	((union skl_connector_node_id) { UINT_MAX })

enum skl_channel_index {
	SKL_CHANNEL_LEFT = 0,
	SKL_CHANNEL_RIGHT = 1,
	SKL_CHANNEL_CENTER = 2,
	SKL_CHANNEL_LEFT_SURROUND = 3,
	SKL_CHANNEL_CENTER_SURROUND = 3,
	SKL_CHANNEL_RIGHT_SURROUND = 4,
	SKL_CHANNEL_LFE = 7,
	SKL_CHANNEL_INVALID = 0xF,
};

enum skl_bitdepth {
	SKL_DEPTH_8BIT = 8,
	SKL_DEPTH_16BIT = 16,
	SKL_DEPTH_24BIT = 24,
	SKL_DEPTH_32BIT = 32,
	SKL_DEPTH_INVALID
};

enum skl_s_freq {
	SKL_FS_8000 = 8000,
	SKL_FS_11025 = 11025,
	SKL_FS_12000 = 12000,
	SKL_FS_16000 = 16000,
	SKL_FS_22050 = 22050,
	SKL_FS_24000 = 24000,
	SKL_FS_32000 = 32000,
	SKL_FS_44100 = 44100,
	SKL_FS_48000 = 48000,
	SKL_FS_64000 = 64000,
	SKL_FS_88200 = 88200,
	SKL_FS_96000 = 96000,
	SKL_FS_128000 = 128000,
	SKL_FS_176400 = 176400,
	SKL_FS_192000 = 192000,
	SKL_FS_INVALID
};

struct skl_audio_data_format {
	enum skl_s_freq s_freq;
	enum skl_bitdepth bit_depth;
	u32 channel_map;
	enum skl_ch_cfg ch_cfg;
	enum skl_interleaving interleaving;
	u8 number_of_channels;
	u8 valid_bit_depth;
	u8 sample_type;
	u8 reserved[1];
} __packed;

struct skl_base_cfg {
	u32 cpc;
	u32 ibs;
	u32 obs;
	u32 is_pages;
	struct skl_audio_data_format audio_fmt;
};

struct skl_cpr_gtw_cfg {
	u32 node_id;
	u32 dma_buffer_size;
	u32 config_length;
	/* not mandatory; required only for DMIC/I2S */
	u32 config_data[1];
} __packed;

struct skl_cpr_cfg {
	struct skl_base_cfg base_cfg;
	struct skl_audio_data_format out_fmt;
	u32 cpr_feature_mask;
	struct skl_cpr_gtw_cfg gtw_cfg;
} __packed;

struct skl_cpr_pin_fmt {
	u32 sink_id;
	struct skl_audio_data_format src_fmt;
	struct skl_audio_data_format dst_fmt;
} __packed;

struct skl_src_module_cfg {
	struct skl_base_cfg base_cfg;
	enum skl_s_freq src_cfg;
	u32 mode;
} __packed;

/* Maximum number of coefficients up down mixer module */
#define UP_DOWN_MIXER_MAX_COEFF		8

struct skl_up_down_mixer_cfg {
	struct skl_base_cfg base_cfg;
	enum skl_ch_cfg out_ch_cfg;
	/* This should be set to 1 if user coefficients are required */
	u32 coeff_sel;
	/* Pass the user coeff in this array */
	s32 coeff[UP_DOWN_MIXER_MAX_COEFF];
	u32 ch_map;
} __packed;

struct skl_algo_cfg {
	struct skl_base_cfg  base_cfg;
	char params[0];
} __packed;

struct skl_base_outfmt_cfg {
	struct skl_base_cfg base_cfg;
	struct skl_audio_data_format out_fmt;
} __packed;

struct skl_tlv {
	u32 type;
	u32 length;
	u8 value[0];
};

static const guid_t skl_copier_mod_uuid =
	GUID_INIT(0x9BA00C83, 0xCA12, 0x4A83, 0x94, 0x3C,
		0x1F, 0xA2, 0xE8, 0x2F, 0x9D, 0xDA);

static const guid_t skl_probe_mod_uuid =
	GUID_INIT(0x7CAD0808, 0xAB10, 0xCD23, 0xEF, 0x45,
		0x12, 0xAB, 0x34, 0xCD, 0x56, 0xEF);

struct skl_probe_gtw_cfg {
	union skl_connector_node_id node_id;
	u32 dma_buffer_size;
} __packed;

struct skl_probe_mod_cfg {
	struct skl_base_cfg base_cfg;
	struct skl_probe_gtw_cfg gtw_cfg;
} __packed;

enum skl_probe_runtime_param {
	SKL_PROBE_INJECTION_DMA = 1,
	SKL_PROBE_INJECTION_DMA_DETACH,
	SKL_PROBE_POINTS,
	SKL_PROBE_POINTS_DISCONNECT,
};

struct skl_probe_dma {
	union skl_connector_node_id node_id;
	unsigned int dma_buffer_size;
} __packed;

enum skl_probe_type {
	SKL_PROBE_TYPE_INPUT = 0,
	SKL_PROBE_TYPE_OUTPUT,
	SKL_PROBE_TYPE_INTERNAL
};

union skl_probe_point_id {
	unsigned int value;
	struct {
		unsigned int module_id:16;
		unsigned int instance_id:8;
		enum skl_probe_type type:2;
		unsigned int index:6;
	} id;
} __packed;

enum skl_connection_purpose {
	SKL_CONNECTION_PURPOSE_EXTRACT = 0,
	SKL_CONNECTION_PURPOSE_INJECT,
	SKL_CONNECTION_PURPOSE_INJECT_REEXTRACT,
};

struct skl_probe_point_desc {
	union skl_probe_point_id id;
	enum skl_connection_purpose purpose __aligned(4);
	union skl_connector_node_id node_id;
} __packed;

enum skl_ipc_msg_target {
	IPC_FW_GEN_MSG = 0,
	IPC_MOD_MSG = 1
};

enum skl_ipc_msg_direction {
	IPC_MSG_REQUEST = 0,
	IPC_MSG_REPLY = 1
};

/* Global Message Types */
enum skl_ipc_glb_type {
	IPC_GLB_GET_FW_VERSION = 0, /* Retrieves firmware version */
	IPC_GLB_LOAD_MULTIPLE_MODS = 15,
	IPC_GLB_UNLOAD_MULTIPLE_MODS = 16,
	IPC_GLB_CREATE_PPL = 17,
	IPC_GLB_DELETE_PPL = 18,
	IPC_GLB_SET_PPL_STATE = 19,
	IPC_GLB_GET_PPL_STATE = 20,
	IPC_GLB_GET_PPL_CONTEXT_SIZE = 21,
	IPC_GLB_SAVE_PPL = 22,
	IPC_GLB_RESTORE_PPL = 23,
	IPC_GLB_LOAD_LIBRARY = 24,
	IPC_GLB_NOTIFY = 26,
	IPC_GLB_MAX_IPC_MSG_NUMBER = 31 /* Maximum message number */
};

/* Resource Event Types */
enum skl_ipc_resource_event_type {
	SKL_BUDGET_VIOLATION = 0,
	SKL_MIXER_UNDERRUN = 1,
	SKL_STREAM_DATA_SEGMENT = 2,
	SKL_PROCESS_DATA_ERR = 3,
	SKL_STACK_OVERFLOW = 4,
	SKL_BUFFERING_MODE_CHANGED = 5,
	SKL_GATEWAY_UNDERRUN = 6,
	SKL_GATEWAY_OVERRUN = 7,
	SKL_EDF_DOMAIN_UNSTABLE = 8,
	SKL_WCLK_SAMPLE_COUNT = 9,
	SKL_GATEWAY_HIGH_THRESHOLD = 10,
	SKL_GATEWAY_LOW_THRESHOLD = 11,
	SKL_I2S_BCE_DETECTED = 12,
	SKL_I2S_CLK_STATE_CHANGED = 13,
	SKL_I2S_SINK_MODE_CHANGED = 14,
	SKL_I2S_SOURCE_MODE_CHANGED = 15,
	SKL_SRE_DRIFT_TOO_HIGH = 16,
	SKL_INVALID_RESOURCE_EVENT_TYPE = 17
};

#define IPC_IXC_STATUS_BITS		24

enum skl_ipc_glb_reply {
	IPC_GLB_REPLY_SUCCESS = 0,
	IPC_GLB_REPLY_UNKNOWN_MSG_TYPE = 1,
	IPC_GLB_REPLY_ERROR_INVALID_PARAM = 2,
	IPC_GLB_REPLY_BUSY = 3,
	IPC_GLB_REPLY_PENDING = 4,
	IPC_GLB_REPLY_FAILURE = 5,
	IPC_GLB_REPLY_INVALID_REQUEST = 6,
	IPC_GLB_REPLY_OUT_OF_MEMORY = 7,
	IPC_GLB_REPLY_INVALID_RESOURCE_ID = 9,
	IPC_GLB_REPLY_OUT_OF_MIPS = 11,
	IPC_GLB_REPLY_INVALID_RESOURCE_STATE = 12,
	IPC_GLB_REPLY_UNAVAILABLE = 15,
	IPC_GLB_REPLY_MOD_MGMT_ERROR = 100,
	IPC_GLB_REPLY_MOD_LOAD_CL_FAILED = 101,
	IPC_GLB_REPLY_MOD_LOAD_INVALID_HASH = 102,
	IPC_GLB_REPLY_MOD_UNLOAD_INST_EXIST = 103,
	IPC_GLB_REPLY_MOD_NOT_INITIALIZED = 104,
	IPC_GLB_REPLY_INVALID_CONFIG_PARAM_ID = 120,
	IPC_GLB_REPLY_INVALID_CONFIG_DATA_LEN = 121,
	IPC_GLB_REPLY_GATEWAY_NOT_INITIALIZED = 140,
	IPC_GLB_REPLY_GATEWAY_NOT_EXIST = 141,
	IPC_GLB_REPLY_SCLK_ALREADY_RUNNING = 150,
	IPC_GLB_REPLY_MCLK_ALREADY_RUNNING = 151,
	IPC_GLB_REPLY_PPL_NOT_INITIALIZED = 160,
	IPC_GLB_REPLY_PPL_NOT_EXIST = 161,
	IPC_GLB_REPLY_PPL_SAVE_FAILED = 162,
	IPC_GLB_REPLY_PPL_RESTORE_FAILED = 163,

	IPC_MAX_STATUS = ((1<<IPC_IXC_STATUS_BITS)-1)
};

enum skl_ipc_notification_type {
	IPC_GLB_NOTIFY_GLITCH = 0,
	IPC_GLB_NOTIFY_OVERRUN = 1,
	IPC_GLB_NOTIFY_UNDERRUN = 2,
	IPC_GLB_NOTIFY_END_STREAM = 3,
	IPC_GLB_NOTIFY_PHRASE_DETECTED = 4,
	IPC_GLB_NOTIFY_RESOURCE_EVENT = 5,
	IPC_GLB_NOTIFY_LOG_BUFFER_STATUS = 6,
	IPC_GLB_NOTIFY_TIMESTAMP_CAPTURED = 7,
	IPC_GLB_NOTIFY_FW_READY = 8,
	IPC_GLB_NOTIFY_FW_AUD_CLASS_RESULT = 9,
	IPC_GLB_MODULE_NOTIFICATION = 12,
};

/* Module Message Types */
enum skl_ipc_module_msg {
	IPC_MOD_INIT_INSTANCE = 0,
	IPC_MOD_CONFIG_GET = 1,
	IPC_MOD_CONFIG_SET = 2,
	IPC_MOD_LARGE_CONFIG_GET = 3,
	IPC_MOD_LARGE_CONFIG_SET = 4,
	IPC_MOD_BIND = 5,
	IPC_MOD_UNBIND = 6,
	IPC_MOD_SET_DX = 7,
	IPC_MOD_SET_D0IX = 8,
	IPC_MOD_DELETE_INSTANCE = 11
};

struct skl_notify_msg {
	union {
		u16 word_id;
		struct {
			u16 rsvd:12;
			u16 core:4;
		} log;
	};
	enum skl_ipc_notification_type notif_type:8;
	enum skl_ipc_glb_type type:5;
	enum skl_ipc_msg_direction dir:1;
	enum skl_ipc_msg_target target:1;
	u32 rsvd:1;
	union {
		u32 sv_score:16;
		struct {
			u32 core_id:2;
			u32 stack_size:16;
		};
		struct {
			u32 rsvd:30;
			u32 done:1;
			u32 error:1;
		} mod_evt;
	};
} __packed;

struct skl_gain_module_config {
	struct skl_base_cfg mconf;
	struct skl_gain_config gain_cfg;
};

enum skl_ipc_pipeline_state {
	PPL_INVALID_STATE =	0,
	PPL_UNINITIALIZED =	1,
	PPL_RESET =		2,
	PPL_PAUSED =		3,
	PPL_RUNNING =		4,
	PPL_ERROR_STOP =	5,
	PPL_SAVED =		6,
	PPL_RESTORED =		7
};

enum skl_copier_runtime_param {
	SKL_COPIER_TIMESTAMP_INIT = 1,
};

struct skl_ipc_dxstate_info {
	u32 core_mask;
	u32 dx_mask;
};

struct skl_ipc_header {
	u32 primary;
	u32 extension;
};

struct skl_dsp_cores {
	unsigned int count;
	enum skl_dsp_states *state;
	int *usage_count;
};

struct skl_module_notify {
	u32 unique_id;
	u32 event_id;
	u32 event_data_size;
	u32 event_data[0];
} __packed;

/**
 * skl_d0i3_data: skl D0i3 counters data struct
 *
 * @streaming: Count of usecases that can attempt streaming D0i3
 * @non_streaming: Count of usecases that can attempt non-streaming D0i3
 * @non_d0i3: Count of usecases that cannot attempt D0i3
 * @state: current state
 * @work: D0i3 worker thread
 */
struct skl_d0i3_data {
	int streaming;
	int non_streaming;
	int non_d0i3;
	enum skl_dsp_d0i3_states state;
	struct delayed_work work;
};

#define SKL_LIB_NAME_LENGTH 128
#define SKL_MAX_LIB 16

struct skl_lib_info {
	char name[SKL_LIB_NAME_LENGTH];
	const struct firmware *fw;
};

enum skl_basefw_runtime_param {
	SKL_BASEFW_ASTATE_TABLE = 4,
	SKL_BASEFW_DMA_CONTROL = 5,
	SKL_BASEFW_ENABLE_LOGS = 6,
	SKL_BASEFW_FIRMWARE_CONFIG = 7,
	SKL_BASEFW_HARDWARE_CONFIG = 8,
	SKL_BASEFW_SYSTEM_TIME = 20,
};

enum skl_fw_cfg_params {
	SKL_FW_CFG_FW_VERSION = 0,
	SKL_FW_CFG_MEMORY_RECLAIMED,
	SKL_FW_CFG_SLOW_CLOCK_FREQ_HZ,
	SKL_FW_CFG_FAST_CLOCK_FREQ_HZ,
	SKL_FW_CFG_DMA_BUFFER_CONFIG,
	SKL_FW_CFG_ALH_SUPPORT_LEVEL,
	SKL_FW_CFG_IPC_DL_MAILBOX_BYTES,
	SKL_FW_CFG_IPC_UL_MAILBOX_BYTES,
	SKL_FW_CFG_TRACE_LOG_BYTES,
	SKL_FW_CFG_MAX_PPL_COUNT,
	SKL_FW_CFG_MAX_ASTATE_COUNT,
	SKL_FW_CFG_MAX_MODULE_PIN_COUNT,
	SKL_FW_CFG_MODULES_COUNT,
	SKL_FW_CFG_MAX_MOD_INST_COUNT,
	SKL_FW_CFG_MAX_LL_TASKS_PER_PRI_COUNT,
	SKL_FW_CFG_LL_PRI_COUNT,
	SKL_FW_CFG_MAX_DP_TASKS_COUNT,
	SKL_FW_CFG_MAX_LIBS_COUNT,
	SKL_FW_CFG_SCHEDULER_CONFIG,
	SKL_FW_CFG_XTAL_FREQ_HZ,
	SKL_FW_CFG_CLOCKS_CONFIG,
	SKL_FW_CFG_UAOL_SUPPORT,
	SKL_FW_CFG_POWER_GATING_POLICY,
	SKL_FW_CFG_ASSERT_MODE,
};

struct skl_fw_version {
	u16 major;
	u16 minor;
	u16 hotfix;
	u16 build;
};

enum skl_alh_support_level {
	ALH_NO_SUPPORT = 0x00000,
	ALH_CAVS_1_8_CNL = 0x10000,
};

struct skl_fw_cfg {
	struct skl_fw_version fw_version;
	u32 memory_reclaimed;
	u32 slow_clock_freq_hz;
	u32 fast_clock_freq_hz;
	enum skl_alh_support_level alh_support;
	u32 ipc_dl_mailbox_bytes;
	u32 ipc_ul_mailbox_bytes;
	u32 trace_log_bytes;
	u32 max_ppl_count;
	u32 max_astate_count;
	u32 max_module_pin_count;
	u32 modules_count;
	u32 max_mod_inst_count;
	u32 max_ll_tasks_per_pri_count;
	u32 ll_pri_count;
	u32 max_dp_tasks_count;
	u32 max_libs_count;
	u32 xtal_freq_hz;
	u32 uaol_support;
	u32 power_gating_policy;
};

enum skl_hw_cfg_params {
	SKL_HW_CFG_CAVS_VER,
	SKL_HW_CFG_DSP_CORES,
	SKL_HW_CFG_MEM_PAGE_BYTES,
	SKL_HW_CFG_TOTAL_PHYS_MEM_PAGES,
	SKL_HW_CFG_I2S_CAPS,
	SKL_HW_CFG_GPDMA_CAPS,
	SKL_HW_CFG_GATEWAY_COUNT,
	SKL_HW_CFG_HP_EBB_COUNT,
	SKL_HW_CFG_LP_EBB_COUNT,
	SKL_HW_CFG_EBB_SIZE_BYTES,
	SKL_HW_CFG_UAOL_CAPS
};

enum skl_cavs_version {
	SKL_CAVS_VER_1_5 = 0x10005,
	SKL_CAVS_VER_1_8 = 0x10008,
};

enum skl_i2s_version {
	SKL_I2S_VER_15_SKYLAKE   = 0x00000,
	SKL_I2S_VER_15_BROXTON   = 0x10000,
	SKL_I2S_VER_15_BROXTON_P = 0x20000,
	SKL_I2S_VER_18_KBL_CNL   = 0x30000,
};

struct skl_i2s_caps {
	enum skl_i2s_version version;
	u32 ctrl_count;
	u32 *ctrl_base_addr;
};

struct skl_hw_cfg {
	enum skl_cavs_version cavs_version;
	u32 dsp_cores;
	u32 mem_page_bytes;
	u32 total_phys_mem_pages;
	struct skl_i2s_caps i2s_caps;
	u32 gateway_count;
	u32 hp_ebb_count;
	u32 lp_ebb_count;
	u32 ebb_size_bytes;
};

struct skl_sys_time {
	u32 val_l;
	u32 val_u;
} __packed;

enum skl_log_enable {
	SKL_LOG_DISABLE = 0,
	SKL_LOG_ENABLE = 1
};

enum skl_log_priority {
	SKL_LOG_CRITICAL = 1,
	SKL_LOG_HIGH,
	SKL_LOG_MEDIUM,
	SKL_LOG_LOW,
	SKL_LOG_VERBOSE
};

enum icl_log_priority {
	ICL_LOG_CRITICAL = 0,
	ICL_LOG_HIGH,
	ICL_LOG_MEDIUM,
	ICL_LOG_LOW,
	ICL_LOG_VERBOSE
};

enum icl_log_source {
	ICL_LOG_INFRA = 0,
	ICL_LOG_HAL,
	ICL_LOG_MODULE,
	ICL_LOG_AUDIO,
	ICL_LOG_SENSING,
	ICL_LOG_ULP_INFRA
};

struct skl_log_state {
	enum skl_log_enable enable __aligned(4);
	enum skl_log_priority min_priority __aligned(4);
} __packed;

struct skl_log_state_info {
	u32 core_mask;
	struct skl_log_state logs_core[0];
} __packed;

struct bxt_log_state_info {
	u32 aging_timer_period;
	u32 fifo_full_timer_period;
	u32 core_mask;
	struct skl_log_state logs_core[0];
} __packed;

struct icl_log_state_info {
	u32 aging_timer_period;
	u32 fifo_full_timer_period;
	enum skl_log_enable enable __aligned(4);
	u32 logs_priorities_mask[0];
} __packed;

struct skl_notify_kctrl_info {
	struct list_head list;
	u32 notify_id;
	struct snd_kcontrol *notify_kctl;
};

struct skl_ipc_init_instance_msg {
	u32 module_id;
	u32 instance_id;
	u16 param_data_size;
	u8 ppl_instance_id;
	u8 core_id;
	u8 domain;
};

struct skl_ipc_bind_unbind_msg {
	u32 module_id;
	u32 instance_id;
	u32 dst_module_id;
	u32 dst_instance_id;
	u8 src_queue;
	u8 dst_queue;
	bool bind;
};

struct skl_ipc_large_config_msg {
	u32 module_id;
	u32 instance_id;
	u32 large_param_id;
	u32 param_data_size;
};

struct skl_ipc_d0ix_msg {
	u32 module_id;
	u32 instance_id;
	u8 streaming;
	u8 wake;
};

#define SKL_IPC_BOOT_MSECS		3000

#define SKL_IPC_D3_MASK	0
#define SKL_IPC_D0_MASK	3

irqreturn_t skl_dsp_irq_thread_handler(int irq, void *context);

int skl_ipc_create_pipeline(struct sst_generic_ipc *sst_ipc,
		u16 ppl_mem_size, u8 ppl_type, u8 instance_id, u8 lp_mode);

int skl_ipc_delete_pipeline(struct sst_generic_ipc *sst_ipc, u8 instance_id);

int skl_ipc_set_pipeline_state(struct sst_generic_ipc *sst_ipc,
		u8 instance_id,	enum skl_ipc_pipeline_state state);

int skl_ipc_save_pipeline(struct sst_generic_ipc *ipc,
		u8 instance_id, int dma_id);

int skl_ipc_restore_pipeline(struct sst_generic_ipc *ipc, u8 instance_id);

int skl_ipc_init_instance(struct sst_generic_ipc *sst_ipc,
		struct skl_ipc_init_instance_msg *msg, void *param_data);

int skl_ipc_bind_unbind(struct sst_generic_ipc *sst_ipc,
		struct skl_ipc_bind_unbind_msg *msg);

int skl_ipc_load_modules(struct sst_generic_ipc *ipc,
				u8 module_cnt, void *data);

int skl_ipc_unload_modules(struct sst_generic_ipc *ipc,
				u8 module_cnt, void *data);

int skl_ipc_set_dx(struct sst_generic_ipc *ipc,
		u8 instance_id, u16 module_id, struct skl_ipc_dxstate_info *dx);

int skl_ipc_set_large_config(struct sst_generic_ipc *ipc,
		struct skl_ipc_large_config_msg *msg, u32 *param);

int skl_ipc_get_large_config(struct sst_generic_ipc *ipc,
		struct skl_ipc_large_config_msg *msg,
		u32 **payload, size_t *bytes);

int skl_sst_ipc_load_library(struct sst_generic_ipc *ipc,
			u8 dma_id, u8 table_id, bool wait);

int skl_ipc_set_d0ix(struct sst_generic_ipc *ipc,
		struct skl_ipc_d0ix_msg *msg);
int skl_ipc_delete_instance(struct sst_generic_ipc *ipc,
		unsigned int module_id, unsigned int instance_id);

int skl_ipc_check_D0i0(struct sst_dsp *dsp, bool state);

void skl_ipc_int_enable(struct sst_dsp *dsp);
void skl_ipc_op_int_enable(struct sst_dsp *ctx);
void skl_ipc_op_int_disable(struct sst_dsp *ctx);
void skl_ipc_int_disable(struct sst_dsp *dsp);

bool skl_ipc_int_status(struct sst_dsp *dsp);
int skl_ipc_init(struct device *dev, struct skl_dev *skl);
void skl_clear_module_cnt(struct sst_dsp *ctx);

void skl_ipc_process_reply(struct sst_generic_ipc *ipc,
		struct skl_ipc_header header);
int skl_ipc_process_notification(struct sst_generic_ipc *ipc,
		struct skl_ipc_header header);
void skl_ipc_tx_data_copy(struct ipc_message *msg, char *tx_data,
		size_t tx_size);
void skl_ipc_set_fw_cfg(struct sst_generic_ipc *ipc, u8 instance_id,
			u16 module_id, u32 *data);

int skl_ipc_fw_cfg_get(struct sst_generic_ipc *ipc, struct skl_fw_cfg *cfg);
int skl_ipc_hw_cfg_get(struct sst_generic_ipc *ipc, struct skl_hw_cfg *cfg);

int skl_probe_init_module(struct skl_dev *skl, size_t buffer_size);
int skl_probe_delete_module(struct skl_dev *skl);
int skl_probe_get_dma(struct skl_dev *skl,
		struct skl_probe_dma **dma, size_t *num_dma);
int skl_probe_dma_attach(struct skl_dev *skl,
		struct skl_probe_dma *dma, size_t num_dma);
int skl_probe_dma_detach(struct skl_dev *skl,
		union skl_connector_node_id *node_id, size_t num_node_id);
int skl_probe_get_points(struct skl_dev *skl,
		struct skl_probe_point_desc **desc, size_t *num_desc);
int skl_probe_points_connect(struct skl_dev *skl,
		struct skl_probe_point_desc *desc, size_t num_desc);
int skl_probe_points_disconnect(struct skl_dev *skl,
		union skl_probe_point_id *id, size_t num_id);
int skl_system_time_set(struct sst_generic_ipc *ipc);
int skl_enable_logs_set(struct sst_generic_ipc *ipc, u32 *info, size_t size);
int bxt_enable_logs(struct sst_dsp *dsp, enum skl_log_enable enable,
		u32 aging_period, u32 fifo_full_period,
		unsigned long resource_mask, u32 *priorities);

unsigned int
skl_kfifo_fromio_locked(struct kfifo *fifo, const void __iomem *src,
		unsigned int len, spinlock_t *lock);

#endif /* __SKL_IPC_H */
