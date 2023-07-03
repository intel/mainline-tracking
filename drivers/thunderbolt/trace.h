/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Thunderbolt tracing support
 *
 * Copyright (C) 2023, Intel Corporation
 * Author: Mika Westerberg <mika.westerberg@linux.intel.com>
 *	   Gil Fine <gil.fine@intel.com>
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM thunderbolt

#if !defined(TB_TRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define TB_TRACE_H_

#include <linux/trace_seq.h>
#include <linux/tracepoint.h>

#include "tb_msgs.h"

#define show_type_name(val)						\
	__print_symbolic(val,						\
		{ TB_CFG_PKG_READ, "read" },				\
		{ TB_CFG_PKG_WRITE, "write" },				\
		{ TB_CFG_PKG_ERROR, "error" },				\
		{ TB_CFG_PKG_NOTIFY_ACK, "ack" },			\
		{ TB_CFG_PKG_EVENT, "event" },				\
		{ TB_CFG_PKG_XDOMAIN_REQ, "xdomain_request" },		\
		{ TB_CFG_PKG_XDOMAIN_RESP, "xdomain_response" },	\
		{ TB_CFG_PKG_OVERRIDE, "cm_override" },			\
		{ TB_CFG_PKG_RESET, "reset" },				\
		{ TB_CFG_PKG_ICM_EVENT, "icm_event" },			\
		{ TB_CFG_PKG_ICM_CMD, "icm_request" },			\
		{ TB_CFG_PKG_ICM_RESP, "icm_response" })

#ifndef TB_TRACE_HELPERS
#define TB_TRACE_HELPERS
static inline const char *show_data_read_write(struct trace_seq *p,
					       const u32 *data)
{
	const struct cfg_read_pkg *msg = (const struct cfg_read_pkg *)data;
	const char *ret = trace_seq_buffer_ptr(p);

	trace_seq_printf(p, "offset=0x%x len=0x%x port=%d config=0x%x seq=%d ",
			 msg->addr.offset, msg->addr.length, msg->addr.port,
			 msg->addr.space, msg->addr.seq);

	return ret;
}

static inline const char *show_data_error(struct trace_seq *p,
					  const u32 *data)
{
	const struct cfg_error_pkg *msg = (const struct cfg_error_pkg *)data;
	const char *ret = trace_seq_buffer_ptr(p);

	/* For error code 33, info field is not used as a port number */
	trace_seq_printf(p, "code=0x%x info=0x%x plug=0x%x ", msg->error,
			 msg->port, msg->pg);

	return ret;
}

static inline const char *show_data_event(struct trace_seq *p,
					  const u32 *data)
{
	const struct cfg_event_pkg *msg = (const struct cfg_event_pkg *)data;
	const char *ret = trace_seq_buffer_ptr(p);

	trace_seq_printf(p, "port=%d unplug=0x%x ", msg->port, msg->unplug);

	return ret;
}

static inline const char *show_route(struct trace_seq *p, const u32 *data)
{
	const struct tb_cfg_header *header = (const struct tb_cfg_header *)data;
	const char *ret = trace_seq_buffer_ptr(p);

	trace_seq_printf(p, "route=%llx ", tb_cfg_get_route(header));

	return ret;
}

static inline const char *show_data(struct trace_seq *p, u8 type,
				    const u32 *data, u32 length)
{
	const char *ret = trace_seq_buffer_ptr(p);
	const char *prefix = "";
	int i;

	show_route(p, data);

	switch (type) {
	case TB_CFG_PKG_READ:
	case TB_CFG_PKG_WRITE:
		show_data_read_write(p, data);
		break;

	case TB_CFG_PKG_ERROR:
		show_data_error(p, data);
		break;

	case TB_CFG_PKG_EVENT:
		show_data_event(p, data);
		break;

	default:
		break;
	}

	trace_seq_printf(p, "[ ");
	for (i = 0; i < length; i++) {
		trace_seq_printf(p, "%s0x%08x", prefix, data[i]);
		prefix = " ";
	}
	trace_seq_printf(p, " ]");
	trace_seq_putc(p, 0);

	return ret;
}
#endif

DECLARE_EVENT_CLASS(tb_raw,
	TP_PROTO(u8 type, const void *data, size_t size, bool dropped),

	TP_ARGS(type, data, size, dropped),

	TP_STRUCT__entry(
		__field(u8, type)
		__field(size_t, size)
		__dynamic_array(u32, data, size / 4)
		__field(bool, dropped)
	),

	TP_fast_assign(
		__entry->type = type;
		__entry->size = size / 4;
		memcpy(__get_dynamic_array(data), data, size);
		__entry->dropped = dropped;
	),

	TP_printk("pdf=%s size=%zd %s",
		show_type_name(__entry->type), __entry->size,
		show_data(p, __entry->type, __get_dynamic_array(data), __entry->size)
	)
);

DEFINE_EVENT(tb_raw, tb_tx,
	TP_PROTO(u8 type, const void *data, size_t size, bool dropped),
	TP_ARGS(type, data, size, dropped)
);

DEFINE_EVENT(tb_raw, tb_event,
	TP_PROTO(u8 type, const void *data, size_t size, bool dropped),
	TP_ARGS(type, data, size, dropped)
);

DEFINE_EVENT_PRINT(tb_raw, tb_rx,
	TP_PROTO(u8 type, const void *data, size_t size, bool dropped),
	TP_ARGS(type, data, size, dropped),
	TP_printk("pdf=%s%s size=%zd %s",
		show_type_name(__entry->type),
		__entry->dropped ? " D" : "", __entry->size,
		show_data(p, __entry->type, __get_dynamic_array(data), __entry->size)
	)
);

#endif /* TB_TRACE_H_ */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace

/* This part must be outside protection */
#include <trace/define_trace.h>
