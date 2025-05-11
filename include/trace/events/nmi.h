/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM nmi

#if !defined(_TRACE_NMI_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_NMI_H

#include <linux/ktime.h>
#include <linux/tracepoint.h>

TRACE_EVENT(nmi_handler,

	TP_PROTO(void *handler, s64 delta_ns, int handled, unsigned long source_bitmap),

	TP_ARGS(handler, delta_ns, handled, source_bitmap),

	TP_STRUCT__entry(
		__field(	void *,		handler	)
		__field(	s64,		delta_ns)
		__field(	int,		handled	)
		__field(unsigned long,	source_bitmap)
	),

	TP_fast_assign(
		__entry->handler = handler;
		__entry->delta_ns = delta_ns;
		__entry->handled = handled;
		__entry->source_bitmap = source_bitmap;
	),

	TP_printk("%ps() delta_ns: %lld handled: %d source_bitmap: 0x%lx",
		__entry->handler,
		__entry->delta_ns,
		__entry->handled,
		__entry->source_bitmap)
);

#endif /* _TRACE_NMI_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
