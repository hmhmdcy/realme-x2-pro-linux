/* SPDX-License-Identifier: GPL-2.0-only */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM drm_msm_dsi

#if !defined(_MSM_DSI_TRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _MSM_DSI_TRACE_H_

#include <linux/tracepoint.h>

TRACE_EVENT(msm_dsi_error_irq,
	TP_PROTO(int id, u32 isr),
	TP_ARGS(id, isr),
	TP_STRUCT__entry(
		__field(int, id)
		__field(u32, isr)
	),
	TP_fast_assign(
		__entry->id = id;
		__entry->isr = isr;
	),
	TP_printk("id=%d isr=0x%x", __entry->id, __entry->isr)
);

TRACE_EVENT(msm_dsi_error_status,
	TP_PROTO(int id, u32 reg, u32 raw, u32 state),
	TP_ARGS(id, reg, raw, state),
	TP_STRUCT__entry(
		__field(int, id)
		__field(u32, reg)
		__field(u32, raw)
		__field(u32, state)
	),
	TP_fast_assign(
		__entry->id = id;
		__entry->reg = reg;
		__entry->raw = raw;
		__entry->state = state;
	),
	TP_printk("id=%d reg=0x%x raw=0x%x state=0x%x", __entry->id,
		  __entry->reg, __entry->raw, __entry->state)
);

TRACE_EVENT(msm_dsi_error_work,
	TP_PROTO(int id, u32 state, bool done),
	TP_ARGS(id, state, done),
	TP_STRUCT__entry(
		__field(int, id)
		__field(u32, state)
		__field(bool, done)
	),
	TP_fast_assign(
		__entry->id = id;
		__entry->state = state;
		__entry->done = done;
	),
	TP_printk("id=%d state=0x%x done=%d", __entry->id,
		  __entry->state, __entry->done)
);

#endif /* _MSM_DSI_TRACE_H_ */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../drivers/gpu/drm/msm
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE msm_dsi_trace
#include <trace/define_trace.h>
