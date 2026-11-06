/* SPDX-License-Identifier: GPL-2.0-only */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM drm_panel

#if !defined(_TRACE_DRM_PANEL_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_DRM_PANEL_H

#include <linux/tracepoint.h>

TRACE_EVENT(drm_panel_backlight,
	    TP_PROTO(const char *panel, int phase, int brightness),
	    TP_ARGS(panel, phase, brightness),
	    TP_STRUCT__entry(
		    __string(panel, panel)
		    __field(int, phase)
		    __field(int, brightness)
	    ),
	    TP_fast_assign(
		    __assign_str(panel);
		    __entry->phase = phase;
		    __entry->brightness = brightness;
	    ),
	    TP_printk("panel=%s phase=%d brightness=%d", __get_str(panel),
		      __entry->phase, __entry->brightness)
);

#endif

#include <trace/define_trace.h>
