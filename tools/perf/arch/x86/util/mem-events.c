// SPDX-License-Identifier: GPL-2.0
#include "util/pmu.h"
#include "map_symbol.h"
#include "mem-events.h"

static char mem_loads_name[100];
static char mem_stores_name[100];

#define MEM_LOADS_AUX		0x8203
#define MEM_LOADS_AUX_NAME     "{%s/mem-loads-aux/,%s/mem-loads,ldlat=%u/}:P"

bool is_mem_loads_aux_event(struct evsel *leader)
{
	if (perf_pmu__find("cpu")) {
		if (!pmu_have_event("cpu", "mem-loads-aux"))
			return false;
	} else if (perf_pmu__find("cpu_core")) {
		if (!pmu_have_event("cpu_core", "mem-loads-aux"))
			return false;
	}

	return leader->core.attr.config == MEM_LOADS_AUX;
}

char *perf_mem_events__name(int i, char *pmu_name)
{
	struct perf_mem_event *e = perf_mem_events__ptr(i);

	if (!e)
		return NULL;

	if (!pmu_name)
		pmu_name = (char *)"cpu";

	if (i == PERF_MEM_EVENTS__LOAD) {
		if (pmu_have_event(pmu_name, "mem-loads-aux")) {
			scnprintf(mem_loads_name, sizeof(mem_loads_name),
				  MEM_LOADS_AUX_NAME, pmu_name, pmu_name,
				  perf_mem_events__loads_ldlat);
                } else {
                        scnprintf(mem_loads_name, sizeof(mem_loads_name),
				  e->name, pmu_name,
				  perf_mem_events__loads_ldlat);
		}
		return mem_loads_name;
	}

	if (i == PERF_MEM_EVENTS__STORE) {
		scnprintf(mem_stores_name, sizeof(mem_stores_name),
			  e->name, pmu_name);
		return mem_stores_name;
	}

	return (char *)e->name;
}
