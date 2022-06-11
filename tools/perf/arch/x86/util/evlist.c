// SPDX-License-Identifier: GPL-2.0
#include <stdio.h>
#include "util/pmu.h"
#include "util/evlist.h"
#include "util/parse-events.h"

#define TOPDOWN_L1_EVENTS	"{slots,topdown-retiring,topdown-bad-spec,topdown-fe-bound,topdown-be-bound}"
#define TOPDOWN_L1_EVENTS_ATOM  "{cpu_atom/topdown-retiring/,cpu_atom/topdown-bad-spec/,cpu_atom/topdown-fe-bound/,cpu_atom/topdown-be-bound/}"
#define TOPDOWN_L1_EVENTS_CORE  "{slots,cpu_core/topdown-retiring/,cpu_core/topdown-bad-spec/,cpu_core/topdown-fe-bound/,cpu_core/topdown-be-bound/}"
#define TOPDOWN_L2_EVENTS	"{slots,topdown-retiring,topdown-bad-spec,topdown-fe-bound,topdown-be-bound,topdown-heavy-ops,topdown-br-mispredict,topdown-fetch-lat,topdown-mem-bound}"
#define TOPDOWN_L2_EVENTS_ATOM  "{cpu_atom/topdown-retiring/,cpu_atom/topdown-bad-spec/,cpu_atom/topdown-fe-bound/,cpu_atom/topdown-be-bound/,cpu_atom/topdown-heavy-ops/,cpu_atom/topdown-br-mispredict/,cpu_atom/topdown-fetch-lat/,cpu_atom/topdown-mem-bound/}"
#define TOPDOWN_L2_EVENTS_CORE  "{slots,cpu_core/topdown-retiring/,cpu_core/topdown-bad-spec/,cpu_core/topdown-fe-bound/,cpu_core/topdown-be-bound/,cpu_core/topdown-heavy-ops/,cpu_core/topdown-br-mispredict/,cpu_core/topdown-fetch-lat/,cpu_core/topdown-mem-bound/}"

int arch_evlist__add_default_attrs(struct evlist *evlist, const char *pmu_name)
{
	const char *topdown_events;

	if (!pmu_name)
		return 0;
	if (strcmp(pmu_name, "cpu_atom") && !pmu_have_event(pmu_name, "slots"))
		return 0;
	if (pmu_have_event(pmu_name, "topdown-heavy-ops")){
		if(!strcmp(pmu_name, "cpu_core"))
			topdown_events = TOPDOWN_L2_EVENTS_CORE;
		else if(!strcmp(pmu_name, "cpu_atom"))
			topdown_events = TOPDOWN_L2_EVENTS_ATOM;
		else
			topdown_events = TOPDOWN_L2_EVENTS;
	}else{
		if(!strcmp(pmu_name, "cpu_core"))
			topdown_events = TOPDOWN_L1_EVENTS_CORE;
		else if(!strcmp(pmu_name, "cpu_atom"))
			topdown_events = TOPDOWN_L1_EVENTS_ATOM;
		else
			topdown_events = TOPDOWN_L1_EVENTS;
	}

	return parse_events(evlist, topdown_events, NULL);
}

struct evsel *arch_evlist__leader(struct list_head *list)
{
	struct evsel *evsel, *first, *slots = NULL;
	bool has_topdown = false;

	first = list_first_entry(list, struct evsel, core.node);

	if (!pmu_have_event("cpu", "slots"))
		return first;

	/* If there is a slots event and a topdown event then the slots event comes first. */
	__evlist__for_each_entry(list, evsel) {
		if (evsel->pmu_name && !strcmp(evsel->pmu_name, "cpu") && evsel->name) {
			if (strcasestr(evsel->name, "slots")) {
				slots = evsel;
				if (slots == first)
					return first;
			}
			if (!strncasecmp(evsel->name, "topdown", 7))
				has_topdown = true;
			if (slots && has_topdown)
				return slots;
		}
	}
	return first;
}
