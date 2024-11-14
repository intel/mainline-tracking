// SPDX-License-Identifier: GPL-2.0
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "util/debug.h"
#include <subcmd/parse-options.h>
#include "util/perf_regs.h"
#include "util/parse-regs-options.h"

static int
__parse_regs(const struct option *opt, const char *str, int unset, bool intr)
{
	unsigned int size = intr ? PERF_NUM_INTR_REGS * 64 : 64;
	uint64_t *mode = (uint64_t *)opt->value;
	const struct sample_reg *r = NULL;
	char *s, *os = NULL, *p;
	int ret = -1;
	DECLARE_BITMAP(mask, size);
	DECLARE_BITMAP(mask_tmp, size);

	if (unset)
		return 0;

	/*
	 * cannot set it twice
	 */
	if (bitmap_weight((unsigned long *)mode, size))
		return -1;

	bitmap_zero(mask, size);
	if (intr)
		arch__intr_reg_mask(mask);
	else
		*(uint64_t *)mask = arch__user_reg_mask();

	/* str may be NULL in case no arg is passed to -I */
	if (str) {
		/* because str is read-only */
		s = os = strdup(str);
		if (!s)
			return -1;

		for (;;) {
			p = strchr(s, ',');
			if (p)
				*p = '\0';

			if (!strcmp(s, "?")) {
				fprintf(stderr, "available registers: ");
				for (r = arch__sample_reg_masks(); r->name; r++) {
					bitmap_and(mask_tmp, mask, r->mask_ext, size);
					if (bitmap_weight(mask_tmp, size))
						fprintf(stderr, "%s ", r->name);
				}
				fputc('\n', stderr);
				/* just printing available regs */
				goto error;
			}
			for (r = arch__sample_reg_masks(); r->name; r++) {
				bitmap_and(mask_tmp, mask, r->mask_ext, size);
				if (bitmap_weight(mask_tmp, size) && !strcasecmp(s, r->name))
					break;
			}
			if (!r || !r->name) {
				ui__warning("Unknown register \"%s\", check man page or run \"perf record %s?\"\n",
					    s, intr ? "-I" : "--user-regs=");
				goto error;
			}

			bitmap_or((unsigned long *)mode, (unsigned long *)mode, r->mask_ext, size);

			if (!p)
				break;

			s = p + 1;
		}
	}
	ret = 0;

	/* default to all possible regs */
	if (!bitmap_weight((unsigned long *)mode, size))
		bitmap_or((unsigned long *)mode, (unsigned long *)mode, mask, size);
error:
	free(os);
	return ret;
}

int
parse_user_regs(const struct option *opt, const char *str, int unset)
{
	return __parse_regs(opt, str, unset, false);
}

int
parse_intr_regs(const struct option *opt, const char *str, int unset)
{
	return __parse_regs(opt, str, unset, true);
}
