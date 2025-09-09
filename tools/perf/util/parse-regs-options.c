// SPDX-License-Identifier: GPL-2.0
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <linux/bitops.h>
#include "util/debug.h"
#include <subcmd/parse-options.h>
#include "util/perf_regs.h"
#include "util/parse-regs-options.h"
#include "record.h"

static int
__parse_regs(const struct option *opt, const char *str, int unset, bool intr)
{
	uint64_t *mode = (uint64_t *)opt->value;
	const struct sample_reg *r = NULL;
	struct record_opts *opts;
	char *s, *os = NULL, *p;
	int ret = -1;
	uint64_t mask;
	uint64_t simd_mask;
	uint64_t pred_mask;
	uint64_t bitmap;
	u16 qwords;
	int reg_idx;

	if (unset)
		return 0;

	/*
	 * cannot set it twice
	 */
	if (*mode)
		return -1;

	if (intr) {
		opts = container_of(opt->value, struct record_opts, sample_intr_regs);
		mask = arch__intr_reg_mask();
		simd_mask = arch__intr_simd_reg_mask();
		pred_mask = arch__intr_pred_reg_mask();
	} else {
		opts = container_of(opt->value, struct record_opts, sample_user_regs);
		mask = arch__user_reg_mask();
		simd_mask = arch__user_simd_reg_mask();
		pred_mask = arch__user_pred_reg_mask();
	}

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
					if (r->mask & mask)
						fprintf(stderr, "%s ", r->name);
				}
				if (simd_mask) {
					bitmap = qwords = 0;
					for (r = arch__sample_simd_reg_masks(); r->name; r++) {
						if (r->mask & simd_mask) {
							reg_idx = fls64(r->mask) - 1;
							if (intr)
								bitmap = arch__intr_simd_reg_bitmap_qwords(reg_idx, &qwords);
							else
								bitmap = arch__user_simd_reg_bitmap_qwords(reg_idx, &qwords);
							if (bitmap)
								fprintf(stderr, "%s0-%d ", r->name, fls64(bitmap) - 1);
						}
					}
				}
				if (pred_mask) {
					bitmap = qwords = 0;
					for (r = arch__sample_pred_reg_masks(); r->name; r++) {
						if (r->mask & pred_mask) {
							reg_idx = fls64(r->mask) - 1;
							if (intr)
								bitmap = arch__intr_pred_reg_bitmap_qwords(reg_idx, &qwords);
							else
								bitmap = arch__user_pred_reg_bitmap_qwords(reg_idx, &qwords);
							if (bitmap)
								fprintf(stderr, "%s0-%d ", r->name, fls64(bitmap) - 1);
						}
					}
				}

				fputc('\n', stderr);
				/* just printing available regs */
				goto error;
			}

			if (simd_mask) {
				bitmap = qwords = 0;
				for (r = arch__sample_simd_reg_masks(); r->name; r++) {
					if (!strcasecmp(s, r->name)) {
						if (!fls64(r->mask))
							continue;
						reg_idx = fls64(r->mask) - 1;
						if (intr)
							bitmap = arch__intr_simd_reg_bitmap_qwords(reg_idx, &qwords);
						else
							bitmap = arch__user_simd_reg_bitmap_qwords(reg_idx, &qwords);
						break;
					}
				}

				/* Just need the highest qwords */
				if (qwords > opts->sample_vec_regs_qwords) {
					opts->sample_vec_regs_qwords = qwords;
					if (intr)
						opts->sample_intr_vec_regs = bitmap;
					else
						opts->sample_user_vec_regs = bitmap;
				}

				if (r->name)
					goto next;
			}
			if (pred_mask) {
				bitmap = qwords = 0;
				for (r = arch__sample_pred_reg_masks(); r->name; r++) {
					if (!strcasecmp(s, r->name)) {
						if (!fls64(r->mask))
							continue;
						reg_idx = fls64(r->mask) - 1;
						if (intr)
							bitmap = arch__intr_pred_reg_bitmap_qwords(reg_idx, &qwords);
						else
							bitmap = arch__user_pred_reg_bitmap_qwords(reg_idx, &qwords);
						break;
					}
				}

				/* Just need the highest qwords */
				if (qwords > opts->sample_pred_regs_qwords) {
					opts->sample_pred_regs_qwords = qwords;
					if (intr)
						opts->sample_intr_pred_regs = bitmap;
					else
						opts->sample_user_pred_regs = bitmap;
				}

				if (r->name)
					goto next;
			}

			for (r = arch__sample_reg_masks(); r->name; r++) {
				if ((r->mask & mask) && !strcasecmp(s, r->name))
					break;
			}
			if (!r || !r->name) {
				ui__warning("Unknown register \"%s\", check man page or run \"perf record %s?\"\n",
					    s, intr ? "-I" : "--user-regs=");
				goto error;
			}

			*mode |= r->mask;
next:
			if (!p)
				break;

			s = p + 1;
		}
	}
	ret = 0;

	/* default to all possible regs */
	if (*mode == 0)
		*mode = mask;
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
