// SPDX-License-Identifier: GPL-2.0
/*  Copyright(c) 2016-20 Intel Corporation. */

#include <stddef.h>
#include "defines.h"

static uint8_t encl_buffer[8192] = { 1 };

enum sgx_enclu_function {
	EACCEPT = 0x5,
	EMODPE = 0x6,
};

static void do_encl_emodpe(void *_op)
{
	struct sgx_secinfo secinfo __aligned(sizeof(struct sgx_secinfo)) = {0};
	struct encl_op_emodpe *op = _op;

	secinfo.flags = op->flags;

	asm volatile(".byte 0x0f, 0x01, 0xd7"
				:
				: "a" (EMODPE),
				  "b" (&secinfo),
				  "c" (op->epc_addr));
}

static void do_encl_eaccept(void *_op)
{
	struct sgx_secinfo secinfo __aligned(sizeof(struct sgx_secinfo)) = {0};
	struct encl_op_eaccept *op = _op;
	int rax;

	secinfo.flags = op->flags;

	asm volatile(".byte 0x0f, 0x01, 0xd7"
				: "=a" (rax)
				: "a" (EACCEPT),
				  "b" (&secinfo),
				  "c" (op->epc_addr));

	op->ret = rax;
}

static void *memcpy(void *dest, const void *src, size_t n)
{
	size_t i;

	for (i = 0; i < n; i++)
		((char *)dest)[i] = ((char *)src)[i];

	return dest;
}

void encl_body(void *rdi,  void *rsi)
{
	struct encl_op *op = (struct encl_op *)rdi;

	switch (op->type) {
	case ENCL_OP_PUT:
		memcpy(&encl_buffer[0], &op->buffer, 8);
		break;

	case ENCL_OP_GET:
		memcpy(&op->buffer, &encl_buffer[0], 8);
		break;

	default:
		break;
	}
}
