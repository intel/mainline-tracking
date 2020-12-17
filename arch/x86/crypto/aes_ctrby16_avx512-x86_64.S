/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright © 2020 Intel Corporation.
 *
 * Implement AES CTR mode by16 optimization with VAES instructions. (x86_64)
 *
 * This is AES128/192/256 CTR mode optimization implementation. It requires
 * the support of Intel(R) AVX512VL and VAES instructions.
 *
 * This work was inspired by the AES CTR mode optimization published
 * in Intel Optimized IPSEC Cryptographic library.
 * https://github.com/intel/intel-ipsec-mb/avx512/cntr_vaes.asm
 *
 * Authors:
 * Tomasz Kantecki <tomasz.kantecki@intel.com>
 * Megha Dey <megha.dey@intel.com>
 */

#include "avx512_vaes_common.S"

#define ZKEY0	%zmm17
#define ZKEY1	%zmm18
#define ZKEY2	%zmm19
#define ZKEY3	%zmm20
#define ZKEY4	%zmm21
#define ZKEY5	%zmm22
#define ZKEY6	%zmm23
#define ZKEY7	%zmm24
#define ZKEY8	%zmm25
#define ZKEY9	%zmm26
#define ZKEY10	%zmm27
#define ZKEY11	%zmm28
#define ZKEY12	%zmm29
#define ZKEY13	%zmm30
#define ZKEY14	%zmm31

#define TMP0		%r10
#define TMP1		%r11
#define TMP2		%r12
#define	TMP3		%rax
#define DATA_OFFSET	%r13
#define RBITS		%r14
#define MASKREG		%k1
#define SHUFREG		%zmm13
#define ADD8REG		%zmm14

#define CTR_BLOCKx	%xmm0
#define CTR_BLOCK_1_4	%zmm1
#define CTR_BLOCK_5_8	%zmm2
#define CTR_BLOCK_9_12	%zmm3
#define CTR_BLOCK_13_16	%zmm4

#define ZTMP0		%zmm5
#define ZTMP1		%zmm6
#define ZTMP2		%zmm7
#define ZTMP3		%zmm8
#define ZTMP4		%zmm9
#define ZTMP5		%zmm10
#define ZTMP6		%zmm11
#define ZTMP7		%zmm12

#define	XTMP		%xmm15

#define STACK_FRAME_SIZE_CTR	(5*8)	/* space for 5 GP registers */

.text

/* Save register content for the caller */
#define FUNC_SAVE_CTR()				\
	mov	%rsp, %rax;			\
	sub	$STACK_FRAME_SIZE_CTR, %rsp;	\
	and	$~63, %rsp;			\
	mov	%r12, (%rsp);			\
	mov	%r13, 0x8(%rsp);		\
	mov	%rax, 0x18(%rsp);

/* Restore register content for the caller */
#define FUNC_RESTORE_CTR()			\
	vzeroupper;			\
	mov	(%rsp), %r12;		\
	mov	0x8(%rsp), %r13;	\
	mov	0x18(%rsp), %rsp;

/*
 * Maintain the bits from the output text when writing out the output blocks,
 * in case there are some bits that do not require encryption
 */
#define PRESERVE_BITS(RBITS, LENGTH, CYPH_PLAIN_OUT, ZIN_OUT, ZTMP0, ZTMP1, ZTMP2, IA0, IA1, blocks_to_skip, FULL_PARTIAL, MASKREG, DATA_OFFSET, NUM_ARGS)	\
/* offset = number of sets of 4 blocks to skip */			\
.set offset, (((blocks_to_skip) / 4) * 64);				\
\
/* num_left_blocks = blocks in the last set, range 1-4 blocks	*/	\
.set num_left_blocks,(((blocks_to_skip) & 3) + 1);			\
\
.if NUM_ARGS == 13;							\
	/* Load output to get last partial byte */			\
	.ifc FULL_PARTIAL, partial;					\
		vmovdqu8	offset(CYPH_PLAIN_OUT, DATA_OFFSET), ZTMP0{MASKREG};	\
	.else;								\
		vmovdqu8	offset(CYPH_PLAIN_OUT, DATA_OFFSET), ZTMP0;		\
	.endif;								\
.else;									\
	/* Load o/p to get last partial byte (upto the last 4 blocks) */\
	ZMM_LOAD_MASKED_BLOCKS_0_16(num_left_blocks, CYPH_PLAIN_OUT, offset, ZTMP0, no_zmm, no_zmm, no_zmm, MASKREG)	\
.endif;									\
\
	/* Save rcx in temporary GP register */				\
	mov	%rcx, IA0;						\
	mov	$0xff, DWORD(IA1);					\
	mov	BYTE(RBITS), %cl;					\
	/* e.g. 3 remaining bits=> mask = 00011111 */			\
	shr	%cl, DWORD(IA1);					\
	mov	IA0, %rcx;						\
\
	vmovq	IA1, XWORD(ZTMP1);					\
\
	/*
	 * Get number of full bytes in last block.			\
	 * Subtracting the bytes in the blocks to skip to the length of	\
	 * whole set of blocks gives us the number of bytes in the last	\
	 * block, but the last block has a partial byte at the end, so	\
	 * an extra byte needs to be subtracted.			\
	 */								\
	mov	LENGTH, IA1;						\
	sub	$(blocks_to_skip * 16 + 1), IA1;			\
	lea	shift_tab_16 + 16(%rip), IA0;				\
	sub	IA1, IA0;						\
	vmovdqu (IA0), XWORD(ZTMP2);					\
	vpshufb XWORD(ZTMP2), XWORD(ZTMP1), XWORD(ZTMP1);		\
.if num_left_blocks == 4;						\
	vshufi64x2	$0x15, ZTMP1, ZTMP1, ZTMP1;			\
.elseif num_left_blocks == 3;						\
	vshufi64x2	$0x45, ZTMP1, ZTMP1, ZTMP1;			\
.elseif num_left_blocks == 2;						\
	vshufi64x2	$0x51, ZTMP1, ZTMP1, ZTMP1;			\
.endif;	/* No need to shift if there is only one block */		\
\
	/*								\
	 * At this point, ZTMP1 contains a mask with all 0s, but with	\
	 * some 1s in the partial byte.					\
	 * First, clear last bits (not to be ciphered) of last output	\
	 * block. ZIN_OUT = ZIN_OUT AND NOT ZTMP1 (0x50 = andA!C)	\
	 */								\
	vpternlogq	$0x50, ZTMP1, ZTMP1, ZIN_OUT;			\
\
	/*								\
	 * Then, set these last bits to the bits coming from output.	\
	 * ZIN_OUT = ZIN_OUT OR (ZTMP0 AND ZTMP1) (0xF8 = orAandBC)	\
	 */								\
	vpternlogq	$0xF8, ZTMP1, ZTMP0, ZIN_OUT;

/*
 * CTR(128 bits) needs to be incremented. Since there is no 128-bit add
 * instruction, we need to increment 64-bit (least significant) and if an
 * overflow is detected, increment the most significant 64-bits.
 */
#define INCR_CNTR_128(CTR, ZT, const, num)			\
	vpaddq	const(%rip), XWORD(CTR), XTMP;			\
	vptest	ddq_low_msk(%rip), XTMP;			\
	jnz	64f;						\
	vpaddq	ddq_high_add_1(%rip), XTMP, XTMP;		\
	vpaddq	ddq_high_add_1(%rip), XWORD(CTR), XWORD(CTR);	\
64:;	\
	vinserti64x2	$num, XTMP, ZT, ZT;

/* Increment 4, 128 bit counters stored in a ZMM register */
#define INCR_CNTR_4_128(CTR, ZT)		\
	vmovdqa64	XWORD(CTR), XWORD(ZT);	\
	vshufi64x2	$0, ZT, ZT, ZT;		\
	INCR_CNTR_128(CTR, ZT, ddq_add_1, 1)	\
	INCR_CNTR_128(CTR, ZT, ddq_add_2, 2)	\
	INCR_CNTR_128(CTR, ZT, ddq_add_3, 3)	\
	vextracti32x4	$3, ZT, XWORD(CTR);

#define up_count(CTR, NUM_blocks, num, ZTMP)		\
.if NUM_blocks == num;					\
	jmp	76f;					\
.endif;							\
.if NUM_blocks > num;					\
	vpaddq	ddq_add_1(%rip), XWORD(CTR), XWORD(CTR);\
	INCR_CNTR_4_128(CTR, ZTMP)			\
.endif;							\
76:;

/* Increment 1 to 16 counters (1 to 4 ZMM registers based on number of blocks */
#define INCR_CNTR_NUM_BLOCKS(CNTR, ZTMP0, ZTMP1, ZTMP2, ZTMP3, NUM)	\
	INCR_CNTR_4_128(CNTR, ZTMP0)	\
	up_count(CNTR, NUM, 1, ZTMP1)	\
	up_count(CNTR, NUM, 2, ZTMP2)	\
	up_count(CNTR, NUM, 3, ZTMP3)

#define UPDATE_COUNTERS(CTR, ZT1, ZT2, ZT3, ZT4, of_num, num)	\
	vshufi64x2	$0, ZWORD(CTR), ZWORD(CTR), ZWORD(CTR); \
	vmovq		XWORD(CTR), TMP3;			\
	cmp		$~of_num, TMP3;				\
	jb		77f;					\
	INCR_CNTR_NUM_BLOCKS(CTR, ZT1, ZT2, ZT3, ZT4, num)	\
	jmp		78f;					\
77:;								\
	vpaddd	ddq_add_0_3(%rip), ZWORD(CTR), ZT1;		\
.if (num > 1);							\
	vpaddd	ddq_add_4_7(%rip), ZWORD(CTR), ZT2;		\
.endif;								\
.if (num > 2);							\
	vpaddd	ddq_add_8_11(%rip), ZWORD(CTR), ZT3;		\
.endif;								\
.if (num > 3);							\
	vpaddd	ddq_add_12_15(%rip), ZWORD(CTR), ZT4;		\
.endif;								\
78:;

/* Prepares the AES counter blocka */
#define PREPARE_COUNTER_BLOCKS(CTR, ZT1, ZT2, ZT3, ZT4, num_initial_blocks)	\
.if num_initial_blocks == 1;					\
	vmovdqa64	XWORD(CTR), XWORD(ZT1);			\
.elseif num_initial_blocks == 2;				\
	vshufi64x2	$0, YWORD(CTR), YWORD(CTR), YWORD(ZT1); \
	vmovq		XWORD(CTR), TMP3;			\
	cmp		$~1, TMP3;				\
	jb		50f;					\
	vmovdqa64	XWORD(CTR), XWORD(ZT1);			\
	vshufi64x2	$0, YWORD(ZT1), YWORD(ZT1), YWORD(ZT1); \
	INCR_CNTR_128(CTR, ZT1, ddq_add_1, 1)			\
	vextracti32x4	$1, YWORD(ZT1), XWORD(CTR);		\
	jmp		55f;					\
50:;								\
	vpaddd	ddq_add_0_3(%rip), YWORD(ZT1), YWORD(ZT1);	\
.elseif num_initial_blocks <= 4;				\
	UPDATE_COUNTERS(CTR, ZT1, ZT2, ZT3, ZT4, 3, 1)		\
.elseif num_initial_blocks <= 8;				\
	UPDATE_COUNTERS(CTR, ZT1, ZT2, ZT3, ZT4, 7, 2)		\
.elseif num_initial_blocks <= 12;				\
	UPDATE_COUNTERS(CTR, ZT1, ZT2, ZT3, ZT4, 11, 3)		\
.else;								\
	UPDATE_COUNTERS(CTR, ZT1, ZT2, ZT3, ZT4, 15, 4)		\
.endif;								\
55:;

/* Extract and Shuffle the updated counters for AES rounds */
#define	EXTRACT_CNTR_VAL(ZT1, ZT2, ZT3, ZT4, SHUFREG, CTR, num_initial_blocks)	\
.if num_initial_blocks == 1;					\
	vpshufb		XWORD(SHUFREG), CTR, XWORD(ZT1);	\
.elseif num_initial_blocks == 2;				\
	vextracti32x4	$1, YWORD(ZT1), CTR;			\
	vpshufb		YWORD(SHUFREG), YWORD(ZT1), YWORD(ZT1); \
.elseif num_initial_blocks <= 4;				\
	vextracti32x4	$(num_initial_blocks - 1), ZT1, CTR;	\
	vpshufb		SHUFREG, ZT1, ZT1;			\
.elseif num_initial_blocks == 5;				\
	vmovdqa64	XWORD(ZT2), CTR;			\
	vpshufb		SHUFREG, ZT1, ZT1;			\
	vpshufb		XWORD(SHUFREG), XWORD(ZT2), XWORD(ZT2); \
.elseif num_initial_blocks == 6;				\
	vextracti32x4	$1, YWORD(ZT2), CTR;			\
	vpshufb		SHUFREG, ZT1, ZT1;			\
	vpshufb		YWORD(SHUFREG), YWORD(ZT2), YWORD(ZT2); \
.elseif num_initial_blocks == 7;				\
	vextracti32x4	$2, ZT2, CTR;				\
	vpshufb		SHUFREG, ZT1, ZT1;			\
	vpshufb		SHUFREG, ZT2, ZT2;			\
.elseif num_initial_blocks == 8;				\
	vextracti32x4	$3, ZT2, CTR;				\
	vpshufb		SHUFREG, ZT1, ZT1;			\
	vpshufb		SHUFREG, ZT2, ZT2;			\
.elseif num_initial_blocks == 9;				\
	vmovdqa64	XWORD(ZT3), CTR;			\
	vpshufb		SHUFREG, ZT1, ZT1;			\
	vpshufb		SHUFREG, ZT2, ZT2;			\
	vpshufb		XWORD(SHUFREG), XWORD(ZT3), XWORD(ZT3); \
.elseif num_initial_blocks == 10;				\
	vextracti32x4	$1, YWORD(ZT3), CTR;			\
	vpshufb		SHUFREG, ZT1, ZT1;			\
	vpshufb		SHUFREG, ZT2, ZT2;			\
	vpshufb		YWORD(SHUFREG), YWORD(ZT3), YWORD(ZT3); \
.elseif num_initial_blocks == 11;				\
	vextracti32x4	$2, ZT3, CTR;				\
	vpshufb		SHUFREG, ZT1, ZT1;			\
	vpshufb		SHUFREG, ZT2, ZT2;			\
	vpshufb		SHUFREG, ZT3, ZT3;			\
.elseif num_initial_blocks == 12;				\
	vextracti32x4	$3, ZT3, CTR;				\
	vpshufb		SHUFREG, ZT1, ZT1;			\
	vpshufb		SHUFREG, ZT2, ZT2;			\
	vpshufb		SHUFREG, ZT3, ZT3;			\
.elseif num_initial_blocks == 13;				\
	vmovdqa64	XWORD(ZT4), CTR;			\
	vpshufb		SHUFREG, ZT1, ZT1;			\
	vpshufb		SHUFREG, ZT2, ZT2;			\
	vpshufb		SHUFREG, ZT3, ZT3;			\
	vpshufb		XWORD(SHUFREG), XWORD(ZT4), XWORD(ZT4);	\
.elseif num_initial_blocks == 14;				\
	vextracti32x4	$1, YWORD(ZT4), CTR;			\
	vpshufb		SHUFREG, ZT1, ZT1;			\
	vpshufb		SHUFREG, ZT2, ZT2;			\
	vpshufb		SHUFREG, ZT3, ZT3;			\
	vpshufb		YWORD(SHUFREG), YWORD(ZT4), YWORD(ZT4);	\
.elseif num_initial_blocks == 15;				\
	vextracti32x4	$2, ZT4, CTR;				\
	vpshufb		SHUFREG, ZT1, ZT1;			\
	vpshufb		SHUFREG, ZT2, ZT2;			\
	vpshufb		SHUFREG, ZT3, ZT3;			\
	vpshufb		SHUFREG, ZT4, ZT4;			\
.endif;

/* AES rounds and XOR with plain/cipher text */
#define AES_XOR_ROUNDS(ZT1, ZT2, ZT3, ZT4, ZKEY_0, ZKEY_1, ZKEY_2, ZKEY_3, ZKEY_4, ZKEY_5, ZKEY_6, ZKEY_7, ZKEY_8, ZKEY_9, ZKEY_10, ZKEY_11, ZKEY_12, ZKEY_13, ZKEY_14, ZT5, ZT6, ZT7, ZT8, num_initial_blocks, NROUNDS)	\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, ZT3, ZT4, ZKEY0, 0, ZT5, ZT6, ZT7, ZT8, num_initial_blocks, NROUNDS)	\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, ZT3, ZT4, ZKEY1, 1, ZT5, ZT6, ZT7, ZT8, num_initial_blocks, NROUNDS)	\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, ZT3, ZT4, ZKEY2, 2, ZT5, ZT6, ZT7, ZT8, num_initial_blocks, NROUNDS)	\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, ZT3, ZT4, ZKEY3, 3, ZT5, ZT6, ZT7, ZT8, num_initial_blocks, NROUNDS)	\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, ZT3, ZT4, ZKEY4, 4, ZT5, ZT6, ZT7, ZT8, num_initial_blocks, NROUNDS)	\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, ZT3, ZT4, ZKEY5, 5, ZT5, ZT6, ZT7, ZT8, num_initial_blocks, NROUNDS)	\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, ZT3, ZT4, ZKEY6, 6, ZT5, ZT6, ZT7, ZT8, num_initial_blocks, NROUNDS)	\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, ZT3, ZT4, ZKEY7, 7, ZT5, ZT6, ZT7, ZT8, num_initial_blocks, NROUNDS)	\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, ZT3, ZT4, ZKEY8, 8, ZT5, ZT6, ZT7, ZT8, num_initial_blocks, NROUNDS)	\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, ZT3, ZT4, ZKEY9, 9, ZT5, ZT6, ZT7, ZT8, num_initial_blocks, NROUNDS)	\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, ZT3, ZT4, ZKEY10, 10, ZT5, ZT6, ZT7, ZT8, num_initial_blocks, NROUNDS)	\
.if NROUNDS == 9;	\
	jmp 28f;	\
.else;			\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, ZT3, ZT4, ZKEY11, 11, ZT5, ZT6, ZT7, ZT8, num_initial_blocks, NROUNDS)	\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, ZT3, ZT4, ZKEY12, 12, ZT5, ZT6, ZT7, ZT8, num_initial_blocks, NROUNDS)	\
.if NROUNDS == 11;	\
	jmp 28f;	\
.else;			\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, ZT3, ZT4, ZKEY13, 13, ZT5, ZT6, ZT7, ZT8, num_initial_blocks, NROUNDS)	\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, ZT3, ZT4, ZKEY14, 14, ZT5, ZT6, ZT7, ZT8, num_initial_blocks, NROUNDS)	\
.endif;			\
.endif;			\
28:;

/* IV is updated to current counter + 1 and returned to the upper glue layers */
#define UPDATE_IV(CTR)				\
	vpaddd	ONE(%rip), CTR, CTR;		\
	vptest	ddq_low_msk(%rip), CTR;		\
	jnz 27f;				\
	vpaddq	ddq_high_add_1(%rip), CTR, CTR;	\
27:;						\
	vpshufb SHUF_MASK(%rip), CTR, CTR;	\
	vmovdqu CTR, (arg5);

/*
 * Macro with support for a partial final block. It may look similar to
 * INITIAL_BLOCKS but its usage is different. It is not meant to cipher
 * counter blocks for the main by16 loop. Just ciphers amount of blocks.
 * Used for small packets (<256 bytes). num_initial_blocks is expected
 * to include the partial final block in the count.
 */
#define INITIAL_BLOCKS_PARTIAL(KEY, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, num_initial_blocks, CTR, ZT1, ZT2, ZT3, ZT4, ZT5, ZT6, ZT7, ZT8, IA0, IA1, MASKREG, SHUFREG, NROUNDS, RBITS)      \
	/* get load/store mask */				\
	lea	byte64_len_to_mask_table(%rip), IA0;		\
	mov	LENGTH, IA1;					\
.if num_initial_blocks > 12;					\
	sub	 $192, IA1;					\
.elseif num_initial_blocks > 8;					\
	sub	$128, IA1;					\
.elseif num_initial_blocks > 4;					\
	sub	$64, IA1;					\
.endif;								\
	kmovq	(IA0, IA1, 8), MASKREG;				\
\
	ZMM_LOAD_MASKED_BLOCKS_0_16(num_initial_blocks, PLAIN_CYPH_IN, 1, ZT5, ZT6, ZT7, ZT8, MASKREG)	\
	PREPARE_COUNTER_BLOCKS(CTR, ZT1, ZT2, ZT3, ZT4, num_initial_blocks)	\
	EXTRACT_CNTR_VAL(ZT1, ZT2, ZT3, ZT4, SHUFREG, CTR, num_initial_blocks)	\
	AES_XOR_ROUNDS(ZT1, ZT2, ZT3, ZT4, ZKEY0, ZKEY1, ZKEY2, ZKEY3, ZKEY4, ZKEY5, ZKEY6, ZKEY7, ZKEY8, ZKEY9, ZKEY10, ZKEY11, ZKEY12, ZKEY13, ZKEY14, ZT5, ZT6, ZT7, ZT8, num_initial_blocks, NROUNDS)     \
	/* write cipher/plain text back to output */		\
	ZMM_STORE_MASKED_BLOCKS_0_16(num_initial_blocks, CYPH_PLAIN_OUT, 1, ZT1, ZT2, ZT3, ZT4, MASKREG)	\
	UPDATE_IV(XWORD(CTR))

/* This macro is used to "warm-up" pipeline for ENCRYPT_16_PARALLEL macro
 * code. It is called only for data lengths 256 and above. The flow is as
 * follows:
 * - encrypt the initial num_initial_blocks blocks (can be 0)
 * - encrypt the next 16 blocks
 * - the last 16th block can be partial (lengths between 257 and 367)
 * - partial block ciphering is handled within this macro
 */
#define INITIAL_BLOCKS(KEY, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, DATA_OFFSET, num_initial_blocks, CTR, CTR_1_4, CTR_5_8, CTR_9_12, CTR_13_16, ZT1, ZT2, ZT3, ZT4, ZT5, ZT6, ZT7, ZT8, IA0, IA1, MASKREG, SHUFREG, NROUNDS, RBITS)	\
.if num_initial_blocks > 0;						\
	/* load plain/cipher text */					\
	ZMM_LOAD_BLOCKS_0_16(num_initial_blocks, PLAIN_CYPH_IN, 1, ZT5, ZT6, ZT7, ZT8, load_4_instead_of_3)	\
	PREPARE_COUNTER_BLOCKS(CTR, ZT1, ZT2, ZT3, ZT4, num_initial_blocks)	\
	EXTRACT_CNTR_VAL(ZT1, ZT2, ZT3, ZT4, SHUFREG, CTR, num_initial_blocks)	\
	 AES_XOR_ROUNDS(ZT1, ZT2, ZT3, ZT4, ZKEY0, ZKEY1, ZKEY2, ZKEY3, ZKEY4, ZKEY5, ZKEY6, ZKEY7, ZKEY8, ZKEY9, ZKEY10, ZKEY11, ZKEY12, ZKEY13, ZKEY14, ZT5, ZT6, ZT7, ZT8, num_initial_blocks, NROUNDS)	\
	/* write cipher/plain text back to output */			\
	ZMM_STORE_BLOCKS_0_16(num_initial_blocks, CYPH_PLAIN_OUT, 1, ZT1, ZT2, ZT3, ZT4)	\
	/* adjust data offset and length */				\
	sub	$(num_initial_blocks * 16), LENGTH;			\
	add	$(num_initial_blocks * 16), DATA_OFFSET;		\
.endif;									\
\
	/* - cipher of num_initial_blocks is done			\
	 * - prepare counter blocks for the next 16 blocks (ZT5-ZT8)	\
	 * - shuffle the blocks for AES					\
	 * - encrypt the next 16 blocks					\
	 */								\
\
	 /* get text load/store mask (assume full mask by default) */	\
	 mov	 $~0, IA0;						\
.if num_initial_blocks > 0;						\
	/* This macro is executed for length 256 and up, zero length	\
	 * is checked in CNTR_ENC_DEC. We know there is a partial block	\
	 * if: LENGTH - 16*num_initial_blocks < 256			\
	 */								\
	cmp	$256, LENGTH;						\
	jge	56f;							\
	mov	%rcx, IA1;						\
	mov	$256, %ecx;						\
	sub	LENGTH, %rcx;						\
	shr	%cl, IA0;						\
	mov	IA1, %rcx;						\
56:;									\
.endif;									\
	kmovq	IA0, MASKREG;						\
	/* load plain or cipher text */					\
	vmovdqu8	(PLAIN_CYPH_IN, DATA_OFFSET, 1), ZT5;		\
	vmovdqu8	64(PLAIN_CYPH_IN, DATA_OFFSET), ZT6;		\
	vmovdqu8	128(PLAIN_CYPH_IN, DATA_OFFSET), ZT7;		\
	vmovdqu8	192(PLAIN_CYPH_IN, DATA_OFFSET), ZT8{MASKREG}{z};	\
\
	/* prepare next counter blocks */				\
	vshufi64x2	$0, ZWORD(CTR), ZWORD(CTR), ZWORD(CTR);		\
.if num_initial_blocks > 0;						\
	vmovq	XWORD(CTR), TMP3;					\
	cmp	$~16, TMP3;						\
	jb	58f;							\
	vpaddq	ddq_add_1(%rip), XWORD(CTR), XWORD(CTR);		\
	vptest	ddq_low_msk(%rip), XWORD(CTR);				\
	jnz 57f;							\
	vpaddq	ddq_high_add_1(%rip), XWORD(CTR), XWORD(CTR);		\
57:;	\
	INCR_CNTR_NUM_BLOCKS(CTR, CTR_1_4, CTR_5_8, CTR_9_12, CTR_13_16, 4)	\
	jmp 60f;							\
58:;									\
	vpaddd	      ddq_add_1_4(%rip), ZWORD(CTR), CTR_1_4;		\
	vpaddd	      ddq_add_5_8(%rip), ZWORD(CTR), CTR_5_8;		\
	vpaddd	      ddq_add_9_12(%rip), ZWORD(CTR), CTR_9_12;		\
	vpaddd	      ddq_add_13_16(%rip), ZWORD(CTR), CTR_13_16;	\
.else;									\
	vmovq	XWORD(CTR), TMP3;					\
	cmp	$~15, TMP3;						\
	jb	59f;							\
	INCR_CNTR_NUM_BLOCKS(CTR, CTR_1_4, CTR_5_8, CTR_9_12, CTR_13_16, 4)	\
	jmp 60f;							\
59:;									\
	vpaddd	      ddq_add_0_3(%rip), ZWORD(CTR), CTR_1_4;		\
	vpaddd	      ddq_add_4_7(%rip), ZWORD(CTR), CTR_5_8;		\
	vpaddd	      ddq_add_8_11(%rip), ZWORD(CTR), CTR_9_12;		\
	vpaddd	      ddq_add_12_15(%rip), ZWORD(CTR), CTR_13_16;	\
.endif;									\
60:;									\
\
	vpshufb		SHUFREG, CTR_1_4, ZT1;				\
	vpshufb		SHUFREG, CTR_5_8, ZT2;				\
	vpshufb		SHUFREG, CTR_9_12, ZT3;				\
	vpshufb		SHUFREG, CTR_13_16, ZT4;			\
\
	AES_XOR_ROUNDS(ZT1, ZT2, ZT3, ZT4, ZKEY0, ZKEY1, ZKEY2, ZKEY3, ZKEY4, ZKEY5, ZKEY6, ZKEY7, ZKEY8, ZKEY9, ZKEY10, ZKEY11, ZKEY12, ZKEY13, ZKEY14, ZT5, ZT6, ZT7, ZT8, 16, NROUNDS)	\
\
	/* write cipher/plain text back to output */			\
	vmovdqu8	ZT1, (CYPH_PLAIN_OUT, DATA_OFFSET);		\
	vmovdqu8	ZT2, 64(CYPH_PLAIN_OUT, DATA_OFFSET, 1);	\
	vmovdqu8	ZT3, 128(CYPH_PLAIN_OUT, DATA_OFFSET, 1);	\
	vmovdqu8	ZT4, 192(CYPH_PLAIN_OUT, DATA_OFFSET, 1){MASKREG};	\
\
	vextracti32x4 $3, CTR_13_16, XWORD(CTR);			\
	UPDATE_IV(CTR)							\
\
	/* check if there is partial block */				\
	cmp	$256, LENGTH;						\
	jl	61f;							\
	/* adjust offset and length */					\
	add	$256, DATA_OFFSET;					\
	sub	$256, LENGTH;						\
	jmp	62f;							\
61:;									\
	/* zero the length (all encryption is complete) */		\
	xor	LENGTH, LENGTH;						\
62:;

/*
 * This macro ciphers payloads shorter than 256 bytes. The number of blocks in
 * the message comes as an argument. Depending on the number of blocks, an
 * optimized variant of INITIAL_BLOCKS_PARTIAL is invoked
 */
#define CNTR_ENC_DEC_SMALL(KEY, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, NUM_BLOCKS, CTR, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, IA0, IA1, MASKREG, SHUFREG, NROUNDS, RBITS)  \
	cmp	$8, NUM_BLOCKS;		\
	je	38f;			\
	jl	48f;			\
	/* Initial blocks 9-16 */	\
	cmp	$12, NUM_BLOCKS;	\
	je	42f;			\
	jl	49f;			\
	/* Initial blocks 13-16 */	\
	cmp	$16, NUM_BLOCKS;	\
	je	46f;			\
	cmp	$15, NUM_BLOCKS;	\
	je	45f;			\
	cmp	$14, NUM_BLOCKS;	\
	je	44f;			\
	cmp	$13, NUM_BLOCKS;	\
	je	43f;			\
49:;					\
	cmp	$11, NUM_BLOCKS;	\
	je	41f;			\
	cmp	$10, NUM_BLOCKS;	\
	je	40f;			\
	cmp	$9, NUM_BLOCKS;		\
	je	39f;			\
48:;					\
	cmp	$4, NUM_BLOCKS;		\
	je	34f;			\
	jl	47f;			\
	/* Initial blocks 5-7 */	\
	cmp	$7, NUM_BLOCKS;		\
	je	37f;			\
	cmp	$6, NUM_BLOCKS;		\
	je	36f;			\
	cmp	$5, NUM_BLOCKS;		\
	je	35f;			\
47:;					\
	cmp	$3, NUM_BLOCKS;		\
	je	33f;			\
	cmp	$2, NUM_BLOCKS;		\
	je	32f;			\
	jmp	31f;			\
46:;	\
	INITIAL_BLOCKS_PARTIAL(KEY, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, 16, CTR, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, IA0, IA1, MASKREG, SHUFREG, NROUNDS, RBITS)      \
	jmp	30f;	\
45:;	\
	INITIAL_BLOCKS_PARTIAL(KEY, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, 15, CTR, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, IA0, IA1, MASKREG, SHUFREG, NROUNDS, RBITS)      \
	jmp	30f;	\
44:;	\
	INITIAL_BLOCKS_PARTIAL(KEY, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, 14, CTR, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, IA0, IA1, MASKREG, SHUFREG, NROUNDS, RBITS)      \
	jmp	30f;	\
43:;	\
	INITIAL_BLOCKS_PARTIAL(KEY, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, 13, CTR, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, IA0, IA1, MASKREG, SHUFREG, NROUNDS, RBITS)      \
	jmp	30f;	\
42:;	\
	INITIAL_BLOCKS_PARTIAL(KEY, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, 12, CTR, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, IA0, IA1, MASKREG, SHUFREG, NROUNDS, RBITS)      \
	jmp	30f;	\
41:;	\
	INITIAL_BLOCKS_PARTIAL(KEY, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, 11, CTR, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, IA0, IA1, MASKREG, SHUFREG, NROUNDS, RBITS)      \
	jmp	30f;	\
40:;	\
	INITIAL_BLOCKS_PARTIAL(KEY, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, 10, CTR, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, IA0, IA1, MASKREG, SHUFREG, NROUNDS, RBITS)      \
	jmp	30f;	\
39:;	\
	INITIAL_BLOCKS_PARTIAL(KEY, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, 9, CTR, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, IA0, IA1, MASKREG, SHUFREG, NROUNDS, RBITS)	    \
	jmp	30f;	\
38:;	\
	INITIAL_BLOCKS_PARTIAL(KEY, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, 8, CTR, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, IA0, IA1, MASKREG, SHUFREG, NROUNDS, RBITS)	    \
	jmp	30f;	\
37:;	\
	INITIAL_BLOCKS_PARTIAL(KEY, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, 7, CTR, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, IA0, IA1, MASKREG, SHUFREG, NROUNDS, RBITS)	    \
	jmp	30f;	\
36:;	\
	INITIAL_BLOCKS_PARTIAL(KEY, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, 6, CTR, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, IA0, IA1, MASKREG, SHUFREG, NROUNDS, RBITS)	    \
	jmp	30f;	\
35:;	\
	INITIAL_BLOCKS_PARTIAL(KEY, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, 5, CTR, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, IA0, IA1, MASKREG, SHUFREG, NROUNDS, RBITS)	    \
	jmp	30f;	\
34:;	\
	INITIAL_BLOCKS_PARTIAL(KEY, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, 4, CTR, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, IA0, IA1, MASKREG, SHUFREG, NROUNDS, RBITS)	    \
	jmp	30f;	\
33:;	\
	INITIAL_BLOCKS_PARTIAL(KEY, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, 3, CTR, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, IA0, IA1, MASKREG, SHUFREG, NROUNDS, RBITS)	    \
	jmp	30f;	\
32:;	\
	INITIAL_BLOCKS_PARTIAL(KEY, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, 2, CTR, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, IA0, IA1, MASKREG, SHUFREG, NROUNDS, RBITS)	    \
	jmp	30f;	\
31:;	\
	INITIAL_BLOCKS_PARTIAL(KEY, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, 1, CTR, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, IA0, IA1, MASKREG, SHUFREG, NROUNDS, RBITS)	    \
30:;

/*
 * This is the main CNTR macro. It operates on single stream and encrypts 16
 * blocks at a time
 */
#define ENCRYPT_16_PARALLEL(KEY, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, DATA_OFFSET, CTR_1_4, CTR_5_8, CTR_9_12, CTR_13_16, FULL_PARTIAL, IA0, IA1, LENGTH, ZT1, ZT2, ZT3, ZT4, ZT5, ZT6, ZT7, ZT8, MASKREG, SHUFREG, ADD8REG, NROUNDS, RBITS)	\
	/* load/store mask (partial case) and load the text data */	\
.ifc FULL_PARTIAL, full;	\
	vmovdqu8	(PLAIN_CYPH_IN, DATA_OFFSET, 1), ZT5;		\
	vmovdqu8	64(PLAIN_CYPH_IN, DATA_OFFSET, 1), ZT6;		\
	vmovdqu8	128(PLAIN_CYPH_IN, DATA_OFFSET, 1), ZT7;	\
	vmovdqu8	192(PLAIN_CYPH_IN, DATA_OFFSET, 1), ZT8;	\
.else;									\
	lea		byte64_len_to_mask_table(%rip), IA0;		\
	mov		LENGTH, IA1;					\
	sub		$192, IA1;					\
	kmovq		(IA0, IA1, 8), MASKREG;				\
	vmovdqu8	(PLAIN_CYPH_IN, DATA_OFFSET, 1), ZT5;		\
	vmovdqu8	64(PLAIN_CYPH_IN, DATA_OFFSET, 1), ZT6;		\
	vmovdqu8	128(PLAIN_CYPH_IN, DATA_OFFSET, 1), ZT7;	\
	vmovdqu8	192(PLAIN_CYPH_IN, DATA_OFFSET, 1), ZT8{MASKREG}{z};	\
.endif;									\
	/*								\
	 * populate counter blocks					\
	 * CTR is shuffled outside the scope of this macro		\
	 * it has to be kept in unshuffled form				\
	 */								\
	vpaddd		ADD8REG, CTR_1_4, CTR_1_4;			\
	vpaddd		ADD8REG, CTR_5_8, CTR_5_8;			\
	vpaddd		ADD8REG, CTR_9_12, CTR_9_12;			\
	vpaddd		ADD8REG, CTR_13_16, CTR_13_16;			\
	vpshufb		SHUFREG, CTR_1_4, ZT1;				\
	vpshufb		SHUFREG, CTR_5_8, ZT2;				\
	vpshufb		SHUFREG, CTR_9_12, ZT3;				\
	vpshufb		SHUFREG, CTR_13_16, ZT4;			\
	AES_XOR_ROUNDS(ZT1, ZT2, ZT3, ZT4, ZKEY0, ZKEY1, ZKEY2, ZKEY3, ZKEY4, ZKEY5, ZKEY6, ZKEY7, ZKEY8, ZKEY9, ZKEY10, ZKEY11, ZKEY12, ZKEY13, ZKEY14, ZT5, ZT6, ZT7, ZT8, 16, NROUNDS)     \
	/*store the text data */					\
	vmovdqu8	ZT1, (CYPH_PLAIN_OUT, DATA_OFFSET);		\
	vmovdqu8	ZT2, 64(CYPH_PLAIN_OUT, DATA_OFFSET, 1);	\
	vmovdqu8	ZT3, 128(CYPH_PLAIN_OUT, DATA_OFFSET, 1);	\
.ifc FULL_PARTIAL, full;						\
	vmovdqu8	ZT4, 192(CYPH_PLAIN_OUT, DATA_OFFSET, 1);	\
.else;									\
	vmovdqu8	ZT4, 192(CYPH_PLAIN_OUT, DATA_OFFSET, 1){MASKREG};	\
.endif;

/*
 * CNTR_ENC_DEC Encodes/Decodes given data. Requires the input data be
 * at least 1 byte long because of READ_SMALL_INPUT_DATA.
 */
#define CNTR_ENC_DEC(KEYS, DST, SRC, LEN, IV, NROUNDS)		\
	or	LEN, LEN;					\
	je	22f;						\
/*								\
 * Macro flow:							\
 * - calculate the number of 16byte blocks in the message	\
 * - process (number of 16byte blocks) mod 16			\
 * - process 16x16 byte blocks at a time until all are done	\
 */								\
	xor	DATA_OFFSET, DATA_OFFSET;			\
/* Prepare round keys */					\
	vbroadcastf64x2 16*0(KEYS), ZKEY0;			\
	vbroadcastf64x2 16*1(KEYS), ZKEY1;			\
	vbroadcastf64x2 16*2(KEYS), ZKEY2;			\
	vbroadcastf64x2 16*3(KEYS), ZKEY3;			\
	vbroadcastf64x2 16*4(KEYS), ZKEY4;			\
	vbroadcastf64x2 16*5(KEYS), ZKEY5;			\
	vbroadcastf64x2 16*6(KEYS), ZKEY6;			\
	vbroadcastf64x2 16*7(KEYS), ZKEY7;			\
	vbroadcastf64x2 16*8(KEYS), ZKEY8;			\
	vbroadcastf64x2 16*9(KEYS), ZKEY9;			\
	vbroadcastf64x2 16*10(KEYS), ZKEY10;			\
.if NROUNDS == 9;						\
	jmp 23f;						\
.else;								\
	vbroadcastf64x2 16*11(KEYS), ZKEY11;			\
	vbroadcastf64x2 16*12(KEYS), ZKEY12;			\
.if NROUNDS == 11;						\
	jmp 23f;						\
.else;								\
	vbroadcastf64x2 16*13(KEYS), ZKEY13;			\
	vbroadcastf64x2 16*14(KEYS), ZKEY14;			\
.endif;								\
.endif;								\
23:;								\
	mov	$16, TMP2;					\
	/* Set mask to read 16 IV bytes */			\
	mov	mask_16_bytes(%rip), TMP0;			\
	kmovq	TMP0, MASKREG;					\
	vmovdqu8	(IV), CTR_BLOCKx{MASKREG};		\
	vmovdqa64	SHUF_MASK(%rip), SHUFREG;		\
	/* store IV as counter in LE format */			\
	vpshufb XWORD(SHUFREG), CTR_BLOCKx, CTR_BLOCKx;		\
	/* Determine how many blocks to process in INITIAL */	\
	mov	LEN, TMP1;					\
	shr	$4, TMP1;					\
	and	$0xf, TMP1;					\
	/*							\
	 * Process one additional block in INITIAL_ macros if	\
	 * there is a partial block.				\
	 */							\
	mov	LEN, TMP0;					\
	and	$0xf, TMP0;					\
	add	$0xf, TMP0;					\
	shr	$4, TMP0;					\
	add	TMP0, TMP1;					\
	/* IA1 can be in the range from 0 to 16 */		\
\
	/* Less than 256B will be handled by the small message	\
	 * code, which can process up to 16x blocks (16 bytes)	\
	 */							\
	cmp	$256, LEN;					\
	jge	20f;						\
	CNTR_ENC_DEC_SMALL(KEYS, DST, SRC, LEN, TMP1, CTR_BLOCKx, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, TMP0, TMP2, MASKREG, SHUFREG, NROUNDS, RBITS)	\
	jmp	22f;						\
20:;								\
	and	$0xf, TMP1;					\
	je	16f;						\
	cmp	$15, TMP1;					\
	je	15f;						\
	cmp	$14, TMP1;					\
	je	14f;						\
	cmp	$13, TMP1;					\
	je	13f;						\
	cmp	$12, TMP1;					\
	je	12f;						\
	cmp	$11, TMP1;					\
	je	11f;						\
	cmp	$10, TMP1;					\
	je	10f;						\
	cmp	$9, TMP1;					\
	je	9f;						\
	cmp	$8, TMP1;					\
	je	8f;						\
	cmp	$7, TMP1;					\
	je	7f;						\
	cmp	$6, TMP1;					\
	je	6f;						\
	cmp	$5, TMP1;					\
	je	5f;						\
	cmp	$4, TMP1;					\
	je	4f;						\
	cmp	$3, TMP1;					\
	je	3f;						\
	cmp	$2, TMP1;					\
	je	2f;						\
	jmp	1f;						\
\
	and	$0xf, TMP1;					\
	je	16f;						\
	cmp	$8, TMP1;					\
	je	8f;						\
	jl	18f;						\
	/* Initial blocks 9-15 */				\
	cmp	$12, TMP1;					\
	je	12f;						\
	jl	19f;						\
	/* Initial blocks 13-15 */				\
	cmp	$15, TMP1;					\
	je	15f;						\
	cmp	$14, TMP1;					\
	je	14f;						\
	cmp	$13, TMP1;					\
	je	13f;						\
19:;								\
	cmp	$11, TMP1;					\
	je	11f;						\
	cmp	$10, TMP1;					\
	je	10f;						\
	cmp	$9, TMP1;					\
	je	9f;						\
18:;								\
	cmp	$4, TMP1;					\
	je	4f;						\
	jl	17f;						\
	/* Initial blocks 5-7 */				\
	cmp	$7, TMP1;					\
	je	7f;						\
	cmp	$6, TMP1;					\
	je	6f;						\
	cmp	$5, TMP1;					\
	je	5f;						\
17:;								\
	cmp	$3, TMP1;					\
	je	3f;						\
	cmp	$2, TMP1;					\
	je	2f;						\
	jmp	1f;						\
15:;								\
	INITIAL_BLOCKS(KEYS, DST, SRC, LEN, DATA_OFFSET, 15, CTR_BLOCKx, CTR_BLOCK_1_4, CTR_BLOCK_5_8, CTR_BLOCK_9_12, CTR_BLOCK_13_16, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, TMP0, TMP1, MASKREG, SHUFREG, NROUNDS, RBITS)	\
	jmp	21f;	\
14:;	\
	INITIAL_BLOCKS(KEYS, DST, SRC, LEN, DATA_OFFSET, 14, CTR_BLOCKx, CTR_BLOCK_1_4, CTR_BLOCK_5_8, CTR_BLOCK_9_12, CTR_BLOCK_13_16, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, TMP0, TMP1, MASKREG, SHUFREG, NROUNDS, RBITS)	\
	jmp	21f;	\
13:;	\
	INITIAL_BLOCKS(KEYS, DST, SRC, LEN, DATA_OFFSET, 13, CTR_BLOCKx, CTR_BLOCK_1_4, CTR_BLOCK_5_8, CTR_BLOCK_9_12, CTR_BLOCK_13_16, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, TMP0, TMP1, MASKREG, SHUFREG, NROUNDS, RBITS)	\
	jmp	21f;	\
12:;	\
	INITIAL_BLOCKS(KEYS, DST, SRC, LEN, DATA_OFFSET, 12, CTR_BLOCKx, CTR_BLOCK_1_4, CTR_BLOCK_5_8, CTR_BLOCK_9_12, CTR_BLOCK_13_16, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, TMP0, TMP1, MASKREG, SHUFREG, NROUNDS, RBITS)	\
	jmp	21f;	\
11:;	\
	INITIAL_BLOCKS(KEYS, DST, SRC, LEN, DATA_OFFSET, 11, CTR_BLOCKx, CTR_BLOCK_1_4, CTR_BLOCK_5_8, CTR_BLOCK_9_12, CTR_BLOCK_13_16, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, TMP0, TMP1, MASKREG, SHUFREG, NROUNDS, RBITS)	\
	jmp	21f;	\
10:;	\
	INITIAL_BLOCKS(KEYS, DST, SRC, LEN, DATA_OFFSET, 10, CTR_BLOCKx, CTR_BLOCK_1_4, CTR_BLOCK_5_8, CTR_BLOCK_9_12, CTR_BLOCK_13_16, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, TMP0, TMP1, MASKREG, SHUFREG, NROUNDS, RBITS)	\
	jmp	21f;	\
9:;	\
	INITIAL_BLOCKS(KEYS, DST, SRC, LEN, DATA_OFFSET, 9, CTR_BLOCKx, CTR_BLOCK_1_4, CTR_BLOCK_5_8, CTR_BLOCK_9_12, CTR_BLOCK_13_16, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, TMP0, TMP1, MASKREG, SHUFREG, NROUNDS, RBITS)	\
	jmp	21f;	\
8:;	\
	INITIAL_BLOCKS(KEYS, DST, SRC, LEN, DATA_OFFSET, 8, CTR_BLOCKx, CTR_BLOCK_1_4, CTR_BLOCK_5_8, CTR_BLOCK_9_12, CTR_BLOCK_13_16, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, TMP0, TMP1, MASKREG, SHUFREG, NROUNDS, RBITS)	\
	jmp	21f;	\
7:;	\
	INITIAL_BLOCKS(KEYS, DST, SRC, LEN, DATA_OFFSET, 7, CTR_BLOCKx, CTR_BLOCK_1_4, CTR_BLOCK_5_8, CTR_BLOCK_9_12, CTR_BLOCK_13_16, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, TMP0, TMP1, MASKREG, SHUFREG, NROUNDS, RBITS)	\
	jmp	21f;	\
6:;	\
	INITIAL_BLOCKS(KEYS, DST, SRC, LEN, DATA_OFFSET, 6, CTR_BLOCKx, CTR_BLOCK_1_4, CTR_BLOCK_5_8, CTR_BLOCK_9_12, CTR_BLOCK_13_16, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, TMP0, TMP1, MASKREG, SHUFREG, NROUNDS, RBITS)	\
	jmp	21f;	\
5:;	\
	INITIAL_BLOCKS(KEYS, DST, SRC, LEN, DATA_OFFSET, 5, CTR_BLOCKx, CTR_BLOCK_1_4, CTR_BLOCK_5_8, CTR_BLOCK_9_12, CTR_BLOCK_13_16, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, TMP0, TMP1, MASKREG, SHUFREG, NROUNDS, RBITS)	\
	jmp	21f;	\
4:;	\
	INITIAL_BLOCKS(KEYS, DST, SRC, LEN, DATA_OFFSET, 4, CTR_BLOCKx, CTR_BLOCK_1_4, CTR_BLOCK_5_8, CTR_BLOCK_9_12, CTR_BLOCK_13_16, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, TMP0, TMP1, MASKREG, SHUFREG, NROUNDS, RBITS)	\
	jmp	21f;	\
3:;	\
	INITIAL_BLOCKS(KEYS, DST, SRC, LEN, DATA_OFFSET, 3, CTR_BLOCKx, CTR_BLOCK_1_4, CTR_BLOCK_5_8, CTR_BLOCK_9_12, CTR_BLOCK_13_16, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, TMP0, TMP1, MASKREG, SHUFREG, NROUNDS, RBITS)	\
	jmp	21f;	\
2:;	\
	INITIAL_BLOCKS(KEYS, DST, SRC, LEN, DATA_OFFSET, 2, CTR_BLOCKx, CTR_BLOCK_1_4, CTR_BLOCK_5_8, CTR_BLOCK_9_12, CTR_BLOCK_13_16, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, TMP0, TMP1, MASKREG, SHUFREG, NROUNDS, RBITS)	\
	jmp	21f;	\
1:;	\
	INITIAL_BLOCKS(KEYS, DST, SRC, LEN, DATA_OFFSET, 1, CTR_BLOCKx, CTR_BLOCK_1_4, CTR_BLOCK_5_8, CTR_BLOCK_9_12, CTR_BLOCK_13_16, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, TMP0, TMP1, MASKREG, SHUFREG, NROUNDS, RBITS)	\
	jmp	21f;	\
16:;	\
	INITIAL_BLOCKS(KEYS, DST, SRC, LEN, DATA_OFFSET, 0, CTR_BLOCKx, CTR_BLOCK_1_4, CTR_BLOCK_5_8, CTR_BLOCK_9_12, CTR_BLOCK_13_16, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, TMP0, TMP1, MASKREG, SHUFREG, NROUNDS, RBITS)	\
21:;	\
	or		LEN, LEN;				\
	je		22f;					\
	vmovdqa64	ddq_add_16(%rip), ADD8REG;		\
	/* Process 15 full blocks plus a partial block */	\
	cmp		$256, LEN;				\
	jl		24f;					\
25:;								\
	ENCRYPT_16_PARALLEL(KEYS, DST, SRC, DATA_OFFSET, CTR_BLOCK_1_4, CTR_BLOCK_5_8, CTR_BLOCK_9_12, CTR_BLOCK_13_16, full, TMP0, TMP1, LEN, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, MASKREG, SHUFREG, ADD8REG, NROUNDS, RBITS)		\
	add	$256, DATA_OFFSET;				\
	sub	$256, LEN;					\
	vextracti32x4 $3, CTR_BLOCK_13_16, CTR_BLOCKx;		\
	UPDATE_IV(CTR_BLOCKx)					\
	cmp	$256, LEN;					\
	jge	25b;						\
26:;								\
	/*							\
	 * Test to see if we need a by 16 with partial block.	\
	 * At this point bytes remaining should be either zero	\
	 * or between 241-255.					\
	 */							\
	or	LEN, LEN;					\
	je	22f;						\
24:;								\
	ENCRYPT_16_PARALLEL(KEYS, DST, SRC, DATA_OFFSET, CTR_BLOCK_1_4, CTR_BLOCK_5_8, CTR_BLOCK_9_12, CTR_BLOCK_13_16, partial, TMP0, TMP1, LEN, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, MASKREG, SHUFREG, ADD8REG, NROUNDS, RBITS)	\
	vextracti32x4 $3, CTR_BLOCK_13_16, CTR_BLOCKx;		\
	UPDATE_IV(CTR_BLOCKx)					\
22:;

#define AES_CNTR_ENC_AVX512_BY16(keys, out, in, len, iv, NROUNDS)	\
	FUNC_SAVE_CTR()							\
	/* call the aes main loop */					\
	CNTR_ENC_DEC(keys, out, in, len, iv, NROUNDS)			\
	FUNC_RESTORE_CTR()							\
	ret;

/* Routine to do AES128/192/256 CTR enc/decrypt "by16"
 * void aes_ctr_enc_128_avx512_by16/ aes_ctr_enc_192_avx512_by16/
 *	aes_ctr_enc_256_avx512_by16/
 *		(void *keys,
 *		 u8 *out,
 *		 const u8 *in,
 *		 unsigned int num_bytes,
 *		 u8 *iv);
 */
SYM_FUNC_START(aes_ctr_enc_128_avx512_by16)
	AES_CNTR_ENC_AVX512_BY16(arg1, arg2, arg3, arg4, arg5, 9)
SYM_FUNC_END(aes_ctr_enc_128_avx512_by16)

SYM_FUNC_START(aes_ctr_enc_192_avx512_by16)
	AES_CNTR_ENC_AVX512_BY16(arg1, arg2, arg3, arg4, arg5, 11)
SYM_FUNC_END(aes_ctr_enc_192_avx512_by16)

SYM_FUNC_START(aes_ctr_enc_256_avx512_by16)
	AES_CNTR_ENC_AVX512_BY16(arg1, arg2, arg3, arg4, arg5, 13)
SYM_FUNC_END(aes_ctr_enc_256_avx512_by16)
