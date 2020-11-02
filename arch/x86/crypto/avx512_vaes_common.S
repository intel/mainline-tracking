/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright © 2020 Intel Corporation.
 *
 * Collection of macros which can be used by any crypto code using vaes and
 * AVX512 optimizations.
 *
 * This work was inspired by Intel IPSEC Cryptographic library.
 * https://github.com/intel/intel-ipsec-mb/aes_common.asm
 *
 * Authors:
 * Tomasz Kantecki <tomasz.kantecki@intel.com>
 * Megha Dey <megha.dey@intel.com>
 */

#include <linux/linkage.h>
#include <asm/inst.h>

#define zmm31y ymm31
#define zmm30y ymm30
#define zmm29y ymm29
#define zmm28y ymm28
#define zmm27y ymm27
#define zmm26y ymm26
#define zmm25y ymm25
#define zmm24y ymm24
#define zmm23y ymm23
#define zmm22y ymm22
#define zmm21y ymm21
#define zmm20y ymm20
#define zmm19y ymm19
#define zmm18y ymm18
#define zmm17y ymm17
#define zmm16y ymm16
#define zmm15y ymm15
#define zmm13y ymm13
#define zmm12y ymm12
#define zmm11y ymm11
#define zmm10y ymm10
#define zmm9y  ymm9
#define zmm8y  ymm8
#define zmm7y  ymm7
#define zmm6y  ymm6
#define zmm5y  ymm5
#define zmm4y  ymm4
#define zmm3y  ymm3
#define zmm2y  ymm2
#define zmm1y  ymm1
#define zmm0y  ymm0

#define zmm31x xmm31
#define zmm30x xmm30
#define zmm29x xmm29
#define zmm28x xmm28
#define zmm27x xmm27
#define zmm26x xmm26
#define zmm25x xmm25
#define zmm24x xmm24
#define zmm23x xmm23
#define zmm22x xmm22
#define zmm21x xmm21
#define zmm20x xmm20
#define zmm19x xmm19
#define zmm18x xmm18
#define zmm17x xmm17
#define zmm16x xmm16
#define zmm15x xmm15
#define zmm14x xmm14
#define zmm13x xmm13
#define zmm12x xmm12
#define zmm11x xmm11
#define zmm10x xmm10
#define zmm9x  xmm9
#define zmm8x  xmm8
#define zmm7x  xmm7
#define zmm6x  xmm6
#define zmm5x  xmm5
#define zmm4x  xmm4
#define zmm3x  xmm3
#define zmm2x  xmm2
#define zmm1x  xmm1
#define zmm0x xmm0

#define ymm5y ymm5
#define ymm4y ymm4
#define ymm3y ymm3
#define ymm2y ymm2
#define ymm1y ymm1

#define ymm12x xmm12
#define ymm11x xmm11
#define ymm7x xmm7
#define ymm6x xmm6
#define ymm5x xmm5
#define ymm4x xmm4
#define ymm3x xmm3
#define ymm2x xmm2
#define ymm1x xmm1

#define xmm14z zmm14
#define xmm10z zmm10
#define xmm2z zmm2
#define xmm0z zmm0
#define xmm5z zmm5
#define xmm4z zmm4
#define xmm3z zmm3
#define xmm1z zmm1
#define xmm6z zmm6
#define xmm7z zmm7
#define xmm8z zmm8
#define xmm9z zmm9

#define xmm11y ymm11
#define xmm9y ymm9
#define xmm5y ymm5
#define xmm4y ymm4
#define xmm3y ymm3
#define xmm2y ymm2
#define xmm1y ymm1
#define xmm0y ymm0

#define xmm14x xmm14
#define xmm8x xmm8
#define xmm7x xmm7
#define xmm6x xmm6
#define xmm5x xmm5
#define xmm4x xmm4
#define xmm3x xmm3
#define xmm2x xmm2
#define xmm1x xmm1
#define xmm0x xmm0

#define xmm0z  zmm0
#define xmm0y  ymm0
#define xmm0x  xmm0

#define stringify(reg,y)       reg##y
#define str(reg,y)	       stringify(reg,y)
#define concat(reg,y)	       str(reg,y)

#define YWORD(reg)     concat(reg, y)
#define XWORD(reg)     concat(reg, x)
#define ZWORD(reg)     concat(reg, z)
#define DWORD(reg)     concat(reg, d)
#define WORD(reg)      concat(reg, w)
#define BYTE(reg)      concat(reg, b)

#define arg1	%rdi
#define arg2	%rsi
#define arg3	%rdx
#define arg4	%rcx
#define arg5	%r8
#define arg6	%r9

#define STACK_LOCAL_OFFSET	  64
#define LOCAL_STORAGE		  (48*16)	 //space for up to 128 AES blocks
#define STACK_FRAME_SIZE_GHASH	  (STACK_LOCAL_OFFSET + LOCAL_STORAGE)

#define HashKey_48	(16*0)
#define HashKey_47	(16*1)
#define HashKey_46	(16*2)
#define HashKey_45	(16*3)
#define HashKey_44	(16*4)
#define HashKey_43	(16*5)
#define HashKey_42	(16*6)
#define HashKey_41	(16*7)
#define HashKey_40	(16*8)
#define HashKey_39	(16*9)
#define HashKey_38	(16*10)
#define HashKey_37	(16*11)
#define HashKey_36	(16*12)
#define HashKey_35	(16*13)
#define HashKey_34	(16*14)
#define HashKey_33	(16*15)
#define HashKey_32	(16*16)
#define HashKey_31	(16*17)
#define HashKey_30	(16*18)
#define HashKey_29	(16*19)
#define HashKey_28	(16*20)
#define HashKey_27	(16*21)
#define HashKey_26	(16*22)
#define HashKey_25	(16*23)
#define HashKey_24	(16*24)
#define HashKey_23	(16*25)
#define HashKey_22     (16*26)
#define HashKey_21     (16*27)
#define HashKey_20     (16*28)
#define HashKey_19     (16*29)
#define HashKey_18     (16*30)
#define HashKey_17     (16*31)
#define HashKey_16	(16*32)
#define HashKey_15	(16*33)
#define HashKey_14	(16*34)
#define HashKey_13	(16*35)
#define HashKey_12	(16*36)
#define HashKey_11	(16*37)
#define HashKey_10	(16*38)
#define HashKey_9      (16*39)
#define HashKey_8      (16*40)
#define HashKey_7      (16*41)
#define HashKey_6      (16*42)
#define HashKey_5      (16*43)
#define HashKey_4      (16*44)
#define HashKey_3      (16*45)
#define HashKey_2      (16*46)
#define HashKey_1      (16*47)
#define HashKey      (16*47)

.data

.align 16
ONE:
.octa	0x00000000000000000000000000000001

.align 16
POLY:
.octa	0xC2000000000000000000000000000001

.align 16
TWOONE:
.octa	0x00000001000000000000000000000001

/*
 * Order of these constants should not change.
 * ALL_F should follow SHIFT_MASK, ZERO should follow ALL_F
 */
.align 16
SHIFT_MASK:
.octa	0x0f0e0d0c0b0a09080706050403020100

ALL_F:
.octa	0xffffffffffffffffffffffffffffffff

ZERO:
.octa	0x00000000000000000000000000000000

.align 16
ONEf:
.octa	0x01000000000000000000000000000000

.align 64
SHUF_MASK:
.octa	0x000102030405060708090A0B0C0D0E0F
.octa	0x000102030405060708090A0B0C0D0E0F
.octa	0x000102030405060708090A0B0C0D0E0F
.octa	0x000102030405060708090A0B0C0D0E0F

.align 64
byte_len_to_mask_table:
.quad	0x0007000300010000
.quad	0x007f003f001f000f
.quad	0x07ff03ff01ff00ff
.quad	0x7fff3fff1fff0fff
.quad	0xffff

.align 64
byte64_len_to_mask_table:
.octa	0x00000000000000010000000000000000
.octa	0x00000000000000070000000000000003
.octa	0x000000000000001f000000000000000f
.octa	0x000000000000007f000000000000003f
.octa	0x00000000000001ff00000000000000ff
.octa	0x00000000000007ff00000000000003ff
.octa	0x0000000000001fff0000000000000fff
.octa	0x0000000000007fff0000000000003fff
.octa	0x000000000001ffff000000000000ffff
.octa	0x000000000007ffff000000000003ffff
.octa	0x00000000001fffff00000000000fffff
.octa	0x00000000007fffff00000000003fffff
.octa	0x0000000001ffffff0000000000ffffff
.octa	0x0000000007ffffff0000000003ffffff
.octa	0x000000001fffffff000000000fffffff
.octa	0x000000007fffffff000000003fffffff
.octa	0x00000001ffffffff00000000ffffffff
.octa	0x00000007ffffffff00000003ffffffff
.octa	0x0000001fffffffff0000000fffffffff
.octa	0x0000007fffffffff0000003fffffffff
.octa	0x000001ffffffffff000000ffffffffff
.octa	0x000007ffffffffff000003ffffffffff
.octa	0x00001fffffffffff00000fffffffffff
.octa	0x00007fffffffffff00003fffffffffff
.octa	0x0001ffffffffffff0000ffffffffffff
.octa	0x0007ffffffffffff0003ffffffffffff
.octa	0x001fffffffffffff000fffffffffffff
.octa	0x007fffffffffffff003fffffffffffff
.octa	0x01ffffffffffffff00ffffffffffffff
.octa	0x07ffffffffffffff03ffffffffffffff
.octa	0x1fffffffffffffff0fffffffffffffff
.octa	0x7fffffffffffffff3fffffffffffffff
.octa	0xffffffffffffffff

.align 64
mask_out_top_block:
.octa	0xffffffffffffffffffffffffffffffff
.octa	0xffffffffffffffffffffffffffffffff
.octa	0xffffffffffffffffffffffffffffffff
.octa	0x00000000000000000000000000000000

.align 64
ddq_add_1234:
.octa	0x00000000000000000000000000000001
.octa	0x00000000000000000000000000000002
.octa	0x00000000000000000000000000000003
.octa	0x00000000000000000000000000000004

.align 64
ddq_add_5678:
.octa	0x00000000000000000000000000000005
.octa	0x00000000000000000000000000000006
.octa	0x00000000000000000000000000000007
.octa	0x00000000000000000000000000000008

.align 64
ddq_add_4444:
.octa	0x00000000000000000000000000000004
.octa	0x00000000000000000000000000000004
.octa	0x00000000000000000000000000000004
.octa	0x00000000000000000000000000000004

.align 64
ddq_add_8888:
.octa	0x00000000000000000000000000000008
.octa	0x00000000000000000000000000000008
.octa	0x00000000000000000000000000000008
.octa	0x00000000000000000000000000000008

.align 64
ddq_addbe_1234:
.octa	0x01000000000000000000000000000000
.octa	0x02000000000000000000000000000000
.octa	0x03000000000000000000000000000000
.octa	0x04000000000000000000000000000000

.align 64
ddq_addbe_4444:
.octa	0x04000000000000000000000000000000
.octa	0x04000000000000000000000000000000
.octa	0x04000000000000000000000000000000
.octa	0x04000000000000000000000000000000

.align 64
ddq_addbe_8888:
.octa	0x08000000000000000000000000000000
.octa	0x08000000000000000000000000000000
.octa	0x08000000000000000000000000000000
.octa	0x08000000000000000000000000000000

.align 64
POLY2:
.octa	0xC20000000000000000000001C2000000
.octa	0xC20000000000000000000001C2000000
.octa	0xC20000000000000000000001C2000000
.octa	0xC20000000000000000000001C2000000

.align 16
byteswap_const:
.octa	0x000102030405060708090A0B0C0D0E0F

.text

/* Save register content for the caller */
#define FUNC_SAVE_GHASH()			\
	mov	%rsp, %rax;		\
	sub	$STACK_FRAME_SIZE_GHASH, %rsp;\
	and	$~63, %rsp;		\
	mov	%r12, 0*8(%rsp);	\
	mov	%r13, 1*8(%rsp);	\
	mov	%r14, 2*8(%rsp);	\
	mov	%r15, 3*8(%rsp);	\
	mov	%rax, 4*8(%rsp);	\
	mov	%rax, 4*8(%rsp);	\
	mov	%rax, %r14;		\
	mov	%rbp, 5*8(%rsp);	\
	mov	%rbx, 6*8(%rsp);	\

/* Restore register content for the caller */
#define FUNC_RESTORE_GHASH()		  \
	mov	5*8(%rsp), %rbp;	\
	mov	6*8(%rsp), %rbx;	\
	mov	0*8(%rsp), %r12;	\
	mov	1*8(%rsp), %r13;	\
	mov	2*8(%rsp), %r14;	\
	mov	3*8(%rsp), %r15;	\
	mov	4*8(%rsp), %rsp;	\

/*
 * GHASH school book multiplication
 */
#define GHASH_MUL(GH, HK, T1, T2, T3, T4, T5)			\
	vpclmulqdq	$0x11, HK, GH, T1;			\
	vpclmulqdq	$0x00, HK, GH, T2;			\
	vpclmulqdq	$0x01, HK, GH, T3;			\
	vpclmulqdq	$0x10, HK, GH, GH;			\
	vpxorq		T3, GH, GH;				\
	vpsrldq		$8, GH, T3;				\
	vpslldq		$8, GH, GH;				\
	vpxorq		T3, T1, T1;				\
	vpxorq		T2, GH, GH;				\
	vmovdqu64	POLY2(%rip), T3;			\
	vpclmulqdq	$0x01, GH, T3, T2;			\
	vpslldq		$8, T2, T2;				\
	vpxorq		T2, GH, GH;				\
	vpclmulqdq	$0x00, GH, T3, T2;			\
	vpsrldq		$4, T2, T2;				\
	vpclmulqdq	$0x10, GH, T3, GH;			\
	vpslldq		$4, GH, GH;				\
	vpternlogq	$0x96, T2, T1, GH;

/*
 * Precomputation of hash keys. These precomputated keys
 * are saved in memory and reused for as many 8 blocks sets
 * as necessary.
 */
#define PRECOMPUTE(GDATA, HK, T1, T2, T3, T4, T5, T6, T7, T8) \
\
	vmovdqa64	HK, T5; \
	vinserti64x2	$3, HK, ZWORD(T7), ZWORD(T7); \
	GHASH_MUL(T5, HK, T1, T3, T4, T6, T2) \
	vmovdqu64	T5, HashKey_2(GDATA); \
	vinserti64x2	$2, T5, ZWORD(T7), ZWORD(T7); \
	GHASH_MUL(T5, HK, T1, T3, T4, T6, T2) \
	vmovdqu64	T5, HashKey_3(GDATA); \
	vinserti64x2	$1, T5, ZWORD(T7), ZWORD(T7); \
	GHASH_MUL(T5, HK, T1, T3, T4, T6, T2) \
	vmovdqu64	T5, HashKey_4(GDATA); \
	vinserti64x2	$0, T5, ZWORD(T7), ZWORD(T7); \
	vshufi64x2	$0x00, ZWORD(T5), ZWORD(T5), ZWORD(T5); \
	vmovdqa64	ZWORD(T7), ZWORD(T8); \
	GHASH_MUL(ZWORD(T7), ZWORD(T5), ZWORD(T1), ZWORD(T3), ZWORD(T4), ZWORD(T6), ZWORD(T2)) \
	vmovdqu64	ZWORD(T7), HashKey_8(GDATA); \
	vshufi64x2	$0x00, ZWORD(T7), ZWORD(T7), ZWORD(T5); \
	GHASH_MUL(ZWORD(T8), ZWORD(T5), ZWORD(T1), ZWORD(T3), ZWORD(T4), ZWORD(T6), ZWORD(T2)) \
	vmovdqu64 ZWORD(T8), HashKey_12(GDATA); \
	GHASH_MUL(ZWORD(T7), ZWORD(T5), ZWORD(T1), ZWORD(T3), ZWORD(T4), ZWORD(T6), ZWORD(T2)) \
	vmovdqu64 ZWORD(T7), HashKey_16(GDATA); \
	GHASH_MUL(ZWORD(T8), ZWORD(T5), ZWORD(T1), ZWORD(T3), ZWORD(T4), ZWORD(T6), ZWORD(T2)) \
	vmovdqu64 ZWORD(T8), HashKey_20(GDATA); \
	GHASH_MUL(ZWORD(T7), ZWORD(T5), ZWORD(T1), ZWORD(T3), ZWORD(T4), ZWORD(T6), ZWORD(T2)) \
	vmovdqu64 ZWORD(T7), HashKey_24(GDATA); \
	GHASH_MUL(ZWORD(T8), ZWORD(T5), ZWORD(T1), ZWORD(T3), ZWORD(T4), ZWORD(T6), ZWORD(T2)) \
	vmovdqu64 ZWORD(T8), HashKey_28(GDATA); \
	GHASH_MUL(ZWORD(T7), ZWORD(T5), ZWORD(T1), ZWORD(T3), ZWORD(T4), ZWORD(T6), ZWORD(T2)) \
	vmovdqu64 ZWORD(T7), HashKey_32(GDATA); \
	GHASH_MUL(ZWORD(T8), ZWORD(T5), ZWORD(T1), ZWORD(T3), ZWORD(T4), ZWORD(T6), ZWORD(T2)) \
	vmovdqu64 ZWORD(T8), HashKey_36(GDATA); \
	GHASH_MUL(ZWORD(T7), ZWORD(T5), ZWORD(T1), ZWORD(T3), ZWORD(T4), ZWORD(T6), ZWORD(T2)) \
	vmovdqu64 ZWORD(T7), HashKey_40(GDATA); \
	GHASH_MUL(ZWORD(T8), ZWORD(T5), ZWORD(T1), ZWORD(T3), ZWORD(T4), ZWORD(T6), ZWORD(T2)) \
	vmovdqu64 ZWORD(T8), HashKey_44(GDATA); \
	GHASH_MUL(ZWORD(T7), ZWORD(T5), ZWORD(T1), ZWORD(T3), ZWORD(T4), ZWORD(T6), ZWORD(T2)) \
	vmovdqu64 ZWORD(T7), HashKey_48(GDATA);

#define VHPXORI4x128(REG,TMP)					\
	vextracti64x4	$1, REG, YWORD(TMP);			\
	vpxorq		YWORD(TMP), YWORD(REG), YWORD(REG);	\
	vextracti32x4	$1, YWORD(REG), XWORD(TMP);		\
	vpxorq		XWORD(TMP), XWORD(REG), XWORD(REG);

#define VCLMUL_REDUCE(OUT, POLY, HI128, LO128, TMP0, TMP1)	\
	vpclmulqdq	$0x01, LO128, POLY, TMP0;		\
	vpslldq		$8, TMP0, TMP0;				\
	vpxorq		TMP0, LO128, TMP0;			\
	vpclmulqdq	$0x00, TMP0, POLY, TMP1;		\
	vpsrldq		$4, TMP1, TMP1;				\
	vpclmulqdq	$0x10, TMP0, POLY, OUT;			\
	vpslldq		$4, OUT, OUT;				\
	vpternlogq	$0x96, HI128, TMP1, OUT;

/*
 * GHASH 1 to 16 blocks of the input buffer.
 *  - It performs reduction at the end.
 *  - It can take intermediate GHASH sums as input.
 */
#define GHASH_1_TO_16(KP, OFFSET, GHASH, T1, T2, T3, T4, T5, T6, T7, T8, T9, AAD_HASH_IN, CIPHER_IN0, CIPHER_IN1, CIPHER_IN2, CIPHER_IN3, NUM_BLOCKS, BOOL, INSTANCE_TYPE, ROUND, HKEY_START, PREV_H, PREV_L, PREV_M1, PREV_M2) \
.set	reg_idx, 0;	\
.set	blocks_left, NUM_BLOCKS;	\
.ifc INSTANCE_TYPE, single_call; \
	.if BOOL == 1; \
	.set	hashk, concat(HashKey_, NUM_BLOCKS);	\
	.else; \
	.set	hashk, concat(HashKey_, NUM_BLOCKS) + 0x11;	 \
	.endif; \
	.set	first_result, 1; \
	.set	reduce, 1; \
	vpxorq		AAD_HASH_IN, CIPHER_IN0, CIPHER_IN0; \
.else;	\
	.set	hashk, concat(HashKey_, HKEY_START);	\
	.ifc ROUND, first; \
		.set first_result, 1; \
		.set reduce, 0; \
		vpxorq		AAD_HASH_IN, CIPHER_IN0, CIPHER_IN0; \
	.else; \
		.ifc ROUND, mid; \
		    .set first_result, 0; \
		    .set reduce, 0; \
		    vmovdqa64	    PREV_H, T1; \
		    vmovdqa64	    PREV_L, T2; \
		    vmovdqa64	    PREV_M1, T3; \
		    vmovdqa64	    PREV_M2, T4; \
		.else; \
		    .set first_result, 0; \
		    .set reduce, 1; \
		    vmovdqa64	    PREV_H, T1; \
		    vmovdqa64	    PREV_L, T2; \
		    vmovdqa64	    PREV_M1, T3; \
		    vmovdqa64	    PREV_M2, T4; \
		.endif; \
	.endif; \
.endif; \
.if NUM_BLOCKS < 4;	\
	.if blocks_left == 1; \
		.if first_result == 1;	\
			vmovdqu64	hashk + OFFSET(KP), XWORD(T9); \
			vpclmulqdq	$0x11, XWORD(T9), XWORD(CIPHER_IN0), XWORD(T1); \
			vpclmulqdq	$0x00, XWORD(T9), XWORD(CIPHER_IN0), XWORD(T2); \
			vpclmulqdq	$0x01, XWORD(T9), XWORD(CIPHER_IN0), XWORD(T3); \
			vpclmulqdq	$0x10, XWORD(T9), XWORD(CIPHER_IN0), XWORD(T4); \
		.else;	\
			vmovdqu64	hashk + OFFSET(KP), XWORD(T9); \
			vpclmulqdq	$0x11, XWORD(T9), XWORD(CIPHER_IN0), XWORD(T5); \
			vpclmulqdq	$0x00, XWORD(T9), XWORD(CIPHER_IN0), XWORD(T6); \
			vpclmulqdq	$0x01, XWORD(T9), XWORD(CIPHER_IN0), XWORD(T7); \
			vpclmulqdq	$0x10, XWORD(T9), XWORD(CIPHER_IN0), XWORD(T8); \
		.endif; \
	.elseif blocks_left == 2; \
		.if first_result == 1;	\
			vmovdqu64	hashk + OFFSET(KP), YWORD(T9); \
			vpclmulqdq	$0x11, YWORD(T9), YWORD(CIPHER_IN0), YWORD(T1); \
			vpclmulqdq	$0x00, YWORD(T9), YWORD(CIPHER_IN0), YWORD(T2); \
			vpclmulqdq	$0x01, YWORD(T9), YWORD(CIPHER_IN0), YWORD(T3); \
			vpclmulqdq	$0x10, YWORD(T9), YWORD(CIPHER_IN0), YWORD(T4); \
		.else;	\
			vmovdqu64	hashk + OFFSET(KP), YWORD(T9); \
			vpclmulqdq	$0x11, YWORD(T9), YWORD(CIPHER_IN0), YWORD(T5); \
			vpclmulqdq	$0x00, YWORD(T9), YWORD(CIPHER_IN0), YWORD(T6); \
			vpclmulqdq	$0x01, YWORD(T9), YWORD(CIPHER_IN0), YWORD(T7); \
			vpclmulqdq	$0x10, YWORD(T9), YWORD(CIPHER_IN0), YWORD(T8); \
		.endif; \
	.elseif blocks_left == 3;	\
		.if first_result == 1;	\
			vmovdqu64	hashk + OFFSET(KP), YWORD(T9); \
			vinserti64x2	$2, 32 + hashk + OFFSET(KP), T9, T9; \
			vpclmulqdq	$0x11, T9, CIPHER_IN0, T1; \
			vpclmulqdq	$0x00, T9, CIPHER_IN0, T2; \
			vpclmulqdq	$0x01, T9, CIPHER_IN0, T3; \
			vpclmulqdq	$0x10, T9, CIPHER_IN0, T4; \
		.else;	\
			vmovdqu64	hashk + OFFSET(KP), YWORD(T9); \
			vinserti64x2	$2, 32 + hashk + OFFSET(KP), T9, T9; \
			vpclmulqdq	$0x11, T9, CIPHER_IN0, T5; \
			vpclmulqdq	$0x00, T9, CIPHER_IN0, T6; \
			vpclmulqdq	$0x01, T9, CIPHER_IN0, T7; \
			vpclmulqdq	$0x10, T9, CIPHER_IN0, T8; \
		.endif; \
	.endif; \
	.if first_result != 1; \
		 vpxorq		 T5, T1, T1; \
		 vpxorq		 T6, T2, T2; \
		 vpxorq		 T7, T3, T3; \
		 vpxorq		 T8, T4, T4; \
	.endif; \
.elseif (NUM_BLOCKS >= 4) && (NUM_BLOCKS < 8); \
	vmovdqu64	hashk + OFFSET(KP), T9; \
	.if first_result == 1; \
		vpclmulqdq	$0x11, T9, CIPHER_IN0, T1; \
		vpclmulqdq	$0x00, T9, CIPHER_IN0, T2; \
		vpclmulqdq	$0x01, T9, CIPHER_IN0, T3; \
		vpclmulqdq	$0x10, T9, CIPHER_IN0, T4; \
		.set first_result, 0; \
	.else; \
		vpclmulqdq	$0x11, T9, CIPHER_IN0, T5; \
		vpclmulqdq	$0x00, T9, CIPHER_IN0, T6; \
		vpclmulqdq	$0x01, T9, CIPHER_IN0, T7; \
		vpclmulqdq	$0x10, T9, CIPHER_IN0, T8; \
		vpxorq		T5, T1, T1; \
		vpxorq		T6, T2, T2; \
		vpxorq		T7, T3, T3; \
		vpxorq		T8, T4, T4; \
	.endif; \
	.set hashk, hashk + 64; \
	.set blocks_left, blocks_left - 4;	\
	.set reg_idx, reg_idx + 1;	\
	.if blocks_left > 0;	\
	.if blocks_left == 1; \
		.if first_result == 1;	\
			vmovdqu64	hashk + OFFSET(KP), XWORD(T9); \
			vpclmulqdq	$0x11, XWORD(T9), XWORD(CIPHER_IN1), XWORD(T1); \
			vpclmulqdq	$0x00, XWORD(T9), XWORD(CIPHER_IN1), XWORD(T2); \
			vpclmulqdq	$0x01, XWORD(T9), XWORD(CIPHER_IN1), XWORD(T3); \
			vpclmulqdq	$0x10, XWORD(T9), XWORD(CIPHER_IN1), XWORD(T4); \
		.else;	\
			vmovdqu64	hashk + OFFSET(KP), XWORD(T9); \
			vpclmulqdq	$0x11, XWORD(T9), XWORD(CIPHER_IN1), XWORD(T5); \
			vpclmulqdq	$0x00, XWORD(T9), XWORD(CIPHER_IN1), XWORD(T6); \
			vpclmulqdq	$0x01, XWORD(T9), XWORD(CIPHER_IN1), XWORD(T7); \
			vpclmulqdq	$0x10, XWORD(T9), XWORD(CIPHER_IN1), XWORD(T8); \
		.endif; \
	.elseif blocks_left == 2; \
		.if first_result == 1;	\
			vmovdqu64	hashk + OFFSET(KP), YWORD(T9); \
			vpclmulqdq	$0x11, YWORD(T9), YWORD(CIPHER_IN1), YWORD(T1); \
			vpclmulqdq	$0x00, YWORD(T9), YWORD(CIPHER_IN1), YWORD(T2); \
			vpclmulqdq	$0x01, YWORD(T9), YWORD(CIPHER_IN1), YWORD(T3); \
			vpclmulqdq	$0x10, YWORD(T9), YWORD(CIPHER_IN1), YWORD(T4); \
		.else;	\
			vmovdqu64	hashk + OFFSET(KP), YWORD(T9); \
			vpclmulqdq	$0x11, YWORD(T9), YWORD(CIPHER_IN1), YWORD(T5); \
			vpclmulqdq	$0x00, YWORD(T9), YWORD(CIPHER_IN1), YWORD(T6); \
			vpclmulqdq	$0x01, YWORD(T9), YWORD(CIPHER_IN1), YWORD(T7); \
			vpclmulqdq	$0x10, YWORD(T9), YWORD(CIPHER_IN1), YWORD(T8); \
		.endif; \
	.elseif blocks_left == 3;  \
		.if first_result == 1;	\
			vmovdqu64	hashk + OFFSET(KP), YWORD(T9); \
			vinserti64x2	$2, 32 + hashk + OFFSET(KP), T9, T9; \
			vpclmulqdq	$0x11, T9, CIPHER_IN1, T1; \
			vpclmulqdq	$0x00, T9, CIPHER_IN1, T2; \
			vpclmulqdq	$0x01, T9, CIPHER_IN1, T3; \
			vpclmulqdq	$0x10, T9, CIPHER_IN1, T4; \
		.else;	\
			vmovdqu64	hashk + OFFSET(KP), YWORD(T9); \
			vinserti64x2	$2, 32 + hashk + OFFSET(KP), T9, T9; \
			vpclmulqdq	$0x11, T9, CIPHER_IN1, T5; \
			vpclmulqdq	$0x00, T9, CIPHER_IN1, T6; \
			vpclmulqdq	$0x01, T9, CIPHER_IN1, T7; \
			vpclmulqdq	$0x10, T9, CIPHER_IN1, T8; \
		.endif; \
	.endif; \
	.if first_result != 1; \
			vpxorq		T5, T1, T1; \
			vpxorq		T6, T2, T2; \
			vpxorq		T7, T3, T3; \
			vpxorq		T8, T4, T4; \
	.endif; \
	.endif; \
.elseif (NUM_BLOCKS >= 8) && (NUM_BLOCKS < 12); \
	vmovdqu64	hashk + OFFSET(KP), T9; \
	.if first_result == 1; \
		vpclmulqdq	$0x11, T9, CIPHER_IN0, T1; \
		vpclmulqdq	$0x00, T9, CIPHER_IN0, T2; \
		vpclmulqdq	$0x01, T9, CIPHER_IN0, T3; \
		vpclmulqdq	$0x10, T9, CIPHER_IN0, T4; \
		.set first_result, 0; \
	.else; \
		vpclmulqdq	$0x11, T9, CIPHER_IN0, T5; \
		vpclmulqdq	$0x00, T9, CIPHER_IN0, T6; \
		vpclmulqdq	$0x01, T9, CIPHER_IN0, T7; \
		vpclmulqdq	$0x10, T9, CIPHER_IN0, T8; \
		vpxorq		T5, T1, T1; \
		vpxorq		T6, T2, T2; \
		vpxorq		T7, T3, T3; \
		vpxorq		T8, T4, T4; \
	.endif; \
	.set hashk, hashk + 64; \
	.set blocks_left, blocks_left - 4; \
	.set reg_idx, reg_idx + 1;	\
	vmovdqu64	hashk + OFFSET(KP), T9; \
	.if first_result == 1; \
		vpclmulqdq	$0x11, T9, CIPHER_IN1, T1; \
		vpclmulqdq	$0x00, T9, CIPHER_IN1, T2; \
		vpclmulqdq	$0x01, T9, CIPHER_IN1, T3; \
		vpclmulqdq	$0x10, T9, CIPHER_IN1, T4; \
		.set first_result, 0; \
	.else; \
		vpclmulqdq	$0x11, T9, CIPHER_IN1, T5; \
		vpclmulqdq	$0x00, T9, CIPHER_IN1, T6; \
		vpclmulqdq	$0x01, T9, CIPHER_IN1, T7; \
		vpclmulqdq	$0x10, T9, CIPHER_IN1, T8; \
		vpxorq		T5, T1, T1; \
		vpxorq		T6, T2, T2; \
		vpxorq		T7, T3, T3; \
		vpxorq		T8, T4, T4; \
	.endif; \
	.set hashk, hashk + 64; \
	.set blocks_left, blocks_left - 4; \
	.set reg_idx, reg_idx + 1;	\
	.if blocks_left > 0; \
	.if blocks_left == 1; \
		.if first_result == 1;	\
			vmovdqu64	hashk + OFFSET(KP), XWORD(T9); \
			vpclmulqdq	$0x11, XWORD(T9), XWORD(CIPHER_IN2), XWORD(T1); \
			vpclmulqdq	$0x00, XWORD(T9), XWORD(CIPHER_IN2), XWORD(T2); \
			vpclmulqdq	$0x01, XWORD(T9), XWORD(CIPHER_IN2), XWORD(T3); \
			vpclmulqdq	$0x10, XWORD(T9), XWORD(CIPHER_IN2), XWORD(T4); \
		.else;	\
			vmovdqu64	hashk + OFFSET(KP), XWORD(T9); \
			vpclmulqdq	$0x11, XWORD(T9), XWORD(CIPHER_IN2), XWORD(T5); \
			vpclmulqdq	$0x00, XWORD(T9), XWORD(CIPHER_IN2), XWORD(T6); \
			vpclmulqdq	$0x01, XWORD(T9), XWORD(CIPHER_IN2), XWORD(T7); \
			vpclmulqdq	$0x10, XWORD(T9), XWORD(CIPHER_IN2), XWORD(T8); \
		.endif; \
	.elseif blocks_left == 2; \
		.if first_result == 1;	\
			vmovdqu64	hashk + OFFSET(KP), YWORD(T9); \
			vpclmulqdq	$0x11, YWORD(T9), YWORD(CIPHER_IN2), YWORD(T1); \
			vpclmulqdq	$0x00, YWORD(T9), YWORD(CIPHER_IN2), YWORD(T2); \
			vpclmulqdq	$0x01, YWORD(T9), YWORD(CIPHER_IN2), YWORD(T3); \
			vpclmulqdq	$0x10, YWORD(T9), YWORD(CIPHER_IN2), YWORD(T4); \
		.else;	\
			vmovdqu64	hashk + OFFSET(KP), YWORD(T9); \
			vpclmulqdq	$0x11, YWORD(T9), YWORD(CIPHER_IN2), YWORD(T5); \
			vpclmulqdq	$0x00, YWORD(T9), YWORD(CIPHER_IN2), YWORD(T6); \
			vpclmulqdq	$0x01, YWORD(T9), YWORD(CIPHER_IN2), YWORD(T7); \
			vpclmulqdq	$0x10, YWORD(T9), YWORD(CIPHER_IN2), YWORD(T8); \
		.endif; \
	.elseif blocks_left == 3;  \
		.if first_result == 1;	\
			vmovdqu64	hashk + OFFSET(KP), YWORD(T9); \
			vinserti64x2	$2, 32 + hashk + OFFSET(KP), T9, T9; \
			vpclmulqdq	$0x11, T9, CIPHER_IN2, T1; \
			vpclmulqdq	$0x00, T9, CIPHER_IN2, T2; \
			vpclmulqdq	$0x01, T9, CIPHER_IN2, T3; \
			vpclmulqdq	$0x10, T9, CIPHER_IN2, T4; \
		.else;	\
			vmovdqu64	hashk + OFFSET(KP), YWORD(T9); \
			vinserti64x2	$2, 32 + hashk + OFFSET(KP), T9, T9; \
			vpclmulqdq	$0x11, T9, CIPHER_IN2, T5; \
			vpclmulqdq	$0x00, T9, CIPHER_IN2, T6; \
			vpclmulqdq	$0x01, T9, CIPHER_IN2, T7; \
			vpclmulqdq	$0x10, T9, CIPHER_IN2, T8; \
		.endif; \
	.endif; \
	.if first_result != 1; \
		vpxorq		T5, T1, T1; \
		vpxorq		T6, T2, T2; \
		vpxorq		T7, T3, T3; \
		vpxorq		T8, T4, T4; \
	.endif; \
	.endif; \
.elseif (NUM_BLOCKS >= 12) && (NUM_BLOCKS < 16); \
	vmovdqu64	hashk + OFFSET(KP), T9; \
	.if first_result == 1; \
		vpclmulqdq	$0x11, T9, CIPHER_IN0, T1; \
		vpclmulqdq	$0x00, T9, CIPHER_IN0, T2; \
		vpclmulqdq	$0x01, T9, CIPHER_IN0, T3; \
		vpclmulqdq	$0x10, T9, CIPHER_IN0, T4; \
		first_result = 0; \
	.else; \
		vpclmulqdq	$0x11, T9, CIPHER_IN0, T5; \
		vpclmulqdq	$0x00, T9, CIPHER_IN0, T6; \
		vpclmulqdq	$0x01, T9, CIPHER_IN0, T7; \
		vpclmulqdq	$0x10, T9, CIPHER_IN0, T8; \
		vpxorq		T5, T1, T1; \
		vpxorq		T6, T2, T2; \
		vpxorq		T7, T3, T3; \
		vpxorq		T8, T4, T4; \
	.endif; \
	.set hashk, hashk + 64; \
	.set blocks_left, blocks_left - 4; \
	.set reg_idx, reg_idx + 1;	\
	vmovdqu64	hashk + OFFSET(KP), T9; \
	.if first_result == 1; \
		vpclmulqdq	$0x11, T9, CIPHER_IN1, T1; \
		vpclmulqdq	$0x00, T9, CIPHER_IN1, T2; \
		vpclmulqdq	$0x01, T9, CIPHER_IN1, T3; \
		vpclmulqdq	$0x10, T9, CIPHER_IN1, T4; \
		first_result = 0; \
	.else; \
		vpclmulqdq	$0x11, T9, CIPHER_IN1, T5; \
		vpclmulqdq	$0x00, T9, CIPHER_IN1, T6; \
		vpclmulqdq	$0x01, T9, CIPHER_IN1, T7; \
		vpclmulqdq	$0x10, T9, CIPHER_IN1, T8; \
		vpxorq		T5, T1, T1; \
		vpxorq		T6, T2, T2; \
		vpxorq		T7, T3, T3; \
		vpxorq		T8, T4, T4; \
	.endif; \
	.set hashk, hashk + 64; \
	.set blocks_left, blocks_left - 4; \
	.set reg_idx, reg_idx + 1;	\
	vmovdqu64	hashk + OFFSET(KP), T9; \
	.if first_result == 1; \
		vpclmulqdq	$0x11, T9, CIPHER_IN2, T1; \
		vpclmulqdq	$0x00, T9, CIPHER_IN2, T2; \
		vpclmulqdq	$0x01, T9, CIPHER_IN2, T3; \
		vpclmulqdq	$0x10, T9, CIPHER_IN2, T4; \
		first_result = 0; \
	.else; \
		vpclmulqdq	$0x11, T9, CIPHER_IN2, T5; \
		vpclmulqdq	$0x00, T9, CIPHER_IN2, T6; \
		vpclmulqdq	$0x01, T9, CIPHER_IN2, T7; \
		vpclmulqdq	$0x10, T9, CIPHER_IN2, T8; \
		vpxorq		T5, T1, T1; \
		vpxorq		T6, T2, T2; \
		vpxorq		T7, T3, T3; \
		vpxorq		T8, T4, T4; \
	.endif; \
	.set hashk, hashk + 64; \
	.set blocks_left, blocks_left - 4; \
	.set reg_idx, reg_idx + 1;	\
	.if blocks_left > 0;	\
	.if blocks_left == 1; \
		.if first_result == 1;	\
			vmovdqu64	hashk + OFFSET(KP), XWORD(T9); \
			vpclmulqdq	$0x11, XWORD(T9), XWORD(CIPHER_IN3), XWORD(T1); \
			vpclmulqdq	$0x00, XWORD(T9), XWORD(CIPHER_IN3), XWORD(T2); \
			vpclmulqdq	$0x01, XWORD(T9), XWORD(CIPHER_IN3), XWORD(T3); \
			vpclmulqdq	$0x10, XWORD(T9), XWORD(CIPHER_IN3), XWORD(T4); \
		.else;	\
			vmovdqu64	hashk + OFFSET(KP), XWORD(T9); \
			vpclmulqdq	$0x11, XWORD(T9), XWORD(CIPHER_IN3), XWORD(T5); \
			vpclmulqdq	$0x00, XWORD(T9), XWORD(CIPHER_IN3), XWORD(T6); \
			vpclmulqdq	$0x01, XWORD(T9), XWORD(CIPHER_IN3), XWORD(T7); \
			vpclmulqdq	$0x10, XWORD(T9), XWORD(CIPHER_IN3), XWORD(T8); \
		.endif; \
	.elseif blocks_left == 2; \
		.if first_result == 1;	\
			vmovdqu64	hashk + OFFSET(KP), YWORD(T9); \
			vpclmulqdq	$0x11, YWORD(T9), YWORD(CIPHER_IN3), YWORD(T1); \
			vpclmulqdq	$0x00, YWORD(T9), YWORD(CIPHER_IN3), YWORD(T2); \
			vpclmulqdq	$0x01, YWORD(T9), YWORD(CIPHER_IN3), YWORD(T3); \
			vpclmulqdq	$0x10, YWORD(T9), YWORD(CIPHER_IN3), YWORD(T4); \
		.else;	\
			vmovdqu64	hashk + OFFSET(KP), YWORD(T9); \
			vpclmulqdq	$0x11, YWORD(T9), YWORD(CIPHER_IN3), YWORD(T5); \
			vpclmulqdq	$0x00, YWORD(T9), YWORD(CIPHER_IN3), YWORD(T6); \
			vpclmulqdq	$0x01, YWORD(T9), YWORD(CIPHER_IN3), YWORD(T7); \
			vpclmulqdq	$0x10, YWORD(T9), YWORD(CIPHER_IN3), YWORD(T8); \
		.endif; \
	.elseif blocks_left == 3;  \
		.if first_result == 1;	\
			vmovdqu64	hashk + OFFSET(KP), YWORD(T9); \
			vinserti64x2	$2, 32 + hashk + OFFSET(KP), T9, T9; \
			vpclmulqdq	$0x11, T9, CIPHER_IN3, T1; \
			vpclmulqdq	$0x00, T9, CIPHER_IN3, T2; \
			vpclmulqdq	$0x01, T9, CIPHER_IN3, T3; \
			vpclmulqdq	$0x10, T9, CIPHER_IN3, T4; \
		.else;	\
			vmovdqu64	hashk + OFFSET(KP), YWORD(T9); \
			vinserti64x2	$2, 32 + hashk + OFFSET(KP), T9, T9; \
			vpclmulqdq	$0x11, T9, CIPHER_IN3, T5; \
			vpclmulqdq	$0x00, T9, CIPHER_IN3, T6; \
			vpclmulqdq	$0x01, T9, CIPHER_IN3, T7; \
			vpclmulqdq	$0x10, T9, CIPHER_IN3, T8; \
		.endif; \
	.endif; \
	.if first_result != 1; \
			vpxorq		T5, T1, T1; \
			vpxorq		T6, T2, T2; \
			vpxorq		T7, T3, T3; \
			vpxorq		T8, T4, T4; \
	.endif; \
	.endif; \
.else;	\
	vmovdqu64	hashk + OFFSET(KP), T9; \
	.if first_result == 1; \
		vpclmulqdq	$0x11, T9, CIPHER_IN0, T1; \
		vpclmulqdq	$0x00, T9, CIPHER_IN0, T2; \
		vpclmulqdq	$0x01, T9, CIPHER_IN0, T3; \
		vpclmulqdq	$0x10, T9, CIPHER_IN0, T4; \
		first_result = 0; \
	.else; \
		vpclmulqdq	$0x11, T9, CIPHER_IN0, T5; \
		vpclmulqdq	$0x00, T9, CIPHER_IN0, T6; \
		vpclmulqdq	$0x01, T9, CIPHER_IN0, T7; \
		vpclmulqdq	$0x10, T9, CIPHER_IN0, T8; \
		vpxorq		T5, T1, T1; \
		vpxorq		T6, T2, T2; \
		vpxorq		T7, T3, T3; \
		vpxorq		T8, T4, T4; \
	.endif; \
	.set hashk, hashk + 64; \
	.set blocks_left, blocks_left - 4; \
	.set reg_idx, reg_idx + 1;     \
	vmovdqu64	hashk + OFFSET(KP), T9; \
	.if first_result == 1; \
		vpclmulqdq	$0x11, T9, CIPHER_IN1, T1; \
		vpclmulqdq	$0x00, T9, CIPHER_IN1, T2; \
		vpclmulqdq	$0x01, T9, CIPHER_IN1, T3; \
		vpclmulqdq	$0x10, T9, CIPHER_IN1, T4; \
		first_result = 0; \
	.else; \
		vpclmulqdq	$0x11, T9, CIPHER_IN1, T5; \
		vpclmulqdq	$0x00, T9, CIPHER_IN1, T6; \
		vpclmulqdq	$0x01, T9, CIPHER_IN1, T7; \
		vpclmulqdq	$0x10, T9, CIPHER_IN1, T8; \
		vpxorq		T5, T1, T1; \
		vpxorq		T6, T2, T2; \
		vpxorq		T7, T3, T3; \
		vpxorq		T8, T4, T4; \
	.endif; \
	.set hashk, hashk + 64; \
	.set blocks_left, blocks_left - 4; \
	.set reg_idx, reg_idx + 1;	\
	vmovdqu64	hashk + OFFSET(KP), T9; \
	.if first_result == 1; \
		vpclmulqdq	$0x11, T9, CIPHER_IN2, T1; \
		vpclmulqdq	$0x00, T9, CIPHER_IN2, T2; \
		vpclmulqdq	$0x01, T9, CIPHER_IN2, T3; \
		vpclmulqdq	$0x10, T9, CIPHER_IN2, T4; \
		first_result = 0; \
	.else; \
		vpclmulqdq	$0x11, T9, CIPHER_IN2, T5; \
		vpclmulqdq	$0x00, T9, CIPHER_IN2, T6; \
		vpclmulqdq	$0x01, T9, CIPHER_IN2, T7; \
		vpclmulqdq	$0x10, T9, CIPHER_IN2, T8; \
		vpxorq		T5, T1, T1; \
		vpxorq		T6, T2, T2; \
		vpxorq		T7, T3, T3; \
		vpxorq		T8, T4, T4; \
	.endif; \
	.set hashk, hashk + 64; \
	.set blocks_left, blocks_left - 4; \
	.set reg_idx, reg_idx + 1;	\
	vmovdqu64	hashk + OFFSET(KP), T9; \
	.if first_result == 1; \
		vpclmulqdq	$0x11, T9, CIPHER_IN3, T1; \
		vpclmulqdq	$0x00, T9, CIPHER_IN3, T2; \
		vpclmulqdq	$0x01, T9, CIPHER_IN3, T3; \
		vpclmulqdq	$0x10, T9, CIPHER_IN3, T4; \
		first_result = 0; \
	.else; \
		vpclmulqdq	$0x11, T9, CIPHER_IN3, T5; \
		vpclmulqdq	$0x00, T9, CIPHER_IN3, T6; \
		vpclmulqdq	$0x01, T9, CIPHER_IN3, T7; \
		vpclmulqdq	$0x10, T9, CIPHER_IN3, T8; \
		vpxorq		T5, T1, T1; \
		vpxorq		T6, T2, T2; \
		vpxorq		T7, T3, T3; \
		vpxorq		T8, T4, T4; \
	.endif; \
	.set hashk, hashk + 64; \
	.set blocks_left, blocks_left - 4; \
	.set reg_idx, reg_idx + 1;	\
.endif; \
.if reduce == 1; \
	vpxorq		T4, T3, T3; \
	vpsrldq		$8, T3, T7; \
	vpslldq		$8, T3, T8; \
	vpxorq		T7, T1, T1; \
	vpxorq		T8, T2, T2; \
	VHPXORI4x128(T1, T7); \
	VHPXORI4x128(T2, T8); \
	vmovdqa64	POLY2(%rip), XWORD(T9); \
	VCLMUL_REDUCE(XWORD(GHASH), XWORD(T9), XWORD(T1), XWORD(T2), XWORD(T3), XWORD(T4)) \
.else; \
	vmovdqa64	T1, PREV_H; \
	vmovdqa64	T2, PREV_L; \
	vmovdqa64	T3, PREV_M1; \
	vmovdqa64	T4, PREV_M2; \
.endif;

/*
 * Calculates the hash of the data which will not be encrypted.
 * Input: The input data (A_IN), that data's length (A_LEN), and the hash key (GDATA_KEY).
 * Output: The hash of the data (AAD_HASH).
 */
#define CALC_AAD_HASH(A_IN, A_LEN, AAD_HASH, GDATA_KEY, ZT0, ZT1, ZT2, ZT3, ZT4, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, ZT13, ZT14, ZT15, ZT16, ZT17, T1, T2, T3, MASKREG, OFFSET) \
	mov	A_IN, T1; \
	mov	A_LEN, T2; \
	or	T2, T2; \
	jz	0f; \
	vmovdqa64	SHUF_MASK(%rip), ZT13; \
20:; \
	cmp	$(48*16), T2; \
	jl	21f; \
	vmovdqu64	64*0(T1), ZT1; \
	vmovdqu64	64*1(T1), ZT2; \
	vmovdqu64	64*2(T1), ZT3; \
	vmovdqu64	64*3(T1), ZT4; \
	vpshufb ZT13, ZT1, ZT1; \
	vpshufb ZT13, ZT2, ZT2; \
	vpshufb ZT13, ZT3, ZT3; \
	vpshufb ZT13, ZT4, ZT4; \
	GHASH_1_TO_16(GDATA_KEY, OFFSET, ZWORD(AAD_HASH), ZT0, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, ZWORD(AAD_HASH), ZT1, ZT2, ZT3, ZT4, 16, 1, multi_call, first, 48, ZT14, ZT15, ZT16, ZT17) \
	vmovdqu64     0 + 256(T1), ZT1; \
	vmovdqu64     64 + 256(T1), ZT2; \
	vmovdqu64     128 + 256(T1), ZT3; \
	vmovdqu64     192 + 256(T1), ZT4; \
	vpshufb ZT13, ZT1, ZT1; \
	vpshufb ZT13, ZT2, ZT2; \
	vpshufb ZT13, ZT3, ZT3; \
	vpshufb ZT13, ZT4, ZT4; \
	GHASH_1_TO_16(GDATA_KEY, OFFSET, ZWORD(AAD_HASH), ZT0, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, ZWORD(AAD_HASH), ZT1, ZT2, ZT3, ZT4, 16, 1, multi_call, mid, 32, ZT14, ZT15, ZT16, ZT17) \
	vmovdqu64     0 + 512(T1), ZT1; \
	vmovdqu64     64 + 512(T1), ZT2; \
	vmovdqu64     128 + 512(T1), ZT3; \
	vmovdqu64     192 + 512(T1), ZT4; \
	vpshufb ZT13, ZT1, ZT1; \
	vpshufb ZT13, ZT2, ZT2; \
	vpshufb ZT13, ZT3, ZT3; \
	vpshufb ZT13, ZT4, ZT4; \
	GHASH_1_TO_16(GDATA_KEY, OFFSET, ZWORD(AAD_HASH), ZT0, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, ZWORD(AAD_HASH), ZT1, ZT2, ZT3, ZT4, 16, 1, multi_call, last, 16, ZT14, ZT15, ZT16, ZT17) \
	sub	$(48*16), T2; \
	je	0f; \
	add	$(48*16), T1; \
	jmp	20b; \
21:; \
	cmp	$(32*16), T2; \
	jl	22f; \
	vmovdqu64	64*0(T1), ZT1; \
	vmovdqu64	64*1(T1), ZT2; \
	vmovdqu64	64*2(T1), ZT3; \
	vmovdqu64	64*3(T1), ZT4; \
	vpshufb ZT13, ZT1, ZT1; \
	vpshufb ZT13, ZT2, ZT2; \
	vpshufb ZT13, ZT3, ZT3; \
	vpshufb ZT13, ZT4, ZT4; \
	GHASH_1_TO_16(GDATA_KEY, OFFSET, ZWORD(AAD_HASH), ZT0, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, ZWORD(AAD_HASH), ZT1, ZT2, ZT3, ZT4, 16, 1, multi_call, first, 32, ZT14, ZT15, ZT16, ZT17) \
	vmovdqu64     0 + 256(T1), ZT1; \
	vmovdqu64     64 + 256(T1), ZT2; \
	vmovdqu64     128 + 256(T1), ZT3; \
	vmovdqu64     192 + 256(T1), ZT4; \
	vpshufb ZT13, ZT1, ZT1; \
	vpshufb ZT13, ZT2, ZT2; \
	vpshufb ZT13, ZT3, ZT3; \
	vpshufb ZT13, ZT4, ZT4; \
	GHASH_1_TO_16(GDATA_KEY, OFFSET, ZWORD(AAD_HASH), ZT0, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, ZWORD(AAD_HASH), ZT1, ZT2, ZT3, ZT4, 16, 1, multi_call, last, 16, ZT14, ZT15, ZT16, ZT17) \
	sub	$(32*16), T2; \
	je	0f; \
	add	$(32*16), T1; \
	jmp	23f; \
22:; \
	cmp	$(16*16), T2; \
	jl	23f; \
	vmovdqu64	64*0(T1), ZT1; \
	vmovdqu64	64*1(T1), ZT2; \
	vmovdqu64	64*2(T1), ZT3; \
	vmovdqu64	64*3(T1), ZT4; \
	vpshufb ZT13, ZT1, ZT1; \
	vpshufb ZT13, ZT2, ZT2; \
	vpshufb ZT13, ZT3, ZT3; \
	vpshufb ZT13, ZT4, ZT4; \
	GHASH_1_TO_16(GDATA_KEY, OFFSET, ZWORD(AAD_HASH), ZT0, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, ZWORD(AAD_HASH), ZT1, ZT2, ZT3, ZT4, 16, 1, single_call, NULL, NULL, NULL, NULL, NULL, NULL) \
	sub	$(16*16), T2; \
	je	0f; \
	add	$(16*16), T1; \
23:; \
	lea	byte64_len_to_mask_table(%rip), T3; \
	lea	(T3, T2, 8), T3; \
	add	$15, T2; \
	and	$-16, T2; \
	shr	$4, T2; \
	cmp	$1, T2; \
	je	1f; \
	cmp	$2, T2; \
	je	2f; \
	cmp	$3, T2; \
	je	3f; \
	cmp	$4, T2; \
	je	4f; \
	cmp	$5, T2; \
	je	5f; \
	cmp	$6, T2; \
	je	6f; \
	cmp	$7, T2; \
	je	7f; \
	cmp	$8, T2; \
	je	8f; \
	cmp	$9, T2; \
	je	9f; \
	cmp	$10, T2; \
	je	10f; \
	cmp	$11, T2; \
	je	11f; \
	cmp	$12, T2; \
	je	12f; \
	cmp	$13, T2; \
	je	13f; \
	cmp	$14, T2; \
	je	14f; \
	cmp	$15, T2; \
	je	15f; \
16:; \
	sub $(64*3*8), T3; \
	kmovq	(T3), MASKREG; \
	vmovdqu8  64*0(T1), ZT1; \
	vmovdqu8  64*1(T1), ZT2; \
	vmovdqu8  64*2(T1), ZT3; \
	vmovdqu8  64*3(T1), ZT4{MASKREG}{z}; \
	vpshufb ZT13, ZT1, ZT1; \
	vpshufb ZT13, ZT2, ZT2; \
	vpshufb ZT13, ZT3, ZT3; \
	vpshufb ZT13, ZT4, ZT4; \
	GHASH_1_TO_16(GDATA_KEY, OFFSET, ZWORD(AAD_HASH), ZT0, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, ZWORD(AAD_HASH), ZT1, ZT2, ZT3, ZT4, 16, 1, single_call, NULL, NULL, NULL, NULL, NULL, NULL) \
	jmp	0f; \
15:; \
	sub $(64*3*8), T3; \
	kmovq	(T3), MASKREG; \
	vmovdqu8  64*0(T1), ZT1; \
	vmovdqu8  64*1(T1), ZT2; \
	vmovdqu8  64*2(T1), ZT3; \
	vmovdqu8  64*3(T1), ZT4{MASKREG}{z}; \
	vpshufb ZT13, ZT1, ZT1; \
	vpshufb ZT13, ZT2, ZT2; \
	vpshufb ZT13, ZT3, ZT3; \
	vpshufb ZT13, ZT4, ZT4; \
	GHASH_1_TO_16(GDATA_KEY, OFFSET, ZWORD(AAD_HASH), ZT0, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, ZWORD(AAD_HASH), ZT1, ZT2, ZT3, ZT4, 15, 1, single_call, NULL, NULL, NULL, NULL, NULL, NULL) \
	jmp	0f; \
14:; \
	sub $(64*3*8), T3; \
	kmovq	(T3), MASKREG; \
	vmovdqu8  64*0(T1), ZT1; \
	vmovdqu8  64*1(T1), ZT2; \
	vmovdqu8  64*2(T1), ZT3; \
	vmovdqu8  64*3(T1), ZT4{MASKREG}{z}; \
	vpshufb ZT13, ZT1, ZT1; \
	vpshufb ZT13, ZT2, ZT2; \
	vpshufb ZT13, ZT3, ZT3; \
	vpshufb ZT13, ZT4, ZT4; \
	GHASH_1_TO_16(GDATA_KEY, OFFSET, ZWORD(AAD_HASH), ZT0, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, ZWORD(AAD_HASH), ZT1, ZT2, ZT3, ZT4, 14, 1, single_call, NULL, NULL, NULL, NULL, NULL, NULL) \
	jmp	0f; \
13:; \
	sub $(64*3*8), T3; \
	kmovq	(T3), MASKREG; \
	vmovdqu8  64*0(T1), ZT1; \
	vmovdqu8  64*1(T1), ZT2; \
	vmovdqu8  64*2(T1), ZT3; \
	vmovdqu8  64*3(T1), ZT4{MASKREG}{z}; \
	vpshufb ZT13, ZT1, ZT1; \
	vpshufb ZT13, ZT2, ZT2; \
	vpshufb ZT13, ZT3, ZT3; \
	vpshufb ZT13, ZT4, ZT4; \
	GHASH_1_TO_16(GDATA_KEY, OFFSET, ZWORD(AAD_HASH), ZT0, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, ZWORD(AAD_HASH), ZT1, ZT2, ZT3, ZT4, 13, 1, single_call, NULL, NULL, NULL, NULL, NULL, NULL) \
	jmp	0f; \
12:; \
	sub $(64*2*8), T3; \
	kmovq	(T3), MASKREG; \
	vmovdqu8  64*0(T1), ZT1; \
	vmovdqu8  64*1(T1), ZT2; \
	vmovdqu8  64*2(T1), ZT3{MASKREG}{z}; \
	vpshufb ZT13, ZT1, ZT1; \
	vpshufb ZT13, ZT2, ZT2; \
	vpshufb ZT13, ZT3, ZT3; \
	GHASH_1_TO_16(GDATA_KEY, OFFSET, ZWORD(AAD_HASH), ZT0, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, ZWORD(AAD_HASH), ZT1, ZT2, ZT3, no_zmm, 12, 1, single_call, NULL, NULL, NULL, NULL, NULL, NULL) \
	jmp	0f; \
11:; \
	sub $(64*2*8), T3; \
	kmovq	(T3), MASKREG; \
	vmovdqu8  64*0(T1), ZT1; \
	vmovdqu8  64*1(T1), ZT2; \
	vmovdqu8  64*2(T1), ZT3{MASKREG}{z}; \
	vpshufb ZT13, ZT1, ZT1; \
	vpshufb ZT13, ZT2, ZT2; \
	vpshufb ZT13, ZT3, ZT3; \
	GHASH_1_TO_16(GDATA_KEY, OFFSET, ZWORD(AAD_HASH), ZT0, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, ZWORD(AAD_HASH), ZT1, ZT2, ZT3, no_zmm, 11, 1, single_call, NULL, NULL, NULL, NULL, NULL, NULL) \
	jmp	0f; \
10:; \
	sub $(64*2*8), T3; \
	kmovq	(T3), MASKREG; \
	vmovdqu8  64*0(T1), ZT1; \
	vmovdqu8  64*1(T1), ZT2; \
	vmovdqu8  64*2(T1), ZT3{MASKREG}{z}; \
	vpshufb ZT13, ZT1, ZT1; \
	vpshufb ZT13, ZT2, ZT2; \
	vpshufb ZT13, ZT3, ZT3; \
	GHASH_1_TO_16(GDATA_KEY, OFFSET, ZWORD(AAD_HASH), ZT0, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, ZWORD(AAD_HASH), ZT1, ZT2, ZT3, no_zmm, 10, 1, single_call, NULL, NULL, NULL, NULL, NULL, NULL) \
	jmp	0f; \
9:; \
	sub $(64*2*8), T3; \
	kmovq	(T3), MASKREG; \
	vmovdqu8  64*0(T1), ZT1; \
	vmovdqu8  64*1(T1), ZT2; \
	vmovdqu8  64*2(T1), ZT3{MASKREG}{z}; \
	vpshufb ZT13, ZT1, ZT1; \
	vpshufb ZT13, ZT2, ZT2; \
	vpshufb ZT13, ZT3, ZT3; \
	GHASH_1_TO_16(GDATA_KEY, OFFSET, ZWORD(AAD_HASH), ZT0, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, ZWORD(AAD_HASH), ZT1, ZT2, ZT3, no_zmm, 9, 1, single_call, NULL, NULL, NULL, NULL, NULL, NULL) \
	jmp	0f; \
8:; \
	sub $(64*8), T3; \
	kmovq	(T3), MASKREG; \
	vmovdqu8  64*0(T1), ZT1; \
	vmovdqu8  64*1(T1), ZT2{MASKREG}{z}; \
	vpshufb ZT13, ZT1, ZT1; \
	vpshufb ZT13, ZT2, ZT2; \
	GHASH_1_TO_16(GDATA_KEY, OFFSET, ZWORD(AAD_HASH), ZT0, ZT3, ZT4, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZWORD(AAD_HASH), ZT1, ZT2, no_zmm, no_zmm, 8, 1, single_call, NULL, NULL, NULL, NULL, NULL, NULL) \
	jmp	0f; \
7:; \
	sub $(64*8), T3; \
	kmovq	(T3), MASKREG; \
	vmovdqu8  64*0(T1), ZT1; \
	vmovdqu8  64*1(T1), ZT2{MASKREG}{z}; \
	vpshufb ZT13, ZT1, ZT1; \
	vpshufb ZT13, ZT2, ZT2; \
	GHASH_1_TO_16(GDATA_KEY, OFFSET, ZWORD(AAD_HASH), ZT0, ZT3, ZT4, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZWORD(AAD_HASH), ZT1, ZT2, no_zmm, no_zmm, 7, 1, single_call, NULL, NULL, NULL, NULL, NULL, NULL) \
	jmp	0f; \
6:; \
	sub $(64*8), T3; \
	kmovq	(T3), MASKREG; \
	vmovdqu8  64*0(T1), ZT1; \
	vmovdqu8  64*1(T1), YWORD(ZT2){MASKREG}{z}; \
	vpshufb ZT13, ZT1, ZT1; \
	vpshufb YWORD(ZT13), YWORD(ZT2), YWORD(ZT2); \
	GHASH_1_TO_16(GDATA_KEY, OFFSET, ZWORD(AAD_HASH), ZT0, ZT3, ZT4, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZWORD(AAD_HASH), ZT1, ZT2, no_zmm, no_zmm, 6, 1, single_call, NULL, NULL, NULL, NULL, NULL, NULL) \
	jmp	0f; \
5:; \
	sub $(64*8), T3; \
	kmovq	(T3), MASKREG; \
	vmovdqu8  64*0(T1), ZT1; \
	vmovdqu8  64*1(T1), XWORD(ZT2){MASKREG}{z}; \
	vpshufb ZT13, ZT1, ZT1; \
	vpshufb XWORD(ZT13), XWORD(ZT2), XWORD(ZT2); \
	GHASH_1_TO_16(GDATA_KEY, OFFSET, ZWORD(AAD_HASH), ZT0, ZT3, ZT4, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZWORD(AAD_HASH), ZT1, ZT2, no_zmm, no_zmm, 5, 1, single_call, NULL, NULL, NULL, NULL, NULL, NULL) \
	jmp	0f; \
4:; \
	kmovq	(T3), MASKREG; \
	vmovdqu8  64*0(T1), ZT1{MASKREG}{z}; \
	vpshufb ZT13, ZT1, ZT1; \
	GHASH_1_TO_16(GDATA_KEY, OFFSET, ZWORD(AAD_HASH), ZT0, ZT3, ZT4, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZWORD(AAD_HASH), ZT1, no_zmm, no_zmm, no_zmm, 4, 1, single_call, NULL, NULL, NULL, NULL, NULL, NULL) \
	jmp	0f; \
3:; \
	kmovq	(T3), MASKREG; \
	vmovdqu8  64*0(T1), ZT1{MASKREG}{z}; \
	vpshufb ZT13, ZT1, ZT1; \
	GHASH_1_TO_16(GDATA_KEY, OFFSET, ZWORD(AAD_HASH), ZT0, ZT3, ZT4, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZWORD(AAD_HASH), ZT1, no_zmm, no_zmm, no_zmm, 3, 1, single_call, NULL, NULL, NULL, NULL, NULL, NULL) \
	jmp	0f; \
2:; \
	kmovq	(T3), MASKREG; \
	vmovdqu8  64*0(T1), YWORD(ZT1){MASKREG}{z}; \
	vpshufb YWORD(ZT13), YWORD(ZT1), YWORD(ZT1); \
	GHASH_1_TO_16(GDATA_KEY, OFFSET, ZWORD(AAD_HASH), ZT0, ZT3, ZT4, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZWORD(AAD_HASH), ZT1, no_zmm, no_zmm, no_zmm, 2, 1, single_call, NULL, NULL, NULL, NULL, NULL, NULL) \
	jmp	0f; \
1:; \
	kmovq	(T3), MASKREG; \
	vmovdqu8  64*0(T1), XWORD(ZT1){MASKREG}{z}; \
	vpshufb XWORD(ZT13), XWORD(ZT1), XWORD(ZT1); \
	GHASH_1_TO_16(GDATA_KEY, OFFSET, ZWORD(AAD_HASH), ZT0, ZT3, ZT4, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZWORD(AAD_HASH), ZT1, no_zmm, no_zmm, no_zmm, 1, 1, single_call, NULL, NULL, NULL, NULL, NULL, NULL) \
0:;
