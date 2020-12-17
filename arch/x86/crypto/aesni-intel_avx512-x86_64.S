/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright © 2020 Intel Corporation.
 *
 * Implement AES GCM mode optimization with VAES instructions. (x86_64)
 *
 * This is AES128/192/256 GCM mode optimization implementation. It requires
 * the support of Intel(R) AVX512F, VPCLMULQDQ and VAES instructions.
 *
 * This work was inspired by the AES GCM mode optimization published
 * in Intel Optimized IPSEC Cryptographic library.
 * https://github.com/intel/intel-ipsec-mbi/avx512/gcm_vaes_avx512.asm
 *
 * Contact Information:
 * Vinodh Gopal <vinodh.gopal@intel.com>
 * James Guilford <james.guilford@intel.com>
 * Tomasz Kantecki <tomasz.kantecki@intel.com>
 * Megha Dey <megha.dey@intel.com>
 */

#include "avx512_vaes_common.S"

#define HashSubKey	(16*6)
#define PBlockLen	(16*5)
#define CurCount	(16*4)
#define OrigIV		(16*3)
#define PBlockEncKey	(16*2)
#define InLen		((16*1)+8)
#define AadLen		(16*1)
#define AadHash		(16*0)
#define big_loop_nblocks	48

.text

#define ENCRYPT_SINGLE_BLOCK(GDATA, XMM0, NROUNDS)	\
	vpxorq (GDATA), XMM0, XMM0;			\
.set i, 1;						\
.rept 9;						\
	vaesenc 16 * i(GDATA), XMM0, XMM0;		\
	.set i, i+1;					\
.endr;							\
.if NROUNDS == 9;					\
	vaesenclast 16 * 10(GDATA), XMM0, XMM0;		\
.else;							\
	vaesenc 16 * 10(GDATA), XMM0, XMM0;		\
	vaesenc 16 * 11(GDATA), XMM0, XMM0;		\
	.if NROUNDS == 11;				\
		vaesenclast 16 * 12(GDATA), XMM0, XMM0;	\
	.else;						\
		vaesenc 16 * 12(GDATA), XMM0, XMM0;	\
		vaesenc 16 * 13(GDATA), XMM0, XMM0;	\
		vaesenclast 16 * 14(GDATA), XMM0, XMM0;	\
	.endif;						\
.endif;

/* schoolbook multiply - 1st step */
#define VCLMUL_STEP1(HS, HI, TMP, TH, TM, TL, HKEY)		\
.ifc HKEY, NULL;						\
	vmovdqu64	HashKey_4 + HashSubKey(HS), TMP;	\
.else;								\
	vmovdqa64	HKEY , TMP;				\
.endif;								\
	vpclmulqdq	$0x11, TMP, HI, TH;			\
	vpclmulqdq	$0x00, TMP, HI, TL;			\
	vpclmulqdq	$0x01, TMP, HI, TM;			\
	vpclmulqdq	$0x10, TMP, HI, TMP;			\
	vpxorq		TMP, TM, TM;

/* Horizontal XOR - 2 x 128bits xored together */
#define VHPXORI2x128(REG, TMP)					\
	vextracti32x4	$1, REG, XWORD(TMP);			\
	vpxorq		XWORD(TMP), XWORD(REG), XWORD(REG);

/* schoolbook multiply - 2nd step */
#define VCLMUL_STEP2(HS, HI, LO, TMP0, TMP1, TMP2, TH, TM, TL, HKEY, HXOR)	\
.ifc HKEY, NULL;								\
	vmovdqu64	HashKey_8 + HashSubKey(HS), TMP0;			\
.else;										\
	vmovdqa64	HKEY, TMP0;						\
.endif;										\
	vpclmulqdq	$0x10, TMP0, LO, TMP1;					\
	vpclmulqdq	$0x11, TMP0, LO, TMP2;					\
	vpxorq		TMP2, TH, TH;						\
	vpclmulqdq	$0x00, TMP0, LO, TMP2;					\
	vpxorq		TMP2, TL, TL;						\
	vpclmulqdq	$0x01, TMP0, LO, TMP0;					\
	vpternlogq	$0x96, TMP0, TMP1, TM;					\
	/* finish multiplications */						\
	vpsrldq		$8, TM, TMP2;						\
	vpxorq		TMP2, TH, HI;						\
	vpslldq		$8, TM, TMP2;						\
	vpxorq		TMP2, TL, LO;						\
	/* XOR 128 bits horizontally and compute [(X8*H1) + (X7*H2) + ... ((X1+Y0)*H8] */	\
.ifc HXOR, NULL;								\
	VHPXORI4x128(HI, TMP2)							\
	VHPXORI4x128(LO, TMP1)							\
.else;										\
	.if HXOR == 4;								\
		VHPXORI4x128(HI, TMP2)						\
		VHPXORI4x128(LO, TMP1)						\
	.elseif HXOR == 2;							\
		VHPXORI2x128(HI, TMP2)						\
		VHPXORI2x128(LO, TMP1)						\
	.endif;									\
	/* for HXOR == 1 there is nothing to be done */				\
.endif;

/* schoolbook multiply (1 to 8 blocks) - 1st step */
#define VCLMUL_1_TO_8_STEP1(HS, HI, TMP1, TMP2, TH, TM, TL, NBLOCKS)	\
	.if NBLOCKS == 8;						\
		VCLMUL_STEP1(HS, HI, TMP1, TH, TM, TL, NULL)		\
	.elseif NBLOCKS == 7;						\
		vmovdqu64	HashKey_3 + HashSubKey(HS), TMP2;	\
		vmovdqa64	mask_out_top_block(%rip), TMP1;		\
		vpandq		TMP1, TMP2, TMP2;			\
		vpandq		TMP1, HI, HI;				\
		VCLMUL_STEP1(NULL, HI, TMP1, TH, TM, TL, TMP2)		\
	.elseif NBLOCKS == 6;						\
		vmovdqu64	HashKey_2 + HashSubKey(HS), YWORD(TMP2);\
		VCLMUL_STEP1(NULL, YWORD(HI), YWORD(TMP1), YWORD(TH), YWORD(TM), YWORD(TL), YWORD(TMP2))	\
	.elseif NBLOCKS == 5;						\
		vmovdqu64	HashKey_1 + HashSubKey(HS), XWORD(TMP2);\
		VCLMUL_STEP1(NULL, XWORD(HI), XWORD(TMP1), XWORD(TH), XWORD(TM), XWORD(TL), XWORD(TMP2))	\
	.else;								\
		vpxorq		TH, TH, TH;				\
		vpxorq		TM, TM, TM;				\
		vpxorq		TL, TL, TL;				\
	.endif;

/* schoolbook multiply (1 to 8 blocks) - 2nd step */
#define VCLMUL_1_TO_8_STEP2(HS, HI, LO, TMP0, TMP1, TMP2, TH, TM, TL, NBLOCKS)		\
	.if NBLOCKS == 8;								\
		VCLMUL_STEP2(HS, HI, LO, TMP0, TMP1, TMP2, TH, TM, TL, NULL, NULL)	\
	.elseif NBLOCKS == 7;								\
		vmovdqu64	HashKey_7 + HashSubKey(HS), TMP2;			\
		VCLMUL_STEP2(NULL, HI, LO, TMP0, TMP1, TMP2, TH, TM, TL, TMP2, 4)	\
	.elseif NBLOCKS == 6;								\
		vmovdqu64	HashKey_6 + HashSubKey(HS), TMP2;			\
		VCLMUL_STEP2(NULL, HI, LO, TMP0, TMP1, TMP2, TH, TM, TL, TMP2, 4)	\
	.elseif NBLOCKS == 5;								\
		vmovdqu64	HashKey_5 + HashSubKey(HS), TMP2;			\
		VCLMUL_STEP2(NULL, HI, LO, TMP0, TMP1, TMP2, TH, TM, TL, TMP2, 4)	\
	.elseif NBLOCKS == 4;								\
		vmovdqu64	HashKey_4 + HashSubKey(HS), TMP2;			\
		VCLMUL_STEP2(NULL, HI, LO, TMP0, TMP1, TMP2, TH, TM, TL, TMP2, 4)	\
	.elseif NBLOCKS == 3;								\
		vmovdqu64	HashKey_3 + HashSubKey(HS), TMP2;			\
		vmovdqa64	mask_out_top_block(%rip), TMP1;				\
		vpandq		TMP1, TMP2, TMP2;					\
		vpandq		TMP1, LO, LO;						\
		VCLMUL_STEP2(NULL, HI, LO, TMP0, TMP1, TMP2, TH, TM, TL, TMP2, 4)	\
	.elseif NBLOCKS == 2;								\
		vmovdqu64	HashKey_2 + HashSubKey(HS), YWORD(TMP2);		\
		VCLMUL_STEP2(NULL, YWORD(HI), YWORD(LO), YWORD(TMP0), YWORD(TMP1), YWORD(TMP2), YWORD(TH), YWORD(TM), YWORD(TL), YWORD(TMP2), 2)	\
	.elseif NBLOCKS == 1;								\
		vmovdqu64	HashKey_1 + HashSubKey(HS), XWORD(TMP2);		\
		VCLMUL_STEP2(NULL, XWORD(HI), XWORD(LO), XWORD(TMP0), XWORD(TMP1), XWORD(TMP2), XWORD(TH), XWORD(TM), XWORD(TL), XWORD(TMP2), 1)	\
	.else;										\
		vpxorq		HI, HI, HI;						\
		vpxorq		LO, LO, LO;						\
	.endif;

/* Initialize a gcm_context_data struct to prepare for encoding/decoding. */
#define GCM_INIT(GDATA_CTX, IV, HASH_SUBKEY, A_IN, A_LEN, GPR1, GPR2, GPR3, MASKREG, AAD_HASH, CUR_COUNT, ZT0, ZT1, ZT2, ZT3, ZT4, ZT5, ZT6, ZT7, ZT8, ZT9)	  \
	vpxorq           AAD_HASH, AAD_HASH, AAD_HASH;			\
	CALC_AAD_HASH(A_IN, A_LEN, AAD_HASH, GDATA_CTX, ZT0, ZT1, ZT2, ZT3, ZT4, ZT5, ZT6, ZT7, ZT8, ZT9, %zmm11, %zmm12, %zmm13, %zmm15, %zmm16, %zmm17, %zmm18, %zmm19, GPR1, GPR2, GPR3, MASKREG, 96)	\
	mov		A_LEN, GPR1;				\
	vmovdqu64	AAD_HASH, (GDATA_CTX);			\
	mov		GPR1, 16(GDATA_CTX);			\
	xor		GPR1, GPR1;				\
	mov		GPR1, InLen(GDATA_CTX);			\
	mov		GPR1, PBlockLen(GDATA_CTX);		\
	vmovdqu8	ONEf(%rip), CUR_COUNT;			\
	mov		IV, GPR2;				\
	mov		$0xfff, GPR1;				\
	kmovq		GPR1, MASKREG;				\
	vmovdqu8	(GPR2), CUR_COUNT{MASKREG};		\
	vmovdqu64	CUR_COUNT, OrigIV(GDATA_CTX);		\
	vpshufb		SHUF_MASK(%rip), CUR_COUNT, CUR_COUNT;	\
	vmovdqu		CUR_COUNT, CurCount(GDATA_CTX);

/* Packs xmm register with data when data input is less or equal to 16 bytes */
#define READ_SMALL_DATA_INPUT(OUTPUT, INPUT, LEN ,TMP1, MASK)	\
	cmp		$16, LEN;				\
	jge		49f;					\
	lea		byte_len_to_mask_table(%rip), TMP1;	\
	kmovw		(TMP1, LEN, 2), MASK;			\
	vmovdqu8	(INPUT), OUTPUT{MASK}{z};		\
	jmp		50f;					\
49:;								\
	vmovdqu8	(INPUT), OUTPUT;			\
	mov		$0xffff, TMP1;				\
	kmovq		TMP1, MASK;				\
50:;

/*
 * Handles encryption/decryption and the tag partial blocks between update calls.
 * Requires the input data be at least 1 byte long. The output is a cipher/plain
 * of the first partial block (CYPH_PLAIN_OUT), AAD_HASH and updated GDATA_CTX
 */
#define PARTIAL_BLOCK(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, PLAIN_CYPH_LEN, DATA_OFFSET, AAD_HASH, ENC_DEC, GPTMP0, GPTMP1, GPTMP2, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, ZTMP9, MASKREG)	\
	mov		PBlockLen(GDATA_CTX), GPTMP0;			\
	or		GPTMP0, GPTMP0;					\
	je		48f;						\
	READ_SMALL_DATA_INPUT(XWORD(ZTMP0), PLAIN_CYPH_IN, PLAIN_CYPH_LEN, GPTMP1, MASKREG)	\
	vmovdqu64	PBlockEncKey(GDATA_CTX), XWORD(ZTMP1);		\
	vmovdqu64	HashKey + HashSubKey(GDATA_CTX), XWORD(ZTMP2);	\
	lea		SHIFT_MASK(%rip), GPTMP1;			\
	add		GPTMP0, GPTMP1;					\
	vmovdqu64	(GPTMP1), XWORD(ZTMP3);				\
	vpshufb		XWORD(ZTMP3), XWORD(ZTMP1), XWORD(ZTMP1);	\
	.ifc ENC_DEC, DEC;						\
	vmovdqa64	XWORD(ZTMP0), XWORD(ZTMP4);			\
	.endif;								\
	vpxorq		XWORD(ZTMP0), XWORD(ZTMP1), XWORD(ZTMP1);	\
	/* Determine if partial block is being filled & shift mask */	\
	mov		PLAIN_CYPH_LEN, GPTMP2;				\
	add		GPTMP0, GPTMP2;					\
	sub		$16, GPTMP2;					\
	jge		45f;						\
	sub		GPTMP2, GPTMP1;					\
45:;									\
	/* get the mask to mask out bottom GPTMP0 bytes of XTMP1 */	\
	vmovdqu64	(ALL_F - SHIFT_MASK)(GPTMP1), XWORD(ZTMP0);	\
	vpand		XWORD(ZTMP0), XWORD(ZTMP1),  XWORD(ZTMP1);	\
	.ifc ENC_DEC, DEC;						\
	vpand		XWORD(ZTMP0), XWORD(ZTMP4), XWORD(ZTMP4);	\
	vpshufb		SHUF_MASK(%rip), XWORD(ZTMP4), XWORD(ZTMP4);	\
	vpshufb		XWORD(ZTMP3), XWORD(ZTMP4), XWORD(ZTMP4);	\
	vpxorq		XWORD(ZTMP4), AAD_HASH, AAD_HASH;		\
	.else;								\
	vpshufb		SHUF_MASK(%rip), XWORD(ZTMP1), XWORD(ZTMP1);	\
	vpshufb		XWORD(ZTMP3), XWORD(ZTMP1), XWORD(ZTMP1);	\
	vpxorq		XWORD(ZTMP1), AAD_HASH, AAD_HASH;		\
	.endif;								\
	cmp		$0, GPTMP2;					\
	jl		46f;						\
	/* GHASH computation for the last <16 Byte block */		\
	GHASH_MUL(AAD_HASH, XWORD(ZTMP2), XWORD(ZTMP5), XWORD(ZTMP6), XWORD(ZTMP7), XWORD(ZTMP8), XWORD(ZTMP9)) \
	movq		$0, PBlockLen(GDATA_CTX);			\
	mov		GPTMP0, GPTMP1;					\
	mov		$16, GPTMP0;					\
	sub		GPTMP1, GPTMP0;					\
	jmp		47f;						\
46:;									\
	add		PLAIN_CYPH_LEN, PBlockLen(GDATA_CTX);		\
	mov		PLAIN_CYPH_LEN, GPTMP0;				\
47:;									\
	lea		byte_len_to_mask_table(%rip), GPTMP1;		\
	kmovw		(GPTMP1, GPTMP0, 2), MASKREG;			\
	vmovdqu64	AAD_HASH, (GDATA_CTX);				\
	.ifc ENC_DEC, ENC;						\
	/* shuffle XTMP1 back to output as ciphertext */		\
	vpshufb		SHUF_MASK(%rip), XWORD(ZTMP1), XWORD(ZTMP1);	\
	vpshufb		XWORD(ZTMP3), XWORD(ZTMP1), XWORD(ZTMP1);	\
	.endif;								\
	vmovdqu8	XWORD(ZTMP1), (CYPH_PLAIN_OUT, DATA_OFFSET, 1){MASKREG};	\
	add		GPTMP0, DATA_OFFSET;				\
48:;

/* Encrypt/decrypt the initial 16 blocks */
#define INITIAL_BLOCKS_16(IN, OUT, KP, DATA_OFFSET, GHASH, CTR, CTR_CHECK, ADDBE_4x4, ADDBE_1234, T0, T1, T2, T3, T4, T5, T6, T7, T8, SHUF_MASK, ENC_DEC, BLK_OFFSET, DATA_DISPL, NROUNDS) \
	cmp		$(256 - 16), BYTE(CTR_CHECK);		\
	jae		37f;					\
	vpaddd		ADDBE_1234 ,CTR, T5;			\
	vpaddd		ADDBE_4x4, T5, T6;			\
	vpaddd		ADDBE_4x4, T6, T7;			\
	vpaddd		ADDBE_4x4, T7, T8;			\
	jmp		38f;					\
37:;								\
	vpshufb		SHUF_MASK, CTR, CTR;			\
	vmovdqa64	ddq_add_4444(%rip), T8;			\
	vpaddd		ddq_add_1234(%rip), CTR, T5;		\
	vpaddd		T8, T5, T6;				\
	vpaddd		T8, T6, T7;				\
	vpaddd		T8, T7, T8;				\
	vpshufb		SHUF_MASK, T5, T5;			\
	vpshufb		SHUF_MASK, T6, T6;			\
	vpshufb		SHUF_MASK, T7, T7;			\
	vpshufb		SHUF_MASK, T8, T8;			\
38:;								\
	vshufi64x2	$0xff, T8, T8, CTR;			\
	add		$16, BYTE(CTR_CHECK);			\
	/* load 16 blocks of data */				\
	vmovdqu8	DATA_DISPL(IN, DATA_OFFSET), T0;	\
	vmovdqu8	64 + DATA_DISPL(DATA_OFFSET, IN), T1;	\
	vmovdqu8	128 + DATA_DISPL(DATA_OFFSET, IN), T2;	\
	vmovdqu8	192 + DATA_DISPL(DATA_OFFSET, IN), T3;	\
	/* move to AES encryption rounds */			\
	vbroadcastf64x2 (KP), T4;				\
	vpxorq		T4, T5, T5;				\
	vpxorq		T4, T6, T6;				\
	vpxorq		T4, T7, T7;				\
	vpxorq		T4, T8, T8;				\
.set i, 1;							\
.rept 9;							\
	vbroadcastf64x2 16*i(KP), T4;				\
	vaesenc		T4, T5, T5;				\
	vaesenc		T4, T6, T6;				\
	vaesenc		T4, T7, T7;				\
	vaesenc		T4, T8, T8;				\
	.set i, i+1;						\
.endr;								\
.if NROUNDS==9;							\
	vbroadcastf64x2 16*i(KP), T4;				\
.else;								\
	.rept 2;						\
		vbroadcastf64x2 16*i(KP), T4;			\
		vaesenc		T4, T5, T5;			\
		vaesenc		T4, T6, T6;			\
		vaesenc		T4, T7, T7;			\
		vaesenc		T4, T8, T8;			\
		.set i, i+1;					\
	.endr;							\
	.if NROUNDS==11;					\
		vbroadcastf64x2 16*i(KP), T4;			\
	.else;							\
		.rept 2;					\
			vbroadcastf64x2 16*i(KP), T4;		\
			vaesenc		T4, T5, T5;		\
			vaesenc		T4, T6, T6;		\
			vaesenc		T4, T7, T7;		\
			vaesenc		T4, T8, T8;		\
		.set i, i+1;					\
		.endr;						\
		vbroadcastf64x2 16*i(KP), T4;			\
	.endif;							\
.endif;								\
	vaesenclast	T4, T5, T5;				\
	vaesenclast	T4, T6, T6;				\
	vaesenclast	T4, T7, T7;				\
	vaesenclast	T4, T8, T8;				\
	vpxorq		T0, T5, T5;				\
	vpxorq		T1, T6, T6;				\
	vpxorq		T2, T7, T7;				\
	vpxorq		T3, T8, T8;				\
	vmovdqu8	T5, DATA_DISPL(OUT, DATA_OFFSET);	\
	vmovdqu8	T6, 64 + DATA_DISPL(DATA_OFFSET, OUT);	\
	vmovdqu8	T7, 128 + DATA_DISPL(DATA_OFFSET, OUT);	\
	vmovdqu8	T8, 192 + DATA_DISPL(DATA_OFFSET, OUT);	\
.ifc  ENC_DEC, DEC;						\
	vpshufb		SHUF_MASK, T0, T5;			\
	vpshufb		SHUF_MASK, T1, T6;			\
	vpshufb		SHUF_MASK, T2, T7;			\
	vpshufb		SHUF_MASK, T3, T8;			\
.else;								\
	vpshufb		SHUF_MASK, T5, T5;			\
	vpshufb		SHUF_MASK, T6, T6;			\
	vpshufb		SHUF_MASK, T7, T7;			\
	vpshufb		SHUF_MASK, T8, T8;			\
.endif;								\
.ifnc GHASH, no_ghash;						\
	/* xor cipher block0 with GHASH for next GHASH round */	\
	vpxorq		GHASH, T5, T5;				\
.endif;								\
	vmovdqa64	T5, BLK_OFFSET(%rsp);			\
	vmovdqa64	T6, 64 + BLK_OFFSET(%rsp);		\
	vmovdqa64	T7, 128 + BLK_OFFSET(%rsp);		\
	vmovdqa64	T8, 192 + BLK_OFFSET(%rsp);

/*
 * Main GCM macro stitching cipher with GHASH
 * - operates on single stream
 * - encrypts 16 blocks at a time
 * - ghash the 16 previously encrypted ciphertext blocks
 * - no partial block or multi_call handling here
 */
#define GHASH_16_ENCRYPT_16_PARALLEL(GDATA, GCTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, DATA_OFFSET, CTR_BE, CTR_CHECK, HASHKEY_OFFSET, AESOUT_BLK_OFFSET, GHASHIN_BLK_OFFSET, SHFMSK, ZT1, ZT2, ZT3, ZT4, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, ZT13, ZT14, ZT15, ZT16, ZT17, ZT18, ZT19, ZT20, ZT21, ZT22, ZT23, ADDBE_4x4, ADDBE_1234, TO_REDUCE_L, TO_REDUCE_H, TO_REDUCE_M, DO_REDUCTION, ENC_DEC, DATA_DISPL, GHASH_IN, NROUNDS)   \
	cmp		$240, BYTE(CTR_CHECK);		\
	jae		28f;				\
	vpaddd		ADDBE_1234, CTR_BE, ZT1;	\
	vpaddd		ADDBE_4x4, ZT1, ZT2;		\
	vpaddd		 ADDBE_4x4, ZT2, ZT3;		\
	vpaddd		 ADDBE_4x4, ZT3, ZT4;		\
	jmp		29f;				\
28:;							\
	vpshufb		SHFMSK, CTR_BE, CTR_BE;		\
	vmovdqa64	ddq_add_4444(%rip), ZT4;	\
	vpaddd		ddq_add_1234(%rip), CTR_BE, ZT1;\
	vpaddd		ZT4, ZT1, ZT2;			\
	vpaddd		ZT4, ZT2, ZT3;			\
	vpaddd		ZT4, ZT3, ZT4;			\
	vpshufb		SHFMSK, ZT1, ZT1;		\
	vpshufb		SHFMSK, ZT2, ZT2;		\
	vpshufb		SHFMSK, ZT3, ZT3;		\
	vpshufb		SHFMSK, ZT4, ZT4;		\
29:;							\
	vbroadcastf64x2 (GDATA), ZT17;			\
.ifnc GHASH_IN,no_ghash_in;				\
	vpxorq		GHASHIN_BLK_OFFSET(%rsp), GHASH_IN, ZT21;	\
.else;							\
	vmovdqa64	GHASHIN_BLK_OFFSET(%rsp), ZT21;	\
.endif;							\
	vmovdqu64	HASHKEY_OFFSET(GCTX), ZT19;	\
	/*						\
	 * Save counter for the next round, increment	\
	 * counter overflow check register.		\
	 */						\
	vshufi64x2	$0xff, ZT4, ZT4, CTR_BE;	\
	add		$16, BYTE(CTR_CHECK);		\
	vbroadcastf64x2 16*1(GDATA), ZT18;		\
	vmovdqu64	HASHKEY_OFFSET + 64(GCTX), ZT20;\
	vmovdqa64	GHASHIN_BLK_OFFSET + 64(%rsp), ZT22;	\
	vpxorq		ZT17, ZT1, ZT1;			\
	vpxorq		ZT17, ZT2, ZT2;			\
	vpxorq		ZT17, ZT3, ZT3;			\
	vpxorq		ZT17, ZT4, ZT4;			\
	vbroadcastf64x2 16*2(GDATA), ZT17;		\
	/* GHASH 4 blocks (15 to 12) */			\
	vpclmulqdq	$0x11, ZT19, ZT21, ZT5;		\
	vpclmulqdq	$0x00, ZT19, ZT21, ZT6;		\
	vpclmulqdq	$0x01, ZT19, ZT21, ZT7;		\
	vpclmulqdq	$0x10, ZT19, ZT21, ZT8;		\
	vmovdqu64	HASHKEY_OFFSET + 64*2(GCTX), ZT19;	\
	vmovdqa64	GHASHIN_BLK_OFFSET + 64*2(%rsp), ZT21;	\
	/* AES round 1 */				\
	vaesenc		ZT18, ZT1, ZT1;			\
	vaesenc		ZT18, ZT2, ZT2;			\
	vaesenc		ZT18, ZT3, ZT3;			\
	vaesenc		ZT18, ZT4, ZT4;			\
	vbroadcastf64x2 16*3(GDATA), ZT18;		\
	/* GHASH 4 blocks (11 to 8) */			\
	vpclmulqdq	$0x10, ZT20, ZT22, ZT11;	\
	vpclmulqdq	$0x01, ZT20, ZT22, ZT12;	\
	vpclmulqdq	$0x11, ZT20, ZT22, ZT9;		\
	vpclmulqdq	$0x00, ZT20, ZT22, ZT10;	\
	vmovdqu64	HASHKEY_OFFSET + 64*3(GCTX), ZT20;	\
	vmovdqa64	GHASHIN_BLK_OFFSET + 64*3(%rsp), ZT22;	\
	/* AES round 2 */				\
	vaesenc		ZT17, ZT1, ZT1;			\
	vaesenc		ZT17, ZT2, ZT2;			\
	vaesenc		ZT17, ZT3, ZT3;			\
	vaesenc		ZT17, ZT4, ZT4;			\
	vbroadcastf64x2 16*4(GDATA), ZT17;		\
	/* GHASH 4 blocks (7 to 4) */			\
	vpclmulqdq	$0x10, ZT19, ZT21, ZT15;	\
	vpclmulqdq	$0x01, ZT19, ZT21, ZT16;	\
	vpclmulqdq	$0x11, ZT19, ZT21, ZT13;	\
	vpclmulqdq	$0x00, ZT19, ZT21, ZT14;	\
	/* AES round 3 */				\
	vaesenc		ZT18, ZT1, ZT1;			\
	vaesenc		ZT18, ZT2, ZT2;			\
	vaesenc		ZT18, ZT3, ZT3;			\
	vaesenc		ZT18, ZT4, ZT4;			\
	vbroadcastf64x2 16*5(GDATA), ZT18;		\
	/* Gather (XOR) GHASH for 12 blocks */		\
	vpternlogq	$0x96, ZT13, ZT9, ZT5;		\
	vpternlogq	$0x96, ZT14, ZT10, ZT6;		\
	vpternlogq	$0x96, ZT16, ZT12, ZT8;		\
	vpternlogq	$0x96, ZT15, ZT11, ZT7;		\
	/* AES round 4 */				\
	vaesenc		ZT17, ZT1, ZT1;			\
	vaesenc		ZT17, ZT2, ZT2;			\
	vaesenc		ZT17, ZT3, ZT3;			\
	vaesenc		ZT17, ZT4, ZT4;			\
	vbroadcastf64x2 16*6(GDATA), ZT17;		\
	/* Load plain/cipher test */			\
	vmovdqu8	DATA_DISPL(DATA_OFFSET, PLAIN_CYPH_IN), ZT13;	\
	vmovdqu8	64*1 + DATA_DISPL(DATA_OFFSET, PLAIN_CYPH_IN), ZT14;	\
	vmovdqu8	64*2 + DATA_DISPL(DATA_OFFSET, PLAIN_CYPH_IN), ZT15;	\
	vmovdqu8	64*3 + DATA_DISPL(DATA_OFFSET, PLAIN_CYPH_IN), ZT16;	\
	/* AES round 5 */				\
	vaesenc		ZT18, ZT1, ZT1;			\
	vaesenc		ZT18, ZT2, ZT2;			\
	vaesenc		ZT18, ZT3, ZT3;			\
	vaesenc		ZT18, ZT4, ZT4;			\
	vbroadcastf64x2 16*7(GDATA), ZT18;		\
	/* GHASH 4 blocks (3 to 0) */			\
	vpclmulqdq	$0x10, ZT20, ZT22, ZT11;	\
	vpclmulqdq	$0x01, ZT20, ZT22, ZT12;	\
	vpclmulqdq	$0x11, ZT20, ZT22, ZT9;		\
	vpclmulqdq	$0x00, ZT20, ZT22, ZT10;	\
	/* AES round 6 */				\
	vaesenc		ZT17, ZT1, ZT1;			\
	vaesenc		ZT17, ZT2, ZT2;			\
	vaesenc		ZT17, ZT3, ZT3;			\
	vaesenc		ZT17, ZT4, ZT4;			\
	vbroadcastf64x2 16*8(GDATA), ZT17;		\
	/* gather GHASH in GH1L (low) and GH1H (high) */\
	.ifc DO_REDUCTION, first_time;			\
		vpternlogq	$0x96, ZT12, ZT8, ZT7;	\
		vpxorq		ZT11, ZT7, TO_REDUCE_M; \
		vpxorq		ZT9, ZT5, TO_REDUCE_H;	\
		vpxorq		ZT10, ZT6, TO_REDUCE_L; \
	.endif;						\
	.ifc DO_REDUCTION, no_reduction;		\
		vpternlogq	$0x96, ZT12, ZT8, ZT7;	\
		vpternlogq	$0x96, ZT11, ZT7, TO_REDUCE_M;	\
		vpternlogq	$0x96, ZT9, ZT5, TO_REDUCE_H;	\
		vpternlogq	$0x96, ZT10, ZT6, TO_REDUCE_L;	\
	.endif;						\
	.ifc DO_REDUCTION, final_reduction;		\
		/*					\
		 * phase 1: add mid products together,	\
		 * load polynomial constant for reduction	\
		 */					\
		vpternlogq	$0x96, ZT12, ZT8, ZT7;	\
		vpternlogq	$0x96, ZT11, TO_REDUCE_M, ZT7;	\
		vpsrldq		$8, ZT7, ZT11;		\
		vpslldq		$8, ZT7, ZT7;		\
		vmovdqa64	POLY2(%rip), XWORD(ZT12);	\
	.endif;						\
	/* AES round 7 */				\
	vaesenc		ZT18, ZT1, ZT1;			\
	vaesenc		ZT18, ZT2, ZT2;			\
	vaesenc		ZT18, ZT3, ZT3;			\
	vaesenc		ZT18, ZT4, ZT4;			\
	vbroadcastf64x2 16*9(GDATA), ZT18;		\
	/* Add mid product to high and low */		\
	.ifc DO_REDUCTION, final_reduction;		\
		vpternlogq	$0x96, ZT11, ZT9, ZT5;	\
		vpxorq		TO_REDUCE_H, ZT5, ZT5;	\
		vpternlogq	$0x96, ZT7, ZT10, ZT6;	\
		vpxorq		TO_REDUCE_L, ZT6, ZT6;	\
	.endif;						\
	/* AES round 8 */				\
	vaesenc		ZT17, ZT1, ZT1;			\
	vaesenc		ZT17, ZT2, ZT2;			\
	vaesenc		ZT17, ZT3, ZT3;			\
	vaesenc		ZT17, ZT4, ZT4;			\
	vbroadcastf64x2 16*10(GDATA), ZT17;		\
	/* horizontal xor of low and high 4x128 */	\
	.ifc DO_REDUCTION, final_reduction;		\
		VHPXORI4x128(ZT5, ZT9)			\
		VHPXORI4x128(ZT6, ZT10)			\
	.endif;						\
	/* AES round 9 */				\
	vaesenc		ZT18, ZT1, ZT1;			\
	vaesenc		ZT18, ZT2, ZT2;			\
	vaesenc		ZT18, ZT3, ZT3;			\
	vaesenc		ZT18, ZT4, ZT4;			\
	.if NROUNDS >= 11;				\
		vbroadcastf64x2 16*11(GDATA), ZT18;	\
	.endif;						\
	/* First phase of reduction */			\
	.ifc DO_REDUCTION, final_reduction;		\
		vpclmulqdq	$0x01, XWORD(ZT6), XWORD(ZT12), XWORD(ZT10);	\
		vpslldq		$8, XWORD(ZT10), XWORD(ZT10);		\
		vpxorq		XWORD(ZT10), XWORD(ZT6), XWORD(ZT10);	\
	.endif;						\
	/* AES128 done. Continue for AES192 & AES256*/	\
	.if NROUNDS >= 11;				\
		vaesenc		ZT17, ZT1, ZT1;		\
		vaesenc		ZT17, ZT2, ZT2;		\
		vaesenc		ZT17, ZT3, ZT3;		\
		vaesenc		ZT17, ZT4, ZT4;		\
		vbroadcastf64x2 16*12(GDATA), ZT17;	\
		vaesenc		ZT18, ZT1, ZT1;		\
		vaesenc		ZT18, ZT2, ZT2;		\
		vaesenc		ZT18, ZT3, ZT3;		\
		vaesenc		ZT18, ZT4, ZT4;		\
		.if NROUNDS == 13;			\
			vbroadcastf64x2 16*13(GDATA), ZT18;	\
			vaesenc		ZT17, ZT1, ZT1;	\
			vaesenc		ZT17, ZT2, ZT2;	\
			vaesenc		ZT17, ZT3, ZT3; \
			vaesenc		ZT17, ZT4, ZT4; \
			vbroadcastf64x2 16*14(GDATA), ZT17;	\
			vaesenc		ZT18, ZT1, ZT1;		\
			vaesenc		ZT18, ZT2, ZT2;		\
			vaesenc		ZT18, ZT3, ZT3;		\
			vaesenc		ZT18, ZT4, ZT4;		\
		.endif;						\
	.endif;							\
	/* second phase of the reduction */			\
	.ifc DO_REDUCTION, final_reduction;					\
		vpclmulqdq	$0, XWORD(ZT10), XWORD(ZT12), XWORD(ZT9);	\
		vpsrldq		$4, XWORD(ZT9), XWORD(ZT9);			\
		vpclmulqdq	$0x10, XWORD(ZT10), XWORD(ZT12), XWORD(ZT11);	\
		vpslldq		$4, XWORD(ZT11), XWORD(ZT11);			\
		vpternlogq	$0x96, XWORD(ZT9), XWORD(ZT11), XWORD(ZT5);	\
	.endif;									\
	/* Last AES round */			\
	vaesenclast	    ZT17, ZT1, ZT1;	\
	vaesenclast	    ZT17, ZT2, ZT2;	\
	vaesenclast	    ZT17, ZT3, ZT3;	\
	vaesenclast	    ZT17, ZT4, ZT4;	\
	/* XOR against plain/cipher text */	\
	vpxorq		    ZT13, ZT1, ZT1;	\
	vpxorq		 ZT14, ZT2, ZT2;	\
	vpxorq	       ZT15, ZT3, ZT3;		\
	vpxorq	       ZT16, ZT4, ZT4;		\
	/* Store cipher/plain text */		\
	vmovdqu8	ZT1, DATA_DISPL(DATA_OFFSET, CYPH_PLAIN_OUT);		\
	vmovdqu8	ZT2, 64*1 + DATA_DISPL(DATA_OFFSET, CYPH_PLAIN_OUT);	\
	vmovdqu8	ZT3, 64*2 + DATA_DISPL(DATA_OFFSET, CYPH_PLAIN_OUT);	\
	vmovdqu8	ZT4, 64*3 + DATA_DISPL(DATA_OFFSET, CYPH_PLAIN_OUT);	\
	/* Shuffle cipher text blocks for GHASH computation */	\
	.ifc ENC_DEC, ENC;				\
		vpshufb		SHFMSK, ZT1, ZT1;	\
		vpshufb		SHFMSK, ZT2, ZT2;	\
		vpshufb		SHFMSK, ZT3, ZT3;	\
		vpshufb		SHFMSK, ZT4, ZT4;	\
	.else;						\
		vpshufb		SHFMSK, ZT13, ZT1;	\
		vpshufb		SHFMSK, ZT14, ZT2;	\
		vpshufb		SHFMSK, ZT15, ZT3;	\
		vpshufb		SHFMSK, ZT16, ZT4;	\
	.endif;						\
	/* Store shuffled cipher text for ghashing */	\
	vmovdqa64 ZT1, 0*64 + AESOUT_BLK_OFFSET(%rsp);	\
	vmovdqa64 ZT2, 1*64 + AESOUT_BLK_OFFSET(%rsp);	\
	vmovdqa64 ZT3, 2*64 + AESOUT_BLK_OFFSET(%rsp);	\
	vmovdqa64 ZT4, 3*64 + AESOUT_BLK_OFFSET(%rsp);

/* Encrypt the initial N x 16 blocks */
#define INITIAL_BLOCKS_Nx16(IN, OUT, KP, CTX, DATA_OFFSET, GHASH, CTR, CTR_CHECK, T0, T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, T11, T12, T13, T14, T15, T16, T17, T18, T19, T20, T21, T22, GH, GL, GM, ADDBE_4x4, ADDBE_1234, SHUF_MASK, ENC_DEC, NBLOCKS, DEPTH_BLK, NROUNDS) \
	/* set up CTR_CHECK */				\
	vmovd		XWORD(CTR), DWORD(CTR_CHECK);	\
	and		$255, DWORD(CTR_CHECK);		\
	/* In LE format after init, convert to BE */	\
	vshufi64x2	$0, CTR, CTR, CTR;		\
	vpshufb		SHUF_MASK, CTR, CTR;		\
	/* first 16 blocks - just cipher */		\
	INITIAL_BLOCKS_16(IN, OUT, KP, DATA_OFFSET, GHASH, CTR, CTR_CHECK, ADDBE_4x4, ADDBE_1234, T0, T1, T2, T3, T4, T5, T6, T7, T8, SHUF_MASK, ENC_DEC, STACK_LOCAL_OFFSET, 0, NROUNDS)	\
	INITIAL_BLOCKS_16(IN, OUT, KP, DATA_OFFSET, no_ghash, CTR, CTR_CHECK, ADDBE_4x4, ADDBE_1234, T0, T1, T2, T3, T4, T5, T6, T7, T8, SHUF_MASK, ENC_DEC, STACK_LOCAL_OFFSET + 256, 256, NROUNDS)	\
	/* GHASH + AES follows */			\
	GHASH_16_ENCRYPT_16_PARALLEL(KP, CTX, OUT, IN, DATA_OFFSET, CTR, CTR_CHECK, HashSubKey, STACK_LOCAL_OFFSET + 512, STACK_LOCAL_OFFSET, SHUF_MASK, T0, T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, T11, T12, T13, T14, T15, T16, T17, T18, T19, T20, T21, T22, ADDBE_4x4, ADDBE_1234, GL, GH, GM, first_time, ENC_DEC, 512, no_ghash_in, NROUNDS)	\
	add		$(48 * 16), DATA_OFFSET;

/* Encrypt & ghash multiples of 16 blocks */
#define GHASH_ENCRYPT_Nx16_PARALLEL(IN, OUT, GDATA_KEY, GCTX, DATA_OFFSET, CTR_BE, SHFMSK, ZT0, ZT1, ZT2, ZT3, ZT4, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, ZT13, ZT14, ZT15, ZT16, ZT17, ZT18, ZT19, ZT20, ZT21, ZT22, GTH, GTL, GTM, ADDBE_4x4, ADDBE_1234, GHASH, ENC_DEC, NUM_BLOCKS, DEPTH_BLK, CTR_CHECK, NROUNDS)	\
	GHASH_16_ENCRYPT_16_PARALLEL(GDATA_KEY, GCTX, OUT, IN, DATA_OFFSET, CTR_BE, CTR_CHECK, HashSubKey + HashKey_32, STACK_LOCAL_OFFSET, STACK_LOCAL_OFFSET + (16 * 16), SHFMSK, ZT0, ZT1, ZT2, ZT3, ZT4, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, ZT13, ZT14, ZT15, ZT16, ZT17, ZT18, ZT19, ZT20, ZT21, ZT22, ADDBE_4x4, ADDBE_1234, GTL, GTH, GTM, no_reduction, ENC_DEC, 0, no_ghash_in, NROUNDS)	\
	GHASH_16_ENCRYPT_16_PARALLEL(GDATA_KEY, GCTX, OUT, IN, DATA_OFFSET, CTR_BE, CTR_CHECK, HashSubKey + HashKey_16, STACK_LOCAL_OFFSET + 256, STACK_LOCAL_OFFSET + (16 * 16) + 256, SHFMSK, ZT0, ZT1, ZT2, ZT3, ZT4, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, ZT13, ZT14, ZT15, ZT16, ZT17, ZT18, ZT19, ZT20, ZT21, ZT22, ADDBE_4x4, ADDBE_1234, GTL, GTH, GTM, final_reduction, ENC_DEC, 256, no_ghash_in, NROUNDS)	\
	vmovdqa64	ZT4, GHASH;	\
	GHASH_16_ENCRYPT_16_PARALLEL(GDATA_KEY, GCTX, OUT, IN, DATA_OFFSET, CTR_BE, CTR_CHECK, HashSubKey + HashKey_48, STACK_LOCAL_OFFSET + 512, STACK_LOCAL_OFFSET, SHFMSK, ZT0, ZT1, ZT2, ZT3, ZT4, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, ZT13, ZT14, ZT15, ZT16, ZT17, ZT18, ZT19, ZT20, ZT21, ZT22, ADDBE_4x4, ADDBE_1234, GTL, GTH, GTM, first_time, ENC_DEC, 512, GHASH, NROUNDS)	\
	add	$(NUM_BLOCKS * 16), DATA_OFFSET;	\

/* GHASH the last 16 blocks of cipher text */
#define GHASH_LAST_Nx16(KP, GHASH, T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, T11, T12, T13, T14, T15, T16, GH, GL,GM, LOOP_BLK, DEPTH_BLK)	     \
	/* load cipher blocks and ghash keys */		\
	vmovdqa64	(((LOOP_BLK - DEPTH_BLK) * 16) + STACK_LOCAL_OFFSET)(%rsp), T13;	\
	vmovdqa64	(((LOOP_BLK - DEPTH_BLK) * 16) + 64 + STACK_LOCAL_OFFSET)(%rsp), T14;	\
	vmovdqu64	HashKey_32 + HashSubKey(KP), T15;	\
	vmovdqu64	HashKey_32 + 64 + HashSubKey(KP), T16;	\
	/* ghash blocks 0-3 */				\
	vpclmulqdq	$0x11, T15, T13, T1;		\
	vpclmulqdq	$0x00, T15, T13, T2;		\
	vpclmulqdq	$0x01, T15, T13, T3;		\
	vpclmulqdq	$0x10, T15, T13, T4;		\
	/* ghash blocks 4-7 */				\
	vpclmulqdq	$0x11, T16, T14, T5;		\
	vpclmulqdq	$0x00, T16, T14, T6;		\
	vpclmulqdq	$0x01, T16, T14, T7;		\
	vpclmulqdq	$0x10, T16, T14, T8;		\
	vpternlogq	$0x96, GH, T5, T1;		\
	vpternlogq	$0x96, GL, T6, T2;		\
	vpternlogq	$0x96, GM, T7, T3;		\
	vpxorq		T8, T4, T4;			\
	\
.set i, 0;						\
.rept 3;						\
	/* Remaining blocks; load next 8 cipher blocks and corresponding ghash keys */			\
	vmovdqa64	(((LOOP_BLK - DEPTH_BLK) * 16) + STACK_LOCAL_OFFSET + 128)(%rsp), T13;		\
	vmovdqa64	(((LOOP_BLK - DEPTH_BLK) * 16) + 64 + STACK_LOCAL_OFFSET + 128)(%rsp), T14;	\
	vmovdqu64	HashKey_32 + 128 + i*128 + HashSubKey(KP), T15;	\
	vmovdqu64	HashKey_32 + 64 + 128 + i*128 + HashSubKey(KP), T16;	\
	/* ghash blocks 0-3 */				\
	vpclmulqdq	$0x11, T15, T13, T5;		\
	vpclmulqdq	$0x00, T15, T13, T6;		\
	vpclmulqdq	$0x01, T15, T13, T7;		\
	vpclmulqdq	$0x10, T15, T13, T8;		\
	/* ghash blocks 4-7 */				\
	vpclmulqdq	$0x11, T16, T14, T9;		\
	vpclmulqdq	$0x00, T16, T14, T10;		\
	vpclmulqdq	$0x01, T16, T14, T11;		\
	vpclmulqdq	$0x10, T16, T14, T12;		\
	/* update sums */				\
	vpternlogq	$0x96, T9, T5, T1;		\
	vpternlogq	$0x96, T10, T6, T2;		\
	vpternlogq	$0x96, T11, T7, T3;		\
	vpternlogq	$0x96, T12, T8, T4;		\
	.set		i, i+1;				\
.endr;							\
	vpxorq		T4, T3, T3;			\
	vpsrldq		$8, T3, T7;			\
	vpslldq		$8, T3, T8;			\
	vpxorq		T7, T1, T1;			\
	vpxorq		T8, T2, T2;			\
	\
	/* add TH and TL 128-bit words horizontally */	\
	VHPXORI4x128(T1, T11)				\
	VHPXORI4x128(T2, T12)				\
	\
	/* Reduction */					\
	vmovdqa64	POLY2(%rip), T15;		\
	VCLMUL_REDUCE(GHASH, T15, T1, T2, T3, T4);

/*
 * INITIAL_BLOCKS_PARTIAL macro with support for a partial final block.
 * It may look similar to INITIAL_BLOCKS but its usage is different:
 * - first encrypts/decrypts and then ghash these blocks
 * - Small packets or left over data chunks (<256 bytes)
 * - Remaining data chunks below 256 bytes (multi buffer code)
 * num_initial_blocks is expected to include the partial final block
 * in the count.
 */
#define INITIAL_BLOCKS_PARTIAL(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, DATA_OFFSET, num_initial_blocks, CTR, HASH_IN_OUT, ENC_DEC, ZT0, ZT1, ZT2, ZT3, ZT4, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, ZT13, ZT14, ZT15, ZT16, ZT17, ZT18, ZT19, ZT20, ZT21, ZT22, IA0, IA1, MASKREG, SHUFMASK, NROUNDS) \
	/* Copy ghash to temp reg */					\
	vmovdqa64	HASH_IN_OUT, XWORD(ZT2);			\
	/* prepare AES counter blocks */				\
.if num_initial_blocks == 1;						\
	vpaddd		ONE(%rip), CTR, XWORD(ZT3);			\
.elseif num_initial_blocks == 2;					\
	vshufi64x2	$0, YWORD(CTR), YWORD(CTR), YWORD(ZT3);		\
	vpaddd		ddq_add_1234(%rip), YWORD(ZT3), YWORD(ZT3);	\
.else;									\
	vshufi64x2	$0, ZWORD(CTR), ZWORD(CTR), ZWORD(CTR);		\
	vpaddd		ddq_add_1234(%rip), ZWORD(CTR), ZT3;		\
.if num_initial_blocks > 4;						\
	vpaddd		ddq_add_5678(%rip), ZWORD(CTR), ZT4;		\
.endif;									\
.if num_initial_blocks > 8;						\
	vpaddd		ddq_add_8888(%rip), ZT3, ZT8;			\
.endif;									\
.if num_initial_blocks > 12;						\
	vpaddd		ddq_add_8888(%rip), ZT4, ZT9;			\
.endif;									\
.endif;									\
	/* Get load/store mask */					\
	lea		byte64_len_to_mask_table(%rip), IA0;		\
	mov		LENGTH, IA1;					\
.if num_initial_blocks > 12;						\
	sub		$(3 * 64), IA1;					\
.elseif num_initial_blocks > 8;						\
	sub		$(2 * 64), IA1;					\
.elseif num_initial_blocks > 4;						\
	sub		$64, IA1;					\
.endif;									\
	kmovq		(IA0, IA1, 8), MASKREG;				\
	/* Extract new counter value. Shuffle counters for AES rounds */\
.if num_initial_blocks <= 4;						\
	vextracti32x4	$(num_initial_blocks - 1), ZT3, CTR;		\
.elseif num_initial_blocks <= 8;					\
	vextracti32x4	$(num_initial_blocks - 5), ZT4, CTR;		\
.elseif num_initial_blocks <= 12;					\
	vextracti32x4	$(num_initial_blocks - 9), ZT8, CTR;		\
.else;									\
	vextracti32x4	$(num_initial_blocks - 13), ZT9, CTR;		\
.endif;									\
	ZMM_OPCODE3_DSTR_SRC1R_SRC2R_BLOCKS_0_16(num_initial_blocks, vpshufb, ZT3, ZT4, ZT8, ZT9, ZT3, ZT4, ZT8, ZT9, SHUFMASK, SHUFMASK, SHUFMASK, SHUFMASK)	\
	/* Load plain/cipher text */					\
	ZMM_LOAD_MASKED_BLOCKS_0_16(num_initial_blocks, PLAIN_CYPH_IN, DATA_OFFSET, ZT5, ZT6, ZT10, ZT11, MASKREG)	\
	/* AES rounds and XOR with plain/cipher text */			\
.set i, 0;								\
.rept 11;								\
	vbroadcastf64x2 16*i(GDATA_KEY), ZT1;				\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT3, ZT4, ZT8, ZT9, ZT1, i, ZT5, ZT6, ZT10, ZT11, num_initial_blocks, NROUNDS)	\
	.set i, i+1;							\
.endr;									\
.if NROUNDS > 9;							\
.rept 2;								\
	vbroadcastf64x2 16*i(GDATA_KEY), ZT1;				\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT3, ZT4, ZT8, ZT9, ZT1, i, ZT5, ZT6, ZT10, ZT11, num_initial_blocks, NROUNDS)	\
	.set i, i+1;							\
.endr;									\
.endif;									\
.if NROUNDS > 11;							\
.rept 2;								\
	vbroadcastf64x2 16*i(GDATA_KEY), ZT1;				\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT3, ZT4, ZT8, ZT9, ZT1, i, ZT5, ZT6, ZT10, ZT11, num_initial_blocks, NROUNDS)	\
	.set i, i+1;							\
.endr;									\
.endif;									\
/*
 * Retrieve the last cipher counter block (Partially XOR'ed with text	\
 * This is needed for partial block case				\
 */									\
.if num_initial_blocks <= 4;						\
	vextracti32x4	$(num_initial_blocks - 1), ZT3, XWORD(ZT1);	\
.elseif num_initial_blocks <= 8;					\
	vextracti32x4	$(num_initial_blocks - 5), ZT4, XWORD(ZT1);	\
.elseif num_initial_blocks <= 12;					\
	vextracti32x4	$(num_initial_blocks - 9), ZT8, XWORD(ZT1);	\
.else;									\
	vextracti32x4	$(num_initial_blocks - 13), ZT9, XWORD(ZT1);	\
.endif;									\
	/* Write cipher/plain text back to output */			\
	ZMM_STORE_MASKED_BLOCKS_0_16(num_initial_blocks, CYPH_PLAIN_OUT,DATA_OFFSET, ZT3, ZT4, ZT8, ZT9, MASKREG)	\
	/* Zero bytes outside the mask before hashing */		\
.if num_initial_blocks <= 4;						\
	vmovdqu8	ZT3, ZT3{MASKREG}{z};				\
.elseif num_initial_blocks <= 8;					\
	vmovdqu8	ZT4, ZT4{MASKREG}{z};				\
.elseif num_initial_blocks <= 12;					\
	vmovdqu8	ZT8, ZT8{MASKREG}{z};				\
.else;									\
	vmovdqu8	ZT9, ZT9{MASKREG}{z};				\
.endif;									\
/* Shuffle the cipher text blocks for hashing part */			\
.ifc  ENC_DEC, DEC;							\
	ZMM_OPCODE3_DSTR_SRC1R_SRC2R_BLOCKS_0_16(num_initial_blocks, vpshufb,	\
			ZT5, ZT6, ZT10, ZT11,				\
			ZT5, ZT6, ZT10, ZT11,				\
			SHUFMASK, SHUFMASK, SHUFMASK, SHUFMASK)		\
.else;									\
	 ZMM_OPCODE3_DSTR_SRC1R_SRC2R_BLOCKS_0_16(num_initial_blocks, vpshufb,	\
			ZT5, ZT6, ZT10, ZT11,				\
			ZT3, ZT4, ZT8, ZT9,				\
			SHUFMASK, SHUFMASK, SHUFMASK, SHUFMASK)		\
.endif;									\
/* Extract the last block for partial cases */				\
.if num_initial_blocks <= 4;						\
	vextracti32x4	$(num_initial_blocks - 1), ZT5, XWORD(ZT7);	\
.elseif num_initial_blocks <= 8;					\
	vextracti32x4	$(num_initial_blocks - 5), ZT6, XWORD(ZT7);	\
.elseif num_initial_blocks <= 12;					\
	vextracti32x4	$(num_initial_blocks - 9), ZT10, XWORD(ZT7);	\
.else;									\
	vextracti32x4	$(num_initial_blocks - 13), ZT11, XWORD(ZT7);	\
.endif;									\
/* Hash all but the last block of data */				\
.if num_initial_blocks > 1;						\
	add	$(16 * (num_initial_blocks - 1)), DATA_OFFSET;		\
	sub	$(16 * (num_initial_blocks - 1)), LENGTH;		\
.endif;									\
.if num_initial_blocks < 16;						\
	cmp	$16, LENGTH;						\
	jl	25f;							\
	/* Handle a full length final blk; encrypt & hash all blocks */	\
	sub	$16, LENGTH;						\
	add	$16, DATA_OFFSET;					\
	mov	LENGTH, PBlockLen(GDATA_CTX);				\
	/* Hash all of the data */					\
	GHASH_1_TO_16(GDATA_CTX, 96, HASH_IN_OUT, ZT12, ZT13, ZT14, ZT15, ZT16, ZT17, ZT18, ZT19, ZT20, ZT2, ZT5, ZT6, ZT10, ZT11, num_initial_blocks, 1, single_call, null, null, null, null, null, null)	\
	jmp	26f;							\
.endif;									\
25:;									\
	/* Handle ghash for a <16B final block */			\
	mov	LENGTH, PBlockLen(GDATA_CTX);				\
	vmovdqu64	XWORD(ZT1), PBlockEncKey(GDATA_CTX);		\
.if num_initial_blocks > 1;						\
	GHASH_1_TO_16(GDATA_CTX, 96, HASH_IN_OUT, ZT12, ZT13, ZT14, ZT15, ZT16, ZT17, ZT18, ZT19, ZT20, ZT2, ZT5, ZT6, ZT10, ZT11, num_initial_blocks - 1, 0, single_call, null, null, null, null, null, null)	\
.else;									\
	vpxorq		XWORD(ZT7), XWORD(ZT2), HASH_IN_OUT;	\
	jmp		27f;						\
.endif;									\
/* After GHASH reduction */						\
26:;									\
.if num_initial_blocks > 1;						\
	.if num_initial_blocks != 16;					\
		or	LENGTH, LENGTH;					\
		je	27f;						\
	.endif;								\
	vpxorq	    XWORD(ZT7), HASH_IN_OUT, HASH_IN_OUT;		\
	/* Final hash is now in HASH_IN_OUT */				\
.endif;									\
27:;

/* Cipher and ghash of payloads shorter than 256 bytes */
#define GCM_ENC_DEC_SMALL(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, PLAIN_CYPH_LEN, ENC_DEC, DATA_OFFSET, LENGTH, NUM_BLOCKS, CTR, HASH_IN_OUT, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, ZTMP9, ZTMP10, ZTMP11, ZTMP12, ZTMP13, ZTMP14, ZTMP15, ZTMP16, ZTMP17, ZTMP18, ZTMP19, ZTMP20, ZTMP21, ZTMP22, IA0, IA1, MASKREG, SHUFMASK, NROUNDS)	\
	cmp	$8, NUM_BLOCKS;	\
	je	58f;		\
	jl	68f;		\
	cmp	$12, NUM_BLOCKS;\
	je	62f;		\
	jl	67f;		\
	cmp	$16, NUM_BLOCKS;\
	je	66f;		\
	cmp	$15, NUM_BLOCKS;\
	je	65f;		\
	cmp	$14, NUM_BLOCKS;\
	je	64f;		\
	jmp	63f;		\
67:;				\
	cmp	$11, NUM_BLOCKS;\
	je	61f;		\
	cmp	$10, NUM_BLOCKS;\
	je	60f; 		\
	jmp	59f;		\
68:;				\
	cmp	$4, NUM_BLOCKS;	\
	je	54f;		\
	jl	69f;		\
	cmp	$7, NUM_BLOCKS;	\
	je	57f;		\
	cmp	$6, NUM_BLOCKS;	\
	je	56f;		\
	jmp	55f;		\
69:;				\
	cmp	$3, NUM_BLOCKS;	\
	je	53f;		\
	cmp	$2, NUM_BLOCKS;	\
	je	52f;		\
51:;				\
	INITIAL_BLOCKS_PARTIAL(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, DATA_OFFSET, 1, CTR, HASH_IN_OUT, ENC_DEC, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, ZTMP9, ZTMP10, ZTMP11, ZTMP12, ZTMP13, ZTMP14, ZTMP15, ZTMP16, ZTMP17, ZTMP18, ZTMP19, ZTMP20, ZTMP21, ZTMP22, IA0, IA1, MASKREG, SHUFMASK, NROUNDS)	\
	jmp	70f;		\
52:;				\
	INITIAL_BLOCKS_PARTIAL(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, DATA_OFFSET, 2, CTR, HASH_IN_OUT, ENC_DEC, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, ZTMP9, ZTMP10, ZTMP11, ZTMP12, ZTMP13, ZTMP14, ZTMP15, ZTMP16, ZTMP17, ZTMP18, ZTMP19, ZTMP20, ZTMP21, ZTMP22, IA0, IA1, MASKREG, SHUFMASK, NROUNDS)	\
	jmp	70f;		\
53:;				\
	INITIAL_BLOCKS_PARTIAL(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, DATA_OFFSET, 3, CTR, HASH_IN_OUT, ENC_DEC, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, ZTMP9, ZTMP10, ZTMP11, ZTMP12, ZTMP13, ZTMP14, ZTMP15, ZTMP16, ZTMP17, ZTMP18, ZTMP19, ZTMP20, ZTMP21, ZTMP22, IA0, IA1, MASKREG, SHUFMASK, NROUNDS)	\
	jmp	70f;		\
54:;				\
	INITIAL_BLOCKS_PARTIAL(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, DATA_OFFSET, 4, CTR, HASH_IN_OUT, ENC_DEC, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, ZTMP9, ZTMP10, ZTMP11, ZTMP12, ZTMP13, ZTMP14, ZTMP15, ZTMP16, ZTMP17, ZTMP18, ZTMP19, ZTMP20, ZTMP21, ZTMP22, IA0, IA1, MASKREG, SHUFMASK, NROUNDS)	\
	jmp	70f;		\
55:;				\
	INITIAL_BLOCKS_PARTIAL(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, DATA_OFFSET, 5, CTR, HASH_IN_OUT, ENC_DEC, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, ZTMP9, ZTMP10, ZTMP11, ZTMP12, ZTMP13, ZTMP14, ZTMP15, ZTMP16, ZTMP17, ZTMP18, ZTMP19, ZTMP20, ZTMP21, ZTMP22, IA0, IA1, MASKREG, SHUFMASK, NROUNDS)	\
	jmp	70f;		\
56:;				\
	INITIAL_BLOCKS_PARTIAL(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, DATA_OFFSET, 6, CTR, HASH_IN_OUT, ENC_DEC, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, ZTMP9, ZTMP10, ZTMP11, ZTMP12, ZTMP13, ZTMP14, ZTMP15, ZTMP16, ZTMP17, ZTMP18, ZTMP19, ZTMP20, ZTMP21, ZTMP22, IA0, IA1, MASKREG, SHUFMASK, NROUNDS)	\
	jmp	70f;		\
57:;				\
	INITIAL_BLOCKS_PARTIAL(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, DATA_OFFSET, 7, CTR, HASH_IN_OUT, ENC_DEC, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, ZTMP9, ZTMP10, ZTMP11, ZTMP12, ZTMP13, ZTMP14, ZTMP15, ZTMP16, ZTMP17, ZTMP18, ZTMP19, ZTMP20, ZTMP21, ZTMP22, IA0, IA1, MASKREG, SHUFMASK, NROUNDS)	\
	jmp	70f;		\
58:;				\
	INITIAL_BLOCKS_PARTIAL(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, DATA_OFFSET, 8, CTR, HASH_IN_OUT, ENC_DEC, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, ZTMP9, ZTMP10, ZTMP11, ZTMP12, ZTMP13, ZTMP14, ZTMP15, ZTMP16, ZTMP17, ZTMP18, ZTMP19, ZTMP20, ZTMP21, ZTMP22, IA0, IA1, MASKREG, SHUFMASK, NROUNDS)	\
	jmp	70f;		\
59:;				\
	INITIAL_BLOCKS_PARTIAL(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, DATA_OFFSET, 9, CTR, HASH_IN_OUT, ENC_DEC, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, ZTMP9, ZTMP10, ZTMP11, ZTMP12, ZTMP13, ZTMP14, ZTMP15, ZTMP16, ZTMP17, ZTMP18, ZTMP19, ZTMP20, ZTMP21, ZTMP22, IA0, IA1, MASKREG, SHUFMASK, NROUNDS)	\
	jmp	70f;		\
60:;				\
	INITIAL_BLOCKS_PARTIAL(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, DATA_OFFSET, 10, CTR, HASH_IN_OUT, ENC_DEC, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, ZTMP9, ZTMP10, ZTMP11, ZTMP12, ZTMP13, ZTMP14, ZTMP15, ZTMP16, ZTMP17, ZTMP18, ZTMP19, ZTMP20, ZTMP21, ZTMP22, IA0, IA1, MASKREG, SHUFMASK, NROUNDS)	\
	jmp	70f;		\
61:;				\
	INITIAL_BLOCKS_PARTIAL(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, DATA_OFFSET, 11, CTR, HASH_IN_OUT, ENC_DEC, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, ZTMP9, ZTMP10, ZTMP11, ZTMP12, ZTMP13, ZTMP14, ZTMP15, ZTMP16, ZTMP17, ZTMP18, ZTMP19, ZTMP20, ZTMP21, ZTMP22, IA0, IA1, MASKREG, SHUFMASK, NROUNDS)	\
	jmp	70f;		\
62:;				\
	INITIAL_BLOCKS_PARTIAL(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, DATA_OFFSET, 12, CTR, HASH_IN_OUT, ENC_DEC, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, ZTMP9, ZTMP10, ZTMP11, ZTMP12, ZTMP13, ZTMP14, ZTMP15, ZTMP16, ZTMP17, ZTMP18, ZTMP19, ZTMP20, ZTMP21, ZTMP22, IA0, IA1, MASKREG, SHUFMASK, NROUNDS)	\
	jmp	70f;		\
63:;				\
	INITIAL_BLOCKS_PARTIAL(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, DATA_OFFSET, 13, CTR, HASH_IN_OUT, ENC_DEC, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, ZTMP9, ZTMP10, ZTMP11, ZTMP12, ZTMP13, ZTMP14, ZTMP15, ZTMP16, ZTMP17, ZTMP18, ZTMP19, ZTMP20, ZTMP21, ZTMP22, IA0, IA1, MASKREG, SHUFMASK, NROUNDS) \
	jmp	70f;		\
64:;				\
	INITIAL_BLOCKS_PARTIAL(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, DATA_OFFSET, 14, CTR, HASH_IN_OUT, ENC_DEC, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, ZTMP9, ZTMP10, ZTMP11, ZTMP12, ZTMP13, ZTMP14, ZTMP15, ZTMP16, ZTMP17, ZTMP18, ZTMP19, ZTMP20, ZTMP21, ZTMP22, IA0, IA1, MASKREG, SHUFMASK, NROUNDS) \
	jmp	70f;		\
65:;				\
	INITIAL_BLOCKS_PARTIAL(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, DATA_OFFSET, 15, CTR, HASH_IN_OUT, ENC_DEC, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, ZTMP9, ZTMP10, ZTMP11, ZTMP12, ZTMP13, ZTMP14, ZTMP15, ZTMP16, ZTMP17, ZTMP18, ZTMP19, ZTMP20, ZTMP21, ZTMP22, IA0, IA1, MASKREG, SHUFMASK, NROUNDS) \
	jmp	70f;		\
66:;				\
	INITIAL_BLOCKS_PARTIAL(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, DATA_OFFSET, 16, CTR, HASH_IN_OUT, ENC_DEC, ZTMP0, ZTMP1, ZTMP2, ZTMP3, ZTMP4, ZTMP5, ZTMP6, ZTMP7, ZTMP8, ZTMP9, ZTMP10, ZTMP11, ZTMP12, ZTMP13, ZTMP14, ZTMP15, ZTMP16, ZTMP17, ZTMP18, ZTMP19, ZTMP20, ZTMP21, ZTMP22, IA0, IA1, MASKREG, SHUFMASK, NROUNDS) \
70:;

/*
 * This macro is used to "warm-up" pipeline for GHASH_8_ENCRYPT_8_PARALLEL
 * macro code. It is called only for data lenghts 128 and above.
 * The flow is as follows:
 * - encrypt the initial num_initial_blocks blocks (can be 0)
 * - encrypt the next 8 blocks and stitch with GHASH for the first num_initial_blocks
 * - the last 8th block can be partial (lengths between 129 and 239)
 * - partial block ciphering is handled within this macro
 * - top bytes of such block are cleared for the subsequent GHASH calculations
 * - PBlockEncKey needs to be setup
 * - top bytes of the block need to include encrypted counter block so that
 *   when handling partial block case text is read and XOR'ed against it.
 *   This needs to be in un-shuffled format.
 */
#define INITIAL_BLOCKS(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, LENGTH, DATA_OFFSET, num_initial_blocks, CTR, AAD_HASH, ZT1, ZT2, ZT3, ZT4, ZT5, ZT6, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, IA0, IA1, ENC_DEC, MASKREG, SHUFMASK, PARTIAL_PRESENT, NROUNDS)	\
.set partial_block_possible, 1;							\
.ifc PARTIAL_PRESENT, no_partial_block;						\
	.set partial_block_possible, 0;						\
.endif;										\
.if num_initial_blocks > 0;							\
	/* Prepare AES counter blocks */					\
	.if num_initial_blocks == 1;						\
		vpaddd		ONE(%rip), CTR, XWORD(ZT3);			\
	.elseif num_initial_blocks == 2;					\
		vshufi64x2	$0, YWORD(CTR), YWORD(CTR), YWORD(ZT3);		\
		vpaddd		ddq_add_1234(%rip), YWORD(ZT3), YWORD(ZT3);	\
	.else;									\
		vshufi64x2	$0, ZWORD(CTR), ZWORD(CTR), ZWORD(CTR);		\
		vpaddd		ddq_add_1234(%rip), ZWORD(CTR), ZT3;		\
		vpaddd		ddq_add_5678(%rip), ZWORD(CTR), ZT4;		\
	.endif;									\
	/* Extract new counter value; shuffle counters for AES rounds */	\
	.if num_initial_blocks <= 4;						\
		vextracti32x4   $(num_initial_blocks - 1), ZT3, CTR;		\
	.else;									\
		vextracti32x4   $(num_initial_blocks - 5), ZT4, CTR;		\
	.endif;									\
	ZMM_OPCODE3_DSTR_SRC1R_SRC2R_BLOCKS_0_16(num_initial_blocks, vpshufb, ZT3, ZT4, no_zmm, no_zmm, ZT3, ZT4, no_zmm, no_zmm, SHUFMASK, SHUFMASK, SHUFMASK, SHUFMASK)	\
	/* load plain/cipher text */						\
	ZMM_LOAD_BLOCKS_0_16(num_initial_blocks, PLAIN_CYPH_IN, DATA_OFFSET, ZT5, ZT6, no_zmm, no_zmm, NULL)	\
	/* AES rounds and XOR with plain/cipher text */				\
.set i, 0;									\
.rept 11;									\
	vbroadcastf64x2 16*i(GDATA_KEY), ZT1;					\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT3, ZT4, no_zmm, no_zmm, ZT1, i, ZT5, ZT6, no_zmm, no_zmm, num_initial_blocks, NROUNDS)	\
	.set i, i+1;								\
.endr;										\
.if NROUNDS > 9;								\
.rept 2;									\
	vbroadcastf64x2 16*i(GDATA_KEY), ZT1;					\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT3, ZT4, no_zmm, no_zmm, ZT1, i, ZT5, ZT6, no_zmm, no_zmm, num_initial_blocks, NROUNDS)	\
	.set i, i+1;								\
.endr;										\
.endif;										\
.if NROUNDS > 11;								\
.rept 2;									\
	vbroadcastf64x2 16*i(GDATA_KEY), ZT1;					\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT3, ZT4, no_zmm, no_zmm, ZT1, i, ZT5, ZT6, no_zmm, no_zmm, num_initial_blocks, NROUNDS)	\
	.set i, i+1;								\
.endr;										\
.endif;										\
	/* Write cipher/plain text back to output */				\
	ZMM_STORE_BLOCKS_0_16(num_initial_blocks, CYPH_PLAIN_OUT, DATA_OFFSET, ZT3, ZT4, no_zmm, no_zmm)	\
	/* Shuffle the cipher text blocks for hashing part */			\
	.ifc ENC_DEC, DEC;							\
	ZMM_OPCODE3_DSTR_SRC1R_SRC2R_BLOCKS_0_16(num_initial_blocks, vpshufb, ZT5, ZT6, no_zmm, no_zmm, ZT5, ZT6, no_zmm, no_zmm, SHUFMASK, SHUFMASK, SHUFMASK, SHUFMASK) \
	.else;									\
	ZMM_OPCODE3_DSTR_SRC1R_SRC2R_BLOCKS_0_16(num_initial_blocks, vpshufb, ZT5, ZT6, no_zmm, no_zmm, ZT3, ZT4, no_zmm, no_zmm, SHUFMASK, SHUFMASK, SHUFMASK, SHUFMASK) \
	.endif;									\
	/* Adjust data offset and length */					\
	sub		$(num_initial_blocks * 16), LENGTH;			\
	add		$(num_initial_blocks * 16), DATA_OFFSET;		\
.endif;										\
	/*									\
	 * Cipher of num_initial_blocks is done					\
	 * prepare counter blocks for the next 8 blocks (ZT3 & ZT4)		\
	 *   - save the last block in %%CTR					\
	 *   - shuffle the blocks for AES					\
	 *   - stitch encryption of new blocks with GHASHING previous blocks	\
	 */									\
	vshufi64x2	$0, ZWORD(CTR), ZWORD(CTR), ZWORD(CTR);			\
	vpaddd	    	ddq_add_1234(%rip), ZWORD(CTR), ZT3;			\
	vpaddd	    	ddq_add_5678(%rip), ZWORD(CTR), ZT4;			\
	vextracti32x4	$3, ZT4, CTR;						\
	vpshufb		SHUFMASK, ZT3, ZT3;					\
	vpshufb		SHUFMASK, ZT4, ZT4;					\
.if partial_block_possible != 0;						\
	/* get text load/store mask (assume full mask by default) */		\
	mov	0xffffffffffffffff, IA0;					\
	.if num_initial_blocks > 0;						\
		cmp	$128, LENGTH;						\
		jge	22f;							\
		mov	%rcx, IA1;						\
		mov	$128, %rcx;						\
		sub	LENGTH, %rcx;						\
		shr	cl, IA0;						\
		mov	IA1, %rcx;						\
22:;										\
	.endif;									\
	kmovq	IA0, MASKREG;							\
	/* load plain or cipher text */						\
	ZMM_LOAD_MASKED_BLOCKS_0_16(8, PLAIN_CYPH_IN, DATA_OFFSET, ZT1, ZT2, no_zmm, no_zmm, MASKREG)			\
.else;										\
	ZMM_LOAD_BLOCKS_0_16(8, PLAIN_CYPH_IN, DATA_OFFSET, ZT1, ZT2, no_zmm, no_zmm, NULL)				\
.endif;										\
.set aes_round, 0;								\
	vbroadcastf64x2 (aes_round * 16)(GDATA_KEY), ZT8;			\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT3, ZT4, no_zmm, no_zmm, ZT8, aes_round, ZT1, ZT2, no_zmm, no_zmm, 8, NROUNDS)	\
.set aes_round, aes_round + 1;							\
/* GHASH blocks 4-7 */			\
.if num_initial_blocks > 0;							\
	vpxorq	AAD_HASH, ZT5, ZT5;						\
	VCLMUL_1_TO_8_STEP1(GDATA_CTX, ZT6, ZT8, ZT9, ZT10, ZT11, ZT12, num_initial_blocks);				\
.endif;										\
/* 1/3 of AES rounds */		\
.rept ((NROUNDS + 1) / 3);							\
	vbroadcastf64x2 (aes_round * 16)(GDATA_KEY), ZT8;			\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT3, ZT4, no_zmm, no_zmm, ZT8, aes_round, ZT1, ZT2, no_zmm, no_zmm, 8, NROUNDS)	\
.set aes_round, aes_round + 1;							\
.endr;										\
/* GHASH blocks 0-3 and gather */	\
.if num_initial_blocks > 0;							\
	VCLMUL_1_TO_8_STEP2(GDATA_CTX, ZT6, ZT5, ZT7, ZT8, ZT9, ZT10, ZT11, ZT12, num_initial_blocks);			\
.endif;										\
/* 2/3 of AES rounds */			\
.rept ((NROUNDS + 1) / 3);							\
	vbroadcastf64x2		(aes_round * 16)(GDATA_KEY), ZT8;		\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT3, ZT4, no_zmm, no_zmm, ZT8, aes_round, ZT1, ZT2, no_zmm, no_zmm, 8, NROUNDS);	\
	.set aes_round, aes_round + 1;						\
.endr;										\
.if num_initial_blocks > 0;							\
	vmovdqu64	POLY2(%rip), XWORD(ZT8);				\
	VCLMUL_REDUCE(XWORD(AAD_HASH), XWORD(ZT8), XWORD(ZT6), XWORD(ZT5), XWORD(ZT7), XWORD(ZT9))			\
.endif;										\
/* 3/3 of AES rounds */			\
.rept (((NROUNDS + 1) / 3) + 2);						\
.if aes_round < (NROUNDS + 2);							\
	vbroadcastf64x2		(aes_round * 16)(GDATA_KEY), ZT8;		\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT3, ZT4, no_zmm, no_zmm, ZT8, aes_round, ZT1, ZT2, no_zmm, no_zmm, 8, NROUNDS)	\
.set aes_round, aes_round + 1;							\
.endif;										\
.endr;										\
.if partial_block_possible != 0;						\
	/* write cipher/plain text back to output */				\
	ZMM_STORE_MASKED_BLOCKS_0_16(8, CYPH_PLAIN_OUT, DATA_OFFSET, ZT3, ZT4, no_zmm, no_zmm, MASKREG)			\
	/* Check if there is a partial block */					\
	cmp		$128, LENGTH;						\
	jl		23f;							\
	/* Adjust offset and length */						\
	add		$128, DATA_OFFSET;					\
	sub		$128, LENGTH;						\
	jmp		24f;							\
23:;										\
	/* partial block case							\
	 * - save the partial block in unshuffled format			\
	 * - ZT4 is partially XOR'ed with data and top bytes contain		\
	 *   encrypted counter block only					\
	 * - save number of bytes process in the partial block			\
	 * - adjust offset and zero the length					\
	 * - clear top bytes of partial block for subsequent GHASH calculations	\
	 */									\
	vextracti32x4	$3, ZT4, PBlockEncKey(GDATA_CTX);			\
	add		LENGTH, DATA_OFFSET;					\
	sub		$(128 - 16), LENGTH;					\
	mov		LENGTH, PBlockLen(GDATA_CTX);				\
	xor		LENGTH, LENGTH;						\
	vmovdqu8	ZT4, ZT4{MASKREG}{z};					\
24:;										\
.else;										\
	ZMM_STORE_BLOCKS_0_16(8, CYPH_PLAIN_OUT, DATA_OFFSET, ZT3, ZT4, no_zmm, no_zmm)					\
	add		$128, DATA_OFFSET;					\
	sub		$128, LENGTH;						\
.endif;										\
	/* Shuffle AES result for GHASH */					\
.ifc  ENC_DEC, DEC;								\
	vpshufb		SHUFMASK, ZT1, ZT1;					\
	vpshufb		SHUFMASK, ZT2, ZT2;					\
.else;										\
	vpshufb		SHUFMASK, ZT3, ZT1;					\
	vpshufb		SHUFMASK, ZT4, ZT2;					\
.endif;										\
	/* Current hash value in AAD_HASH */					\
	vpxorq		AAD_HASH, ZT1, ZT1;

/*
 * Main GCM macro stitching cipher with GHASH
 * - operates on single stream
 * - encrypts 8 blocks at a time
 * - ghash the 8 previously encrypted ciphertext blocks
 * For partial block case, AES_PARTIAL_BLOCK on output contains encrypted the	\
 * counter block.
 */
#define GHASH_8_ENCRYPT_8_PARALLEL(GDATA, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, DATA_OFFSET, CTR1, CTR2, GHASHIN_AESOUT_B03, GHASHIN_AESOUT_B47, AES_PARTIAL_BLOCK, loop_idx, ENC_DEC, FULL_PARTIAL, IA0, IA1, LENGTH, GH4KEY, GH8KEY, SHFMSK, ZT1, ZT2, ZT3, ZT4, ZT5, ZT10, ZT11, ZT12, ZT13, ZT14, ZT15, ZT16, ZT17, MASKREG, DO_REDUCTION, TO_REDUCE_L, TO_REDUCE_H, TO_REDUCE_M, NROUNDS)	\
.ifc loop_idx, in_order;						\
	vpshufb		SHFMSK, CTR1, ZT1;				\
	vpshufb		SHFMSK, CTR2, ZT2;				\
.else;									\
	vmovdqa64	CTR1, ZT1;					\
	vmovdqa64	CTR2, ZT2;					\
.endif;									\
	/* stitch AES rounds with GHASH */				\
	/* AES round 0 */						\
	vbroadcastf64x2 16*0(GDATA), ZT3;				\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, no_zmm, no_zmm, ZT3, 0, ZT4, ZT5, no_zmm, no_zmm, 8, NROUNDS)	\
	/* GHASH 4 blocks */						\
	vpclmulqdq	$0x11, GH4KEY, GHASHIN_AESOUT_B47, ZT10;	\
	vpclmulqdq	$0x00, GH4KEY, GHASHIN_AESOUT_B47, ZT11;	\
	vpclmulqdq	$0x01, GH4KEY, GHASHIN_AESOUT_B47, ZT12;	\
	vpclmulqdq	$0x10, GH4KEY, GHASHIN_AESOUT_B47, ZT13;	\
	vbroadcastf64x2 16*1(GDATA), ZT3;				\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, no_zmm, no_zmm, ZT3, 1, ZT4, ZT5, no_zmm, no_zmm, 8, NROUNDS)	\
	vbroadcastf64x2 16*2(GDATA), ZT3;				\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, no_zmm, no_zmm, ZT3, 2, ZT4, ZT5, no_zmm, no_zmm, 8, NROUNDS)	\
	vbroadcastf64x2 16*3(GDATA), ZT3;				\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, no_zmm, no_zmm, ZT3, 3, ZT4, ZT5, no_zmm, no_zmm, 8, NROUNDS)	\
	vpclmulqdq	$0x10, GH8KEY, GHASHIN_AESOUT_B03, ZT16;	\
	vpclmulqdq	$0x01, GH8KEY, GHASHIN_AESOUT_B03, ZT17;	\
	vpclmulqdq	$0x11, GH8KEY, GHASHIN_AESOUT_B03, ZT14;	\
	vpclmulqdq	$0x00, GH8KEY, GHASHIN_AESOUT_B03, ZT15;	\
	vbroadcastf64x2 16*4(GDATA), ZT3;				\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, no_zmm, no_zmm, ZT3, 4, ZT4, ZT5, no_zmm, no_zmm, 8, NROUNDS)	\
	vbroadcastf64x2 16*5(GDATA), ZT3;				\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, no_zmm, no_zmm, ZT3, 5, ZT4, ZT5, no_zmm, no_zmm, 8, NROUNDS)	\
	vbroadcastf64x2 16*6(GDATA), ZT3;				\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, no_zmm, no_zmm, ZT3, 6, ZT4, ZT5, no_zmm, no_zmm, 8, NROUNDS)	\
.ifc DO_REDUCTION, no_reduction;					\
	vpternlogq	$0x96, ZT16, ZT13, ZT12;			\
	vpternlogq	$0x96, ZT17, ZT12, TO_REDUCE_M;			\
	vpternlogq	$0x96, ZT14, ZT10, TO_REDUCE_H;			\
	vpternlogq	$0x96, ZT15, ZT11, TO_REDUCE_L;			\
.endif;									\
.ifc DO_REDUCTION, do_reduction;					\
	vpternlogq	$0x96, ZT16, ZT13, ZT12;			\
	vpxorq		ZT17, ZT12, ZT12;				\
	vpsrldq		$8, ZT12, ZT16;					\
	vpslldq		$8, ZT12, ZT12;					\
.endif;									\
.ifc DO_REDUCTION, final_reduction;					\
	vpternlogq	$0x96, ZT16, ZT13, ZT12;			\
	vpternlogq	$0x96, ZT17, TO_REDUCE_M, ZT12;			\
	vpsrldq		$8, ZT12, ZT16;					\
	vpslldq		$8, ZT12, ZT12;					\
.endif;									\
	vbroadcastf64x2 16*7(GDATA), ZT3;				\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, no_zmm, no_zmm, ZT3, 7, ZT4, ZT5, no_zmm, no_zmm, 8, NROUNDS)	\
	vbroadcastf64x2 16*8(GDATA), ZT3;				\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, no_zmm, no_zmm, ZT3, 8, ZT4, ZT5, no_zmm, no_zmm, 8, NROUNDS)	\
.ifc DO_REDUCTION, final_reduction;					\
	vpternlogq	$0x96, ZT16, ZT14, ZT10;			\
	vpxorq		TO_REDUCE_H, ZT10;				\
	vpternlogq	$0x96, ZT12, ZT15, ZT11;			\
	vpxorq		TO_REDUCE_L, ZT11;				\
.endif;									\
.ifc DO_REDUCTION, do_reduction;					\
	vpternlogq	$0x96, ZT16, ZT14, ZT10;			\
	vpternlogq	$0x96, ZT12, ZT15, ZT11;			\
.endif;									\
.ifnc DO_REDUCTION, no_reduction;					\
	VHPXORI4x128(ZT14, ZT10);					\
	VHPXORI4x128(ZT15, ZT11);					\
.endif;									\
.if 9 < (NROUNDS + 1);							\
.if NROUNDS == 9;							\
	vbroadcastf64x2 16*9(GDATA), ZT3;				\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, no_zmm, no_zmm, ZT3, 9, ZT4, ZT5, no_zmm, no_zmm, 8, NROUNDS)	\
.else;									\
	vbroadcastf64x2 16*9(GDATA), ZT3;				\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, no_zmm, no_zmm, ZT3, 9, ZT4, ZT5, no_zmm, no_zmm, 8, NROUNDS)	\
	vbroadcastf64x2 16*10(GDATA), ZT3;				\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, no_zmm, no_zmm, ZT3, 10, ZT4, ZT5, no_zmm, no_zmm, 8, NROUNDS)	\
.endif;									\
.endif;									\
.ifnc DO_REDUCTION, no_reduction;					\
	vmovdqu64	POLY2(%rip), XWORD(ZT17);			\
	vpclmulqdq	$0x01, XWORD(ZT11), XWORD(ZT17), XWORD(ZT15);	\
	vpslldq		$8, XWORD(ZT15), XWORD(ZT15);			\
	vpxorq		XWORD(ZT15), XWORD(ZT11), XWORD(ZT15);		\
.endif;									\
.if 11 < (NROUNDS + 1);							\
.if NROUNDS == 11;							\
	vbroadcastf64x2 16*11(GDATA), ZT3;				\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, no_zmm, no_zmm, ZT3, 11, ZT4, ZT5, no_zmm, no_zmm, 8, NROUNDS)	\
.else;									\
	vbroadcastf64x2 16*11(GDATA), ZT3;				\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, no_zmm, no_zmm, ZT3, 11, ZT4, ZT5, no_zmm, no_zmm, 8, NROUNDS)	\
	vbroadcastf64x2 16*12(GDATA), ZT3;				\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, no_zmm, no_zmm, ZT3, 12, ZT4, ZT5, no_zmm, no_zmm, 8, NROUNDS)	\
.endif;									\
.endif;									\
.ifnc DO_REDUCTION, no_reduction;					\
	vpclmulqdq	$0x00, XWORD(ZT15), XWORD(ZT17), XWORD(ZT16);	\
	vpsrldq		$4, XWORD(ZT16), XWORD(ZT16);			\
	vpclmulqdq	$0x10, XWORD(ZT15), XWORD(ZT17), XWORD(ZT13);	\
	vpslldq		$4, XWORD(ZT13), XWORD(ZT13);			\
	vpternlogq	$0x96, XWORD(ZT10), XWORD(ZT16), XWORD(ZT13);	\
.endif;									\
.if 13 < (NROUNDS + 1);							\
	vbroadcastf64x2 16*13(GDATA), ZT3;				\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, no_zmm, no_zmm, ZT3, 13, ZT4, ZT5, no_zmm, no_zmm, 8, NROUNDS)	\
.endif;									\
/* load/store mask (partial case) and load the text data */		\
.ifc FULL_PARTIAL, full;						\
	vmovdqu8	(PLAIN_CYPH_IN, DATA_OFFSET), ZT4;		\
	vmovdqu8	64(PLAIN_CYPH_IN, DATA_OFFSET), ZT5;		\
.else;									\
	lea		byte64_len_to_mask_table(%rip), IA0;		\
	mov		LENGTH, IA1;					\
	sub		$64, IA1;					\
	kmovq		(IA0, IA1, 8), MASKREG;				\
	vmovdqu8	(PLAIN_CYPH_IN, DATA_OFFSET), ZT4;		\
	vmovdqu8	64(PLAIN_CYPH_IN, DATA_OFFSET), ZT5{MASKREG}{z};\
.endif;									\
.if NROUNDS == 9;							\
	vbroadcastf64x2 16*10(GDATA), ZT3;				\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, no_zmm, no_zmm, ZT3, 10, ZT4, ZT5, no_zmm, no_zmm, 8, NROUNDS)	\
.elseif NROUNDS == 11;							\
	vbroadcastf64x2 16*12(GDATA), ZT3;				\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, no_zmm, no_zmm, ZT3, 12, ZT4, ZT5, no_zmm, no_zmm, 8, NROUNDS)	\
.elseif NROUNDS == 13;							\
	vbroadcastf64x2 16*14(GDATA), ZT3;				\
	ZMM_AESENC_ROUND_BLOCKS_0_16(ZT1, ZT2, no_zmm, no_zmm, ZT3, 14, ZT4, ZT5, no_zmm, no_zmm, 8, NROUNDS)	\
.endif;									\
/* store the cipher/plain text data */					\
.ifc FULL_PARTIAL, full;						\
	vmovdqu8	ZT1, (CYPH_PLAIN_OUT, DATA_OFFSET);		\
	vmovdqu8	ZT2, 64(CYPH_PLAIN_OUT, DATA_OFFSET);		\
.else;									\
	vmovdqu8	ZT1, (CYPH_PLAIN_OUT, DATA_OFFSET);		\
	vmovdqu8	ZT2, 64(CYPH_PLAIN_OUT, DATA_OFFSET){MASKREG};	\
.endif;									\
/* prep cipher text blocks for the next ghash round */			\
.ifnc FULL_PARTIAL, full;						\
	vpxorq		ZT5, ZT2, ZT3;					\
	vextracti32x4	$3, ZT3, AES_PARTIAL_BLOCK;			\
/* for GHASH computation, clear the top bytes of the partial block */	\
.ifc ENC_DEC, ENC;							\
	vmovdqu8	ZT2, ZT2{MASKREG}{z};				\
.else;									\
	vmovdqu8	ZT5, ZT5{MASKREG}{z};				\
.endif;									\
.endif;									\
/* shuffle cipher text blocks for GHASH computation */			\
.ifc ENC_DEC, ENC;							\
	vpshufb		SHFMSK, ZT1, GHASHIN_AESOUT_B03;		\
	vpshufb		SHFMSK, ZT2, GHASHIN_AESOUT_B47;		\
.else;									\
	vpshufb		SHFMSK, ZT4, GHASHIN_AESOUT_B03;		\
	vpshufb		SHFMSK, ZT5, GHASHIN_AESOUT_B47;		\
.endif;									\
.ifc DO_REDUCTION, do_reduction;					\
	/* XOR current GHASH value (ZT13) into block 0 */		\
	vpxorq		ZT13, GHASHIN_AESOUT_B03;			\
.endif;									\
.ifc DO_REDUCTION, final_reduction;					\
	/* Return GHASH value (ZT13) in TO_REDUCE_L */			\
	vmovdqa64	ZT13, TO_REDUCE_L;				\
.endif;

/*
 * GHASH the last 7 cipher text blocks.
 * - it uses same GHASH macros as GHASH_LAST_8 but with some twist
 * - it loads GHASH keys for each of the data blocks, so that:
 * - blocks 4, 5 and 6 will use GHASH keys 3, 2, 1 respectively
 * - code ensures that unused block 7 and corresponding GHASH key are zeroed
 * (clmul product is zero this way and will not affect the result)
 * - blocks 0, 1, 2 and 3 will use USE GHASH keys 7, 6, 5 and 4 respectively
 */
#define GHASH_LAST_7(HASHSUBKEY, BL47, BL03, ZTH, ZTM, ZTL, ZT01, ZT02, ZT03, ZT04, AAD_HASH, MASKREG, IA0, GH, GL,GM)	\
	vmovdqa64	POLY2(%rip), XWORD(ZT04);							\
	VCLMUL_1_TO_8_STEP1(HASHSUBKEY, BL47, ZT01, ZT02, ZTH, ZTM, ZTL, 7)				\
	vpxorq		GH, ZTH, ZTH;									\
	vpxorq		GL, ZTL, ZTL;									\
	vpxorq		GM, ZTM, ZTM;									\
	VCLMUL_1_TO_8_STEP2(HASHSUBKEY, BL47, BL03, ZT01, ZT02, ZT03, ZTH, ZTM, ZTL, 7)			\
	VCLMUL_REDUCE(AAD_HASH, XWORD(ZT04), XWORD(BL47), XWORD(BL03), XWORD(ZT01), XWORD(ZT02))	\

/* GHASH the last 8 ciphertext blocks. */
#define GHASH_LAST_8(HASHSUBKEY, BL47, BL03, ZTH, ZTM, ZTL, ZT01, ZT02, ZT03, AAD_HASH, GH, GL,GM)	\
	VCLMUL_STEP1(HASHSUBKEY, BL47, ZT01, ZTH, ZTM, ZTL, NULL)					\
	vpxorq		GH, ZTH, ZTH;									\
	vpxorq		GL, ZTL, ZTL;									\
	vpxorq		GM, ZTM, ZTM;									\
	VCLMUL_STEP2(HASHSUBKEY, BL47, BL03, ZT01, ZT02, ZT03, ZTH, ZTM, ZTL, NULL, NULL)		\
	vmovdqa64	POLY2(%rip), XWORD(ZT03);							\
	VCLMUL_REDUCE(AAD_HASH, XWORD(ZT03), XWORD(BL47), XWORD(BL03), XWORD(ZT01), XWORD(ZT02))	\

/*
 * Encodes/Decodes given data. Assumes that the passed gcm_context_data struct
 * has been initialized by GCM_INIT
 * Requires the input data be at least 1 byte long because of READ_SMALL_INPUT_DATA.
 * Clobbers rax, r10-r15, and zmm0-zmm31, k1
 * Macro flow:
 * calculate the number of 16byte blocks in the message
 * process (number of 16byte blocks) mod 8
 * process 8, 16 byte blocks at a time until all are done
 */
#define GCM_ENC_DEC(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, PLAIN_CYPH_LEN, ENC_DEC, NROUNDS)	 \
	or		PLAIN_CYPH_LEN, PLAIN_CYPH_LEN;		\
	je		21f;					\
	xor		%r11, %r11;	  			\
	add		PLAIN_CYPH_LEN, InLen(GDATA_CTX);	\
	vmovdqu64	AadHash(GDATA_CTX), %xmm14;		\
	/*							\
	 * Used for the update flow - if there was a previous 	\
	 * partial block fill the remaining bytes here.		\
	 */							\
	PARTIAL_BLOCK(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, PLAIN_CYPH_LEN, %r11, %xmm14, ENC_DEC, %r10, %r12, %r13, %zmm0, %zmm3, %zmm4, %zmm5, %zmm6, %zmm7, %zmm10, %zmm11, %zmm12, %zmm13, %k1)	\
	/* lift counter block from GCM_INIT to here */		\
	vmovdqu64	CurCount(GDATA_CTX), %xmm9;		\
	/* Save the amount of data left to process in %r13 */	\
	mov	PLAIN_CYPH_LEN, %r13;				\
	sub	%r11, %r13;					\
	je	21f;						\
	vmovdqa64	SHUF_MASK(%rip), %zmm29;		\
	vmovdqa64	ddq_addbe_4444(%rip), %zmm27;		\
	cmp		$(big_loop_nblocks * 16), %r13;		\
	jl		12f;					\
	vmovdqa64	ddq_addbe_1234(%rip), %zmm28;		\
	INITIAL_BLOCKS_Nx16(PLAIN_CYPH_IN, CYPH_PLAIN_OUT, GDATA_KEY, GDATA_CTX, %r11, %zmm14, %zmm9, %r15, %zmm0, %zmm3, %zmm4, %zmm5, %zmm6, %zmm7, %zmm10, %zmm11, %zmm12, %zmm13, %zmm15, %zmm16, %zmm17, %zmm19, %zmm20, %zmm21, %zmm30, %zmm31, %zmm1, %zmm2, %zmm8, %zmm22, %zmm23, %zmm24 , %zmm25, %zmm26, %zmm27, %zmm28, %zmm29, ENC_DEC, 48, 32, NROUNDS)	 \
	sub		 $(big_loop_nblocks * 16), %r13;	\
	cmp		$(big_loop_nblocks * 16), %r13;		\
	jl		11f;					\
10:;								\
	GHASH_ENCRYPT_Nx16_PARALLEL(PLAIN_CYPH_IN, CYPH_PLAIN_OUT, GDATA_KEY, GDATA_CTX, %r11, %zmm9, %zmm29, %zmm0, %zmm3, %zmm4, %zmm5, %zmm6, %zmm7, %zmm10, %zmm11, %zmm12, %zmm13, %zmm15, %zmm16, %zmm17, %zmm19, %zmm20, %zmm21, %zmm30, %zmm31, %zmm1, %zmm2, %zmm8, %zmm22, %zmm23, %zmm24, %zmm25, %zmm26, %zmm27, %zmm28, %zmm14, ENC_DEC, 48, 32, %r15, NROUNDS)	   \
	sub		$(big_loop_nblocks * 16), %r13;		\
	cmp		$(big_loop_nblocks * 16), %r13;		\
	jge		10b;					\
11:;								\
	vpshufb		%xmm29, %xmm9, %xmm9;			\
	vmovdqa64	%xmm9, XWORD(%zmm28);			\
	GHASH_LAST_Nx16(GDATA_CTX, %zmm14, %zmm0, %zmm3, %zmm4, %zmm5, %zmm6, %zmm7, %zmm10, %zmm11, %zmm12, %zmm13, %zmm15, %zmm16, %zmm17, %zmm19, %zmm20, %zmm21, %zmm24, %zmm25, %zmm26, 48, 32)					\
	or		%r13, %r13;				\
	jz		20f;					\
12:;								\
	/*							\
	 * Less than 256 bytes will be handled by the small	\
	 * message code, which can process up to 16 x blocks	\
	 * (16 bytes each)					\
	 */							\
	cmp		$256, %r13;				\
	jge		13f;					\
	/*							\
	 * Determine how many blocks to process; process one	\
	 * additional block if there is a partial block		\
	 */							\
	mov		%r13, %r12;				\
	add		$15, %r12;				\
	shr		$4, %r12;				\
	GCM_ENC_DEC_SMALL(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, PLAIN_CYPH_LEN, ENC_DEC, %r11, %r13, %r12, %xmm9, %xmm14, %zmm0, %zmm3, %zmm4, %zmm5, %zmm6, %zmm7, %zmm10, %zmm11, %zmm12, %zmm13, %zmm15, %zmm16, %zmm17, %zmm19, %zmm20, %zmm21, %zmm30, %zmm31, %zmm1, %zmm2, %zmm8, %zmm22, %zmm23, %r10, %r15, %k1, %zmm29, NROUNDS)		\
	vmovdqa64	%xmm9, %xmm28;				\
	jmp	20f;						\
13:;								\
	mov		%r13, %r12;				\
	and		$0xff, %r12;				\
	add		$15, %r12;				\
	shr		$4, %r12;				\
	/*							\
	 * Don't allow 8 INITIAL blocks since this will		\
         * be handled by the x8 partial loop.			\
	 */							\
	and		$7, %r12;				\
	je		8f;					\
	cmp		$1, %r12;				\
	je		1f;					\
	cmp		$2, %r12;				\
	je		2f;					\
	cmp		$3, %r12;				\
	je		3f;					\
	cmp		$4, %r12;				\
	je		4f;					\
	cmp		$5, %r12;				\
	je		5f;					\
	cmp		$6, %r12;				\
	je		6f;					\
7:;								\
	INITIAL_BLOCKS(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, %r13, %r11, 7, %xmm9, %zmm14, %zmm0, %zmm3, %zmm4, %zmm5, %zmm6, %zmm7, %zmm10, %zmm11, %zmm12, %zmm13, %zmm15, %zmm16, %r10, %r12, ENC_DEC, %k1, %zmm29, no_partial_block, NROUNDS)	\
	jmp	9f;						\
6:;								\
	INITIAL_BLOCKS(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, %r13, %r11, 6, %xmm9, %zmm14, %zmm0, %zmm3, %zmm4, %zmm5, %zmm6, %zmm7, %zmm10, %zmm11, %zmm12, %zmm13, %zmm15, %zmm16, %r10, %r12, ENC_DEC, %k1, %zmm29, no_partial_block, NROUNDS)	\
	jmp	9f;						\
5:;								\
	INITIAL_BLOCKS(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, %r13, %r11, 5, %xmm9, %zmm14, %zmm0, %zmm3, %zmm4, %zmm5, %zmm6, %zmm7, %zmm10, %zmm11, %zmm12, %zmm13, %zmm15, %zmm16, %r10, %r12, ENC_DEC, %k1, %zmm29, no_partial_block, NROUNDS)	\
	jmp	9f;						\
4:;								\
	INITIAL_BLOCKS(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, %r13, %r11, 4, %xmm9, %zmm14, %zmm0, %zmm3, %zmm4, %zmm5, %zmm6, %zmm7, %zmm10, %zmm11, %zmm12, %zmm13, %zmm15, %zmm16, %r10, %r12, ENC_DEC, %k1, %zmm29, no_partial_block, NROUNDS)	\
	jmp	9f;						\
3:;								\
	INITIAL_BLOCKS(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, %r13, %r11, 3, %xmm9, %zmm14, %zmm0, %zmm3, %zmm4, %zmm5, %zmm6, %zmm7, %zmm10, %zmm11, %zmm12, %zmm13, %zmm15, %zmm16, %r10, %r12, ENC_DEC, %k1, %zmm29, no_partial_block, NROUNDS)	\
	jmp	9f;						\
2:;								\
	INITIAL_BLOCKS(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, %r13, %r11, 2, %xmm9, %zmm14, %zmm0, %zmm3, %zmm4, %zmm5, %zmm6, %zmm7, %zmm10, %zmm11, %zmm12, %zmm13, %zmm15, %zmm16, %r10, %r12, ENC_DEC, %k1, %zmm29, no_partial_block, NROUNDS)	\
	jmp	9f;						\
1:;								\
	INITIAL_BLOCKS(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, %r13, %r11, 1, %xmm9, %zmm14, %zmm0, %zmm3, %zmm4, %zmm5, %zmm6, %zmm7, %zmm10, %zmm11, %zmm12, %zmm13, %zmm15, %zmm16, %r10, %r12, ENC_DEC, %k1, %zmm29, no_partial_block, NROUNDS)	\
	jmp	9f;						\
8:;								\
	INITIAL_BLOCKS(GDATA_KEY, GDATA_CTX, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, %r13, %r11, 0, %xmm9, %zmm14, %zmm0, %zmm3, %zmm4, %zmm5, %zmm6, %zmm7, %zmm10, %zmm11, %zmm12, %zmm13, %zmm15, %zmm16, %r10, %r12, ENC_DEC, %k1, %zmm29, no_partial_block, NROUNDS)	\
9:;									\
	vmovdqa64	%xmm9, XWORD(%zmm28);				\
	/*								\
	 * Move cipher blocks from initial blocks to input of by8 macro	\
	 * and for GHASH_LAST_8/7					\
	 */								\
	vmovdqa64	%zmm0, %zmm1;					\
	vmovdqa64	%zmm3, %zmm2;					\
	/*								\
	 * The entire message cannot get processed in INITIAL_BLOCKS	\
	 * - GCM_ENC_DEC_SMALL handles up to 16 blocks			\
	 * - INITIAL_BLOCKS processes up to 15 blocks			\
	 * - no need to check for zero length at this stage		\
	 * In order to have only one reduction at the end, the start	\
	 * HASH KEY pointer needs to be determined based on length and	\
	 * call type. Note that 8 blocks are already ciphered in	\
	 * INITIAL_BLOCKS and subtracted from LENGTH(%r13)		\
	 */								\
	lea		128(%r13), %r12;				\
	add		$15, %r12;					\
	and		$0x3f0, %r12;					\
	/* if partial block then change hash key start by one */	\
	mov		%r13, %r10;					\
	and		$15, %r10;					\
	add		$15, %r10;					\
	and		$16, %r10;					\
	sub		%r10, %r12;					\
	lea		(HashKey + 16 + HashSubKey)(GDATA_CTX), %rax;	\
	sub		%r12, %rax;					\
	/*								\
	 * %rax points at the first hash key to start GHASH which	\
	 * needs to be updated as the message is processed		\
	 */								\
	vmovdqa64	ddq_addbe_8888(%rip), %zmm27;			\
	vmovdqa64	ddq_add_8888(%rip), %zmm19;			\
	vpxorq		%zmm24, %zmm24, %zmm24;				\
	vpxorq		%zmm25, %zmm25, %zmm25;				\
	vpxorq		%zmm26, %zmm26, %zmm26;				\
	/* prepare counter 8 blocks */					\
	vshufi64x2	$0, %zmm9, %zmm9, %zmm9;			\
	vpaddd		ddq_add_5678(%rip), %zmm9, %zmm18;		\
	vpaddd		ddq_add_1234(%rip), %zmm9, %zmm9;		\
	vpshufb		%zmm29, %zmm9, %zmm9;				\
	vpshufb		%zmm29, %zmm18, %zmm18;				\
	/* Process 7 full blocks plus a partial block */		\
	cmp		$128, %r13;					\
	jl		17f;						\
14:;									\
	/*								\
	 * in_order vs. out_order is an optimization to increment the	\
	 * counter without shuffling it back into little endian.	\
	 * %r15 keeps track of when we need to increment in_order so	\
	 * that the carry is handled correctly.				\
	 */								\
	vmovq		XWORD(%zmm28), %r15;				\
15:;									\
	and		$255, WORD(%r15);				\
	add		$8, WORD(%r15);					\
	vmovdqu64	64(%rax), %zmm31;				\
	vmovdqu64	(%rax), %zmm30;					\
	GHASH_8_ENCRYPT_8_PARALLEL(GDATA_KEY, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, %r11, %zmm9, %zmm18, %zmm1, %zmm2, %xmm8, out_order, ENC_DEC, full, %r10, %r12, %r13, %zmm31, %zmm30, %zmm29, %zmm0, %zmm3, %zmm4, %zmm5, %zmm6, %zmm7, %zmm10, %zmm11, %zmm12, %zmm13, %zmm15, %zmm16, %zmm17, %k1, no_reduction, %zmm25, %zmm24, %zmm26, NROUNDS)      \
	add		$128, %rax;					\
	add		$128, %r11;					\
	sub		$128, %r13;					\
	jz		18f;						\
	cmp		$248, WORD(%r15);				\
	jae		16f;						\
	vpaddd		%zmm27, %zmm9, %zmm9;				\
	vpaddd		%zmm27, %zmm18, %zmm18;				\
	cmp		$128, %r13;					\
	jl		17f;						\
	jmp		15b;						\
16:;									\
	vpshufb		%zmm29, %zmm9, %zmm9;				\
	vpshufb		%zmm29, %zmm18, %zmm18;				\
	vpaddd		%zmm19, %zmm9, %zmm9;				\
	vpaddd		%zmm19, %zmm18, %zmm18;				\
	vpshufb		%zmm29, %zmm9, %zmm9;				\
	vpshufb		%zmm29, %zmm18, %zmm18;				\
	cmp		$128, %r13;					\
	jge		15b;						\
17:;									\
	/*								\
	 * Test to see if we need a by 8 with partial block. At this	\
	 * point, bytes remaining should be either 0 or between 113-127.\
	 * 'in_order' shuffle needed to align key for partial block xor.\
	 * 'out_order' is faster because it avoids extra shuffles.	\
	 * counter blocks prepared for the next 8 blocks in BE format	\
	 * - we can go ahead with out_order scenario			\
	 */								\
	vmovdqu64	64(%rax), %zmm31;				\
	vmovdqu64	(%rax), %zmm30;					\
	GHASH_8_ENCRYPT_8_PARALLEL(GDATA_KEY, CYPH_PLAIN_OUT, PLAIN_CYPH_IN, %r11, %zmm9, %zmm18, %zmm1, %zmm2, %xmm8, out_order, ENC_DEC, partial, %r10, %r12, %r13, %zmm31, %zmm30, %zmm29, %zmm0, %zmm3, %zmm4, %zmm5, %zmm6, %zmm7, %zmm10, %zmm11, %zmm12, %zmm13, %zmm15, %zmm16, %zmm17, %k1, no_reduction, %zmm25, %zmm24, %zmm26, NROUNDS)	\
	add		$128, %rax;					\
	add		$112, %r11;					\
	sub		$112, %r13;					\
	mov		%r13, PBlockLen(GDATA_CTX);			\
	vmovdqu64	%xmm8, PBlockEncKey(GDATA_CTX);			\
18:;									\
	/* Extract the last counter block in LE format */		\
	vextracti32x4	$3, %zmm18, XWORD(%zmm28);			\
	vpshufb		XWORD(%zmm29), XWORD(%zmm28), XWORD(%zmm28);	\
	/*								\
	 * GHASH last cipher text blocks in xmm1-xmm8			\
	 * if block 8th is partial, then skip the block			\
	 */								\
	cmpq		$0, PBlockLen(GDATA_CTX);			\
	jz		19f;						\
	/* Save 8th partial block: GHASH_LAST_7 will clobber %zmm2 */	\
	vextracti32x4	$3, %zmm2, XWORD(%zmm11);			\
	GHASH_LAST_7(GDATA_CTX, %zmm2, %zmm1, %zmm0, %zmm3, %zmm4, %zmm5, %zmm6, %zmm7, %zmm10, %xmm14, %k1, %r10, %zmm24, %zmm25, %zmm26)	\
	/* XOR the partial word into the hash */			\
	vpxorq		%xmm11, %xmm14, %xmm14;				\
	jmp		20f;						\
19:;									\
	GHASH_LAST_8(GDATA_CTX, %zmm2, %zmm1, %zmm0, %zmm3, %zmm4, %zmm5, %zmm6, %zmm7, %xmm14, %zmm24, %zmm25, %zmm26)		\
20:;									\
	vmovdqu64	XWORD(%zmm28), CurCount(GDATA_CTX);		\
21:;									\
	vmovdqu64	%xmm14, (GDATA_CTX);				\

# Store data from SIMD registers to memory
#define simd_store_avx_15(DST, SRC, SIZE, TMP, IDX)			\
	xor	IDX, IDX;						\
	test	$8, SIZE;						\
	jz	44f;							\
	vmovq	SRC, (DST, IDX, 1);					\
	vpsrldq $8, SRC, SRC;						\
	add	$8, IDX;						\
44:;									\
	vmovq	SRC, TMP;						\
	test	$4, SIZE;						\
	jz	43f;							\
	mov	DWORD(TMP), (DST, IDX, 1);				\
	shr	$32, TMP;						\
	add	$4, IDX;						\
43:;									\
	test	$2, SIZE;						\
	jz	42f;							\
	mov	WORD(TMP), (DST, IDX, 1);				\
	shr	$16, TMP;						\
	add	$2, IDX;						\
42:;									\
	test	$1, SIZE;						\
	jz 41f;								\
	mov	BYTE(TMP), (DST, IDX, 1);				\
41:;

/*
 * Finishes Encryption/Decryption of last partial block after GCM_UPDATE finishes.
 * Clobbers rax, r10-r12, and xmm0-xmm2, xmm5-xmm6, xmm9-xmm11, xmm13-xmm15
 */
#define GCM_COMPLETE(GDATA_KEY, GDATA_CTX, AUTH_TAG, AUTH_TAG_LEN, NROUNDS) \
	vmovdqu HashKey + HashSubKey(GDATA_CTX), %xmm13;		\
	vmovdqu OrigIV(GDATA_CTX), %xmm9;				\
	ENCRYPT_SINGLE_BLOCK(GDATA_KEY, %xmm9, NROUNDS)			\
	vmovdqu (GDATA_CTX), %xmm14;					\
	/* Encrypt the final partial block */				\
	mov PBlockLen(GDATA_CTX), %r12;					\
	cmp $0, %r12;							\
	je 36f;								\
	/* GHASH computation for the last 16 byte block */		\
	GHASH_MUL(%xmm14, %xmm13, %xmm0, %xmm10, %xmm11, %xmm5, %xmm6)	\
	vmovdqu %xmm14, (GDATA_CTX);					\
36:;									\
	mov AadLen(GDATA_CTX), %r12;					\
	mov InLen(GDATA_CTX), %rax;					\
	shl $3, %r12;							\
	vmovd %r12d, %xmm15;						\
	shl $3, %rax;							\
	vmovq %rax, %xmm1;						\
	vpslldq $8, %xmm15, %xmm15;					\
	vpxor %xmm1, %xmm15, %xmm15;					\
	vpxor %xmm15, %xmm14, %xmm14;					\
	GHASH_MUL(%xmm14, %xmm13, %xmm0, %xmm10, %xmm11, %xmm5, %xmm6)	\
	vpshufb SHUF_MASK(%rip), %xmm14, %xmm14;			\
	vpxor %xmm14, %xmm9, %xmm9;					\
31:;									\
	mov AUTH_TAG, %r10;						\
	mov AUTH_TAG_LEN, %r11;						\
	cmp $16, %r11;							\
	je 34f;								\
	cmp $12, %r11;							\
	je 33f;								\
	cmp $8, %r11;							\
	je 32f;								\
	simd_store_avx_15(%r10, %xmm9, %r11, %r12, %rax)		\
	jmp 35f;							\
32:;									\
	vmovq %xmm9, %rax;						\
	mov %rax, (%r10);						\
	jmp 35f;							\
33:;									\
	vmovq %xmm9, %rax;						\
	mov %rax, (%r10);						\
	vpsrldq $8, %xmm9, %xmm9;					\
	vmovd %xmm9, %eax;						\
	mov %eax, 8(%r10);						\
	jmp 35f;							\
34:;									\
	vmovdqu %xmm9, (%r10);						\
35:;

################################################################################################
# void	aesni_gcm_init_avx_512
#	 (gcm_data     *my_ctx_data,
#	  gcm_context_data *data,
#	  u8	  *iv, /* Pre-counter block j0: 4 byte salt
#			(from Security Association) concatenated with 8 byte
#			Initialisation Vector (from IPSec ESP Payload)
#			concatenated with 0x00000001. 16-byte aligned pointer. */
#	  u8	 *hash_subkey	/* Hash sub key input. Data starts on a 16-byte boundary. */
#	  const   u8 *aad,	/* Additional Authentication Data (AAD)*/
#	  u64	  aad_len)	/* Length of AAD in bytes. With RFC4106 this is 8 or 12 Bytes */
################################################################################################
SYM_FUNC_START(aesni_gcm_init_avx_512)
	FUNC_SAVE_GHASH()

	GCM_INIT(arg2, arg3, arg4, arg5, arg6, %r10, %r11, %r12, %k1, %xmm14, %xmm2, %zmm1, %zmm2, %zmm3, %zmm4, %zmm5, %zmm6, %zmm7, %zmm8, %zmm9, %zmm10)

	FUNC_RESTORE_GHASH()
	ret
SYM_FUNC_END(aesni_gcm_init_avx_512)

###############################################################################
# void	aesni_gcm_enc_update_avx_512(
#	 gcm_data	 *my_ctx_data,	   /* aligned to 16 Bytes */
#	 gcm_context_data *data,
#	 u8	 *out, /* Ciphertext output. Encrypt in-place is allowed.  */
#	 const	 u8 *in, /* Plaintext input */
#	 u64	 plaintext_len) /* Length of data in Bytes for encryption. */
###############################################################################
SYM_FUNC_START(aesni_gcm_enc_update_avx_512)
	FUNC_SAVE_GHASH()

	mov	2 * 15 * 16(arg1),%eax
	cmp	$32, %eax
	je	key_256_enc_update_2
	cmp	$16, %eax
	je	key_128_enc_update
	# must be 192
	GCM_ENC_DEC(arg1, arg2, arg3, arg4, arg5, ENC, 11)
	FUNC_RESTORE_GHASH()
	ret
key_128_enc_update:
	GCM_ENC_DEC(arg1, arg2, arg3, arg4, arg5, ENC, 9)
	FUNC_RESTORE_GHASH()
	ret
key_256_enc_update_2:
	GCM_ENC_DEC(arg1, arg2, arg3, arg4, arg5, ENC, 13)
	FUNC_RESTORE_GHASH()
	ret
SYM_FUNC_END(aesni_gcm_enc_update_avx_512)

###################################################################################
# void	aesni_gcm_dec_update_avx_512(
#	 gcm_data	 *my_ctx_data,	   /* aligned to 16 Bytes */
#	 gcm_context_data *data,
#	 u8	 *out,		/* Plaintext output. Decrypt in-place is allowed */
#	 const	 u8 *in, 	/* Ciphertext input */
#	 u64	 plaintext_len) /* Length of data in Bytes for encryption */
###################################################################################
SYM_FUNC_START(aesni_gcm_dec_update_avx_512)
	FUNC_SAVE_GHASH()

	mov	2 * 15 * 16(arg1),%eax
	cmp	$32, %eax
	je	key_256_dec_update
	cmp	$16, %eax
	je	key_128_dec_update
	# must be 192
	GCM_ENC_DEC(arg1, arg2, arg3, arg4, arg5, DEC, 11)
	FUNC_RESTORE_GHASH()
	ret
key_128_dec_update:
	GCM_ENC_DEC(arg1, arg2, arg3, arg4, arg5, DEC, 9)
	FUNC_RESTORE_GHASH()
	ret
key_256_dec_update:
	GCM_ENC_DEC(arg1, arg2, arg3, arg4, arg5, DEC, 13)
	FUNC_RESTORE_GHASH()
	ret
SYM_FUNC_END(aesni_gcm_dec_update_avx_512)

###############################################################################
# void	aesni_gcm_finalize_avx_512(
#	 gcm_data	 *my_ctx_data,	   /* aligned to 16 Bytes */
#	 gcm_context_data *data,
#	 u8	 *auth_tag,	/* Authenticated Tag output. */
#	 u64	 auth_tag_len)	/* Authenticated Tag Length in bytes. */
###############################################################################
SYM_FUNC_START(aesni_gcm_finalize_avx_512)
	FUNC_SAVE_GHASH()

	mov	2 * 15 * 16(arg1),%eax
	cmp	$32, %eax
	je	key_256_complete
	cmp	$16, %eax
	je	key_128_complete
	# must be 192
	GCM_COMPLETE(arg1, arg2, arg3, arg4, 11)
	FUNC_RESTORE_GHASH()
	ret
key_256_complete:
	GCM_COMPLETE(arg1, arg2, arg3, arg4, 13)
	FUNC_RESTORE_GHASH()
	ret
key_128_complete:
	GCM_COMPLETE(arg1, arg2, arg3, arg4, 9)
	FUNC_RESTORE_GHASH()
	ret
SYM_FUNC_END(aesni_gcm_finalize_avx_512)

###############################################################################
# void aes_gcm_precomp_avx_512(
#	struct crypto_aes_ctx *ctx,	/* Context struct containing the key */
#	u8 *hash_subkey);		/* Output buffer */
###############################################################################
SYM_FUNC_START(aes_gcm_precomp_avx_512)
	FUNC_SAVE_GHASH()
	vpxor	%xmm6, %xmm6, %xmm6
	mov	2 * 15 * 16(arg1),%eax
	cmp	$32, %eax
	je	key_256_precomp
	cmp	$16, %eax
	je	key_128_precomp
	ENCRYPT_SINGLE_BLOCK(%rdi, %xmm6, 11)
	jmp	key_precomp
key_128_precomp:
	ENCRYPT_SINGLE_BLOCK(%rdi, %xmm6, 9)
	jmp	key_precomp
key_256_precomp:
	ENCRYPT_SINGLE_BLOCK(%rdi, %xmm6, 13)
key_precomp:
	vpshufb SHUF_MASK(%rip), %xmm6, %xmm6
	vmovdqa %xmm6, %xmm2
	vpsllq	$1, %xmm6, %xmm6
	vpsrlq	$63, %xmm2, %xmm2
	vmovdqa %xmm2, %xmm1
	vpslldq $8, %xmm2, %xmm2
	vpsrldq $8, %xmm1, %xmm1
	vpor	%xmm2, %xmm6, %xmm6

	vpshufd  $0x24, %xmm1, %xmm2
	vpcmpeqd TWOONE(%rip), %xmm2, %xmm2
	vpand	 POLY(%rip), %xmm2, %xmm2
	vpxor	 %xmm2, %xmm6, %xmm6

	vmovdqu  %xmm6, HashKey(%rsi)

	PRECOMPUTE(%rsi, %xmm6, %xmm0, %xmm1, %xmm2, %xmm3, %xmm4, %xmm5, %xmm7, %xmm8)

	FUNC_RESTORE_GHASH()
	ret

SYM_FUNC_END(aes_gcm_precomp_avx_512)
