/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Copyright(c) 2020 Intel Corporation. All rights rsvd.
 *
 * Implement CRC T10 DIF calculatin with AVX512 instructions. (x86_64)
 *
 * This is CRC T10 DIF calculation with AVX512 instructions. It requires
 * the support of Intel(R) AVX512F and VPCLMULQDQ instructions.
 *
 * Authors:
 * Greg Tucker <greg.b.tucker@intel.com>
 * Tomasz Kantecki <tomasz.kantecki@intel.com>
 * Kyung Min Park <kyung.min.park@intel.com>
 */

#include <linux/linkage.h>

.text
#define		init_crc	%edi
#define		buf		%rsi
#define		len		%rdx
#define		VARIABLE_OFFSET 16*2+8

/*
 * u16 crct10dif-avx512-asm_64(u16 init_crc, const u8 *buf, size_t len);
 */
.align 16
SYM_FUNC_START(crct10dif_pcl_avx512)

	/* adjust the 16-bit initial_crc value, scale it to 32 bits. */
	shl		$16, init_crc

	/*
	 * After this point, code flow is exactly same as a 32-bit CRC.
	 * The only difference is before returning eax, we will shift it
	 * right 16 bits, to scale back to 16 bits.
	 */
	sub		$(VARIABLE_OFFSET), %rsp

	vbroadcasti32x4 SHUF_MASK(%rip), %zmm18

	/* For sizes less than 256 bytes, we can't fold 256 bytes at a time. */
	cmp		$256, len
	jl		.less_than_256

	/* load the initial crc value */
	vmovd		init_crc, %xmm10

	/*
	 * crc value does not need to be byte-reflected, but it needs to be
	 * moved to the high part of the register because data will be
	 * byte-reflected and will align with initial crc at correct place.
	 */
	vpslldq		$12, %xmm10, %xmm10

	/* receive the initial 64B data, xor the initial crc value. */
	vmovdqu8	(buf), %zmm0
	vmovdqu8	16*4(buf), %zmm4
	vpshufb		%zmm18, %zmm0, %zmm0
	vpshufb		%zmm18, %zmm4, %zmm4
	vpxorq		%zmm10, %zmm0, %zmm0
	vbroadcasti32x4	rk3(%rip), %zmm10

	sub		$256, len
	cmp		$256, len
	jl		.fold_128_B_loop

	vmovdqu8	16*8(buf), %zmm7
	vmovdqu8	16*12(buf), %zmm8
	vpshufb		%zmm18, %zmm7, %zmm7
	vpshufb		%zmm18, %zmm8, %zmm8
	vbroadcasti32x4 rk_1(%rip), %zmm16
	sub		$256, len

.fold_256_B_loop:
	add		$256, buf
	vmovdqu8	(buf), %zmm3
	vpshufb		%zmm18, %zmm3, %zmm3
	vpclmulqdq	$0x00, %zmm16, %zmm0, %zmm1
	vpclmulqdq	$0x11, %zmm16, %zmm0, %zmm2
	vpxorq		%zmm2, %zmm1, %zmm0
	vpxorq		%zmm3, %zmm0, %zmm0

	vmovdqu8	16*4(buf), %zmm9
	vpshufb		%zmm18, %zmm9, %zmm9
	vpclmulqdq	$0x00, %zmm16, %zmm4, %zmm5
	vpclmulqdq	$0x11, %zmm16, %zmm4, %zmm6
	vpxorq		%zmm6, %zmm5, %zmm4
	vpxorq		%zmm9, %zmm4, %zmm4

	vmovdqu8	16*8(buf), %zmm11
	vpshufb		%zmm18, %zmm11, %zmm11
	vpclmulqdq	$0x00, %zmm16, %zmm7, %zmm12
	vpclmulqdq	$0x11, %zmm16, %zmm7, %zmm13
	vpxorq		%zmm13, %zmm12, %zmm7
	vpxorq		%zmm11, %zmm7, %zmm7

	vmovdqu8	16*12(buf), %zmm17
	vpshufb		%zmm18, %zmm17, %zmm17
	vpclmulqdq	$0x00, %zmm16, %zmm8, %zmm14
	vpclmulqdq	$0x11, %zmm16, %zmm8, %zmm15
	vpxorq		%zmm15, %zmm14, %zmm8
	vpxorq		%zmm17, %zmm8, %zmm8

	sub		$256, len
	jge		.fold_256_B_loop

	/* Fold 256 into 128 */
	add		$256, buf
	vpclmulqdq	$0x00, %zmm10, %zmm0, %zmm1
	vpclmulqdq	$0x11, %zmm10, %zmm0, %zmm2
	vpternlogq	$0x96, %zmm2, %zmm1, %zmm7

	vpclmulqdq	$0x00, %zmm10, %zmm4, %zmm5
	vpclmulqdq	$0x11, %zmm10, %zmm4, %zmm6
	vpternlogq	$0x96, %zmm6, %zmm5, %zmm8

	vmovdqa32	%zmm7, %zmm0
	vmovdqa32	%zmm8, %zmm4

	add		$128, len
	jmp		.fold_128_B_register

	/*
	 * At this section of the code, there is 128*x + y (0 <= y < 128) bytes
	 * of buffer. The fold_128_B_loop will fold 128B at a time until we have
	 * 128 + y Bytes of buffer.
	 * Fold 128B at a time. This section of the code folds 8 xmm registers
	 * in parallel.
	 */
.fold_128_B_loop:
	add		$128, buf
	vmovdqu8	(buf), %zmm8
	vpshufb		%zmm18, %zmm8, %zmm8
	vpclmulqdq	$0x00, %zmm10, %zmm0, %zmm2
	vpclmulqdq	$0x11, %zmm10, %zmm0, %zmm1
	vpxorq		%zmm1, %zmm2, %zmm0
	vpxorq		%zmm8, %zmm0, %zmm0

	vmovdqu8	16*4(buf), %zmm9
	vpshufb		%zmm18, %zmm9, %zmm9
	vpclmulqdq	$0x00, %zmm10, %zmm4, %zmm5
	vpclmulqdq	$0x11, %zmm10, %zmm4, %zmm6
	vpxorq		%zmm6, %zmm5, %zmm4
	vpxorq		%zmm9, %zmm4, %zmm4

	sub		$128, len
	jge		.fold_128_B_loop

	add		$128, buf

	/*
	 * At this point, the buffer pointer is pointing at the last y Bytes
	 * of the buffer, where 0 <= y < 128. The 128B of folded data is in
	 * 8 of the xmm registers: xmm0 - xmm7.
	 */
.fold_128_B_register:
	/* fold the 8 128b parts into 1 xmm register with different constant. */
	vmovdqu8	rk9(%rip), %zmm16
	vmovdqu8	rk17(%rip), %zmm11
	vpclmulqdq	$0x00, %zmm16, %zmm0, %zmm1
	vpclmulqdq	$0x11, %zmm16, %zmm0, %zmm2
	vextracti64x2	$3, %zmm4, %xmm7

	vpclmulqdq	$0x00, %zmm11, %zmm4, %zmm5
	vpclmulqdq	$0x11, %zmm11, %zmm4, %zmm6
	vmovdqa		rk1(%rip), %xmm10
	vpternlogq	$0x96, %zmm5, %zmm2, %zmm1
	vpternlogq	$0x96, %zmm7, %zmm6, %zmm1

	vshufi64x2      $0x4e, %zmm1, %zmm1, %zmm8
	vpxorq          %ymm1, %ymm8, %ymm8
	vextracti64x2   $1, %ymm8, %xmm5
	vpxorq          %xmm8, %xmm5, %xmm7

	/*
	 * Instead of 128, we add 128 - 16 to the loop counter to save one
	 * instruction from the loop. Instead of a cmp instruction, we use
	 * the negative flag with the jl instruction.
	 */
	add		$(128 - 16), len
	jl		.final_reduction_for_128

	/*
	 * Now we have 16 + y bytes left to reduce. 16 Bytes is in register xmm7
	 * and the rest is in memory we can fold 16 bytes at a time if y >= 16.
	 * continue folding 16B at a time.
	 */
.16B_reduction_loop:
	vpclmulqdq	$0x11, %xmm10, %xmm7, %xmm8
	vpclmulqdq	$0x00, %xmm10, %xmm7, %xmm7
	vpxor		%xmm8, %xmm7, %xmm7
	vmovdqu		(buf), %xmm0
	vpshufb		%xmm18, %xmm0, %xmm0
	vpxor		%xmm0, %xmm7, %xmm7
	add		$16, buf
	sub		$16, len

	/*
	 * Instead of a cmp instruction, we utilize the flags with the jge
	 * instruction equivalent of: cmp len, 16-16. Check if there is any
	 * more 16B in the buffer to be able to fold.
	 */
	jge		.16B_reduction_loop

	/*
	 * now we have 16+z bytes left to reduce, where 0 <= z < 16.
	 * first, we reduce the data in the xmm7 register.
	 */
.final_reduction_for_128:
	add		$16, len
	je		.128_done

	/*
	 * Here we are getting data that is less than 16 bytes. since we know
	 * that there was data before the pointer, we can offset the input
	 * pointer before the actual point to receive exactly 16 bytes.
	 * After that, the registers need to be adjusted.
	 */
.get_last_two_xmms:
	vmovdqa		%xmm7, %xmm2
	vmovdqu		-16(buf, len), %xmm1
	vpshufb		%xmm18, %xmm1, %xmm1

	/*
	 * get rid of the extra data that was loaded before.
	 * load the shift constant
	 */
	lea		16 + pshufb_shf_table(%rip), %rax
	sub		len, %rax
	vmovdqu		(%rax), %xmm0

	vpshufb		%xmm0, %xmm2, %xmm2
	vpxor		mask1(%rip), %xmm0, %xmm0
	vpshufb		%xmm0, %xmm7, %xmm7
	vpblendvb	%xmm0, %xmm2, %xmm1, %xmm1

	vpclmulqdq	$0x11, %xmm10, %xmm7, %xmm8
	vpclmulqdq	$0x00, %xmm10, %xmm7, %xmm7
	vpxor		%xmm8, %xmm7, %xmm7
	vpxor		%xmm1, %xmm7, %xmm7

.128_done:
	/* compute crc of a 128-bit value. */
	vmovdqa		rk5(%rip), %xmm10
	vmovdqa		%xmm7, %xmm0

	vpclmulqdq	$0x01, %xmm10, %xmm7, %xmm7
	vpslldq		$8, %xmm0, %xmm0
	vpxor		%xmm0, %xmm7, %xmm7

	vmovdqa		%xmm7, %xmm0
	vpand		mask2(%rip), %xmm0, %xmm0
	vpsrldq		$12, %xmm7, %xmm7
	vpclmulqdq	$0x10, %xmm10, %xmm7, %xmm7
	vpxor		%xmm0, %xmm7, %xmm7

	/* barrett reduction */
.barrett:
	vmovdqa		rk7(%rip), %xmm10
	vmovdqa		%xmm7, %xmm0
	vpclmulqdq	$0x01, %xmm10, %xmm7, %xmm7
	vpslldq		$4, %xmm7, %xmm7
	vpclmulqdq	$0x11, %xmm10, %xmm7, %xmm7

	vpslldq		$4, %xmm7, %xmm7
	vpxor		%xmm0, %xmm7, %xmm7
	vpextrd		$1, %xmm7, %eax

.cleanup:
	/* scale the result back to 16 bits. */
	shr		$16, %eax
	add		$(VARIABLE_OFFSET), %rsp
	ret

.align 16
.less_than_256:
	/* check if there is enough buffer to be able to fold 16B at a time. */
	cmp		$32, len
	jl		.less_than_32

	/* If there is, load the constants. */
	vmovdqa		rk1(%rip), %xmm10

	/*
	 * get the initial crc value and align it to its correct place.
	 * And load the plaintext and byte-reflect it.
	 */
	vmovd		init_crc, %xmm0
	vpslldq		$12, %xmm0, %xmm0
	vmovdqu		(buf), %xmm7
	vpshufb		%xmm18, %xmm7, %xmm7
	vpxor		%xmm0, %xmm7, %xmm7

	/* update the buffer pointer */
	add		$16, buf

	/* subtract 32 instead of 16 to save one instruction from the loop */
	sub		$32, len

	jmp		.16B_reduction_loop

.align 16
.less_than_32:
	/*
	 * mov initial crc to the return value. This is necessary for
	 * zero-length buffers.
	 */
	mov		init_crc, %eax
	test		len, len
	je		.cleanup

	vmovd		init_crc, %xmm0
	vpslldq		$12, %xmm0, %xmm0

	cmp		$16, len
	je		.exact_16_left
	jl		.less_than_16_left

	vmovdqu		(buf), %xmm7
	vpshufb		%xmm18, %xmm7, %xmm7
	vpxor		%xmm0, %xmm7, %xmm7
	add		$16, buf
	sub		$16, len
	vmovdqa		rk1(%rip), %xmm10
	jmp		.get_last_two_xmms

.align 16
.less_than_16_left:
	/*
	 * use stack space to load data less than 16 bytes, zero-out the 16B
	 * in the memory first.
	 */
	vpxor		%xmm1, %xmm1, %xmm1
	mov		%rsp, %r11
	vmovdqa		%xmm1, (%r11)

	cmp		$4, len
	jl		.only_less_than_4

	mov		len, %r9
	cmp		$8, len
	jl		.less_than_8_left

	mov		(buf), %rax
	mov		%rax, (%r11)
	add		$8, %r11
	sub		$8, len
	add		$8, buf
.less_than_8_left:
	cmp		$4, len
	jl		.less_than_4_left

	mov		(buf), %eax
	mov		%eax, (%r11)
	add		$4, %r11
	sub		$4, len
	add		$4, buf

.less_than_4_left:
	cmp		$2, len
	jl		.less_than_2_left

	mov		(buf), %ax
	mov		%ax, (%r11)
	add		$2, %r11
	sub		$2, len
	add		$2, buf
.less_than_2_left:
	cmp		$1, len
	jl		.zero_left

	mov		(buf), %al
	mov		%al, (%r11)

.zero_left:
	vmovdqa		(%rsp), %xmm7
	vpshufb		%xmm18, %xmm7, %xmm7
	vpxor		%xmm0, %xmm7, %xmm7

	lea		16 + pshufb_shf_table(%rip), %rax
	sub		%r9, %rax
	vmovdqu		(%rax), %xmm0
	vpxor		mask1(%rip), %xmm0, %xmm0

	vpshufb		%xmm0,%xmm7, %xmm7
	jmp		.128_done

.align 16
.exact_16_left:
	vmovdqu		(buf), %xmm7
	vpshufb		%xmm18, %xmm7, %xmm7
	vpxor		%xmm0, %xmm7, %xmm7
	jmp		.128_done

.only_less_than_4:
	cmp		$3, len
	jl		.only_less_than_3

	mov		(buf), %al
	mov		%al, (%r11)

	mov		1(buf), %al
	mov		%al, 1(%r11)

	mov		2(buf), %al
	mov		%al, 2(%r11)

	vmovdqa		(%rsp), %xmm7
	vpshufb		%xmm18, %xmm7, %xmm7
	vpxor		%xmm0, %xmm7, %xmm7

	vpsrldq		$5, %xmm7, %xmm7
	jmp		.barrett

.only_less_than_3:
	cmp		$2, len
	jl		.only_less_than_2

	mov		(buf), %al
	mov		%al, (%r11)

	mov		1(buf), %al
	mov		%al, 1(%r11)

	vmovdqa		(%rsp), %xmm7
	vpshufb		%xmm18, %xmm7, %xmm7
	vpxor		%xmm0, %xmm7, %xmm7

	vpsrldq		$6, %xmm7, %xmm7
	jmp		.barrett

.only_less_than_2:
	mov		(buf), %al
	mov		%al, (%r11)

	vmovdqa		(%rsp), %xmm7
	vpshufb		%xmm18, %xmm7, %xmm7
	vpxor		%xmm0, %xmm7, %xmm7

	vpsrldq		$7, %xmm7, %xmm7
	jmp		.barrett
SYM_FUNC_END(crct10dif_pcl_avx512)

.section        .data
.align 32
rk_1:		.quad 0xdccf000000000000
rk_2:		.quad 0x4b0b000000000000
rk1:		.quad 0x2d56000000000000
rk2:		.quad 0x06df000000000000
rk3:		.quad 0x9d9d000000000000
rk4:		.quad 0x7cf5000000000000
rk5:		.quad 0x2d56000000000000
rk6:		.quad 0x1368000000000000
rk7:		.quad 0x00000001f65a57f8
rk8:		.quad 0x000000018bb70000
rk9:		.quad 0xceae000000000000
rk10:		.quad 0xbfd6000000000000
rk11:		.quad 0x1e16000000000000
rk12:		.quad 0x713c000000000000
rk13:		.quad 0xf7f9000000000000
rk14:		.quad 0x80a6000000000000
rk15:		.quad 0x044c000000000000
rk16:		.quad 0xe658000000000000
rk17:		.quad 0xad18000000000000
rk18:		.quad 0xa497000000000000
rk19:		.quad 0x6ee3000000000000
rk20:		.quad 0xe7b5000000000000
rk_1b:		.quad 0x2d56000000000000
rk_2b:		.quad 0x06df000000000000
		.quad 0x0000000000000000
		.quad 0x0000000000000000

.align 16
mask1:
	.octa	0x80808080808080808080808080808080

.align 16
mask2:
	.octa	0x00000000FFFFFFFFFFFFFFFFFFFFFFFF

.align 16
SHUF_MASK:
	.octa	0x000102030405060708090A0B0C0D0E0F

.align 16
pshufb_shf_table:	.octa 0x8f8e8d8c8b8a89888786858483828100
			.octa 0x000e0d0c0b0a09080706050403020100
			.octa 0x0f0e0d0c0b0a09088080808080808080
			.octa 0x80808080808080808080808080808080
