/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright © 2020 Intel Corporation.
 *
 * Implement GHASH calculation with AVX512 instructions. (x86_64)
 *
 * This is GHASH calculation with AVX512 instructions. It requires
 * the support of Intel(R) AVX512F and VPCLMULQDQ instructions.
 *
 * Authors:
 * Tomasz Kantecki <tomasz.kantecki@intel.com>
 * Megha Dey <megha.dey@intel.com>
 * Kyung Min Park <kyung.min.park@intel.com>
 */

#include "avx512_vaes_common.S"

/*
 * void ghash_precomp_avx512(u8 *key_data);
 */
SYM_FUNC_START(ghash_precomp_avx512)
        FUNC_SAVE_GHASH()

        /* move original key to xmm6 */
        vmovdqu HashKey_1(arg1), %xmm6

        vpshufb SHUF_MASK(%rip), %xmm6, %xmm6

        vmovdqa %xmm6, %xmm2
        vpsllq  $1, %xmm6, %xmm6
        vpsrlq  $63, %xmm2, %xmm2
        vmovdqa %xmm2, %xmm1
        vpslldq $8, %xmm2, %xmm2
        vpsrldq $8, %xmm1, %xmm1
        vpor %xmm2, %xmm6, %xmm6
        vpshufd $36, %xmm1, %xmm2
        vpcmpeqd TWOONE(%rip), %xmm2, %xmm2
        vpand POLY(%rip), %xmm2, %xmm2
        vpxor %xmm2, %xmm6, %xmm6
        vmovdqu %xmm6, HashKey_1(arg1)

        PRECOMPUTE(arg1, %xmm6, %xmm0, %xmm1, %xmm2, %xmm3, %xmm4, %xmm5, %xmm7, %xmm8)

        FUNC_RESTORE_GHASH()

        ret
SYM_FUNC_END(ghash_precomp_avx512)

/*
 * void clmul_ghash_update_avx512
 *      (uint8_t *dst,
 *       const uint8_t *src,
 *       unsigned int srclen,
 *       struct ghash_ctx_new *key_data);
 */
SYM_FUNC_START(clmul_ghash_update_avx512)
        FUNC_SAVE_GHASH()

        /* Read current hash value from dst */
        vmovdqa (arg1), %xmm0

        /* Bswap current hash value */
        vpshufb SHUF_MASK(%rip), %xmm0, %xmm0

        CALC_AAD_HASH(arg2, arg3, %xmm0, arg4, %zmm1, %zmm2, %zmm3, %zmm4, %zmm5, %zmm6, %zmm7, %zmm8, %zmm9, %zmm10, %zmm11, %zmm12, %zmm13, %zmm15, %zmm16, %zmm17, %zmm18, %zmm19, %r10, %r11, %r12, %k1, 0)

        /* Bswap current hash value before storing */
        vpshufb SHUF_MASK(%rip), %xmm0, %xmm0
        vmovdqu %xmm0, (arg1)

        FUNC_RESTORE_GHASH()

        ret
SYM_FUNC_END(clmul_ghash_update_avx512)
