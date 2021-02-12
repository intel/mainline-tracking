/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_PKEYS_COMMON_H
#define _ASM_X86_PKEYS_COMMON_H

#define PKR_AD_BIT 0x1
#define PKR_WD_BIT 0x2
#define PKR_BITS_PER_PKEY 2

#define PKR_PKEY_SHIFT(pkey) (pkey * PKR_BITS_PER_PKEY)
#define PKR_PKEY_MASK(pkey)  (((1 << PKR_BITS_PER_PKEY) - 1) << PKR_PKEY_SHIFT(pkey))

/*
 * Generate an Access-Disable and Write-Disable mask for the given pkey.
 * Several of the AD's are OR'd together to generate a default pkey register
 * value.
 */
#define PKR_AD_KEY(pkey)     (PKR_AD_BIT << PKR_PKEY_SHIFT(pkey))
#define PKR_WD_KEY(pkey)     (PKR_WD_BIT << PKR_PKEY_SHIFT(pkey))

#endif /*_ASM_X86_PKEYS_COMMON_H */
