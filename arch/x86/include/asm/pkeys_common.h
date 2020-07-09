/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_PKEYS_COMMON_H
#define _ASM_X86_PKEYS_COMMON_H

#define PKR_AD_BIT 0x1
#define PKR_WD_BIT 0x2
#define PKR_BITS_PER_PKEY 2

/*
 * Generate an Access-Disable mask for the given pkey.  Several of these can be
 * OR'd together to generate pkey register values.
 */
#define PKR_AD_KEY(pkey)	(PKR_AD_BIT << ((pkey) * PKR_BITS_PER_PKEY))

#endif /*_ASM_X86_PKEYS_COMMON_H */
