/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2012-2016, Freescale Semiconductor, Inc.
 * Copyright 2018 NXP
 */

#ifndef __CAAM_H__
#define	__CAAM_H__

#if !defined(SUCCESS)
#define SUCCESS (0)
#endif

#define ERROR_ANY           (-1)
#define ERROR_IN_PAGE_ALLOC (1)

void caam_open(void);

u32 caam_gen_blob(u32 plain_data_addr, u32 blob_addr, u32 size);

u32 caam_decap_blob(u32 plain_text, u32 blob_addr, u32 size);
u32 caam_hwrng(uint8_t *output_ptr, u32 output_len);

#endif /* __CAAM_H__ */
