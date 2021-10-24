/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ALSA SoC Audio Layer - Rockchip SPDIF_RX Controller driver
 *
 * Copyright (C) 2018 Fuzhou Rockchip Electronics Co., Ltd
 *
 */

#ifndef _ROCKCHIP_SPDIFRX_H
#define _ROCKCHIP_SPDIFRX_H

/* CFGR */
#define SPDIFRX_CFGR_TWAD_SHIFT		1
#define SPDIFRX_CFGR_TWAD_DATA_ONLY	(0 << 1)
#define SPDIFRX_CFGR_TWAD_STREAM	BIT(1)
#define SPDIFRX_EN_MASK			BIT(0)
#define SPDIFRX_EN			BIT(0)
#define SPDIFRX_DIS			0

/* CLR */
#define SPDIFRX_CLR_RXSC		BIT(0)

/* CDR */
#define SPDIFRX_CDR_CS_MASK		GENMASK(10, 9)
#define SPDIFRX_CDR_CS_IDLE		0
#define SPDIFRX_CDR_CS_DETECT		BIT(9)
#define SPDIFRX_CDR_CS_MEASUREMENT	BIT(10)
#define SPDIFRX_CDR_CS_SYNC		(BIT(9) | BIT(10))
#define SPDIFRX_CDR_AVGSEL_MASK		BIT(1)
#define SPDIFRX_CDR_AVGSEL_MIN		(0 << 1)
#define SPDIFRX_CDR_AVGSEL_AVG		BIT(1)
#define SPDIFRX_CDR_BYPASS_MASK		BIT(0)
#define SPDIFRX_CDR_BYPASS_EN		BIT(0)
#define SPDIFRX_CDR_BYPASS_DIS		0

/* CDRST */
#define SPDIFRX_CDRST_NOSTRTHR_MASK	GENMASK(31, 16)
#define SPDIFRX_CDRST_MAXCNT_MASK	GENMASK(15, 8)
#define SPDIFRX_CDRST_MINCNT_MASK	GENMASK(7, 0)

/* DMACR */
#define SPDIFRX_DMACR_RDE_MASK		BIT(5)
#define SPDIFRX_DMACR_RDE_DISABLE	(0 << 5)
#define SPDIFRX_DMACR_RDE_ENABLE	BIT(5)
#define SPDIFRX_DMACR_RDL_MASK		GENMASK(4, 0)
#define SPDIFRX_DMACR_RDL(x)		(((x) - 1) << 0)

/* FIFOCTRL */
#define SPDIFRX_FIFOCTRL_RFL_MASK	GENMASK(13, 8)
#define SPDIFRX_FIFOCTRL_RFT_MASK	GENMASK(4, 0)

/* Interrupts */
#define SPDIFRX_INT_UBC			BIT(10)  /* User bit changed */
#define SPDIFRX_INT_ESYNC		BIT(9)   /* Enter sync */
#define SPDIFRX_INT_BTE			BIT(8)   /* Block end */
#define SPDIFRX_INT_NSYNC		BIT(7)   /* Exit sync */
#define SPDIFRX_INT_BMDE		BIT(5)   /* Biphase mark decode error */
#define SPDIFRX_INT_RXO			BIT(5)   /* RX Overrun */
#define SPDIFRX_INT_RXF			BIT(4)   /* RX Full */
#define SPDIFRX_INT_NPSP		BIT(3)   /* ??? */
#define SPDIFRX_INT_NVLD		BIT(2)   /* ??? */
#define SPDIFRX_INT_CSC			BIT(1)   /* Channel status bit changed */
#define SPDIFRX_INT_PE			BIT(0)   /* Parity error */

/* BURSTINFO */
#define SPDIFRX_BURSTINFO_PD_MASK	GENMASK(31, 16)
#define SPDIFRX_BURSTINFO_BSNUM_MASK	GENMASK(15, 13)
#define SPDIFRX_BURSTINFO_DATAINFO_MASK	GENMASK(12, 8)
#define SPDIFRX_BURSTINFO_ERRFLAG_MASK	BIT(7)
#define SPDIFRX_BURSTINFO_ERR		BIT(7)
#define SPDIFRX_BURSTINFO_VALID		(0 << 7)
#define SPDIFRX_BURSTINFO_DATATYPE_MASK	GENMASK(6, 0)

#define SPDIFRX_VERSION			(0x0000)
#define SPDIFRX_CFGR			(0x0004)
#define SPDIFRX_CLR			(0x0008)
#define SPDIFRX_CDR			(0x000c)
#define SPDIFRX_CDRST			(0x0010)
#define SPDIFRX_DMACR			(0x0014)
#define SPDIFRX_FIFOCTRL		(0x0018)
#define SPDIFRX_INTEN			(0x001C)
#define SPDIFRX_INTMASK			(0x0020)
#define SPDIFRX_INTSR			(0x0024)
#define SPDIFRX_INTCLR			(0x0028)
#define SPDIFRX_SMPDR			(0x002C)
#define SPDIFRX_USRDR(n)		(0x0030 + (n << 2))
#define SPDIFRX_CHNSR(n)		(0x0060 + (n << 2))
#define SPDIFRX_BURSTINFO		(0x0100)

#endif /* _ROCKCHIP_SPDIFRX_H */
