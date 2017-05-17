/*
 * Copyright (C) 2017 Álvaro Fernández Rojas <noltari@gmail.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_BMIPS_COMMON_H
#define __CONFIG_BMIPS_COMMON_H

/* DMA */
#define ARCH_DMA_MINALIGN		32

/* RAM */
#define CONFIG_SYS_MEMTEST_START	0xa0000000
#define CONFIG_SYS_MEMTEST_END		0xa2000000

/* USB */
#define CONFIG_EHCI_DESC_BIG_ENDIAN
#define CONFIG_EHCI_MMIO_BIG_ENDIAN

/* Memory usage */
#define CONFIG_SYS_MAXARGS		24
#define CONFIG_SYS_MALLOC_LEN		(1024 * 1024)
#define CONFIG_SYS_BOOTPARAMS_LEN	(128 * 1024)
#define CONFIG_SYS_CBSIZE		512

/* U-Boot */
#define CONFIG_SYS_MONITOR_BASE		CONFIG_SYS_TEXT_BASE

#endif /* __CONFIG_BMIPS_COMMON_H */
