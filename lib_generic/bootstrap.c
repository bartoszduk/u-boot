/*
 * (C) Copyright 2010 Industrie Dial Face S.p.A.
 * Luigi 'Comio' Mantellini, luigi.mantellini@idf-hit.com
 *
 * (C) Copyright 2003
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <command.h>
#include <stdio_dev.h>

#ifdef CONFIG_BOOTSTRAP_LZMA
#include <lzma/LzmaTypes.h>
#include <lzma/LzmaDec.h>
#include <lzma/LzmaTools.h>
#endif /* CONFIG_BOOTSTRAP_LZMA */

#ifdef CONFIG_BOOTSTRAP_LZO
#include <linux/lzo.h>
#endif /* CONFIG_BOOTSTRAP_LZO */

#ifdef CONFIG_BOOTSTRAP_BZIP2
#include <bzlib.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

#if defined(CONFIG_BOOTSTRAP_SERIAL)
static const char *algo = 
#if defined(CONFIG_BOOTSTRAP_GZIP)
    "gzip";
#elif defined(CONFIG_BOOTSTRAP_LZMA)
    "lzma";
#elif defined(CONFIG_BOOTSTRAP_LZO)
    "lzo";
#elif defined(CONFIG_BOOTSTRAP_BZIP2)
    "bzip2";
#else
    "flat";
#endif
#endif

int copy_uboot(void *dst, size_t unc_size, void *src, size_t size)
{
	int ret;
	debug("copy from %p (%d) to %p (%d)\n", src, size, dst, unc_size);
#if defined(CONFIG_BOOTSTRAP_SERIAL)
	printf("Uncompressing payload (%s)...", algo);
#endif
#if defined(CONFIG_BOOTSTRAP_GZIP)
	ret = gunzip(dst, unc_size, src, &size);
#elif defined(CONFIG_BOOTSTRAP_LZMA)
	SizeT outsize = unc_size;
	ret = lzmaBuffToBuffDecompress(dst, &outsize, src, size);
#elif defined(CONFIG_BOOTSTRAP_LZO)
	uint unc_len = unc_size;
	ret = lzop_decompress(src, size, dst, &unc_len);
#elif defined(CONFIG_BOOTSTRAP_BZIP2)
	uint unc_len = unc_size;
	ret = BZ2_bzBuffToBuffDecompress ((char*)dst, &unc_len, (char *)src, size, CONFIG_SYS_MALLOC_LEN < (4096 * 1024), 0);
#else
	memcpy(dst, src, size);
	ret = 0;
#endif
	if (ret) {
#if defined(CONFIG_BOOTSTRAP_SERIAL)
	    printf("failed with error %d.\n", ret);
#endif
	    bootstrap_hang();
	} else {
#if defined(CONFIG_BOOTSTRAP_SERIAL)
	    puts("done.\n");
#endif
	}
	return ret;
}
