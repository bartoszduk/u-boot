#
# (C) Copyright 2000-2006
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# See file CREDITS for list of people who contributed to this
# project.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#

include $(TOPDIR)/config.mk

LIB	= $(obj)libgeneric.a
BOOTSTRAP_LIB	= $(obj)libgeneric_bootstrap.a

BOOTSTRAP_LIB-$(CONFIG_BOOTSTRAP)	= $(BOOTSTRAP_LIB)

COBJS-$(CONFIG_ADDR_MAP) += addr_map.o
COBJS-$(CONFIG_BZIP2) += bzlib.o
COBJS-$(CONFIG_BZIP2) += bzlib_crctable.o
COBJS-$(CONFIG_BZIP2) += bzlib_decompress.o
COBJS-$(CONFIG_BZIP2) += bzlib_randtable.o
COBJS-$(CONFIG_BZIP2) += bzlib_huffman.o
COBJS-$(CONFIG_USB_TTY) += circbuf.o
COBJS-y += crc16.o
COBJS-y += crc32.o
COBJS-y += ctype.o
COBJS-y += display_options.o
COBJS-y += div64.o
COBJS-$(CONFIG_GZIP) += gunzip.o
COBJS-$(CONFIG_LMB) += lmb.o
COBJS-y += ldiv.o
COBJS-$(CONFIG_MD5) += md5.o
COBJS-y += net_utils.o
COBJS-$(CONFIG_SHA1) += sha1.o
COBJS-$(CONFIG_SHA256) += sha256.o
COBJS-y += string.o
COBJS-y	+= strmhz.o
COBJS-y += time.o
COBJS-y += vsprintf.o
COBJS-$(CONFIG_ZLIB) += zlib.o
COBJS-$(CONFIG_RBTREE)	+= rbtree.o
BOOTSTRAP_COBJS-$(CONFIG_BOOTSTRAP) += string.o
BOOTSTRAP_COBJS-$(CONFIG_BOOTSTRAP) += vsprintf.o
BOOTSTRAP_COBJS-$(CONFIG_BOOTSTRAP) += div64.o
BOOTSTRAP_COBJS-$(CONFIG_BOOTSTRAP) += ctype.o
BOOTSTRAP_COBJS-$(CONFIG_BOOTSTRAP) += time.o
BOOTSTRAP_COBJS-$(CONFIG_BOOTSTRAP) += bootstrap.o
BOOTSTRAP_COBJS-$(CONFIG_BOOTSTRAP_GZIP) += zlib.o
BOOTSTRAP_COBJS-$(CONFIG_BOOTSTRAP_GZIP) += gunzip.o
BOOTSTRAP_COBJS-$(CONFIG_BOOTSTRAP_GZIP) += crc32.o
BOOTSTRAP_COBJS-$(CONFIG_BOOTSTRAP_BZIP2) += bzlib.o
BOOTSTRAP_COBJS-$(CONFIG_BOOTSTRAP_BZIP2) += bzlib_crctable.o
BOOTSTRAP_COBJS-$(CONFIG_BOOTSTRAP_BZIP2) += bzlib_decompress.o
BOOTSTRAP_COBJS-$(CONFIG_BOOTSTRAP_BZIP2) += bzlib_randtable.o
BOOTSTRAP_COBJS-$(CONFIG_BOOTSTRAP_BZIP2) += bzlib_huffman.o

BOOTSTRAP_COBJS	:= $(BOOTSTRAP_COBJS-y)
BOOTSTRAP_OBJS	:= $(addprefix $(obj),$(BOOTSTRAP_COBJS))

COBJS	:= $(COBJS-y)
SRCS	:= $(COBJS:.o=.c) $(BOOTSTRAP_COBJS:.o=.c)
OBJS	:= $(addprefix $(obj),$(COBJS))

all: $(obj).depend $(LIB) $(BOOTSTRAP_LIB-y)

$(LIB):	$(obj).depend $(OBJS)
	$(AR) $(ARFLAGS) $@ $(OBJS)

$(BOOTSTRAP_LIB):	$(obj).depend $(BOOTSTRAP_OBJS)
	$(AR) $(ARFLAGS) $@ $(BOOTSTRAP_OBJS)


#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
