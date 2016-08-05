#
# (C) Copyright 2003-2006
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

LIB	= $(obj)lib$(CPU).a
BOOTSTRAP_LIB = $(obj)lib$(CPU)_bootstrap.a

BOOTSTRAP_LIB-$(CONFIG_BOOTSTRAP) = $(BOOTSTRAP_LIB)

START	= start.o
SOBJS-y	= cache.o
COBJS-y	= cpu.o reset.o interrupts.o

SOBJS-$(CONFIG_INCA_IP)	+= incaip_wdt.o
COBJS-$(CONFIG_INCA_IP)	+= asc_serial.o incaip_clock.o
#COBJS-$(CONFIG_IFX_ASC) += ifx_asc.o
COBJS-$(CONFIG_PURPLE)	+= asc_serial.o
COBJS-$(CONFIG_SOC_AU1X00) += au1x00_eth.o au1x00_serial.o au1x00_usb_ohci.o

#BOOTSTRAP_START = start_bootstrap.o
BOOTSTRAP_START-$(CONFIG_BOOTSTRAP) += start_bootstrap.o #$(BOOTSTRAP_START)
BOOTSTRAP_COBJS-$(CONFIG_BOOTSTRAP) += cpu.o interrupts.o reset_bootstrap.o
BOOTSTRAP_SOBJS-$(CONFIG_BOOTSTRAP) += cache.o
BOOTSTRAP_COBJS-$(CONFIG_DANUBE) += danube-clock.o
BOOTSTRAP_COBJS-$(CONFIG_AR9) += ar9-clock.o
BOOTSTRAP_COBJS-$(CONFIG_BOOTSTRAP_SERIAL) += ifx_asc.o

BOOTSTRAP_OBJS	:= $(addprefix $(obj),$(BOOTSTRAP_SOBJS-y) $(BOOTSTRAP_COBJS-y))
BOOTSTRAP_START := $(addprefix $(obj),$(BOOTSTRAP_START-y))

SRCS	:= $(SOBJS-y:.o=.S) $(COBJS-y:.o=.c) $(BOOTSTRAP_START-y:.o=.S) $(BOOTSTRAP_SOBJS-y:.o=.S) $(BOOTSTRAP_COBJS-y:.o=.c)
OBJS	:= $(addprefix $(obj),$(SOBJS-y) $(COBJS-y))
START	:= $(addprefix $(obj),$(START))

all:	$(obj).depend $(START) $(LIB) $(BOOTSTRAP_START-y) $(BOOTSTRAP_LIB-y)

$(LIB):	 $(OBJS)
	$(AR) $(ARFLAGS) $@ $(OBJS)

#$(BOOTSTRAP_START):	$(obj).depend

$(BOOTSTRAP_LIB):	$(BOOTSTRAP_OBJS)
	$(AR) $(ARFLAGS) $@ $(BOOTSTRAP_OBJS)


#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
