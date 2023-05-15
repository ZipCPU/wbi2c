################################################################################
##
## Filename:	Makefile
## {{{
## Project:	WBI2C ... a set of Wishbone controlled I2C controller(s)
##
## Purpose:	This makefile builds the final verilator simulation of the
##		i2c core(s).  Specifically, it builds the final C++ portion
##	of the simulator, and thus the final simulator executable.
##
##	Useful targets of this makefile include:
##
##	all
##		Build the I2C test bench programs
##	test
##		Run the various I2C tests that are built within here
##	wbi2cm_tb
##		Build the test bench for the i2c master
##	wbi2cs_tb
##		Build the test bench for the i2c slave
##
##	clean
##		Removes all the products of compilation
##
##
## Creator:	Dan Gisselquist, Ph.D.
##		Gisselquist Technology, LLC
##
################################################################################
## }}}
## Copyright (C) 2015-2023, Gisselquist Technology, LLC
## {{{
## This program is free software (firmware): you can redistribute it and/or
## modify it under the terms of the GNU General Public License as published
## by the Free Software Foundation, either version 3 of the License, or (at
## your option) any later version.
##
## This program is distributed in the hope that it will be useful, but WITHOUT
## ANY WARRANTY; without even the implied warranty of MERCHANTIBILITY or
## FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
## for more details.
##
## You should have received a copy of the GNU General Public License along
## with this program.  (It's in the $(ROOT)/doc directory.  Run make with no
## target there if the PDF file isn't present.)  If not, see
## <http://www.gnu.org/licenses/> for a copy.
## }}}
## License:	GPL, v3, as defined and found on www.gnu.org,
## {{{
##		http://www.gnu.org/licenses/gpl.html
##
################################################################################
##
## }}}
all: wbi2cs_tb wbi2cm_tb test
PROGRAMS := wbi2cs_tb wbi2cm_tb
all: $(PROGRAMS)

CXX	:= g++
OBJDIR  := obj-pc
RTLD	:= ../../rtl
RTLOBJD := $(RTLD)/obj_dir
ifneq ($(VERILATOR_ROOT),)
VERILATOR:=$(VERILATOR_ROOT)/bin/verilator
else
VERILATOR_ROOT ?= $(shell bash -c 'verilator -V|grep VERILATOR_ROOT | head -1 | sed -e " s/^.*=\s*//"')
endif
export	$(VERILATOR)
VROOT   := $(VERILATOR_ROOT)
VDEFS   := $(shell ./vversion.sh)
VINCS	:= -I$(VROOT)/include -I$(VROOT)/include/vltstd
INCS	:= -I$(RTLOBJD) $(VINCS)
COMNSRC := byteswap.cpp
I2CSRCS := wbi2cs_tb.cpp
I2COBJS := $(addprefix $(OBJDIR)/,$(subst .cpp,.o,$(I2CSRCS) $(COMNSRC)))
I2CSRCM := wbi2cm_tb.cpp i2csim.cpp
I2COBJM := $(addprefix $(OBJDIR)/,$(subst .cpp,.o,$(I2CSRCM) $(COMNSRC)))
SOURCES := $(I2CSRCS) $(I2CSRCM) $(COMNSRC)
VLSRCS	:= verilated.cpp verilated_vcd_c.cpp
VLOBJS  := $(OBJDIR)/verilated.o $(OBJDIR)/verilated_vcd_c.o
VLIB	:= $(addprefix $(VROOT)/include/,$(VLSRCS))
LIBS	:= $(RTLOBJD)/Vwbi2cslave__ALL.a
LIBM	:= $(RTLOBJD)/Vwbi2cmaster__ALL.a
CFLAGS	:= -Wall -Og -g

$(OBJDIR)/%.o: %.cpp
	$(mk-objdir)
	$(CXX) $(CFLAGS) $(VDEFS) $(INCS) -c $< -o $@


$(OBJDIR)/%.o: $(VROOT)/include/%.cpp
	$(mk-objdir)
	$(CXX) $(CFLAGS) $(INCS) -c $< -o $@

wbi2cs_tb: $(I2COBJS) $(VLOBJS) $(RTLOBJD)/Vwbi2cslave__ALL.a
	$(CXX) $(CFLAGS) $(INCS) $(I2COBJS) $(VLOBJS) $(RTLOBJD)/Vwbi2cslave__ALL.a -o $@
wbi2cm_tb: $(I2COBJM) $(VLOBJS) $(RTLOBJD)/Vwbi2cmaster__ALL.a
	$(CXX) $(CFLAGS) $(INCS) $(I2COBJM) $(VLOBJS) $(RTLOBJD)/Vwbi2cmaster__ALL.a -o $@

.PHONY: test
test: wbi2cs_tbtest wbi2cm_tbtest

.PHONY: wbi2cs_tbtest
wbi2cs_tbtest: wbi2cs_tb
	./wbi2cs_tb

.PHONY: wbi2cm_tbtest
wbi2cm_tbtest: wbi2cm_tb
	./wbi2cm_tb

define	mk-objdir
	@bash -c "if [ ! -e $(OBJDIR) ]; then mkdir -p $(OBJDIR); fi"
endef

define	build-depends
	@echo "Building dependencies"
	@$(CXX) $(CPPFLAGS) $(INCS) -MM $(VLIB) $(SOURCES) > $(OBJDIR)/xdep.txt
	@sed -e 's/^.*.o: /$(OBJDIR)\/&/' < $(OBJDIR)/xdep.txt > $(OBJDIR)/depends.txt
	@rm $(OBJDIR)/xdep.txt
endef

tags: $(VLIB) $(SOURCES) $(HEADERS)
	@ctags $(SOURCES) $(HEADERS) $(VLIB)

.PHONY: depends
depends: tags
	$(mk-objdir)
	$(build-depends)

$(OBJDIR)/depends.txt: $(SOURCES) $(HEADERS)

.PHONY: clean
clean:
	rm -rf $(OBJDIR)/ $(PROGRAMS)

-include $(OBJDIR)/depends.txt
