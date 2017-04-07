################################################################################
##
## Filename:	Makefile
##
## Project:	WBI2C ... a set of Wishbone controlled I2C controller(s)
##
## Purpose:	This makefile builds the final verilator simulation of the
##		i2c core(s).  Specifically, it builds the final C++ portion
##	of the simulator, and thus the final simulator executable.
##
##	Useful targets of this makefile include:
##
##	test
##	i2c_tb
##
##	clean
##		Removes all the products of compilation
##
##
## Creator:	Dan Gisselquist, Ph.D.
##		Gisselquist Technology, LLC
##
################################################################################
##
## Copyright (C) 2015-2017, Gisselquist Technology, LLC
##
## This program is free software (firmware): you can redistribute it and/or
## modify it under the terms of  the GNU General Public License as published
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
##
## License:	GPL, v3, as defined and found on www.gnu.org,
##		http://www.gnu.org/licenses/gpl.html
##
################################################################################
##
##
all: wbi2c_tb

CXX	:= g++
CFLAGS	:= -Wall -Og -g
OBJDIR  := obj-pc
RTLD	:= ../../rtl
RTLOBJD := $(RTLD)/obj_dir
VERILATOR_ROOT ?= $(shell bash -c 'verilator -V|grep VERILATOR_ROOT | head -1 | sed -e " s/^.*=\s*//"')
VROOT   := $(VERILATOR_ROOT)
VINCS	:= -I$(VROOT)/include -I$(VROOT)/include/vltstd
INCS	:= -I$(RTLOBJD) $(VINCS)
I2CSRCS := wbi2c_tb.cpp byteswap.cpp
I2COBJS := $(addprefix $(OBJDIR)/,$(subst .cpp,.o,$(I2CSRCS)))
SOURCES := $(I2CSRCS)
VLSRCS	:= verilated.cpp verilated_vcd_c.cpp
VLOBJS  := $(OBJDIR)/verilated.o $(OBJDIR)/verilated_vcd_c.o
VLIB	:= $(addprefix $(VROOT)/include/,$(VLSRCS))
LIBS	:= $(RTLOBJD)/Vwbi2cslave__ALL.a

$(OBJDIR)/%.o: %.cpp
	$(CXX) $(CFLAGS) $(INCS) -c $< -o $@

$(OBJDIR)/%.o: $(ZASM)/%.cpp
	$(CXX) $(CFLAGS) $(INCS) -c $< -o $@

$(OBJDIR)/%.o: $(VROOT)/include/%.cpp
	$(CXX) $(CFLAGS) $(INCS) -c $< -o $@

wbi2c_tb: $(I2COBJS) $(VLOBJS) $(RTLOBJD)/Vwbi2cslave__ALL.a
	$(CXX) $(CFLAGS) $(INCS) $(I2COBJS) $(VLOBJS) $(RTLOBJD)/Vwbi2cslave__ALL.a -o $@

.PHONY: test
test: wbi2c_tb
	./wbi2c_tb

define	build-depends
	@echo "Building dependencies"
	@$(CXX) $(CPPFLAGS) $(INCS) -MM $(VLIB) $(SOURCES) > $(OBJDIR)/xdepends.txt
	@sed -e 's/^.*.o: /$(OBJDIR)\/&/' < $(OBJDIR)/xdepends.txt > $(OBJDIR)/depends.txt
	@rm $(OBJDIR)/xdepends.txt
endef

tags: $(VLIB) $(SOURCES)
	@ctags $(SOURCES) $(VLIB)

.PHONY: depends
depends: tags $(OBJDIR)/
	$(build-depends)

$(OBJDIR)/:
	@bash -c "if [ ! -e $(OBJDIR) ]; then mkdir -p $(OBJDIR)/; fi"

$(OBJDIR)/depends.txt: $(OBJDIR)/ depends

.PHONY: clean
clean:
	rm -rf $(OBJDIR)/ ./wbi2c_tb *.vcd

-include $(OBJDIR)/depends.txt
