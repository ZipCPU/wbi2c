################################################################################
##
## Filename:	sw/Makefile
## {{{
## Project:	WBI2C ... a set of (Wishbone controlled) I2C controller(s)
##
## Purpose:	Since the I2C CPU/microcontroller approach requires a set of
##		instructions, we need some software to generate those
##	instructions.  That's the purpose of the I2C microcontroller assembler,
##	or i2casm as it is called here.  The i2casm is found in this directory
##	in the form of Flex script, i2casm.l that needs to be built into an
##	executable.  This makefile coordinates the build of this executable.
##
## Creator:	Dan Gisselquist, Ph.D.
##		Gisselquist Technology, LLC
##
################################################################################
## }}}
## Copyright (C) 2021-2023, Gisselquist Technology, LLC
## {{{
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
## }}}
## License:	GPL, v3, as defined and found on www.gnu.org,
## {{{
##		http://www.gnu.org/licenses/gpl.html
##
################################################################################
##
## }}}
all: i2casm

## Build i2casm
## {{{
lex.yy.c: i2casm.l
	flex i2casm.l

i2casm: lex.yy.c
	g++ lex.yy.c -o i2casm
## }}}

## A "test" target
## {{{
dump.bin: testfil.s i2casm
	./i2casm testfil.s -o dump.bin

.PHONY: test
test: dump.bin
	./i2casm -d dump.bin
## }}}

.PHONY: clean
## {{{
clean:
	rm -f i2casm lex.yy.c
## }}}
