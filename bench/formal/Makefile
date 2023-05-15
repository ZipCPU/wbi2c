################################################################################
##
## Filename: 	Makefile
## {{{
## Project:	WBI2C ... a set of Wishbone controlled I2C controller(s)
##
## Purpose:	Coordinates the proofs of all items that might be out of date.
##
## Creator:	Dan Gisselquist, Ph.D.
##		Gisselquist Technology, LLC
##
################################################################################
## }}}
## Copyright (C) 2015-2023, Gisselquist Technology, LLC
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
all:
.PHONY: axili2ccpu wbi2ccpu axisi2c
RTL := ../../rtl
ZIP := ../../../../../zipcpu/trunk
BUS := ../../../../wb2axip/trunk
all: axili2ccpu wbi2ccpu axisi2c

axili2ccpu: axili2ccpu_prf/PASS axili2ccpu_cvr/PASS
axili2ccpu_prf/PASS: wbi2ccpu.sby $(RTL)/axili2ccpu.v $(RTL)/axisi2c.v $(ZIP)/bench/formal/ffetch.v $(BUS)/rtl/skidbuffer.v $(BUS)/rtl/axilfetch.v $(BUS)/bench/formal/faxil_slave.v
	sbyx.pl axili2ccpu.sby prf
axili2ccpu_cvr/PASS: wbi2ccpu.sby $(RTL)/axili2ccpu.v $(RTL)/axisi2c.v $(ZIP)/bench/formal/ffetch.v $(BUS)/rtl/skidbuffer.v $(BUS)/rtl/axilfetch.v $(BUS)/bench/formal/faxil_slave.v
	sby -f axili2ccpu.sby cvr


wbi2ccpu: wbi2ccpu_prf/PASS wbi2ccpu_cvr/PASS
wbi2ccpu_prf/PASS: wbi2ccpu.sby $(RTL)/wbi2ccpu.v $(ZIP)/bench/formal/ffetch.v $(ZIP)/rtl/ex/fwb_slave.v
	sbyx.pl wbi2ccpu.sby prf
wbi2ccpu_cvr/PASS: wbi2ccpu.sby $(RTL)/wbi2ccpu.v $(ZIP)/bench/formal/ffetch.v $(ZIP)/rtl/ex/fwb_slave.v
	sby -f wbi2ccpu.sby cvr

axisi2c: axisi2c_prf/PASS axisi2c_prflp/PASS axisi2c_prfw/PASS axisi2c_cvr/PASS
axisi2c_prf/PASS: axisi2c.sby $(RTL)/axisi2c.v
	sbyx.pl axisi2c.sby prf
axisi2c_prflp/PASS: axisi2c.sby $(RTL)/axisi2c.v
	sbyx.pl axisi2c.sby prflp
axisi2c_prfw/PASS: axisi2c.sby $(RTL)/axisi2c.v
	sbyx.pl axisi2c.sby prfw
axisi2c_cvr/PASS: axisi2c.sby $(RTL)/axisi2c.v
	sby -f axisi2c.sby cvr

