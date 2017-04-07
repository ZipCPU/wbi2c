////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	i2cslave.v
//
// Project:	WBI2C ... a set of Wishbone controlled I2C controllers
//
// Purpose:	To create an I2C Slave that can be accessed via a wishbone bus.
//
//	This core works by creating a dual-port 128-byte memory, that can be
//	written to via either the wishbone bus it is connected to, or the I2C
//	bus which it acts as a slave upon.
//	Via either bus, the 128 bytes of slave memory may be referenced, read,
//	and written to.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2017, Gisselquist Technology, LLC
//
// This program is free software (firmware): you can redistribute it and/or
// modify it under the terms of  the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTIBILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program.  (It's in the $(ROOT)/doc directory.  Run make with no
// target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
//
// License:	GPL, v3, as defined and found on www.gnu.org,
//		http://www.gnu.org/licenses/gpl.html
//
//
////////////////////////////////////////////////////////////////////////////////
//
//
`define	I2CIDLE		3'h0
`define	I2CSTART	3'h1
`define	I2CADDR		3'h2
`define	I2CSACK		3'h3	// Slave ack's (that's us)
`define	I2CRX		3'h4	// Slave receive's (that's us)
`define	I2CMACK		3'h5	// Master acks's
`define	I2CTX		3'h6	// Slave transmits
`define	I2CILLEGAL	3'h7
//
//
module	i2cslave(i_clk, i_rst,
		i_wb_cyc, i_wb_stb, i_wb_we, i_wb_addr, i_wb_data, i_wb_sel,
			o_wb_ack, o_wb_stall, o_wb_data,
		i_i2c_sck, i_i2c_sda, o_i2c_sck, o_i2c_sda);
	parameter	INITIAL_MEM = "";
	parameter [0:0]	WB_READ_ONLY = 1'b0;
	parameter [0:0]	I2C_READ_ONLY = 1'b0;
	localparam [0:0]	READ_ONLY = (WB_READ_ONLY)&&(I2C_READ_ONLY);
	input			i_clk, i_rst;
	input			i_wb_cyc, i_wb_stb, i_wb_we;
	input		[4:0]	i_wb_addr;
	input		[31:0]	i_wb_data;
	input		[3:0]	i_wb_sel;
	output	reg		o_wb_ack;
	output	wire		o_wb_stall;
	output	reg	[31:0]	o_wb_data;
	input			i_i2c_sck, i_i2c_sda;
	output	reg		o_i2c_sck, o_i2c_sda;

	reg	[7:0]	mem	[0:127];
	initial begin
		if (INITIAL_MEM != "")
			$readmemh(INITIAL_MEM, mem);
	end

	reg		rd_stb, wr_stb;
	reg	[6:0]	i2c_addr;
	reg	[7:0]	wr_data;

	reg	[3:0]	r_we;
	reg	[31:0]	r_data;
	reg	[4:0]	r_addr;
	initial	r_we = 4'h0;
	always @(posedge i_clk)
	begin
		if (!READ_ONLY)
		begin
			r_we[3] <= (i_wb_stb)&&(i_wb_we)&&(i_wb_sel[3])
				||(wr_stb)&&(i2c_addr[1:0]==2'b00);
			r_we[2] <= (i_wb_stb)&&(i_wb_we)&&(i_wb_sel[2])
				||(wr_stb)&&(i2c_addr[1:0]==2'b01);
			r_we[1] <= (i_wb_stb)&&(i_wb_we)&&(i_wb_sel[1])
				||(wr_stb)&&(i2c_addr[1:0]==2'b10);
			r_we[0] <= (i_wb_stb)&&(i_wb_we)&&(i_wb_sel[0])
				||(wr_stb)&&(i2c_addr[1:0]==2'b11);
			r_data  <= (wr_stb)? {(4){wr_data}} : i_wb_data;
			r_addr  <= (wr_stb) ? i2c_addr[6:2] : i_wb_addr;
		end else
			r_we <= 4'h0;
			// data and address are don't cares if READ_ONLY is set
	end

	always @(posedge i_clk)
	begin
		if (r_we[3])
			mem[{r_addr, 2'b00}] <= r_data[31:24];
		if (r_we[2])
			mem[{r_addr, 2'b01}] <= r_data[23:16];
		if (r_we[1])
			mem[{r_addr, 2'b10}] <= r_data[15: 8];
		if (r_we[0])
			mem[{r_addr, 2'b11}] <= r_data[ 7: 0];
	end

	always @(posedge i_clk)
		o_wb_data <= { mem[{ i_wb_addr[4:0], 2'b00 }], 
			mem[{ i_wb_addr[4:0], 2'b01 }],
			mem[{ i_wb_addr[4:0], 2'b10 }],
			mem[{ i_wb_addr[4:0], 2'b11 }] };

	initial	o_wb_ack = 1'b0;
	always @(posedge i_clk)
		o_wb_ack <= i_wb_stb;

	assign	o_wb_stall = 1'b0;

	//
	//
	// Okay, that builds our memory, and gives us access to the bus.
	// Now ... let's build the I2C slave portion to interact over that bus
	//
	//
	localparam	PL=2;
	reg	[(2*PL-1):0]	i2c_pipe;
	always @(posedge i_clk)
		i2c_pipe <= { i2c_pipe[(2*PL-3):0], i_i2c_sck, i_i2c_sda };

	reg	last_sck, last_sda;
	always @(posedge i_clk)
	begin
		last_sck <= i2c_pipe[(2*PL-1)];
		last_sda <= i2c_pipe[(2*PL-2)];
	end

	wire	this_sck   = i2c_pipe[(2*PL-1)];
	wire	this_sda   = i2c_pipe[(2*PL-2)];
	wire	i2c_posedge= (!last_sck)&&( this_sck);
	wire	i2c_negedge= ( last_sck)&&(!this_sck);
	wire	i2c_start  = ( last_sck)&&( this_sck)&&( last_sda)&&(!this_sda);
	wire	i2c_stop   = ( last_sck)&&( this_sck)&&(!last_sda)&&( this_sda);

	reg	[2:0]	state;
	reg	[7:0]	dreg, oreg, rd_val;
	reg	[2:0]	dbits;
	reg		tx_rx_n;
	initial	state = `I2CIDLE;
	initial	o_i2c_sck = 1'b1;
	initial	o_i2c_sda = 1'b1;
	always	@(posedge i_clk)
	begin
		// Default is to do nothing with the output ports.  A 1'b1 does
		// that.
		o_i2c_sck <= 1'b1;
		o_i2c_sda <= 1'b1;
		if (i2c_posedge)
			dreg  <= { dreg[6:0], this_sda };
		if (i2c_negedge)
			oreg  <= { oreg[6:0], oreg[0] };
		case(state)
		`I2CIDLE: begin
				dbits <= 0;
				if (i2c_start)
					state <= `I2CSTART;
			end
		`I2CSTART: begin
				dbits <= 0;
				if (i2c_negedge)
					state <= `I2CADDR;
			end
		`I2CADDR: begin
				if (i2c_negedge)
					dbits <= dbits + 1'b1;
				if ((i2c_negedge)&&(dbits == 3'h7))
					state <= `I2CSACK;
				tx_rx_n <= dreg[0];
			end
		`I2CSACK: begin
				dbits <= 3'h0;
				o_i2c_sda <= 1'b0;
				oreg <= rd_val;
				if (i2c_negedge)
					state <= (tx_rx_n)?`I2CTX:`I2CRX;
			end
		`I2CRX: begin	// Slave reads
				// Write to the slave (that's us)
				if (i2c_negedge)
					dbits <= dbits + 1'b1;
				if ((i2c_negedge)&&(dbits == 3'h7))
					state <= `I2CSACK;
			end
		`I2CTX: begin	// Slave transmits
				// Read from the slave (that's us)
				if (i2c_negedge)
					dbits <= dbits + 1'b1;
				if ((i2c_negedge)&&(dbits == 3'h7))
					state <= `I2CMACK;
				o_i2c_sda <= oreg[7];
			end
		`I2CMACK: begin
				dbits <= 3'h0;
				if (i2c_negedge)
					state <= `I2CTX;
				oreg <= rd_val;
			end
		`I2CILLEGAL:	dbits <= 3'h0;
		// default:	dbits <= 3'h0;
		endcase
		if (i2c_stop)
			state <= `I2CIDLE;
	end

	initial	rd_stb = 1'b0;
	always @(posedge i_clk)
		rd_stb <= (i2c_posedge)&&((state == `I2CSACK)
						||(state == `I2CMACK));

	reg	pre_write;
	initial	pre_write = 1'b0;
	always @(posedge i_clk)
		if (i2c_stop)
			pre_write <= 1'b0;
		else if (state == `I2CRX)
			pre_write <= (!I2C_READ_ONLY);
		else if (state == `I2CIDLE)
			pre_write <= 1'b0;

	initial	wr_stb = 1'b0;
	always @(posedge i_clk)
		wr_stb  <= (i2c_negedge)&&(state == `I2CSACK)&&(pre_write);

	always @(posedge i_clk)
		if ((this_sck)&&(state == `I2CRX))
			wr_data <= dreg;

	always @(posedge i_clk)
		if (state == `I2CADDR)
			i2c_addr <= dreg[7:1] + ((tx_rx_n)? 7'h7f:7'h7e);
		else if (i2c_posedge)
		begin
			// Slave ACK's during master write slave reads, and
			// following an address
			if (state == `I2CSACK)
				i2c_addr <= i2c_addr + 1'b1;
			else if (state == `I2CMACK)
				i2c_addr <= i2c_addr + 1'b1;
		end

	always @(posedge i_clk)
		if (rd_stb)
			rd_val <= mem[i2c_addr];
		else if ((i2c_negedge)&&(state == `I2CMACK))
			rd_val <= { rd_val[6:0], rd_val[0] };

endmodule
