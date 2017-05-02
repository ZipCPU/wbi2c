////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	wbi2cslave.v
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
`default_nettype	none
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
`define	BUS_IDLE	2'b00
`define	BUS_READ	2'b01
`define	BUS_SEND	2'b10
//
//
module	wbi2cslave(i_clk, i_rst,
		i_wb_cyc, i_wb_stb, i_wb_we, i_wb_addr, i_wb_data, i_wb_sel,
			o_wb_ack, o_wb_stall, o_wb_data,
		i_i2c_sck, i_i2c_sda, o_i2c_sck, o_i2c_sda,
		o_dbg);
	parameter	INITIAL_MEM = "";
	parameter [0:0]	WB_READ_ONLY = 1'b0;
	parameter [0:0]	I2C_READ_ONLY = 1'b0;
	parameter [6:0]	SLAVE_ADDRESS = 7'h50;
	parameter	MEM_ADDR_BITS = 8;
	localparam [0:0]	READ_ONLY = (WB_READ_ONLY)&&(I2C_READ_ONLY);
	input	wire		i_clk, i_rst;
	input	wire		i_wb_cyc, i_wb_stb, i_wb_we;
	input	wire	[(MEM_ADDR_BITS-3):0]	i_wb_addr;
	input	wire	[31:0]	i_wb_data;
	input	wire	[3:0]	i_wb_sel;
	output	reg		o_wb_ack;
	output	wire		o_wb_stall;
	output	reg	[31:0]	o_wb_data;
	input	wire		i_i2c_sck, i_i2c_sda;
	output	reg		o_i2c_sck, o_i2c_sda;
	//
	output	wire	[31:0]	o_dbg;

	reg	[31:0]	mem	[0:((1<<(MEM_ADDR_BITS-2))-1)];

`ifndef	VERILATOR
	initial begin
		if (INITIAL_MEM != "")
			$readmemh(INITIAL_MEM, mem);
	end
`endif

	reg	[4:0]	wr_stb;
	reg	[7:0]	i2c_addr;
	wire	[7:0]	wr_data;

	reg	[3:0]	r_we;
	reg	[31:0]	r_data;
	reg	[(MEM_ADDR_BITS-3):0]	r_addr;
	initial	r_we = 4'h0;
	always @(posedge i_clk)
	begin
		if (!READ_ONLY)
		begin
			if ((!I2C_READ_ONLY)&&(wr_stb[4]))
				r_we <= wr_stb[3:0];
			else if ((!WB_READ_ONLY)&&(i_wb_stb)&&(i_wb_we))
				r_we <= i_wb_sel;
			else
				r_we <= 4'h0;
			r_data  <= (wr_stb[4])? {(4){wr_data}} : i_wb_data;
			r_addr  <= (wr_stb[4]) ? i2c_addr[(MEM_ADDR_BITS-1):2]
					: i_wb_addr;
		end else
			r_we <= 4'h0;
			// data and address are don't cares if READ_ONLY is set
	end

	always @(posedge i_clk)
	begin
		if (!READ_ONLY)
		begin
			if (r_we[3])
				mem[r_addr][31:24] <= r_data[31:24];
			if (r_we[2])
				mem[r_addr][23:16] <= r_data[23:16];
			if (r_we[1])
				mem[r_addr][15: 8] <= r_data[15: 8];
			if (r_we[0])
				mem[r_addr][ 7: 0] <= r_data[ 7: 0];
		end
	end

	always @(posedge i_clk)
		o_wb_data <= mem[ i_wb_addr[(MEM_ADDR_BITS-3):0] ];

	initial	o_wb_ack = 1'b0;
	always @(posedge i_clk)
		o_wb_ack <= (i_wb_stb)&&(!o_wb_stall);

	assign	o_wb_stall = (!READ_ONLY)&&(wr_stb[4]);

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

	reg	[2:0]	i2c_state;
	reg	[7:0]	dreg, oreg, rd_val, i2c_rx_byte;
	wire	[7:0]	i2c_tx_byte;
	reg	[2:0]	dbits;
	reg		slave_tx_rx_n, i2c_slave_ack, i2c_rx_stb, i2c_tx_stb;
	initial	i2c_state = `I2CIDLE;
	initial	o_i2c_sck = 1'b1;
	initial	o_i2c_sda = 1'b1;
	initial	i2c_slave_ack  = 1'b1;
	always	@(posedge i_clk)
	begin
		// Default is to do nothing with the output ports.  A 1'b1 does
		// that.
		o_i2c_sck <= 1'b1;
		o_i2c_sda <= 1'b1;
		i2c_tx_stb <= 1'b0;
		i2c_rx_stb <= 1'b0;
		if (i2c_posedge)
			dreg  <= { dreg[6:0], this_sda };
		if (i2c_negedge)
			oreg  <= { oreg[6:0], oreg[0] };
		case(i2c_state)
		`I2CIDLE: begin
				dbits <= 0;
				if (i2c_start)
					i2c_state <= `I2CSTART;
			end
		`I2CSTART: begin
				dbits <= 0;
				if (i2c_negedge)
					i2c_state <= `I2CADDR;
			end
		`I2CADDR: begin
				if (i2c_negedge)
					dbits <= dbits + 1'b1;
				if ((i2c_negedge)&&(dbits == 3'h7))
				begin
					slave_tx_rx_n <= dreg[0];
					if (dreg[7:1] == SLAVE_ADDRESS)
					begin
						i2c_state <= `I2CSACK;
						i2c_slave_ack <= 1'b0;
					end else begin
						// Ignore this, its not for
						// me.
						i2c_state <= `I2CILLEGAL;
						i2c_slave_ack <= 1'b1;
					end
				end
			end
		`I2CSACK: begin
				dbits <= 3'h0;
				// NACK anything outside of our address range
				o_i2c_sda <= i2c_slave_ack;
				oreg <= rd_val;
				if (i2c_negedge)
				begin
					i2c_state <= (slave_tx_rx_n)?`I2CTX:`I2CRX;
					oreg <= i2c_tx_byte;
				end
			end
		`I2CRX: begin	// Slave reads from the bus
				//
				// First byte received is always the memory
				// address.
				//
				if (i2c_negedge)
					dbits <= dbits + 1'b1;
				if ((i2c_negedge)&&(dbits == 3'h7))
				begin
					i2c_rx_byte <= dreg;
					i2c_rx_stb  <= 1'b1;
					i2c_state <= `I2CSACK;
				end
			end
		`I2CTX: begin	// Slave transmits
				// Read from the slave (that's us)
				if (i2c_negedge)
					dbits <= dbits + 1'b1;
				if ((i2c_negedge)&&(dbits == 3'h7))
				begin
					i2c_tx_stb <= 1'b1;
					i2c_state <= `I2CMACK;
				end
				o_i2c_sda <= oreg[7];
			end
		`I2CMACK: begin
				dbits <= 3'h0;
				if (i2c_negedge)
				begin
					i2c_state <= `I2CTX;
					oreg <= i2c_tx_byte;
				end
				oreg <= rd_val;
			end
		`I2CILLEGAL:	dbits <= 3'h0;
		// default:	dbits <= 3'h0;
		endcase
		if (i2c_stop)
			i2c_state <= `I2CIDLE;
		else if (i2c_start)
			i2c_state <= `I2CSTART;
	end

	reg	[1:0]	bus_state;
	reg		go_bus_idle, wr_complete,
			bus_rd_stb, bus_wr_stb;
	//
	initial		wr_complete = 1'b0;
	initial		bus_rd_stb  = 1'b0;
	initial		bus_wr_stb  = 1'b0;
	initial		bus_state   = `BUS_IDLE;
	initial		go_bus_idle = 1'b0;
	always @(posedge i_clk)
	begin
		go_bus_idle <= ((i2c_state == `I2CIDLE)
				||(i2c_state == `I2CSTART)
				||(i2c_state == `I2CILLEGAL));
		bus_rd_stb <= 1'b0;
		bus_wr_stb <= 1'b0;
		if (go_bus_idle)
			bus_state <= `BUS_IDLE;
		else case(bus_state)
			`BUS_IDLE: begin
					if (i2c_rx_stb)
					begin
						i2c_addr <= i2c_rx_byte;
						bus_state <= `BUS_READ;
						bus_rd_stb <= 1'b1;
					end else if (i2c_tx_stb)
					begin
						bus_state <= `BUS_SEND;
						i2c_addr <= i2c_addr + 1'b1;
						bus_rd_stb <= 1'b1;
					end
				end
			`BUS_READ: if (i2c_rx_stb)
				begin
					// Reading from the bus means we are
					// writing to memory
					bus_wr_stb <= (!I2C_READ_ONLY);
				end
				// Increment the address once a write completes
				// if (wr_complete)
				//	i2c_addr <= i2c_addr + 1'b1;
			`BUS_SEND: if (i2c_tx_stb)
				begin
					// Once we've finished transmitting,
					// increment the address to read the
					// next item
					i2c_addr <= i2c_addr + 1'b1;
					bus_rd_stb <= 1'b1;
				end
			default: begin end
		endcase

		if (wr_complete)
		begin
			i2c_addr <= i2c_addr + 1'b1;
			bus_rd_stb <= 1'b1;
		end
	end

	reg	[2:0]	wr_pipe;
	initial	wr_pipe     = 3'h0;
	initial	wr_complete = 1'b0;
	always @(posedge i_clk)
		wr_pipe <= { wr_pipe[1:0], bus_wr_stb };
	always @(posedge i_clk)
		wr_complete <= wr_pipe[2];

	initial	wr_stb = 5'b0;
	always @(posedge i_clk)
	begin
		if (!I2C_READ_ONLY)
		begin
			wr_stb[4]  <= bus_wr_stb;
			wr_stb[3]  <= (bus_wr_stb)&&(i2c_addr[1:0]==2'b00);
			wr_stb[2]  <= (bus_wr_stb)&&(i2c_addr[1:0]==2'b01);
			wr_stb[1]  <= (bus_wr_stb)&&(i2c_addr[1:0]==2'b10);
			wr_stb[0]  <= (bus_wr_stb)&&(i2c_addr[1:0]==2'b11);
		end
	end

	assign	wr_data = i2c_rx_byte;


	reg	[31:0]	pipe_mem;
	reg	[1:0]	pipe_sel;
	always @(posedge i_clk)
		if(bus_rd_stb)
			pipe_mem <= mem[i2c_addr[(MEM_ADDR_BITS-1):2]];
	always @(posedge i_clk)
		if (bus_rd_stb)
			pipe_sel <= i2c_addr[1:0];
	//
	//
	always @(posedge i_clk)
		case(pipe_sel)
		2'b00: rd_val <= pipe_mem[31:24];
		2'b01: rd_val <= pipe_mem[23:16];
		2'b10: rd_val <= pipe_mem[15: 8];
		2'b11: rd_val <= pipe_mem[ 7: 0];
		endcase

	assign	i2c_tx_byte = rd_val;

	//
	//
	//
	reg	r_trigger;
	initial	r_trigger = 1'b0;
	always @(posedge i_clk)
		r_trigger <= i2c_start;
	assign	o_dbg = { r_trigger, 27'h0,
			i_i2c_sck, i_i2c_sda, o_i2c_sck, o_i2c_sda // 4b
			};
endmodule
