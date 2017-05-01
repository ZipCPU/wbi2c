////////////////////////////////////////////////////////////////////////////////
//
// Filename:	wbi2master.v
//
// Project:	WBI2C ... a set of Wishbone controlled I2C controller(s)
//
// Purpose:	This module communicates with an external I2C slave, allowing
//		the WB-master to command the reading and/or writing of I2C
//	values.  The slave is assumed to have 128 bytes of data memory.  This
//	core then allows those bytes to be read or written as commanded.
//
// Registers:
//
//	0,CMD	This is the command register.  Writes to this register will
//		initiate an I2C bus transaction.
//
//	   bits
//		15	Writing a '1' to this bit will reset the interface,
//			and abandon any transaction currently underway.
//		14.. 8	Number of values to read or write.  Ignored if zero.
//		 7.. 1	Initial address to read from or write to
//		     0	1 if reading from the slave, 0 if writing to it
//
//		On read, bits 14.. 8 will indicate the number of bytes
//			remaining to be read or written.  A 0 indicates the
//			interface is idle.
//
//		DEVICE ADDRESS (up to two bytes ?)
//			R/W
//		Address within the device (up to two bytes?)
//		Data address (one byte only)
//		Number of bytes to transmit (one byte)
//
//		Simple commands: DEV(R/W), Addr, NBytes
//		On read: DEV(R/W), and addr, if the address was accepted, and
//			references where the address pointer is at.
//		   BUSY bit	bit[31]
//		   ERR bit	bit[30]
//		NBytes is the number of bytes remaining in the transfer.
//
//		Features (not yet) supported:
//			2-byte device addresses
//			device addresses that don't match their memory addresses
//
//	1,SPD	Indicates the number of clocks per I2C bit.  This is a 20-bit
//		number that cannot be zero.  The actual speed of the port,
//		given this number, will be the system CLKFREQHZ/SPD.
//
//	2-17	address mapped onto 0-1.
//
//	128-255	This is the local copy of the memory shared between the master
//		and the slave.  When commanded to initiate a bus transaction,
//		the bus controller will read from or write to this memory.
//		In all other cases, it is completely accessable from the WB
//		bus.
//
// 
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2015-2017, Gisselquist Technology, LLC
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
//
`default_nettype	none
//
//
`define	I2MIDLE		3'h0
`define	I2MDEVADDR	3'h1
`define	I2MRDSTOP	3'h2
`define	I2MRDDEV	3'h3
`define	I2MTXDATA	3'h4
`define	I2MRXDATA	3'h5
`define	I2MCLEANUP	3'h6
//
//
module	wbi2cmaster(i_clk, i_rst,
		i_wb_cyc, i_wb_stb, i_wb_we, i_wb_addr, i_wb_data, i_wb_sel,
			o_wb_ack, o_wb_stall, o_wb_data,
		i_i2c_scl, i_i2c_sda, o_i2c_scl, o_i2c_sda, o_int,
		o_dbg
`ifdef	VERILATOR
		, i_vstate
`endif
		);
	parameter [0:0]	CONSTANT_SPEED = 1'b0, READ_ONLY = 1'b0;
	parameter [5:0]	TICKBITS = 6'd20;
	parameter [(TICKBITS-1):0]	CLOCKS_PER_TICK = 20'd1000;
	input	wire		i_clk, i_rst;
	// Input bus wires
	input	wire		i_wb_cyc, i_wb_stb, i_wb_we;
	input	wire	[5:0]	i_wb_addr;
	input	wire	[31:0]	i_wb_data;
	input	wire	[3:0]	i_wb_sel;
	// Output bus wires
	output	reg		o_wb_ack;
	output	wire		o_wb_stall;
	output	reg	[31:0]	o_wb_data;
	// I2C clock and data wires
	input	wire		i_i2c_scl, i_i2c_sda;
	output	wire		o_i2c_scl, o_i2c_sda;
	// And our output interrupt
	output	wire		o_int;
	// And some debug wires
	output	wire	[31:0]	o_dbg;
`ifdef	VERILATOR
	input	wire	[31:0]	i_vstate;
`endif



	//
	// Our shared memory structure -- it gets no initial value(s)
	//
	reg	[31:0]	mem	[0:31];

	// r_speed ... the programmable number of system clocks per I2C
	// wait state.  Nominally, this is one quarter the clock speed of the
	// I2C.
	reg				zero_speed_err;
	reg	[(TICKBITS-1):0]	r_speed;

	// Parameters used to control and read values from the lower level
	// I2C device driver we are working with.
	reg		ll_i2c_cyc, ll_i2c_stb, ll_i2c_we;
	reg	[7:0]	ll_i2c_tx_data;
	wire		ll_i2c_ack, ll_i2c_stall, ll_i2c_err;
	wire	[7:0]	ll_i2c_rx_data;
	wire	[31:0]	ll_dbg;
	//
	// The lower level module we are trying to drive
	//
	lli2cm lowlvl(i_clk, r_speed, ll_i2c_cyc, ll_i2c_stb, ll_i2c_we,
				ll_i2c_tx_data,
			ll_i2c_ack, ll_i2c_stall, ll_i2c_err, ll_i2c_rx_data,
			i_i2c_scl, i_i2c_sda, o_i2c_scl, o_i2c_sda, ll_dbg);


	//
	// Let's interact with the wishbone bus
	//

	// First, to arbitrate who has access to memory, and yet to keep our
	// block RAM, we'll create an intermediate data structure and delay
	// any writes to RAM by one clock.
	reg	[6:0]	wr_addr;
	reg	[3:0]	wr_sel;
	reg	[31:0]	wr_data;
	reg		wr_inc;
	reg		r_write_lock;
	//
	// Variables to define the request we are in the process of making
	reg		start_request;
	reg	[7:1]	newdev;
	reg		newrx_txn;
	reg	[6:0]	newadr;
	reg	[6:0]	newcnt;
	//
	reg		r_busy;
	//
	initial	start_request = 1'b0;
	initial	newdev    = 7'h0;
	initial	newrx_txn = 1'b0;
	initial	newadr    = 7'h0;
	initial	newcnt    = 7'h0;
	initial	r_speed   = CLOCKS_PER_TICK;
	initial	zero_speed_err = 1'b0;
	always @(posedge i_clk)
	begin	// Writes from the master wishbone bus
		start_request <= 1'b0;
		if ((i_wb_stb)&&(i_wb_we)&&(!r_busy)&&(!i_wb_addr[5]))
		begin
			if (!i_wb_addr[0])
			begin
				newdev     <= i_wb_data[23:17];
				newrx_txn  <= i_wb_data[16];
				newadr     <= i_wb_data[14: 8];
				newcnt     <= i_wb_data[ 6: 0];

				start_request <= (i_wb_data[6:0] != 0)
					&&((!READ_ONLY)||(newrx_txn));
			end

			if ((i_wb_addr[0])&&(!CONSTANT_SPEED))
				r_speed <= i_wb_data[(TICKBITS-1):0];
		end else if (zero_speed_err)
			r_speed <= CLOCKS_PER_TICK;
		zero_speed_err <= (r_speed == 0);

		wr_sel <= 4'h0;
		wr_inc <= 1'b0;
		if (r_write_lock)
		begin
			if (ll_i2c_ack)
			begin
				wr_data <= { (4) {ll_i2c_rx_data} };
				wr_addr <= newadr[6:0];
				case(newadr[1:0])
				2'b00: wr_sel <= 4'b1000;
				2'b01: wr_sel <= 4'b0100;
				2'b10: wr_sel <= 4'b0010;
				2'b11: wr_sel <= 4'b0001;
				endcase
				newadr <= newadr + 1'b1;
				wr_inc <= 1'b1;
			end
		end else if (!READ_ONLY) begin
			wr_data <= i_wb_data;
			wr_sel  <= ((i_wb_stb)&&(i_wb_we)&&(i_wb_addr[5]))
					? i_wb_sel:4'h0;
			wr_addr <= { i_wb_addr[4:0], 2'b00 };
		end

		if (wr_sel[3])
			mem[wr_addr[6:2]][31:24] <= wr_data[31:24];
		if (wr_sel[2])
			mem[wr_addr[6:2]][23:16] <= wr_data[23:16];
		if (wr_sel[1])
			mem[wr_addr[6:2]][15: 8] <= wr_data[15: 8];
		if (wr_sel[0])
			mem[wr_addr[6:2]][ 7: 0] <= wr_data[ 7: 0];
	end

	reg		last_op;
	reg		rd_inc;
	reg		last_err;
	reg	[6:0]	last_dev;
	reg	[6:0]	last_adr;
	reg	[7:0]	count_left;
	initial	rd_inc = 1'b0;
	wire	[31:0]	w_wb_status;
	assign	w_wb_status = { r_busy, last_err, 6'h0,
				last_dev, 2'b0, last_adr, count_left };
	always @(posedge i_clk)
	begin // Read values and place them on the master wishbone bus.
		last_op <= (count_left[7:0] == 0);
		if ((i_wb_stb)&&(i_wb_we)&&(!r_busy)&&(!i_wb_addr[0]))
			count_left  <= i_wb_data[ 7: 0];
		if (wr_inc)
		begin
			last_dev <= newdev;
			last_adr <= wr_addr+1'b1;
			if (|count_left)
				count_left <= count_left - 1'b1;
		end else if (rd_inc)
		begin
			last_dev <= newdev;
			last_adr <= rd_addr - 1'b1;
			if (|count_left)
				count_left <= count_left - 1'b1;
		end

		casez({i_wb_addr[5], i_wb_addr[0]})
		2'b00: o_wb_data <= w_wb_status;
		2'b01: o_wb_data <= { {(32-TICKBITS){1'b0}}, r_speed };
		2'b1?: o_wb_data <= mem[i_wb_addr[4:0]];
		endcase
	end

	always @(posedge i_clk)
		o_wb_ack <= i_wb_stb;
	assign	o_wb_stall = 1'b0;


	reg		rd_stb;
	reg	[6:0]	rd_addr;
	reg	[31:0]	rd_word;
	reg	[7:0]	rd_byte;
	reg	[1:0]	rd_sel;
	always @(posedge i_clk)
	begin
		if (rd_stb)
		begin
			rd_word <= mem[rd_addr[6:2]];
			rd_sel  <= rd_addr[1:0];
		end

		case(rd_sel)
		2'b00: rd_byte <= rd_word[31:24];
		2'b01: rd_byte <= rd_word[23:16];
		2'b10: rd_byte <= rd_word[15: 8];
		2'b11: rd_byte <= rd_word[ 7: 0];
		endcase
	end


	//
	//
	// The master state machine
	//
	//
	reg		last_ack, last_addr_flag;
	reg	[2:0]	mstate;
	reg	[1:0]	acks_pending;
	initial		r_write_lock = 1'b0;
	reg	[1:0]	r_write_pause;
	initial	mstate = `I2MIDLE;
	initial	r_busy = 1'b0;
	always @(posedge i_clk)
	begin
		if (!ll_i2c_cyc)
			last_addr_flag <= 1'b0;
		else if (last_op)
			last_addr_flag <= 1'b1;
		rd_stb <= 1'b0;

		if ((!r_busy)&&(i_wb_stb)&&(!i_wb_addr[0]))
			last_err <= 1'b0;
		else if ((r_busy)&&(ll_i2c_err))
			last_err <= 1'b1;

		if (mstate == `I2MIDLE)
			acks_pending <= 2'h0;
		else case( { (ll_i2c_stb)&&(!ll_i2c_stall), ll_i2c_ack })
		2'b00: acks_pending <= acks_pending;
		2'b01: acks_pending <= (|acks_pending)? (acks_pending - 1'b1):0;
		2'b10: acks_pending <= acks_pending + 1'b1;
		2'b11: acks_pending <= acks_pending;
		endcase

		last_ack <= (acks_pending[1] == 1'b0)&&(!ll_i2c_stb);

		rd_inc <= 1'b0;
		case(mstate)
		`I2MIDLE: begin
			ll_i2c_cyc <= 1'b0;
			ll_i2c_stb <= 1'b0;
			r_write_lock <= 1'b0;
			if ((start_request)&&(!ll_i2c_stall))
			begin
				ll_i2c_cyc <= 1'b1;
				ll_i2c_stb <= 1'b1;
				ll_i2c_we  <= 1'b1;
				// We start, always, by writing the address out
				ll_i2c_tx_data<= { newdev, 1'b0 };
				rd_addr <= newadr;
				mstate <= `I2MDEVADDR;
				rd_stb <= 1'b1;
				r_busy <= 1'b1;
			end else r_busy <= ll_i2c_stall;
			end
		`I2MDEVADDR: begin
			r_write_lock <= 1'b0;
			if (!ll_i2c_stall)
			begin
				ll_i2c_we  <= 1'b1;	// Still writing
				ll_i2c_stb <= 1'b1;
				ll_i2c_tx_data <= { 1'b0, newadr };
				if (newrx_txn)
					mstate <= `I2MRDSTOP;
				else begin
					mstate <= `I2MTXDATA;
				end
			end end
		`I2MRDSTOP: begin // going to read, need to send the dev addr
			// First thing we have to do is end our transaction
			r_write_lock <= 1'b0;
			if (!ll_i2c_stall)
			begin
				ll_i2c_stb <= 1'b0;
			end

			if ((!ll_i2c_stb)&&(last_ack)&&(ll_i2c_ack))
			begin
				ll_i2c_cyc <= 1'b0;
				mstate <= `I2MRDDEV;
			end end
		`I2MRDDEV: begin
			ll_i2c_stb <= 1'b0;
			r_write_lock <= 1'b0;
			if (!ll_i2c_stall) // Wait 'til its no longer busy
			begin // Fire us up again
				ll_i2c_cyc <= 1'b1;
				ll_i2c_stb <= 1'b1;
				ll_i2c_we  <= 1'b1;
				ll_i2c_tx_data <= { newdev, 1'b1 };
				mstate <= `I2MRXDATA;
				r_write_pause <= 2'b01;
			end end
		`I2MTXDATA: begin // We are sending to the slave
			ll_i2c_stb <= 1'b1;
			r_write_lock <= 1'b0;
			if (!ll_i2c_stall)
			begin
				rd_inc <= 1'b1;
				rd_addr <= rd_addr + 1'b1;
				ll_i2c_tx_data <= rd_byte;
				rd_stb <= 1'b1;
				if (last_addr_flag)
				begin
					ll_i2c_stb <= 1'b0;
					mstate <= `I2MCLEANUP;
				end
			end end
		`I2MRXDATA: begin
			ll_i2c_we <= 1'b0;
			if (!ll_i2c_stall)
			begin
				if (|r_write_pause)
					r_write_pause <= r_write_pause - 1'b1;
				r_write_lock <= (r_write_pause == 2'b00);
				ll_i2c_tx_data <= rd_byte;
			end
			if (last_addr_flag)
			begin
				ll_i2c_stb <= 1'b0;
				mstate <= `I2MCLEANUP;
			end end
		`I2MCLEANUP: begin
			ll_i2c_cyc <= 1'b1;
			ll_i2c_stb <= 1'b0;
			if ((ll_i2c_we)&&(ll_i2c_ack))
				rd_inc <= 1'b1;
			if (last_ack)
			begin
				mstate <= `I2MIDLE;
				ll_i2c_cyc <= 1'b1;
			end end
		default: mstate <= `I2MIDLE;
		endcase
	end

	assign	o_int = !r_busy;

	assign	o_dbg = { ll_dbg[31:29],
		last_adr, wr_inc, count_left[5:0],
		ll_dbg[14:0] };
endmodule

