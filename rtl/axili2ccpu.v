////////////////////////////////////////////////////////////////////////////////
//
// Filename:	axili2ccpu.v
// {{{
// Project:	WBI2C ... a set of Wishbone controlled I2C controller(s)
//
// Purpose:	This is a copy of the WBI2CCPU, save that it has been modified
//		to work with AXI4 instead of Wishbone.
//
// Registers
// {{{
//	0. Stop control
//		SDA,SCL status & override
//		Soft halt request (halt on STOP or o.w. inactive)
//		Hard halt request (halt in any state)
//		Watchdog timeout
//		Issue direct instructions if not active
//		Halt control
//	1. Override
//		Writes instructions to the port
//	2. Address control
//		Writes set the address, unstop the CPU, and cause a jump to that
//			address
//		Reads return the current address
//	3. Clock control
//		(May not be required)
// }}}
//
// Instruction set:
// {{{
//	4'h,4'h		Two instructions per byte
//
//	4'h0		NOP
//	4'h1		START
//	4'h2		STOP
//	4'h3		RXK
//	4'h4		RXN	// RX byte, NAK result
//	4'h5		RXLK	// Cn't include STOP, bc we might want rptd strt
//	4'h6		RXLN	// ditto
//	4'h7		SEND
//	4'h8		WAIT
//	4'h9		HALT
//	4'ha		ABORT
//	4'hb		TARGET
//	4'hc		JUMP
//	4'hd		(Undef/Illegal Insn)
//	4'he		(Undef/Illegal Insn)
//	4'hf		(Undef/Illegal Insn)
// }}}
//
// Dependencies:
//	axilfetch.v	From the wb2axip repo, rtl/ directory
//	skidbuffer.v	From the wb2axip repo, rtl/ directory
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2021, Gisselquist Technology, LLC
// {{{
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
// }}}
// License:	GPL, v3, as defined and found on www.gnu.org,
// {{{
//		http://www.gnu.org/licenses/gpl.html
//
////////////////////////////////////////////////////////////////////////////////
//
`default_nettype none
// }}}
module	axili2ccpu #(
		// {{{
		parameter	ADDRESS_WIDTH = 32,
		parameter	DATA_WIDTH = 32,
		parameter	C_AXI_ADDR_WIDTH = ADDRESS_WIDTH,
		parameter	C_AXI_DATA_WIDTH = DATA_WIDTH,
		localparam	AW = C_AXI_ADDR_WIDTH,
		localparam	DW = C_AXI_DATA_WIDTH,
		parameter [AW-1:0]	RESET_ADDRESS = 0,
		parameter [0:0]	OPT_START_HALTED = (RESET_ADDRESS == 0),
`ifdef	FORMAL
		parameter [11:0]	DEF_CKCOUNT = 2,
`else
		parameter [11:0]	DEF_CKCOUNT = -1,
`endif
		parameter [0:0]	OPT_LOWPOWER = 1'b0
		// }}}
	) (
		// {{{
		input	wire		S_AXI_ACLK, S_AXI_ARESETN,
		// Bus slave interface
		// {{{
		input	wire		S_AXI_AWVALID,
		output	wire		S_AXI_AWREADY,
		input	wire	[3:0]	S_AXI_AWADDR,
		input	wire	[2:0]	S_AXI_AWPROT,
		//
		input	wire		S_AXI_WVALID,
		output	wire		S_AXI_WREADY,
		input	wire	[31:0]	S_AXI_WDATA,
		input	wire	[3:0]	S_AXI_WSTRB,
		//
		output	wire		S_AXI_BVALID,
		input	wire		S_AXI_BREADY,
		output	wire	[1:0]	S_AXI_BRESP,
		//
		input	wire		S_AXI_ARVALID,
		output	wire		S_AXI_ARREADY,
		input	wire	[3:0]	S_AXI_ARADDR,
		input	wire	[2:0]	S_AXI_ARPROT,
		//
		output	wire		S_AXI_RVALID,
		input	wire		S_AXI_RREADY,
		output	wire	[31:0]	S_AXI_RDATA,
		output	wire	[1:0]	S_AXI_RRESP,
		// }}}
		// Bus master interface
		// {{{
		output	wire		M_INSN_ARVALID,
		input	wire		M_INSN_ARREADY,
		output	wire [AW-1:0]	M_INSN_ARADDR,
		output	wire	[2:0]	M_INSN_ARPROT,
		//
		input	wire		M_INSN_RVALID,
		output	wire		M_INSN_RREADY,
		input	wire [DW-1:0]	M_INSN_RDATA,
		input	wire	[1:0]	M_INSN_RRESP,
		// }}}
		// I2C interfacce
		// {{{
		input	wire			i_i2c_sda, i_i2c_scl,
		output	wire			o_i2c_sda, o_i2c_scl,
		// }}}
		// Outgoing stream interface
		//  {{{
		output	wire			M_AXIS_TVALID,
		input	wire			M_AXIS_TREADY,
		output	wire	[7:0]		M_AXIS_TDATA,
		output	wire			M_AXIS_TLAST,
		// OPT output wire		M_AXIS_TABORT,
		// }}}
		input	wire	i_sync_signal
		// }}}
	);

	// Local declarations
	// {{{
	wire	i_clk = S_AXI_ACLK;
	wire	i_reset = !S_AXI_ARESETN;

	// Addresses
	// {{{
	localparam	[1:0]	ADR_CONTROL = 2'b00,
				ADR_OVERRIDE= 2'b01,
				ADR_ADDRESS = 2'b10,
				ADR_CKCOUNT = 2'b11;
	// }}}

	// Command register bit enumeration(s)
	// {{{
	localparam	HALT_BIT = 16,
			ERR_BIT = 17,
			SOFTHALT_BIT = 18;
			//
			// WAIT_BIT = 19,
			// IMMCYCLE_BIT = 20,
			// HALFVALID_BIT = 21;
	// }}}
	localparam	OVW_VALID = 9;

	// Instruction set / Commands
	// {{{
	localparam	[3:0]	CMD_NOOP  = 4'h0,
				// CMD_START = 4'h1,
				CMD_STOP  = 4'h2,
				// CMD_RXK   = 4'h3,
				// CMD_RXN   = 4'h4,
				// CMD_RXLK  = 4'h5,
				// CMD_RXLN  = 4'h6,
				CMD_SEND  = 4'h7,
				CMD_WAIT  = 4'h8,
				CMD_HALT  = 4'h9,
				CMD_ABORT = 4'ha,
				CMD_TARGET= 4'hb,
				CMD_JUMP  = 4'hc;
	// }}}

	wire			cpu_reset, cpu_clear_cache;
	reg			cpu_new_pc;
	reg	[AW-1:0]	pf_jump_addr;
	wire			pf_valid;
	wire			pf_ready;
	wire	[7:0]		pf_insn;
	wire	[AW-1:0]	pf_insn_addr;
	wire			pf_illegal;

	reg			half_valid, imm_cycle;

	wire			insn_ready, half_ready, i2c_abort;
	reg			insn_valid;
	reg	[11:0]		insn;
	reg	[3:0]		half_insn;
	reg			i2c_ckedge, i2c_stretch;
	reg	[11:0]		i2c_ckcount, ckcount;
	reg	[AW-1:0]	abort_address, jump_target;
	reg			r_wait, soft_halt_request, r_halted, r_err;
	wire			w_stopped;

	reg		s_axi_bvalid, s_axi_rvalid;
	wire		skd_awvalid, skd_wvalid, skd_arvalid;
	wire		bus_read, bus_write, bus_override, ovw_ready;
	wire	[1:0]	bus_write_addr, bus_read_addr;
	wire	[31:0]	bus_write_data;
	wire	[3:0]	bus_write_strb;
	reg	[31:0]	bus_read_data;

	wire	s_tvalid;
	reg	[9:0]	ovw_data;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// AXI4-lite Bus handling
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	// Write address skid buffer
	skidbuffer #(
		// {{{
		.OPT_LOWPOWER(OPT_LOWPOWER),
		.OPT_OUTREG(0),
		.DW(2)
		// }}}
	) awskd (
		// {{{
		.i_clk(S_AXI_ACLK), .i_reset(!S_AXI_ARESETN),
		.i_valid(S_AXI_AWVALID), .o_ready(S_AXI_AWREADY),
			.i_data(S_AXI_AWADDR[3:2]),
		.o_valid(skd_awvalid), .i_ready(bus_write),
			.o_data(bus_write_addr)
		// }}}
	);

	// Write data skid buffer
	skidbuffer #(
		// {{{
		.OPT_LOWPOWER(OPT_LOWPOWER),
		.OPT_OUTREG(0),
		.DW(32+32/8)
		// }}}
	) wskd (
		// {{{
		.i_clk(S_AXI_ACLK), .i_reset(!S_AXI_ARESETN),
		.i_valid(S_AXI_WVALID), .o_ready(S_AXI_WREADY),
			.i_data({ S_AXI_WDATA, S_AXI_WSTRB }),
		.o_valid(skd_wvalid), .i_ready(bus_write),
			.o_data({ bus_write_data, bus_write_strb })
		// }}}
	);

	// Read address skid buffer
	skidbuffer #(
		// {{{
		.OPT_LOWPOWER(OPT_LOWPOWER),
		.OPT_OUTREG(0),
		.DW(2)
		// }}}
	) arskd (
		// {{{
		.i_clk(S_AXI_ACLK), .i_reset(!S_AXI_ARESETN),
		.i_valid(S_AXI_ARVALID), .o_ready(S_AXI_ARREADY),
			.i_data(S_AXI_ARADDR[3:2]),
		.o_valid(skd_arvalid), .i_ready(bus_read),
			.o_data(bus_read_addr)
		// }}}
	);


	assign	bus_write      = skd_awvalid && skd_wvalid && (!S_AXI_BVALID || S_AXI_BREADY);
	assign	bus_read       = skd_arvalid && (!S_AXI_RVALID || S_AXI_RREADY);

	// Write response
	// {{{
	initial	s_axi_bvalid = 1'b0;
	always @(posedge i_clk)
	if (i_reset)
		s_axi_bvalid <= 1'b0;
	else if (bus_write)
		s_axi_bvalid <= 1'b1;
	else if (S_AXI_BREADY)
		s_axi_bvalid <= 1'b0;

	assign	S_AXI_BVALID = s_axi_bvalid;
	// }}}

	assign	S_AXI_BRESP = 2'b00;
	assign	S_AXI_RRESP = 2'b00;

	// Read handling
	// {{{
	initial	s_axi_rvalid = 1'b0;
	always @(posedge i_clk)
	if (i_reset)
		s_axi_rvalid <= 0;
	else if (bus_read)
		s_axi_rvalid <= 1;
	else if (S_AXI_RREADY)
		s_axi_rvalid <= 0;

	always @(posedge i_clk)
	if (OPT_LOWPOWER && i_reset)
		bus_read_data <= 0;
	else if (bus_read)
	begin
		bus_read_data <= 0;
		case(bus_read_addr)
		ADR_CONTROL:	bus_read_data <= {
					half_insn, 4'h0,
					//
					2'b0, half_valid, imm_cycle,
					r_wait, soft_halt_request,
						r_err, r_halted,
					//
					o_i2c_scl, o_i2c_sda,
						i_i2c_scl, i_i2c_sda,
					//
					insn	// 12 bits
					};
		ADR_OVERRIDE:	bus_read_data[9:0] <= ovw_data;
		ADR_ADDRESS:	bus_read_data[AW-1:0] <= pf_insn_addr;
		ADR_CKCOUNT:	bus_read_data[11:0] <= ckcount;
		// default:	bus_read_data <= 0;
		endcase

		// if(OPT_LOWPOWER && !bus_read)
		//	bus_read_data <= 0;
	end else if (OPT_LOWPOWER && S_AXI_RREADY)
		bus_read_data <= 0;

	assign	S_AXI_RVALID = s_axi_rvalid;
	assign	S_AXI_RDATA  = bus_read_data;
	// }}}

	assign	bus_override   = r_halted && bus_write
			&& bus_write_addr == ADR_OVERRIDE && bus_write_strb[0];
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Instruction fetch
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	assign	cpu_reset = r_halted;
	assign	cpu_clear_cache = 1'b0;

`ifndef	FORMAL
	axilfetch #(
		// {{{
		.C_AXI_ADDR_WIDTH(AW),
		.C_AXI_DATA_WIDTH(DW),
		.INSN_WIDTH(8),
		.FETCH_LIMIT(2)
		// }}}
	) u_fetch (
		// {{{
		.S_AXI_ACLK(S_AXI_ACLK), .S_AXI_ARESETN(S_AXI_ARESETN),
		// {{{
		.i_cpu_reset(cpu_reset),
		.i_new_pc(cpu_new_pc), .i_clear_cache(cpu_clear_cache),
		.i_ready(pf_ready), .i_pc(pf_jump_addr),
		.o_valid(pf_valid), .o_illegal(pf_illegal),
		.o_insn(pf_insn), .o_pc(pf_insn_addr),
		// }}}
		// AXI-lite bus master interface
		// {{{
		.M_AXI_ARVALID(M_INSN_ARVALID), .M_AXI_ARREADY(M_INSN_ARREADY),
		.M_AXI_ARADDR(M_INSN_ARADDR), .M_AXI_ARPROT(M_INSN_ARPROT),
		.M_AXI_RVALID(M_INSN_RVALID), .M_AXI_RREADY(M_INSN_RREADY),
		.M_AXI_RDATA(M_INSN_RDATA), .M_AXI_RRESP(M_INSN_RRESP)
		// }}}
		// }}}
	);
`endif

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Decode pipeline, and non-i2c handling
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	// half_valid
	// {{{
	always @(posedge i_clk)
	if (i_reset || i2c_abort)
		half_valid <= 1'b0;
	else if (!imm_cycle && pf_valid && pf_ready
				&& pf_insn[7:4] != CMD_SEND
				&& pf_insn[3:0] != CMD_NOOP)
		half_valid <= 1'b1;
	else if (!imm_cycle && bus_override && ovw_ready
			&& bus_write_data[7:4] != CMD_SEND
			&& bus_write_data[3:0] != CMD_NOOP)
		half_valid <= 1'b1;
	else if (bus_write && bus_write_addr == ADR_ADDRESS)
		half_valid <= 1'b0;
	else if (half_ready)
		half_valid <= 1'b0;
	// }}}

	// imm_cycle
	// {{{
	always @(posedge i_clk)
	if (i_reset || cpu_new_pc || cpu_clear_cache || i2c_abort)
		imm_cycle <= 1'b0;
	else if (!imm_cycle && (
		(pf_valid && pf_ready && pf_insn[7:4]== CMD_SEND)
		||(bus_override && ovw_ready && bus_write_data[7:4]== CMD_SEND)
		||(half_valid && half_ready && half_insn[3:0] == CMD_SEND)))
		imm_cycle <= 1'b1;
	else if (bus_write && bus_write_addr == ADR_ADDRESS)
		imm_cycle <= 1'b0;
	else if ((pf_valid && pf_ready) || (bus_override && ovw_ready))
		imm_cycle <= 1'b0;
	// }}}

	// cpu_new_pc, pf_jump_addr
	// {{{
	always @(posedge i_clk)
	if (i_reset)
	begin
		cpu_new_pc   <= !OPT_START_HALTED;
		pf_jump_addr <= RESET_ADDRESS;
	end else begin

		cpu_new_pc <= 1'b0;
		if (cpu_new_pc || (pf_valid && pf_ready))
			pf_jump_addr <= pf_jump_addr + 1;

		// Jump instruction
		// {{{
		if ((pf_valid && pf_ready && pf_insn[7:4] == CMD_JUMP)
			||(half_valid && half_ready
						&& half_insn[3:0] == CMD_JUMP))
		begin
			cpu_new_pc   <= 1'b1;
			pf_jump_addr <= jump_target;
		end
		// }}}

		// Abort an I2C command
		// {{{
		if (i2c_abort)
		begin
			cpu_new_pc   <= 1'b1;
			pf_jump_addr <= abort_address;
		end
		// }}}

		// CPU commanded jump
		// {{{
		if (bus_write && bus_write_addr == ADR_ADDRESS)
		begin
			cpu_new_pc   <= 1'b1;
			pf_jump_addr <= bus_write_data[AW-1:0];
		end
		// }}}
	end
	// }}}	

	assign	pf_ready = !w_stopped && !half_valid
			&& (!insn_valid || ((insn[11] || insn_ready)&&!r_wait)
				) && !cpu_new_pc;
	assign	half_ready = !r_wait
		&& (!insn_valid || ((insn[11] || insn_ready) && !r_wait));

	assign	ovw_ready = !half_valid
			&& (!insn_valid || ((insn[11] || insn_ready)&&!r_wait)
				);

	// insn_valid
	// {{{
	initial	insn_valid = 1'b0;
	always @(posedge i_clk)
	if (i_reset || i2c_abort)
		insn_valid <= 1'b0;
	else if (insn_valid && insn_ready && insn[11:8] == CMD_HALT)
		insn_valid <= 1'b0;
	else if (pf_valid && pf_ready)
	begin
		insn_valid <= imm_cycle || (pf_insn[7:4] != CMD_SEND
					&& pf_insn[7:4] != CMD_NOOP);
	end else if (bus_override && ovw_ready)
		insn_valid <= imm_cycle || (bus_write_data[7:4] != CMD_SEND
					&& bus_write_data[7:4] != CMD_NOOP);
	else if ((!half_valid || half_insn == CMD_SEND) && insn_ready)
		insn_valid <= 1'b0;
	// }}}

	// insn
	// {{{
	always @(posedge i_clk)
	if (pf_valid && pf_ready)
	begin
		if (imm_cycle)
		begin
			insn[7:0] <= pf_insn;
			half_insn <= CMD_NOOP;
		end else
			{ insn[11:8], half_insn } <= pf_insn;
	end else if (bus_override && ovw_ready)
	begin
		if (imm_cycle)
		begin
			insn[7:0] <= bus_write_data[7:0];
			half_insn <= CMD_NOOP;
		end else
			{ insn[11:8], half_insn } <= bus_write_data[7:0];
	end else if (!imm_cycle && insn_ready)
		{ insn[11:8], half_insn } <= { half_insn, CMD_NOOP };
	// }}}

	// ckcount
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		ckcount <= DEF_CKCOUNT;
	else if (bus_write && bus_write_addr == ADR_CKCOUNT)
		ckcount <= bus_write_data[11:0];
	// }}}

	// i2c_ckedge
	// {{{
	always @(posedge i_clk)
	if (i_reset)
	begin
		i2c_ckedge <= 0;
		i2c_ckcount <= DEF_CKCOUNT;
	end else if (!i2c_ckedge || !i2c_stretch)
	begin
		if (i2c_ckcount > 0)
			i2c_ckcount <= i2c_ckcount - 1;
		else
			i2c_ckcount <= ckcount;

		if (i2c_ckedge)
		begin
			i2c_ckedge  <= (ckcount == 0);
		end else
			i2c_ckedge  <= (i2c_ckcount <= 1);
	end
`ifdef	FORMAL
	always @(*)
	if (!i_reset)
		assert(i2c_ckedge == (i2c_ckcount == 0));

`ifdef	COVER
	(* anyconst *)	reg	f_cvrspeed;
	reg	cvr_active;

	initial	cvr_active <= 0;
	always @(posedge i_clk)
	if (insn_valid)
		cvr_active <= 1;

	always @(*)
		assume(f_cvrspeed);

	always @(posedge i_clk)
	if (cvr_active && f_cvrspeed)
		assume(ckcount == 1);
`endif
`endif
	// }}}

	// abort_address
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		abort_address <= RESET_ADDRESS;
	else if (bus_write && bus_write_addr == ADR_ADDRESS)
		abort_address <= bus_write_data[AW-1:0];
	else if (pf_valid && pf_ready && !imm_cycle && pf_insn[7:4]== CMD_ABORT)
			// || pf_insn == { CMD_START, CMD_SEND })
		abort_address <= pf_insn_addr + 1;
	// }}}

	// jump_target
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		jump_target <= RESET_ADDRESS;
	else if (bus_write && bus_write_addr == ADR_ADDRESS)
		jump_target <= bus_write_data[AW-1:0];
	else if (pf_valid && pf_ready && !imm_cycle
			&& pf_insn[7:4] == CMD_TARGET)
		jump_target <= pf_insn_addr + 1;
	// }}}

	// r_wait
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		r_wait <= 1'b0;
	else if (!r_halted && i_sync_signal)
		r_wait <= 1'b0;
	else begin
		if (insn_valid && insn_ready && insn[7:4] == CMD_WAIT)
			r_wait <= 1'b1;
		if (bus_write && bus_write_addr == ADR_ADDRESS)
			r_wait <= 1'b0;
	end
	// }}}

	// soft_halt_request
	// {{{
	always @(posedge i_clk)
	if (i_reset || r_halted)
		soft_halt_request <= 1'b0;
	else if (bus_write && bus_write_addr == ADR_CONTROL
			&& bus_write_data[SOFTHALT_BIT]
			&& bus_write_strb[SOFTHALT_BIT/8])
		soft_halt_request <= 1'b1;
	// }}}

	// r_halted
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		r_halted <= OPT_START_HALTED;
	else begin
		if (insn_valid && insn_ready && insn[11:8] == CMD_STOP
				&& soft_halt_request)
			r_halted <= 1'b1;
		if (pf_valid && pf_ready && pf_illegal)
			r_halted <= 1'b1;
		if (insn_valid && insn_ready && insn[11:8] == CMD_HALT)
			r_halted <= 1'b1;

		if (bus_write)
		begin
			if (bus_write_addr == ADR_CONTROL
					&& bus_write_data[HALT_BIT]
					&& bus_write_strb[HALT_BIT/8])
				r_halted <= 1'b1;
			if (bus_write_addr == ADR_ADDRESS && r_halted)
				r_halted <= 1'b0;
		end
	end
	// }}}

	// r_err
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		r_err <= 1'b0;
	else begin
		if (pf_valid && pf_ready && pf_illegal)
			r_err <= 1'b1;

		if (bus_write)
		begin
			if (bus_write_addr == ADR_CONTROL && bus_write_data[ERR_BIT]
						&& bus_write_strb[ERR_BIT/8])
				r_err <= 1'b0;
		end
	end
	// }}}

	// ovw_data
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		ovw_data <= 10'h00;
	else if (!r_halted)
		ovw_data[OVW_VALID] <= 1'b0;
	else if (M_AXIS_TVALID)
		ovw_data <= { 1'b1, M_AXIS_TLAST, M_AXIS_TDATA };
	else if (bus_write && bus_write_addr == ADR_OVERRIDE)
		ovw_data[OVW_VALID] <= 1'b0;
	// }}}

	assign	w_stopped = r_wait || r_halted;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// I2C Sub-module
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	assign	s_tvalid = insn_valid && !insn[11] && !r_wait;

	axisi2c #(
		// {{{
		.OPT_WATCHDOG(0),
		.OPT_LOWPOWER(OPT_LOWPOWER)
		// }}}
	) u_axisi2c (
		// {{{
		.S_AXI_ACLK(i_clk), .S_AXI_ARESETN(!i_reset),
		//
		// Incoming instruction stream
		// {{{
		.S_AXIS_TVALID(s_tvalid), .S_AXIS_TREADY(insn_ready),
			.S_AXIS_TDATA(insn[10:0]),
		// }}}
		// Outgoing received data stream
		// {{{
		.M_AXIS_TVALID(M_AXIS_TVALID), .M_AXIS_TREADY(M_AXIS_TREADY),
			.M_AXIS_TDATA(M_AXIS_TDATA),
			.M_AXIS_TLAST(M_AXIS_TLAST),
		// }}}
		// Control interface
		// {{{
		.i_ckedge(i2c_ckedge),
		.o_stretch(i2c_stretch),
		.o_abort(i2c_abort),
		// }}}
		.i_scl(i_i2c_scl), .i_sda(i_i2c_sda),
		.o_scl(o_i2c_scl), .o_sda(o_i2c_sda)
		// }}}
	);
	// }}}

	// Keep Verilator happy
	// {{{
	// Verilator lint_off UNUSED
	wire	unused;
	assign	unused = &{ 1'b0, S_AXI_AWADDR[1:0], S_AXI_ARADDR[1:0],
			S_AXI_AWPROT, S_AXI_ARPROT };
	// Verilator lint_off UNUSED
	// }}}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// Formal properties
// {{{
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
`ifdef	FORMAL
	reg	f_past_valid;
	wire	[1:0]	faxil_rd_outstanding, faxil_wr_outstanding,
			faxil_awr_outstanding;
	wire	[7:0]	f_const_insn;
	wire		f_const_illegal;
	wire [AW-1:0]	f_address, f_const_addr;


	initial	f_past_valid = 1'b0;
	always @(posedge i_clk)
		f_past_valid <= 1'b1;

	always @(*)
	if (!f_past_valid)
		assume(i_reset);

	////////////////////////////////////////////////////////////////////////
	//
	// Assert bus compliance
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	faxil_slave #(
		// {{{
		.C_AXI_DATA_WIDTH(32), .C_AXI_ADDR_WIDTH(4),
		// .F_MAX_ACK_DELAY(2),
		.F_LGDEPTH(2)
		// }}}
	) slv (
		// {{{
		.i_clk(i_clk), .i_axi_reset_n(S_AXI_ARESETN),
		.i_axi_awvalid(S_AXI_AWVALID), .i_axi_awready(S_AXI_AWREADY),
			.i_axi_awaddr(S_AXI_AWADDR),.i_axi_awprot(S_AXI_AWPROT),
		.i_axi_wvalid(S_AXI_WVALID), .i_axi_wready(S_AXI_WREADY),
			.i_axi_wdata(S_AXI_WDATA),.i_axi_wstrb(S_AXI_WSTRB),
		.i_axi_bvalid(S_AXI_BVALID), .i_axi_bready(S_AXI_BREADY),
			.i_axi_bresp(S_AXI_BRESP),
		.i_axi_arvalid(S_AXI_ARVALID), .i_axi_arready(S_AXI_ARREADY),
			.i_axi_araddr(S_AXI_ARADDR),.i_axi_arprot(S_AXI_ARPROT),
		.i_axi_rvalid(S_AXI_RVALID), .i_axi_rready(S_AXI_RREADY),
			.i_axi_rdata(S_AXI_RDATA),.i_axi_rresp(S_AXI_RRESP),
		//
		.f_axi_rd_outstanding(faxil_rd_outstanding),
		.f_axi_wr_outstanding(faxil_wr_outstanding),
		.f_axi_awr_outstanding(faxil_awr_outstanding)
		// }}}
	);

	always @(*)
	if (f_past_valid)
	begin
		assert(faxil_awr_outstanding == (s_axi_bvalid ? 1:0)
			+ (S_AXI_AWREADY ? 0:1));
		assert(faxil_wr_outstanding  == (s_axi_bvalid ? 1:0)
			+ (S_AXI_WREADY ? 0:1));
		assert(faxil_rd_outstanding  == (s_axi_rvalid ? 1:0)
			+ (S_AXI_ARREADY ? 0:1));
	end
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Fetch interface compliance
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	ffetch #(
		// {{{
		.ADDRESS_WIDTH(AW),
		.INSN_WIDTH(8)
		// }}}
	) f_fetch (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset || cpu_reset),
		.cpu_new_pc(cpu_new_pc), .cpu_clear_cache(cpu_clear_cache),
		.cpu_pc(pf_jump_addr), .pf_valid(pf_valid),.cpu_ready(pf_ready),
		.pf_pc(pf_insn_addr),.pf_insn(pf_insn), .pf_illegal(pf_illegal),
		.fc_illegal(f_const_illegal), .fc_insn(f_const_insn),
		.fc_pc(f_const_addr), .f_address(f_address)
		// }}}
	);

	/*
	wire	[AW-1:0]	f_next_addr;
	assign	f_next_addr = f_address + 1;
	always @(*)
	if (!i_reset && !cpu_reset && !cpu_new_pc && !cpu_clear_cache)
		assert(pf_jump_addr == f_next_addr);
	*/
	// }}}

	// Follow commands

	// First, the prefetch and decoding

	always @(*)
	if (f_past_valid)
		assert(!imm_cycle || !half_valid);

	////////////////////////////////////////////////////////////////////////
	//
	// Model the (missing) I2C controller
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
`ifndef	I2CCPU
	(* anyseq *) wire	m_tvalid, m_tlast;
	(* anyseq *) wire [7:0]	m_tdata;

	always @(posedge i_clk)
	if (!f_past_valid)
	begin
		assert(!insn_valid);
	end else if ($past(i_reset || i2c_abort))
	begin
		assert(!insn_valid);
	end else if ($past(s_tvalid && !insn_ready))
	begin
		assert(s_tvalid);
		assert($stable(insn[10:0]));
	end

	always @(posedge i_clk)
	if (!f_past_valid)
	begin
		assume(!m_tvalid);
	end else if ($past(i_reset))
	begin
		assume(!m_tvalid);
	end else if ($past(m_tvalid && !M_AXIS_TREADY))
	begin
		assume(m_tvalid);
		assume($stable(m_tdata));
		assume($stable(m_tlast));
	end

	assign	M_AXIS_TVALID = m_tvalid;
	assign	M_AXIS_TDATA  = m_tdata;
	assign	M_AXIS_TLAST  = m_tlast;
`endif
	// }}}
`endif
// }}}
endmodule
