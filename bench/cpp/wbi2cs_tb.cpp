////////////////////////////////////////////////////////////////////////////////
//
// Filename:	wbi2cs_tb.cpp
// {{{
// Project:	WBI2C ... a set of Wishbone controlled I2C controllers
//
// Purpose:	Bench testing for the I2C slave controller
//
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2017-2023, Gisselquist Technology, LLC
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
//
////////////////////////////////////////////////////////////////////////////////
//
// }}}
#include <signal.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <assert.h>

#include <ctype.h>

#include "verilated.h"
#include "Vwbi2cslave.h"

#include "byteswap.h"
#include "testb.h"
#include "wb_tb.h"
// #include "twoc.h"

#ifdef	OLD_VERILATOR
#define	VVAR(A)	v__DOT_ ## A
#else
#define	VVAR(A)	wbi2cslave__DOT_ ## A
#endif

#define	mem	VVAR(_mem)

#define	MEM_ADDR_BITS	8
#define	FULMEMSZ	(1<<(MEM_ADDR_BITS))

#define	SLAVE_ADDRESS	0x50
#define	SCK	m_core->i_i2c_sck
#define	SDA	m_core->i_i2c_sda
#define	MASTER_WR	0
#define	MASTER_RD	1

class	I2CS_TB : public WB_TB<Vwbi2cslave> {
public:
	I2CS_TB(void) {
		m_core->i_i2c_sck = 1;
		m_core->i_i2c_sda = 1;
	}

	~I2CS_TB(void) {}

	void	reset(void) {
		// m_flash.debug(false);
		TESTB<Vwbi2cslave>::reset();
	}

	void	dbgdump(void) {}

	void	tick(void) {
		const bool	debug = false;
		int	sck = SCK, sda = SDA;

		SCK &= m_core->o_i2c_sck;
		SDA &= m_core->o_i2c_sda;

		if (debug)
			dbgdump();
		WB_TB<Vwbi2cslave>::tick();

		SCK = sck & m_core->o_i2c_sck;
		SDA = sda & m_core->o_i2c_sda;

	}

	// Internally, the design keeps things in one memory 32-bits wide.
	// To get at a byte, we need to select which byte from within it.
	unsigned char operator[](const int addr) const {
		unsigned int *mem;
		int	wv;

		mem = m_core->mem;
		wv = mem[(addr>>2)&((FULMEMSZ-1)>>2)];
		wv >>= 8*(3-(addr&0x03));
		return wv & 0x0ff;
	}

	void	i2c_halfwait(void) {
		for(int i=0; i<8; i++)
			tick();
	}

	void	i2c_wait(void) {
		i2c_halfwait();
		i2c_halfwait();
	}

	void	i2c_idle(void) {
		for(int i=0; i<26; i++)
			i2c_wait();
	}

	void	i2c_start() {
		// printf("I2C-START\n");
		TBASSERT(*this, ((SCK)&&(SDA)));
		SDA = 0;
		i2c_halfwait();
		SCK = 0;
		i2c_halfwait();
	}

	void	i2c_repeat_start() {
		// printf("I2C-REPEAT-START\n");
		TBASSERT(*this, (!SCK));
		SDA = 1;
		i2c_halfwait();
		SCK = 1;
		i2c_halfwait();
		i2c_start();
	}

	void	i2c_stop() {
		TBASSERT(*this, ((!SCK)&&(!SDA)));
		SCK = 1;
		i2c_halfwait();
		SDA = 1;
		i2c_halfwait();
		// printf("I2C-STOP\n");
	}

	int	i2c_rxbit(void) {
		int	r;

		SDA = 1;
		i2c_halfwait();
		SCK = 1;
		do {
			i2c_halfwait();
		} while(SCK == 0);
		i2c_halfwait();
		r = SDA;
		SCK = 0;
		i2c_halfwait();
		TBASSERT(*this, (!SCK));

		// printf("I2C-RX: %d\n", r);
		return r;
	}

	void	i2c_txbit(int b) {
		SDA = b;
		i2c_halfwait();
		SCK = 1;
		do {
			i2c_halfwait();
		} while(SCK == 0);
		i2c_halfwait();
		SCK = 0;
		i2c_halfwait();
		TBASSERT(*this, (!SCK));
	}

	void	i2c_txbyte(const int b) {
		int	tx = b;
		for(int i=0; i<8; i++) {
			i2c_txbit((tx>>7)&1);
			tx <<= 1;
		} // printf("TRANSMITTED %02x\n", b);
	}

	int	i2c_rxbyte(void) {
		int	b = 0;
		for(int i=0; i<8; i++) {
			b = (b<<1) | i2c_rxbit();
		}
		// printf("I2C-READ: %02x\n", b);
		return b;
	}

	void	i2c_read(int slave_addr, int addr,
			const unsigned cnt, char *buf) {
		int	ack;

		if (cnt == 0)
			return;

		// printf("I2C_READ(SLV=%02x, ADR=%02x, CNT=%d,...)\n",
		//	slave_addr, addr, cnt);

		slave_addr <<= 1;
		i2c_start();

		// First, set the address
		i2c_txbyte((slave_addr&0xfe)|MASTER_WR);//Master is sending data
		ack = i2c_rxbit();	// (i.e., the address to rd from)
		// printf("RXACK = %d\n", ack);
		TBASSERT(*this, (ack==0));

		i2c_txbyte(addr);	// Address we wish to read from
		ack = i2c_rxbit();
		// printf("RXACK = %d\n", ack);
		TBASSERT(*this, (ack==0));

		i2c_repeat_start();


		// Then, read the data
		i2c_txbyte((slave_addr&0xfe)|MASTER_RD); // Request data
		ack = i2c_rxbit();
		// printf("RXACK = %d\n", ack);
		TBASSERT(*this, (ack==0));

		for(unsigned i=0; i<cnt-1; i++) {
			buf[i] = i2c_rxbyte();
			i2c_txbit(0);
			// printf("TX-ACK SENT\n");
		}

		buf[cnt-1] = i2c_rxbyte();

		// Send a stop bit instead of an ack
		SDA = 0;
		i2c_halfwait();
		i2c_stop();
	}

	void	i2c_read(int addr, const unsigned cnt, char *buf) {
		i2c_read(SLAVE_ADDRESS, addr, cnt, buf);
	}


	void	i2c_write(int slave_addr, int addr,
			const unsigned cnt, const char *buf) {
		int	ack;

		// printf("I2C_WRITE(SLV=%02x, ADR=%02x, CNT=%d,...)\n",
		//	slave_addr, addr, cnt);

		slave_addr <<= 1;
		i2c_start();

		i2c_txbyte((slave_addr&0xfe)|MASTER_WR);
		ack = i2c_rxbit();
		// printf("RXACK = %d\n", ack);
		TBASSERT(*this, (ack==0));

		i2c_txbyte(addr);
		ack = i2c_rxbit();
		// printf("RXACK = %d\n", ack);
		TBASSERT(*this, (ack==0));

		for(unsigned i=0; i<cnt; i++) {
			i2c_txbyte(buf[i] & 0xff);
			ack = i2c_rxbit();
			// printf("RXACK = %d\n", ack);
			TBASSERT(*this, (ack==0));
		}

		i2c_stop();
	}

	void	i2c_write(int addr, const unsigned cnt, const char *buf) {
		i2c_write(SLAVE_ADDRESS, addr, cnt, buf);
	}

};

void	randomize_buffer(unsigned nc, char *buf) {
	if (true) {
		const char	*fname = "/dev/urandom";
		FILE	*fp = fopen(fname, "r");
		unsigned	nr;
		assert(fp);
		nr = fread(buf, sizeof(char), nc, fp);
		assert(nr == nc);
		fclose(fp);
	} else {
		for(unsigned i=0; i<nc; i++)
			buf[i] = (rand()&0x0ff);
	}
}

//
// Standard usage functions.
//
// Notice that the test bench provides no options.  Everything is
// self-contained.
void	usage(void) {
	printf("USAGE: wbi2cs_tb\n");
	printf("\n");
	printf("\t\n");
}

//
int	main(int argc, char **argv) {
	// Setup
	Verilated::commandArgs(argc, argv);
	I2CS_TB	*tb = new I2CS_TB();
	char	buf[FULMEMSZ], tbuf[FULMEMSZ];

	tb->reset();
	tb->opentrace("i2cs_tb.vcd");
	srand(2);

	randomize_buffer(sizeof(buf), &buf[0]);

	tb->wb_write(0, sizeof(buf)/4, (unsigned *)buf);

	// Test point 1 : check that what we've written to the controller is
	// what we can read back.
	tb->wb_read(0, (unsigned)sizeof(buf)/4, (unsigned *)tbuf);
	for(unsigned i=0; i<sizeof(buf); i++)
		TBASSERT(*tb, (buf[i] == tbuf[i]));

	byteswapbuf(sizeof(buf)/4, (unsigned *)buf);

	//
	tb->i2c_idle();
	//

	// Test point 2 : Read from i2c, verify that reads work.
	tb->i2c_read(0, 1, &tbuf[0]);
	printf("COMPARING: %02x(RD) to %02x(EXP)\n",
			tbuf[0]&0x0ff, buf[0]&0x0ff);
	TBASSERT(*tb, (buf[0] == tbuf[0]));

	//
	tb->i2c_idle();
	//

	tb->i2c_read(0, sizeof(buf), tbuf);
	for(unsigned i=0; i<sizeof(buf); i++) {
		if (buf[i] != tbuf[i]) {
			printf("%3d: RX(%02x) != (%02x)EXP\n",
				i, tbuf[i] & 0x0ff, buf[i]&0x0ff);
			TBASSERT(*tb, (buf[i] == tbuf[i]));
		}
	}

	//
	tb->i2c_idle();
	//

	// Test point 3: Walk through this, reading random bytes
	for(unsigned i=0, a=7; i<sizeof(buf); i++, a += 41) {
		a &= 127;
		tb->i2c_read(a, 1, &tbuf[a]);

		//
		tb->i2c_idle();
		//

		if (buf[a] != tbuf[a]) {
			printf("%3d[%3d]: RX(%02x) != (%02x)EXP\n",
				i, a, tbuf[a] & 0x0ff, buf[a]&0x0ff);
			TBASSERT(*tb, (buf[a] == tbuf[a]));
		}
	}

	// Test point 4: Walk through this, reading random byte pairs
	for(unsigned i=0, a=7; i<sizeof(tbuf); i++, a += 97) {
		a &= (FULMEMSZ-2);
		tb->i2c_read(a, 2, &tbuf[a]);

		//
		tb->i2c_idle();
		//

		TBASSERT(*tb, (buf[a  ] == tbuf[a  ]));
		TBASSERT(*tb, (buf[a+1] == tbuf[a+1]));
	}

	// Test point 5: Make sure we haven't changed anything up to this point
	for(unsigned i=0; i<sizeof(buf); i++) {
		if ((i&15)==0)
			printf("READ[%02x]: %02x ", i, buf[i]&0x0ff);
		else if ((i&15)==15)
			printf(" %02x\n", buf[i]&0x0ff);
		else
			printf(" %02x ", buf[i]&0x0ff);
		fflush(stdout);
		if (((buf[i]^(*tb)[i])&0x0ff) != 0) {
			fprintf(stderr, "ERR: %02x (RD) != %02x (EXP) @ %02x\n",
				buf[i], (*tb)[i], i);
			TBASSERT(*tb, (((buf[i]^(*tb)[i])&0x0ff) == 0));
		}
	}

	// Test point 6: Write a new buffer, pairs of addresses at a time
	randomize_buffer(sizeof(buf), buf);
	printf("\n\nWRITE-TEST\n\n");
	for(unsigned i=0, a=0; i<sizeof(buf); i++, a += 61*2) {
		a &= (FULMEMSZ-2);
		printf("PRE-WRITE[%02x] := %02x:%02x (MEM)\n", a,
			(*tb)[a]&0x0ff, (*tb)[a+1]&0x0ff);
		tb->i2c_write(a, 2, &buf[a]);

		//
		tb->i2c_idle();
		//

		// Cheaters test of success here
		printf("   READING FROM ADDR[%02x] := %02x:%02x (MEM) vs %02x:%02x(EXP)\n",
			a, (*tb)[a]&0x0ff, (*tb)[a+1]&0x0ff,
			buf[a]&0x0ff, buf[a+1]&0x0ff);
		TBASSERT(*tb, ((buf[a  ]&0x0ff) == ((*tb)[a  ]&0x0ff)));
		TBASSERT(*tb, ((buf[a+1]&0x0ff) == ((*tb)[a+1]&0x0ff)));
	} // Read the data back ... the cheaters way
	for(unsigned i=0; i<sizeof(buf); i++) {
		printf("TST[%02x]: %02x =?= %02x(EXP)\n", i,
			((*tb)[i]&0x0ff), (buf[i]&0x0ff));
		TBASSERT(*tb, ((buf[i]&0x0ff) == ((*tb)[i]&0x0ff)));
	}

	delete	tb;

	// And declare success
	printf("SUCCESS!\n");
	exit(EXIT_SUCCESS);
}

