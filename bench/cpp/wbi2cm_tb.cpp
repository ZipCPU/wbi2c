////////////////////////////////////////////////////////////////////////////////
//
// Filename:	wbi2cs_tb.cpp
//
// Project:	WBI2C ... a set of Wishbone controlled I2C controllers
//
// Purpose:	Bench testing for the divide unit found within the Zip CPU.
//
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
#include <signal.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <assert.h>

#include <ctype.h>

#include "verilated.h"
#include "Vwbi2cmaster.h"

#include "byteswap.h"
#include "testb.h"
#include "wb_tb.h"
#include "i2csim.h"
// #include "twoc.h"

#define	SLAVE_ADDRESS	0x50
#define	MASTER_WR	0
#define	MASTER_RD	1

// Address locations
#define	R_CMD		0
#define	R_CONTROL	R_CMD
#define	R_COMMAND	R_CMD
#define	R_SPEED		1
#define	R_MEM		32

// Speed to command things
#define	I2CSPEED	40

// Command format(s)
#define	GENCMD(DEV,ADDR,CNT)	((((DEV)&0x07f)<<17)|(((ADDR)&0x07f)<<8)|((CNT)&0x07f))
#define	READCMD(DEV,ADDR,CNT)	(GENCMD(DEV,ADDR,CNT)|(MASTER_RD<<16))
#define	WRITECMD(DEV,ADDR,CNT)	(GENCMD(DEV,ADDR,CNT))

class	I2CM_TB : public WB_TB<Vwbi2cmaster> {
	I2CSIMSLAVE	m_slave;
public:
	I2CM_TB(void) {
		m_core->i_i2c_scl = 1;
		m_core->i_i2c_sda = 1;
	}

	~I2CM_TB(void) {}

	void	reset(void) {
		// m_flash.debug(false);
		TESTB<Vwbi2cmaster>::reset();
	}

	void	dbgdump(void) {}

	void	tick(void) {
		const bool	debug = false;
		I2CBUS	ib;

		ib = m_slave(m_core->o_i2c_scl, m_core->o_i2c_sda);
		m_core->i_i2c_scl = ib.m_scl;
		m_core->i_i2c_sda = ib.m_sda;
		m_core->i_vstate = m_slave.vstate();

		if (debug)
			dbgdump();
		WB_TB<Vwbi2cmaster>::tick();
	}

	// Internally, the design keeps things in one memory 32-bits wide.
	// To get at a byte, we need to select which byte from within it.
	unsigned char operator[](const int addr) const {
		unsigned int *mem;
		int	wv;

		mem = m_core->v__DOT__mem;
		wv = mem[(addr>>2)&0x01f];
		wv >>= 8*(3-(addr&0x03));
		return wv & 0x0ff;
	}

	I2CSIMSLAVE &slave(void) {
		return m_slave;
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
	printf("USAGE: wbi2cm_tb\n");
	printf("\n");
	printf("\tIf the last line returns in SUCCESS, then the test was successful\n");
}

//
int	main(int argc, char **argv) {
	// Setup
	Verilated::commandArgs(argc, argv);
	I2CM_TB	*tb = new I2CM_TB();
	char	buf[128], tbuf[128];

	tb->reset();
	tb->opentrace("i2cm_tb.vcd");
	srand(2);

	randomize_buffer(sizeof(buf), &buf[0]);

	tb->wb_write(R_MEM, sizeof(buf)/4, (unsigned *)buf);

	//
	//
	//
	//
	//
	//
	//
	//
	// Test point 1 : check that what we've written to the controller is
	// what we can read back.
	tb->wb_read(R_MEM, (unsigned)sizeof(buf)/4, (unsigned *)tbuf);
	for(unsigned i=0; i<sizeof(buf); i++)
		TBASSERT(*tb, (buf[i] == tbuf[i]));

	byteswapbuf(sizeof(buf)/4, (unsigned *)buf);


	tb->wb_write(R_SPEED, I2CSPEED);
	{
		unsigned	spd = tb->wb_read(R_SPEED);
		if (spd != I2CSPEED) {
			fprintf(stderr, "ERR: WRONG SPEED READ AFTER SETTING DEV SPD, %d != %d\n", spd, I2CSPEED);
			TBASSERT(*tb, (spd == I2CSPEED));
		}
	}

	//
	//
	//
	//
	//
	//
	//
	//
	// Now, let's randomize our buffer again, and set the slaves data
	randomize_buffer(sizeof(buf), &buf[0]);

	tb->wb_write(R_CMD, WRITECMD(SLAVE_ADDRESS,0,64));
	tb->tick();
	tb->tick();
	{
		bool	busy = true;

		tb->tick();
		tb->tick();

		do {
			unsigned	status, o_int;

			o_int = tb->m_core->o_int;
			status = tb->wb_read(R_CMD);
			busy = ((status >> 31)&1)?true:false;

			if (busy) {
				TBASSERT(*tb, (o_int == 0));
			} else {
				TBASSERT(*tb, (tb->m_core->o_int != 0));
			}
			TBASSERT(*tb, (0 == ((status >> 30)&1)));
		} while(busy);
	}

	tb->wb_read(R_MEM, (unsigned)sizeof(buf)/4, (unsigned *)tbuf);
	for(unsigned i=0; i<20; i++) {
		int	adr = i & -4;
		adr |= 3-(i&3);
		printf("PRE-COMPR[%3d] 0x%02x RCV to 0x%02x SLV\n", i,
			tbuf[adr] & 0x0ff,
			tb->slave()[i] & 0x0ff);
	}
	for(unsigned i=0; i<64; i++) {
		int	adr = i & -4;
		adr |= 3-(i&3);
		printf("COMPARING[%3d] 0x%02x RCV to 0x%02x SLV\n", i,
			tbuf[adr] & 0x0ff,
			tb->slave()[i] & 0x0ff);
		TBASSERT(*tb, ((tbuf[adr]&0x0ff) == (tb->slave()[i]&0x0ff)));
	}

	//
	//
	//
	//
	//
	//
	//
	//
	//
	// Now, write the second half ...
	tb->wb_write(R_CMD, WRITECMD(SLAVE_ADDRESS,64,64));
	tb->tick();
	tb->tick();
	{
		bool	busy = true;
		do {
			tb->tick();
			busy = (0 == tb->m_core->o_int);
		} while(busy);
	}

	tb->wb_read(R_MEM+(64>>2), (unsigned)sizeof(buf)/4, (unsigned *)tbuf);
	for(unsigned i=0; i<20; i++) {
		int	adr = i & -4;
		adr |= 3-(i&3);
		printf("PRE-COMPR[%3d] 0x%02x RCV to 0x%02x SLV\n", i,
			tbuf[adr] & 0x0ff,
			tb->slave()[64+i] & 0x0ff);
	}
	for(unsigned i=0; i<64; i++) {
		int	adr = i & -4;
		adr |= 3-(i&3);
		printf("COMPARING[%3d] 0x%02x RCV to 0x%02x SLV\n", i+64,
			tbuf[adr] & 0x0ff,
			tb->slave()[64+i] & 0x0ff);
		TBASSERT(*tb, ((tbuf[adr]&0x0ff) == (tb->slave()[64+i]&0x0ff)));
	}

	//
	//
	//
	//
	//
	//
	//
	//
	// Now, let's try reading.  First, we'll scramble our buffer again.
	randomize_buffer(sizeof(buf), &buf[0]);

	tb->wb_write(R_MEM, sizeof(buf)/4, (unsigned *)buf);
	// And verify that it was properly scrambled
	tb->wb_read(R_MEM, (unsigned)sizeof(buf)/4, (unsigned *)tbuf);
	for(unsigned i=0; i<sizeof(buf); i++)
		TBASSERT(*tb, (buf[i] == tbuf[i]));

	byteswapbuf(sizeof(buf)/4, (unsigned *)buf);

	// Now, let's read back the data from the port
	tb->wb_write(R_CMD, READCMD(SLAVE_ADDRESS,0,64));
	tb->tick();
	tb->tick();
	tb->tick();
	do {
		tb->tick();
	} while(0 == tb->m_core->o_int);
	
	tb->wb_read(R_MEM, (unsigned)sizeof(buf)/4, (unsigned *)tbuf);
	for(unsigned i=0; i<64; i++) {
		int	adr = i & -4;
		adr |= 3-(i&3);
		if ((tbuf[adr]&0x0ff)!=(tb->slave()[i]&0x0ff)) {
			printf("COMPARING[%3d] 0x%02x RCV to 0x%02x SLV\n", i,
				tbuf[adr] & 0x0ff,
				tb->slave()[i] & 0x0ff);
			TBASSERT(*tb, ((tbuf[adr]&0x0ff)
					== (tb->slave()[i]&0x0ff)));
		}
	}



	delete	tb;

	// And declare success
	printf("SUCCESS!\n");
	exit(EXIT_SUCCESS);
}

