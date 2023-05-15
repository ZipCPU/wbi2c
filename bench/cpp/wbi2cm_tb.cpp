////////////////////////////////////////////////////////////////////////////////
//
// Filename:	wbi2cm_tb.cpp
// {{{
// Project:	WBI2C ... a set of Wishbone controlled I2C controllers
//
// Purpose:	Bench test for the I2C master
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
#include "Vwbi2cmaster.h"

#include "byteswap.h"
#include "testb.h"
#include "wb_tb.h"
#include "i2csim.h"
// #include "twoc.h"

#ifdef	OLD_VERILATOR
#define	VVAR(A)	v__DOT_ ## A
#else
#define	VVAR(A)	wbi2cmaster__DOT_ ## A
#endif

#define	mem	VVAR(_mem)


#define	MEM_ADDR_BITS	7
#define	CMEMMSK		((1<<(MEM_ADDR_BITS))-1)
#define	WMEMMSK		(CMEMMSK >> 2)
#define	HALFMEM		(1<<(MEM_ADDR_BITS-1))
#define	FULMEMSZ	(1<<(MEM_ADDR_BITS))

#define	SLAVE_ADDRESS	0x50
#define	MASTER_WR	0
#define	MASTER_RD	1

// Address locations
#define	R_CMD		0
#define	R_CONTROL	R_CMD
#define	R_COMMAND	R_CMD
#define	R_SPEED		1
#define	R_MEM		(1<<(MEM_ADDR_BITS-2))

// Speed to command things
#define	I2CSPEED	40

// Command format(s)
#define	GENCMD(DEV,ADDR,CNT)	((((DEV)&0x07f)<<17)|(((ADDR)&CMEMMSK)<<8)|((CNT)&CMEMMSK))
#define	READCMD(DEV,ADDR,CNT)	(GENCMD(DEV,ADDR,CNT)|(MASTER_RD<<16))
#define	WRITECMD(DEV,ADDR,CNT)	(GENCMD(DEV,ADDR,CNT))

#define	TESTBREAK	for(int i=0; i<I2CSPEED * 1000; i++) tb->tick()

class	I2CM_TB : public WB_TB<Vwbi2cmaster> {
	I2CSIMSLAVE	m_slave;
public:
	I2CM_TB(void) : m_slave(0x50, MEM_ADDR_BITS) {
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

		mem = m_core->mem;
		wv = mem[(addr>>2)&WMEMMSK];
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
	char	buf[FULMEMSZ], tbuf[FULMEMSZ];
	unsigned	addr, pre, post, wbaddr, rval;
	unsigned long	prel, postl, rvall;

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
	// Test point 1 : check that what we've written to the controller (WB)
	// is what we can read back (WB).
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

	TESTBREAK;

	//
	//
	//
	//
	//
	//
	//
	//
	// Now, let's write our local data to the slave via I2C, and verify that
	// the slave gets the right answer
	tb->wb_write(R_CMD, WRITECMD(SLAVE_ADDRESS,0,HALFMEM));
	tb->tick();
	tb->tick();
	{
		bool	busy = true;
		unsigned	status, o_int;

		// Give the interface a couple ticks to get going
		tb->tick();
		tb->tick();

		// Now wait while the interface is busy
		do {

			// Check the 'int' line before checking if we are busy,
			// being aware that 'int' can become true as the bus
			// read completes, but before the data has been read
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

		status = tb->wb_read(R_CMD);
		// printf("WRITE-COMPLETE, STATUS = %08x\n", status);
		// printf("EXPECTED STATUS        = %08x\n", WRITECMD(SLAVE_ADDRESS, (1<<(MEM_ADDR_BITS-1)), 0));
		TBASSERT(*tb, (status == WRITECMD(SLAVE_ADDRESS,HALFMEM,0)));
	}

	// Now, let's the data memory from the master device back into memory
	// using WB, and then comare it against what the slave says it has
	// by accessing the slave's memory directly
	tb->wb_read(R_MEM, (unsigned)sizeof(buf)/4, (unsigned *)tbuf);
	for(unsigned i=0; i<20; i++) {
		int	adr = i & -4;
		adr |= 3-(i&3);
		printf("PRE-COMPR[%3d] 0x%02x RCV to 0x%02x SLV\n", i,
			tbuf[adr] & 0x0ff,
			tb->slave()[i] & 0x0ff);
	}
	for(unsigned i=0; i<HALFMEM; i++) {
		int	adr = i & -4;
		adr |= 3-(i&3);
		printf("COMPARING[%3d] 0x%02x RCV to 0x%02x SLV\n", i,
			tbuf[adr] & 0x0ff, tb->slave()[i] & 0x0ff);
		if (0 != ((tbuf[adr]^tb->slave()[i])&0x0ff)) {
			TBASSERT(*tb, ((tbuf[adr]&0x0ff) == (tb->slave()[i]&0x0ff)));
		}
	}


	TESTBREAK;

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
	tb->wb_write(R_CMD, WRITECMD(SLAVE_ADDRESS,HALFMEM,HALFMEM));
	tb->tick();
	tb->tick();
	{
		bool		busy = true;
		unsigned	status;

		do {
			tb->tick();
			busy = (0 == tb->m_core->o_int);
		} while(busy);

		status = tb->wb_read(R_CMD);
		// printf("WRITE-COMPLETE, STATUS = %08x\n", status);
		// printf("EXPECTED STATUS        = %08x\n", WRITECMD(SLAVE_ADDRESS, 0, 0));
		TBASSERT(*tb, (status == WRITECMD(SLAVE_ADDRESS,0,0)));
	}

	tb->wb_read(R_MEM+(HALFMEM>>2), (unsigned)sizeof(buf)/4, (unsigned *)tbuf);
	for(unsigned i=0; i<20; i++) {
		int	adr = i & -4;
		adr |= 3-(i&3);
		printf("PRE-COMPR[%3d] 0x%02x RCV to 0x%02x SLV\n", i,
			tbuf[adr] & 0x0ff,
			tb->slave()[HALFMEM+i] & 0x0ff);
	}
	for(unsigned i=0; i<HALFMEM; i++) {
		int	adr = i & -4;
		adr |= 3-(i&3);
		printf("COMPARING[%3d] 0x%02x RCV to 0x%02x SLV\n", i+HALFMEM,
			tbuf[adr] & 0x0ff,
			tb->slave()[HALFMEM+i] & 0x0ff);
		TBASSERT(*tb, ((tbuf[adr]&0x0ff) == (tb->slave()[HALFMEM+i]&0x0ff)));
	}

	TESTBREAK;


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
	tb->wb_write(R_CMD, READCMD(SLAVE_ADDRESS,0,HALFMEM));
	tb->tick();
	tb->tick();
	tb->tick();
	do {
		tb->tick();
	} while(0 == tb->m_core->o_int);
	
	tb->wb_read(R_MEM, (unsigned)sizeof(buf)/4, (unsigned *)tbuf);
	for(unsigned i=0; i<HALFMEM; i++) {
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

	TESTBREAK;

	//
	//
	//
	//
	//
	//
	//
	//
	// Now let's try random access reading
	printf("\n\nNext test: Reads from random I2C addresses\n\n\n");
	// Randomize the buffer
	randomize_buffer(sizeof(buf), &buf[0]);
	// We'll fill the slave's buffer with these values
	for(unsigned i=0; i<FULMEMSZ; i++) {
		tb->slave()[i] = buf[i];
	}
	// Now, let's read one byte from the slave at a time, in a pseudo-random
	// order, and see if we read the proper byte from the slave
	for(unsigned i=0; i<FULMEMSZ; i++) {
		addr = (i * 23) & CMEMMSK;
		wbaddr = (addr >> 2);
		pre = tb->wb_read(R_MEM + wbaddr);
		// printf("PREV[%02x] = %08x\n", wbaddr, pre);

		tb->wb_write(R_CMD, READCMD(SLAVE_ADDRESS,addr,1));
		tb->tick();
		tb->tick();
		while(0 == tb->m_core->o_int) {
			tb->tick();
		}

		{ unsigned status, expected_status;
		status = tb->wb_read(R_CMD);
		expected_status = WRITECMD(SLAVE_ADDRESS, (addr+1), 0);
		// printf("WRITE-COMPLETE, STATUS = %08x\n", status);
		// printf("EXPECTED STATUS        = %08x\n", expected_status);
		TBASSERT(*tb, (status == expected_status));
		}

		post = tb->wb_read(R_MEM+ wbaddr);
		// printf("POST[%02x] = %08x\n", wbaddr, post);
		if (pre != post) {
			unsigned msk;
			msk = (0x0ff)<<((3-(addr&3))*8);
			// Assert that nothing but our data changes
			if (((pre^post)&(~msk))!=0) {
				fprintf(stderr, "1. SINGLE-TEST, Wrong data changed, ADDR=%02x, PRE=%08x, POST=%08x\n", addr, pre, post);
				goto test_failure;
			}
		}

		rval = buf[addr] & 0x0ff;
		rvall= (post >> ((3-(addr&3))*8))&0x0ff;
		if (rval != rvall) {
			fprintf(stderr, "2. ERR, EXPECTED TO READ %02x from %02x, GOT %02x\n",
				rval, addr, (unsigned)rvall);
			fprintf(stderr, "SLAVE[%02x] = %02x\n", addr,
				buf[addr] & 0x0ff);
			goto test_failure;
		}

		// printf("CHECK\n");
	}

	TESTBREAK;

	//
	//
	//
	//
	//
	//
	//
	//
	// Now let's try random access reading, but two at a time this time.
	// We're not even going to guarantee alignment
	//
	printf("\n\nNext test: Reads from random I2C addresses, 2x at a time\n\n\n");
	// Randomize the buffer (again)
	randomize_buffer(sizeof(buf), &buf[0]);
	// We'll fill the slave's buffer with these values
	for(unsigned i=0; i<FULMEMSZ; i++) {
		// Set the data ... the cheaters way that can only be done on a
		// test bench
		tb->slave()[i] = buf[i];
	}
	// Now, let's read one byte from the slave at a time, in a pseudo-random
	// order, and see if we read the proper byte from the slave
	for(unsigned i=0; i<FULMEMSZ; i++) {
		addr = (i * 31) & CMEMMSK;
		wbaddr = addr >> 2;
		prel = tb->wb_read(R_MEM + wbaddr);
		prel = (prel<<32) | tb->wb_read(R_MEM + ((wbaddr+1)& WMEMMSK));
		// printf("PREL=%016lx\n", prel);

		tb->wb_write(R_CMD, READCMD(SLAVE_ADDRESS,addr,2));
		tb->tick();
		tb->tick();
		while(0 == tb->m_core->o_int) {
			tb->tick();
		}

		{ unsigned status, expected_status;
		status = tb->wb_read(R_CMD);
		expected_status = WRITECMD(SLAVE_ADDRESS, (addr+2), 0);
		// printf("WRITE-COMPLETE, STATUS = %08x\n", status);
		// printf("EXPECTED STATUS        = %08x\n", expected_status);
		TBASSERT(*tb, (status == expected_status));
		}

		postl = tb->wb_read(R_MEM + wbaddr);
		postl = (postl<<32)|tb->wb_read(R_MEM + ((wbaddr+1)& WMEMMSK));
		if (pre != post) {
			unsigned long msk;
			msk = (0x0ffffl)<<(24+(3-(addr&3))*8);
			// Assert that nothing but our data changes
			if (((prel^postl)&(~msk))!=0) {
				fprintf(stderr, "3. DBL-TEST, Wrong data changed, ADDR=%02x, PRE=%016lx, POST=%016lx\n", addr, prel, postl);
				fprintf(stderr, "PRE = %016lx\n", prel);
				fprintf(stderr, "POST= %016lx\n", postl);
				goto test_failure;
			}
		}

		rval  = ((buf[addr] << 8) | (buf[(addr+1)&CMEMMSK]&0x0ff)) & 0x0ffff;
		rvall = ((postl >> (24+(3-(addr&3))*8))&0x0ffffl);
		if (rvall != rval) {
			fprintf(stderr, "4. ERR, EXPECTED TO READ %04x from %02x+1, GOT %04lx\n",
				rval, addr, rvall);
			fprintf(stderr, "PRE = %016lx\n", prel);
			fprintf(stderr, "POST= %016lx\n", postl);
			goto test_failure;
		}
	}


	delete	tb;

	// And declare success
	printf("SUCCESS!\n");
	exit(EXIT_SUCCESS);

test_failure:
	delete	tb;
	printf("FAIL\n");
	exit(EXIT_FAILURE);
}

