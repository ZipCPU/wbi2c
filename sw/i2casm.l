/*******************************************************************************
**
** Filename:	i2casm.l
** {{{
** Project:	WBI2C ... a set of Wishbone controlled I2C controller(s)
**
** Purpose:	
**
** Creator:	Dan Gisselquist, Ph.D.
**		Gisselquist Technology, LLC
**
********************************************************************************
** }}}
** Copyright (C) 2021-2024, Gisselquist Technology, LLC
** {{{
** This program is free software (firmware): you can redistribute it and/or
** modify it under the terms of  the GNU General Public License as published
** by the Free Software Foundation, either version 3 of the License, or (at
** your option) any later version.
**
** This program is distributed in the hope that it will be useful, but WITHOUT
** ANY WARRANTY; without even the implied warranty of MERCHANTIBILITY or
** FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
** for more details.
**
** You should have received a copy of the GNU General Public License along
** with this program.  (It's in the $(ROOT)/doc directory.  Run make with no
** target there if the PDF file isn't present.)  If not, see
** <http://www.gnu.org/licenses/> for a copy.
** }}}
** License:	GPL, v3, as defined and found on www.gnu.org,
* {{{
*		http://www.gnu.org/licenses/gpl.html
*
********************************************************************************
*
* }}}
*/

%{
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <ctype.h>

typedef struct SYMBOL_S {
	unsigned	addr;
	char		*str;
} SYMBOL;

typedef struct EQDEFN_S {
	unsigned	val;
	char		*str;
} EQDEFN;

#define	MAX_SYMBOLS	512
SYMBOL	symlist[MAX_SYMBOLS];
unsigned	nsyms = 0;

EQDEFN	defnlist[MAX_SYMBOLS];
unsigned	ndefns = 0;

extern "C" int yylex();
extern "C" int	yywrap() { return 1;}

#define	I_NOOP		0
#define	I_START		1
#define	I_STOP		2
#define	I_SEND		3
#define	I_RXK		4
#define	I_RXN		5
#define	I_RXLK		6
#define	I_RXLN		7
#define	I_WAIT		8
#define	I_HALT		9
#define	I_ABORT		10
#define	I_TARGET	11
#define	I_JUMP		12
#define	I_CHANNEL	13
#define	I_ILLE		14
#define	I_ILLF		15

#define	D_WR		0
#define	D_RD		1

int	posn = 0;
bool	m_debug = false;

void	addinsn(int);
void	addimm_lbl(const char *str);
void	addimm(int);
void	adddefn(const char *str);
void	adddir(int);
void	ordir(int);
void	label(const char *);
%}

%option yylineno
%option warn

%%

(?i:NOP)	{ addinsn(I_NOOP); }
(?i:NOOP)	{ addinsn(I_NOOP); }
(?i:START)	{ addinsn(I_START); }
(?i:STOP)	{ addinsn(I_STOP); }
(?i:RXK)	{ addinsn(I_RXK); }
(?i:RXN)	{ addinsn(I_RXN); }
(?i:RXLK)	{ addinsn(I_RXLK); }
(?i:RXLN)	{ addinsn(I_RXLN); }
(?i:SEND)	{ addinsn(I_SEND); }
(?i:WAIT)	{ addinsn(I_WAIT); }
(?i:HALT)	{ addinsn(I_HALT); }
(?i:ABORT)	{ addinsn(I_ABORT); }
(?i:TGT)	{ addinsn(I_TARGET); }
(?i:TARGET)	{ addinsn(I_TARGET); }
(?i:JUMP)	{ addinsn(I_JUMP); }
(?i:CHANNEL)	{ addinsn(I_CHANNEL); }
(?i:CHNL)	{ addinsn(I_CHANNEL); }
(?i:CHAN)	{ addinsn(I_CHANNEL); }
[A-Za-z_][A-Za-z_0-9]*[ \t]*=[ \t]*0[xX][0-9A-Fa-f]+ { adddefn(yytext);}
[A-Za-z_][A-Za-z_0-9]*:	{ label(yytext); }
[A-Za-z_][A-Za-z_0-9]*  { addimm_lbl(yytext); }
,\s*(?i:WR)	{ adddir(D_WR); }
,\s*(?i:RD)	{ adddir(D_RD); }
,\s*(?i:W)	{ adddir(D_WR); }
,\s*(?i:R)	{ adddir(D_RD); }
[|]\s*(?i:WR)	{ ordir(D_WR); }
[|]\s*(?i:RD)	{ ordir(D_RD); }
[|]\s*(?i:W)	{ ordir(D_WR); }
[|]\s*(?i:R)	{ ordir(D_RD); }
[,\s]*0[xX][0-9A-Fa-f]+ { addimm(strtoul(yytext,NULL,16));}
[,\s]*0[0-7]+		  { addimm(strtoul(yytext,NULL, 8));}
[,\s]*[1-9][0-9]*	  { addimm(strtoul(yytext,NULL,10));}
[,\s]*0		  { addimm(0); }
[ \t]+		{ }
";".*\n		{ }
[,\s]*"#".*\n	{ }
[,\s]*"//".*\n	{ }
,		{ }
\n { }
%%

bool	verbose_flag = false;
FILE	*hfile = NULL;		// C++ data file
// FILE	*dbgfp  = NULL;		// Debug file

const char	*INSN[] = {
		"NOOP","START","STOP", "SEND",  "RXK",   "RXN", "RXLK", "RXLN",
		"WAIT","HALT", "ABORT","TARGET","JUMP","CHAN",  "ILL",  "ILL"};

int	m_binsz, m_pos;
char	*m_binary = NULL;
int	m_last_insn = I_NOOP;
bool	m_half = false, m_halted = false, m_stopped = true, m_long = false,
	m_rx = false, m_rx_implied = false;
unsigned m_count = 0;
char	m_send[256];

void	init_buffer(void) {
	// {{{
	m_binsz = 512;
	m_binary = new char[m_binsz];
	m_pos = 0;
}
// }}}

void	grow_buffer(void) {
	// {{{
	char *newbuf;

	// if (verbose_flag)
	//	fprintf(stderr, "Growing buffer from %d to %d\n", m_binsz, m_binsz*2);
	assert(m_binsz >= 512);
	newbuf = new char[m_binsz * 2];
	memcpy(newbuf, m_binary, m_pos);
	m_binsz *= 2;
	delete[] m_binary;
	m_binary = newbuf;
}
// }}}

void	addinsn(int i) {
	// {{{
	if (verbose_flag) {
		fprintf(stderr, "%02d: ", m_pos);
		if (m_half)
			fprintf(stderr, "Insn #%2d: | %s\n", i, INSN[i]);
		else
			fprintf(stderr, "Insn #%2d:   %s\n", i, INSN[i]);
	}

	char	b;

	// Encode any outstanding strings before moving forwards
	// {{{
	if (m_last_insn == I_RXK && (i == I_RXK || i == I_RXN
					|| i == I_RXLN || i == I_RXLK)) {
		// {{{
		if(m_rx_implied)
			m_count++;
		m_rx = true; m_rx_implied = true;
		m_last_insn = i;
		if (m_binary[m_pos-1] != 0) {
			assert(m_binary[m_pos-1] == (I_RXK << 4));
			m_binary[m_pos-1] = (i << 4);
		} else {
			assert(I_RXK == (m_binary[m_pos-2] & 0x0f));
			m_binary[m_pos-2] &= 0x0f0;
			m_binary[m_pos-2] |= i;
		}
		// Count just continues to grow ...
		return;
		// }}}
	} if (m_rx && m_rx_implied) {
		m_count = 1; // No argument == count of 1
	} if (m_rx) {
		// {{{
		assert(m_count > 0);
		if (m_count <= 16 && m_binary[m_pos-1] != 0) {
			m_binary[m_pos-1] |= (m_count-1);
			m_count = 0;
		} else if (m_binary[m_pos-1] != 0) {
			assert(m_binary[m_pos-1] == (m_last_insn << 4));
			m_binary[m_pos-1] = m_last_insn;
		} if (m_count > 0) {
			if (m_count > 256) {
				m_binary[m_pos-1] &= 0x0f;
				m_binary[m_pos-1] |= I_RXK;
				m_binary[m_pos++] = 0xff;
				m_count -= 256;
			} else {
				m_binary[m_pos++] = (m_count-1);
				m_count = 0;
			}
		}
		while(m_count > 256) {
			m_binary[m_pos++] = I_RXK;
			m_binary[m_pos++] = 0xff;
			m_count -= 256;
		} if (m_count > 16) {
			m_binary[m_pos++] = m_last_insn;
			m_binary[m_pos++] = (m_count-1);
			m_count = 0;
		} else if (m_count > 0) {
			m_binary[m_pos++] = (m_last_insn << 4) | (m_count-1);
			m_count = 0;
		} m_long = false; m_rx = false; m_rx_implied = false;
		// }}}
	} if (m_long || m_last_insn == I_SEND) { // Can only be a SEND instruction
		if (i == I_SEND) {
			// Continue sending bytes.  No change required here.
			return;
		} if (0 == m_count) { // No data.  Remove the send instruction
			// {{{
			assert(m_pos >= 1);
			if (m_binary[m_pos-1] == (I_SEND << 4)) {
				// Half count word ...
				m_binary[--m_pos] = 0;
			} else {
				// Whole count word
				assert(m_binary[m_pos-1] == 0);
				m_pos--; m_half = true;
				m_binary[m_pos] &= 0x0f0;
			}
			m_long = false;
			m_count = 0;
			// }}}
		} else if (m_count <= 16 && m_binary[m_pos-1] != 0) {
			// {{{
			m_binary[m_pos-1] |= (m_count-1);
			m_long = false;
			m_pos += m_count;
			m_count = 0;
			// }}}
		} else { // if (m_count <= 256 )  -- Guaranteed
			// {{{
			m_binary[m_pos-1] |= (m_count-1);
			m_pos += m_count;
			m_long = false;
			m_count = 0;
			// }}}
		}

		m_half = false;
		m_long = false;
		m_count = 0;
	}
	// }}}

	i &= 0x0f;
	m_count = 0;
	m_long  = false;
	m_rx    = false;
	m_rx_implied = false;
	if (m_half) {	// Set the leat-significant nibble
		// {{{
		assert(m_pos > 0);
		b = m_binary[m_pos-1];
		b = (b & 0xf0) | i;
		m_binary[m_pos-1] = b;
		m_half = false;

		m_halted = (i== I_HALT);

		m_rx = false; m_rx_implied = false;
		if (i == I_RXK  || i == I_RXN
				|| i == I_RXLK || i == I_RXLN) {
			m_rx = true;
			m_rx_implied = true;
		} if (i == I_SEND || m_rx) {
			m_binary[m_pos++] = 0;	// Count of 1 ... initially
			m_long  = true;
		} else if (i == I_CHANNEL)  {
			// Assume a channel of zero
			m_binary[m_pos++] = 0;
		}
		// }}}
	} else {	// Set the most significant nibble, if not the whole
		// {{{
		b = (i<<4) | I_NOOP;
		m_binary[m_pos] = b;

		m_rx = (i == I_RXK || i == I_RXN
				|| i == I_RXLK || i == I_RXLN);
		m_rx_implied = m_rx;
		if (i == I_SEND || m_rx || i == I_CHANNEL)  {
			// No second half allowed.
			m_half = false;
		} else
			m_half = true;
		m_pos ++;

		m_halted = (i== I_HALT);
		// }}}
	}

	m_last_insn = i;
	switch(i) {
		case I_START: case I_SEND: case I_RXK: case I_RXN:
		case I_RXLN: case I_RXLK:
			m_stopped = false; break;
		case I_STOP:
			m_stopped = true; break;
		default: break;
	}

	if (m_pos >= m_binsz)
		grow_buffer();
}
// }}}

void	addimm_lbl(const char *id) {
	// {{{
	unsigned	jk;

	// if (verbose_flag)
	//	fprintf(stderr, "IMM Label: %s\n", id);
	for(jk=0; jk<ndefns; jk++) {
		if (strcmp(id, defnlist[jk].str) == 0) {
			addimm(defnlist[jk].val);
			return;
		}
	}
}
// }}}

void	addimm(int imm) {
	// {{{

	if (m_last_insn == I_SEND) {
		// {{{
		if (verbose_flag)
			fprintf(stderr, "SEND IMM: 0x%02x\n", imm & 0x0ff);
		if (m_pos+m_count+3 >= m_binsz)
			grow_buffer();
		m_binary[m_pos+m_count] = imm;
		m_count++;
		if (m_count == 17) {
			if (m_binary[m_pos-1] != 0) {
				assert(m_binary[m_pos-1] == (I_SEND << 4));
				for(unsigned k=0; k<m_count; k++)
					m_binary[m_pos+m_count-k] = m_binary[m_pos+m_count-k-1];
				m_binary[m_pos-1] = I_SEND;
				m_binary[m_pos++] = 0;
				m_long = true;
			}
		} else if (m_count == 257) {
			assert(m_binary[m_pos-1] == 0);
			m_binary[m_pos-1] = 255;
			m_pos += 256;
			m_binary[m_pos++] = I_SEND << 4;
			m_binary[m_pos] = imm;
			m_count = 1;
			m_long = false;
		}
		// }}}
	} else if (m_rx) {
		// {{{
		if (verbose_flag)
			fprintf(stderr, "ADD-RX IMM: 0x%02x\n", imm & 0x0ff);
		m_rx_implied = false;
		m_count += imm;
		// }}}
	} else if (m_last_insn == I_CHANNEL) {
		if (verbose_flag)
			fprintf(stderr, " ADD-IMM: 0x%02x\n", imm & 0x0ff);
		if (m_binary[m_pos-1] == 0 || imm <= 15)
			m_binary[m_pos-1] |= imm;
		else {
			assert(m_binary[m_pos-1] == (I_CHANNEL << 4));
			m_binary[m_pos-1] = I_CHANNEL;
			m_binary[m_pos++] = imm;
		}
		m_half = false;
		m_halted = false;
	}

	if (m_pos >= m_binsz)
		grow_buffer();
}
// }}}

void	adddir(int dir) {
	// {{{
	if (verbose_flag)
		fprintf(stderr, "ADD-DIR: %s\n", (dir == D_RD) ? "RD":"WR");
	if (m_pos < 2) {
		fprintf(stderr, "ERR: Syntax err, direction with no send\n");
	} else if (m_last_insn != I_SEND) {
		fprintf(stderr, "ERR: Syntax err, direction with no send\n");
	} else if (m_count != 1) {
		fprintf(stderr, "ERR: Syntax err, direction with no send immediate\n");
	} else {
		char	b = m_binary[m_pos+m_count-1];
		b = (b << 1) | dir;
		m_binary[m_pos+m_count-1] = b;
	}
}
// }}}

void	ordir(int dir) {
	// {{{
	if (verbose_flag)
		fprintf(stderr, "OR-DIR: %s\n", (dir == D_RD) ? "RD":"WR");
	if (m_pos < 2) {
		fprintf(stderr, "ERR: Syntax err, direction with no send\n");
	} else if (m_last_insn != I_SEND) {
		fprintf(stderr, "ERR: Syntax err, direction with no send\n");
	} else if (m_count != 1) {
		fprintf(stderr, "ERR: Syntax err, direction with no send immediate\n");
	} else {
		char	b = m_binary[m_pos+m_count-1];
		b |= dir;
		m_binary[m_pos+m_count-1] = b;
	}
}
// }}}

void	dump(FILE *fp) {
	// {{{
	bool		prior_start = false;
	unsigned	count;

	for(int p=0; p<m_pos; p++) {
		unsigned h = (m_binary[p] >> 4) & 0x0f,
				n = (m_binary[p] & 0x0f);

		fprintf(fp, "%02x: ", p);
		if (h == I_SEND) {
			// {{{
			count = n+1;
			if (prior_start) {
				// {{{
				unsigned	arg = m_binary[p+1];
				unsigned	d = arg & 1;
				fprintf(fp, "%02x %02x : SEND 0x%02x,%2s\n",
					m_binary[p] & 0x0ff,
					m_binary[p+1] & 0x0ff,
					(arg >> 1) & 0x07f,
					(d == D_RD) ? "RD":"WR");
				p++;
				assert(count != 0);
				if (--count > 0)
					fprintf(fp, "\t");
				// }}}
			} else if (count == 1) {
				fprintf(fp, "%02x %02x : SEND 0x%02x\n",
					m_binary[p] & 0x0ff,
					m_binary[p+1] & 0x0ff,
					m_binary[p+1] & 0x0ff);
				count--; p++;
			} else {
				fprintf(fp, "%02x    : SEND %3d\n\t",
					m_binary[p] & 0x0ff,
					(m_binary[p] & 0x0f) + 1);
			} if (count != 0) for(unsigned k=0; k<count; k++) {
				fprintf(fp, "0x%02x", m_binary[p+k+1] & 0x0ff);
				if (k+1 == count)
					fprintf(fp, "\n");
				else if (7 == (k & 7))
					fprintf(fp, ",\n\t");
				else
					fprintf(fp, ", ");
			} p += count;
			continue;
			// }}}
		} else if (h== I_RXK || h== I_RXN || h== I_RXLK || h== I_RXLN) {
			// {{{
			count = n+1;
			if (h == I_RXK) {
				fprintf(fp, "%02x%3s : %3d RXK\n", m_binary[p] & 0x0ff, "", count);
				continue;
			} else if (count > 1) {
				if (count > 2)
					fprintf(fp, "%02x%3s : %3d RXK\n", m_binary[p] & 0x0ff, "", count-1);
				else
					fprintf(fp, "%02x%3s : RXK\n", m_binary[p] & 0x0ff, "");
			} if (h == I_RXN) {
				fprintf(fp, "%02x: %5s : RXN\n", p, "");
			} else if (h == I_RXLK) {
				fprintf(fp, "%02x: %5s : RXLK\n", p, "");
			} else if (h == I_RXLN) {
				fprintf(fp, "%02x: %5s : RXLN\n", p, "");
			} continue;
			// }}}
		} else if (h == I_CHANNEL) {
			fprintf(fp, "%02x    ", m_binary[p] & 0x0ff);
		} else if (n == I_SEND || n == I_RXK || n == I_RXN
					|| n == I_RXLK || n == I_RXLN
			|| n == I_CHANNEL) {
			fprintf(fp, "%02x %02x ", m_binary[p] & 0x0ff,
				m_binary[p+1] & 0x0ff);
		} else
			fprintf(fp, "%02x%3s ", m_binary[p] & 0x0ff, "");

		switch(h) {
		case I_NOOP:	fprintf(fp, "  NOOP\n"); break;
		case I_START:	fprintf(fp, "  START\n"); break;
		case I_STOP:	fprintf(fp, "  STOP\n"); break;
		case I_WAIT:	fprintf(fp, "  WAIT\n"); break;
		case I_HALT:	fprintf(fp, "  HALT\n"); break;
		case I_ABORT:	fprintf(fp, "  ABORT\n"); break;
		case I_TARGET:	fprintf(fp, "  TARGET\n"); break;
		case I_JUMP:	fprintf(fp, "  JUMP\n"); break;
		case I_CHANNEL: fprintf(fp, "  CHANNEL\t0x%02x\n",
				m_binary[p] & 0x0f); break;
			// case I_SEND: case I_RX*: // handled above
		default:	fprintf(fp, "  ILL\t(0x%x)\n", h & 0x0ff); break;
		}

		prior_start = (h == I_START);

		if (h == I_SEND || h == I_CHANNEL
				|| h == I_RXK || h == I_RXN
				|| h == I_RXLK || h == I_RXLN)
			continue;

		h = n;

		printf("%02x: ", p);
		fprintf(fp, "%5s ", "");
		// fprintf(fp, "%s ", (prior_start) ? "P":" ");

		switch(h) {
		case I_NOOP:	fprintf(fp, "| NOOP\n"); break;
		case I_START:	fprintf(fp, "| START\n"); break;
		case I_STOP:	fprintf(fp, "| STOP\n"); break;
		case I_SEND:
			// {{{
			count = m_binary[++p] + 1;
			if (prior_start) {
				// {{{
				unsigned	arg = m_binary[++p] & 0x0ff;
				int	d = arg & 1;
				arg >>= 1;
				if (d == D_RD)
					// fprintf(fp, "| READ\t0x%02x\n", arg);
					fprintf(fp, "| SEND\t0x%02x,RD\n", arg & 0x0ff);
				else
					// fprintf(fp, "| WRITE\t0x%02x\n", arg);
					fprintf(fp, "| SEND\t0x%02x,WR\n", arg & 0x0ff);
				if (--count > 0)
					fprintf(fp, "\t");
				// }}}
			} else if (count == 1) {
				fprintf(fp, "| SEND 0x%02x\n",
					m_binary[++p] & 0x0ff);
				count--;
			} else {
				fprintf(fp, "| SEND\t0x%02x\n\t",
					m_binary[++p] & 0x0ff);
				count--;
			} for(unsigned k=0; k<count; k++) {
				fprintf(fp, "0x%02x", m_binary[p+k+1] & 0x0ff);
				if (k+1 == count)
					fprintf(fp, "\n");
				else if (7 == (k & 7))
					fprintf(fp, ",\n\t");
				else
					fprintf(fp, ", ");
			} p += count;
			break;
			// }}}
		case I_RXK: case I_RXN: case I_RXLK: case I_RXLN:
			// {{{
			count = (m_binary[++p] & 0x0ff) + 1;
			if (count <= 1) {
				switch(h) {
				case I_RXK:  fprintf(fp, "| RXK\n"); break;
				case I_RXN:  fprintf(fp, "| RXN\n"); break;
				case I_RXLK: fprintf(fp, "| RXLK\n"); break;
				case I_RXLN: fprintf(fp, "| RXLN\n"); break;
				default: break;
				}
			} else switch(n) {
				case I_RXK:
					fprintf(fp, "| %3d RXK\n", count);
					break;
				case I_RXN:
					fprintf(fp, "| %3d RXK\n", count-1);
					fprintf(fp, "%02x: %6s| RXN\n", p-1, "");
					break;
				case I_RXLK:
					fprintf(fp, "| %3d RXK\n", count-1);
					fprintf(fp, "%02x: %6s| RXLK\n", p-1, "");
					break;
				default: // case I_RXLN:
					fprintf(fp, "| %3d RXK\n", count-1);
					fprintf(fp, "%02x: %6s| RXLN\n", p-1, "");
					break;
			} break;
			// }}}
		case I_WAIT:	fprintf(fp, "| WAIT\n"); break;
		case I_HALT:	fprintf(fp, "| HALT\n"); break;
		case I_ABORT:	fprintf(fp, "| ABORT\n"); break;
		case I_TARGET:	fprintf(fp, "| TARGET\n"); break;
		case I_JUMP:	fprintf(fp, "| JUMP\n"); break;
		case I_CHANNEL: fprintf(fp, "| CHANNEL\t0x%02x\n",
					m_binary[++p] & 0x0ff);
			break;
		default:	fprintf(fp, "| ILL\t(0x%x)\n", h); break;
		}

		prior_start = (h == I_START);
	}
}
// }}}

void	adddefn(const char *str) {
	// {{{
	char		*cpy, *ptr, *ptreq;
	unsigned	val;

	if (verbose_flag) {
		fprintf(stderr, "DEF\'n: %s\n", str);
	}

	cpy = strdup(str);
	ptreq = ptr = strchr(cpy,'=');
	if (ptr == NULL) {
		free(cpy); return;
	}

	ptr++;
	while(*ptr && isspace(*ptr))
		ptr++;

	val = strtoul(ptr, NULL, 0);

	*ptreq = '\0';
	ptreq--;
	while(cpy < ptreq && isspace(*ptreq))
		*ptreq-- = '\0';

	defnlist[ndefns].val = val;
	defnlist[ndefns].str = strdup(cpy);
	ndefns++;
	assert(ndefns < MAX_SYMBOLS);

	free(cpy);
}
// }}}

void	label(const char *str) {
	// {{{
	if (m_pos > 0 && !m_halted) {
		addinsn(I_HALT);
	} m_half = false;

	symlist[nsyms].addr = m_pos;
	symlist[nsyms].str  = strdup(str);

	unsigned slen = strlen(symlist[nsyms].str);
	if (symlist[nsyms].str[slen-1] == ':')
		// This should always be true
		symlist[nsyms].str[slen-1] = '\0';
	nsyms++;

	assert(nsyms < MAX_SYMBOLS);
}
// }}}

unsigned	filesz(FILE *fp) {
	// {{{
	unsigned long	here = ftell(fp), endp;

	fseek(fp, 0l, SEEK_END);
	endp = ftell(fp);
	fseek(fp, here, SEEK_SET);
	return (unsigned)(endp-here);
}
// }}}

void	usage(void) {
	// {{{
	fprintf(stderr, ""
"Usage: i2casm [-bcdhvx] [-o <outfile>] [infiles ...]\n"
"\n"
"\t-b\tProduces a binary output\n"
"\t-c\tProduce a C file output, declaring a variable array\n"
"\t-d\tTakes a binary/compiled file, and dumps the instructions\n"
"\t-h\tThis usage statement\n"
"\t-v\tVerbose mode (may or may not do anything)\n"
"\t-x[#[lb]]\tGenerate a hexadecimal output file, suitable for\n"
"\t\t$readmemh with # bit words, and [l]ittle endian or [b]ig\n"
"\t\tendian encoding.  If no width is given, 8b is assumed.  If\n"
"\t\tno encoding is given, big-endian is assumed.\n"
"\t-o <outfile>\tWrite the results to <outfile>.  If <outfile> is not\n"
"\t\tgiven, results will be written to standard out.\n"
"\t<infiles ...>\tA set of filenames, separated by spaces, to be either\n"
"\t\tassembled or (in the case of -d) disassembled.  If no files\n"
"\t\tare given, the script will be read from the standard input.\n");
}
// }}}

int main(int argc, char **argv) {
	bool	dump_flag = false, cpp_flag = false, hex_flag = true;
	bool	big_endian = true;
	int	opt, width=8;
	// dbgfp  = fopen("dump.txt",  "w");

	init_buffer();

	int	nfiles = 0, argn;
	FILE	*finp, *fout = stdout;
	while(-1 != (opt = getopt(argc, argv, "bcdho:vx::"))) {
		// {{{
		switch(opt) {
		case 'b':	// Binary output
			cpp_flag  = false;
			hex_flag  = false;
			dump_flag = false;
			break;
		case 'c':
			cpp_flag  = true;
			hex_flag  = false;
			dump_flag = false;
			break;
		case 'd':
			hex_flag  = false;
			dump_flag = true;
			cpp_flag  = false;
			break;
		case 'h':
			usage();
			exit(EXIT_SUCCESS);
			break;
		case 'o':
			// output_file = strcpy(optarg);
			fout = fopen(optarg, "w");
			break;
		case 'v':
			fprintf(stderr, "Verbose mode enabled\n");
			verbose_flag = true;
			break;
		case 'x':
			hex_flag  = true;
			dump_flag = false;
			cpp_flag  = false;
			if (isdigit(optarg[0]))
				width = atoi(optarg);
			if (optarg[strlen(optarg)-1] == 'l')
				big_endian = false;
			if (optarg[strlen(optarg)-1] == 'b')
				big_endian = true;
			break;
		}
	}
	// }}}

	if (verbose_flag) {
		if (cpp_flag)
			fprintf(stderr, "Attempting to output in C++ format\n");
		if (dump_flag)
			fprintf(stderr, "Attempting to dump input files\n");
	}

	assert(0 == (width & 0x07));
	width >>= 3;

	// Process individual files
	// {{{
	for(argn=optind; argn<argc; argn++) {
		if (verbose_flag)
			fprintf(stderr, "Attempting to read from %s\n", argv[argn]);
		finp = fopen(argv[argn], "r");
		if (finp == NULL) {
			fprintf(stderr, "ERR: Cannot open %s\n", argv[argn]);
			perror("O/S Err:");
			continue;
		}

		nfiles++;
		if (dump_flag) {
			// {{{
			unsigned	fln = filesz(finp);
			size_t		pos;

			if (m_binary) {
				delete[] m_binary;
				m_binsz = 0;
			}

			m_binary = new char[fln+1];
			m_binary[fln] = 0;
			m_pos = (pos = fread(m_binary, 1, fln, finp));
			fprintf(fout, "DUMP: %s\n===============================\n",
				argv[argn]);
			dump(fout);
			fprintf(fout, "\n");
			fclose(finp);
			delete[] m_binary;
			m_binary = NULL;
			// }}}
		} else {
			// yyin = yy_new_buffer(finp, 32768);
			yyin = finp;
			yylex();
		}
	}
	// }}}

	// (Optionally) process stdin
	// {{{
	if (nfiles != 0) {
		// {{{
		if (verbose_flag)
			fprintf(stderr, "All files processed\n");

		if (!dump_flag) {
			if (!m_stopped)
				addinsn(I_STOP);
			if (!m_halted)
				addinsn(I_HALT);
		}
		// }}}
	} else if (dump_flag) {
		// {{{
		size_t	nr;

		assert(m_binsz > 0);
		assert(m_binary != NULL);
		assert(m_pos == 0);

		while((nr = fread(&m_binary[m_pos], 1, m_binsz-m_pos, stdin))
				== m_binsz-m_pos) {

			if (m_binsz > 65536) {
				fprintf(stderr, "ERR: File size exceeds artificial 64kB limit!\n");
				exit(EXIT_FAILURE);
			}

			m_pos += nr;
			grow_buffer();
		}

		fprintf(fout, "DUMP: (stdin)\n====================\n");
		dump(fout);
		fprintf(fout, "\n");
		fclose(finp);
		delete[] m_binary;
		m_binary = NULL;
		// }}}
	} else {
		yylex();	// Use stdin

		if (!m_stopped)
			addinsn(I_STOP);
		if (!m_halted)
			addinsn(I_HALT);
	}
	// }}}

	// Write the file out
	// {{{
	if (!dump_flag) {
		if (cpp_flag) {		// Write a C++ output header file
			// {{{
			unsigned	sympos = 0, tabstart = 0;

			if (nsyms > 0 && symlist[0].addr == 0)
				fprintf(fout, "const char %s[] = {\n\t", symlist[sympos++].str);
			else
				fprintf(fout, "const char i2casm[] = {\n\t");
			for(int p=0; p<m_pos; p++) {
				if (sympos < nsyms && symlist[sympos].addr == p) {
					fprintf(fout, "};\n\nconst char %s[] = {\n\t", symlist[sympos++].str);
					tabstart = p;
				}

				fprintf(fout, "0x%02x", m_binary[p] & 0x0ff);
				if ((p==m_pos-1)
					||(sympos < nsyms
						&& symlist[sympos].addr==p+1))
					fprintf(fout, "\n");
				else if (((p-tabstart) & 7) != 7)
					fprintf(fout, ", ");
				else
					fprintf(fout, ",\n\t");
			} fprintf(fout, "};\n");
			// }}}
		} else if (hex_flag) {	// Writ hex file, user selected width
			// {{{
			unsigned	acc = 0, ln=0, s;
			char		lbuf[128];

printf("WIDTH = %d, big_endian is %s\n", width, big_endian ? "true":"false");
			for(int p=0; p<m_pos; p++) {
				lbuf[0] = '\0';
for(int k=0; k<128; k++)
	lbuf[k] = '\0';

				if (big_endian) {
					// {{{
					for(s=0; s<width && s+p < m_pos; s++) {
						unsigned v = m_binary[p+s];

						v &= 0x0ff;
						sprintf(&lbuf[strlen(lbuf)],
							"%02x", v);
					} for(; s < width; s++) {
						strcat(lbuf, "00");
					}
					// }}}
				} else {
					// {{{
					char *d = &lbuf[2*width];

					*d-- = '\0';

					for(s=0; s<width && s+p < m_pos; s++) {
						unsigned v = m_binary[p+s];

						v &= 0x0ff;
						if ((v & 0x0f) >= 10)
							*d-- = 'a'+(v&0x0f)-10;
						else
							*d-- = '0'+(v&0x0f);
// printf("CH = %02x -> %02x %c\n", v, d[1] & 0x0ff, d[1]);

						v >>= 4;

						if (v >= 10)
							*d-- ='a'+ v -10;
						else
							*d-- ='0'+ v;

					} for(; s < width; s++) {
						*d-- = '0';
						*d-- = '0';
					}
					// }}}
				}

				ln += width*2 + 1;
				fputs(lbuf, fout);
				if (ln + (width*2) >= 80) {
					fputc('\n', fout);
					ln = 0;
				} else {
					fputc(' ', fout);
				} p += width -1;
			}

			if (ln != 0)
				fputc('\n', fout);
			// }}}
		} else {
			fwrite(m_binary, sizeof(char), m_pos, fout);
		}
	}
	// }}}

	fclose(fout); 
	return (EXIT_SUCCESS);
}

