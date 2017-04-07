#include <stdio.h>
#include <assert.h>

class	I2CBUS {
public:
	int	m_sck:1;
	int	m_sda:1;
	I2CBUS(int sck=1, int sda=1) : m_sck(sck), m_sda(sda) {};
	I2CBUS	operator+(const I2CBUS b) const {
		return I2CBUS(m_sck&b.m_sck, m_sda&b.m_sda); }
};

typedef	enum { I2CIDLE, I2CADDR, I2CSACK, I2CSRX, I2CSTX, I2CMACK, I2CILLEGAL
} I2CSTATE;

class	I2CSIMSLAVE {
	char	m_data[128];
	int	m_addr, m_daddr, m_abits, m_dbits, m_dreg, m_ack,
			m_last_sda, m_last_sck, m_counter;
	bool	m_illegal;
	I2CSTATE	m_state;

	I2CSIMSLAVE(void) {
		m_last_sda = 1;
		m_last_sck = 1;
		for(int i=0; i<128; i++)
			m_data[i] = 0;
		m_addr = 0;
		m_illegal = false;
		m_state = I2CIDLE;
	}
	volatile int	getack(int addr) {
		m_ack = 0;
		return m_ack;
	}
	volatile char	read(int addr) {
		m_daddr = addr;
		return m_data[m_daddr];
	}
	volatile char	read(void) {
		m_daddr = (m_daddr+1)&0x07f;
		return m_data[m_daddr];
	}
	volatile void	write(int addr, char data) {
		m_daddr = addr & 0x07f;
		m_data[m_daddr] = data;
	} volatile void	write(char data) {
		m_daddr = (m_daddr++) & 0x07f;
		m_data[m_daddr] = data;
	}
	I2CBUS	operator()(int sck, int sda);
	I2CBUS	operator()(const I2CBUS b) { return (*this)(b.m_sck, b.m_sda); }
};

/*
class	I2CSIMMASTER {
	I2CBUS	operator(int sck, int sda);
	I2CBUS	operator(const I2CBUS b) { return (*this)(b.m_sck, b.m_sda); }
};

*/

I2CBUS	I2CSIMSLAVE::operator()(int sck, int sda) {
	I2CBUS	r(sck, sda); // Our default result

	if ((sck)&&(m_last_sck)&&(sda)&&(!m_last_sda)) {
		// Stop bit: Low to high transition with sck high
		// Leave the bus as is
		m_state = I2CIDLE;
		m_illegal = false;
	} else switch(m_state) {
	case I2CIDLE:
		if (!sda) {
			m_state = I2CADDR;
			m_addr  = 0;
			m_abits = 0;
			m_ack   = 1;
		} else if (!sck) {
			m_state = I2CILLEGAL;
		} // The the bus as it was on entry
		break;
	case	I2CADDR:
		if ((sck)&&(!m_last_sck)) {
			m_addr = (m_addr<<1)|sda;
			m_abits++;
			if (m_abits == 8) {
				m_state = I2CSACK;
				m_ack = getack(m_addr);
			} m_counter = 0;
		} else if (sck) {
			// Can't change when the clock is high
			assert(sda == m_last_sda);
		} // The the bus as it was on entry
		break;
	case	I2CSACK:
		// Ack the master
		if (!sck)
			// Master is not allowed to pull the line low
			assert(r.m_sda);
		r.m_sda = m_ack;
		if (m_counter++ < 40000)
			r.m_sck = 0;
		if ((!r.m_sck)&&(m_last_sck)) {
			if (m_addr&1) {
				m_state = I2CSRX;
			} else {
				m_state = I2CSTX;
				m_dreg = read(m_addr>>1);
			}
		} m_dbits = 0;
		break;
	case	I2CSRX: {
		if (r.m_sck) {
			// Not allowed to change when clock is high
			if (m_last_sck)
				assert(sda == m_last_sda);
			if (!m_last_sck) {
				m_dreg = ((m_dreg<<1) | r.m_sda)&0x0ff;
				m_dbits++;
				if (m_dbits == 8) {
					m_addr = (m_addr + 2)&0x0ff;
					// Get an ack from the master
					m_state = I2CSACK;
					write(m_addr>>1, m_dreg);
				}
			}
		} break;
	case	I2CSTX:
		assert(sda);
		if (r.m_sck) {
			// Not allowed to change when clock is high
			r.m_sda = m_last_sda;
		} else
			r.m_sda = m_dreg>>(7-(m_dbits&0x07));
			if (m_last_sck) {
				m_dbits++;
				if (m_dbits == 8) {
					m_dreg = read();
					// Get an ack from the master
					m_state = I2CMACK;
				}
			}
		} break;
	case	I2CMACK:
		// Insist that the master actually ACK
		//
		// Sadly, we can't.  The master can NAK and ... that's
		// the end.
		// Give the master a chance to ACK
		if ((!r.m_sck)&&(m_last_sck)) {
			if (!sda) {
				// master ACK'd.  Go on
				m_state = I2CSTX;
				m_dreg = read();
			} else {
				m_state = I2CILLEGAL;
			}
		} m_dbits = 0;
		break;
	case	I2CILLEGAL:	// fall through
	default:
		if (!m_illegal) {
			fprintf(stderr, "I2C: Illegal state!!\n");
			m_illegal = true;
			m_state = I2CILLEGAL;
		}
		break;
	}

	m_last_sck = r.m_sck;
	m_last_sda = r.m_sda;
	return r;
}

