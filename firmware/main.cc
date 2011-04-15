#include "avrlib/async_usart.hpp"
#include "avrlib/usart1.hpp"

#include "avrlib/timer1.hpp"
#include "avrlib/counter.hpp"
#include "avrlib/stopwatch.hpp"
#include "avrlib/format.hpp"

#include "avrlib/buffer.hpp"
#include "avrlib/pin.hpp"
#include "avrlib/portd.hpp"
#include "avrlib/assert.hpp"

#include "avrlib/command_parser.hpp"

#include "pdi.hpp"

typedef avrlib::pin<avrlib::portd, 4> pdi_clk;
typedef avrlib::pin<avrlib::portd, 1> pdi_data;

avrlib::async_usart<avrlib::usart1, 64, 64, avrlib::bootseq> com(38400, true);

ISR(USART1_RXC_vect)
{
	com.process_rx();
}

typedef avrlib::counter<avrlib::timer1> clock_t;
clock_t clock;

ISR(TIMER1_OVF_vect)
{
	clock.tov_interrupt();
}

template <typename Clock, typename PdiClk, typename PdiData>
class pdi_t
{
public:
	explicit pdi_t(Clock & clock)
		: m_clock(clock), m_state(st_disabled)
	{
	}

	void clear()
	{
		PdiClk::output(false);
		PdiClk::clear();

		PdiData::output(false);
		PdiData::clear();

		UCSR0B = 0;
		UCSR0C = (1<<URSEL0);

		m_state = st_disabled;
	}

	void init()
	{
		if (m_state != st_disabled)
			return;

		PdiClk::clear();
		PdiClk::output(true);

		PdiData::set();
		PdiData::output(true);

		m_state = st_rst_disable;
		m_time_base = m_clock.value();
	}

	bool tx_ready() const
	{
		return m_state == st_tx && !m_tx_buffer.full() && m_rx_count == 0;
	}

	bool tx_empty() const
	{
		return m_tx_buffer.empty();
	}

	bool rx_ready() const
	{
		return !m_rx_buffer.empty();
	}

	void write(uint8_t data, uint8_t rx_count = 0)
	{
		AVRLIB_ASSERT(this->tx_ready());

		UCSR0B = (1<<TXEN0);
		m_tx_buffer.push(data);
		m_rx_count = rx_count;
		UCSR0A = (1<<TXC0);
		UCSR0B = (1<<TXEN0)|(1<<UDRIE0);
	}

	uint8_t read()
	{
		while (!this->rx_ready())
		{
		}

		uint8_t res = m_rx_buffer.top();
		m_rx_buffer.pop();
		return res;
	}

	// Must be called at least every 50us
	void process()
	{
		switch (m_state)
		{
		case st_disabled:
			break;
		case st_rst_disable:
			if (m_clock.value() - m_time_base >= 8) // FIXME: time constants
			{
				UBRR0H = 0;
				UBRR0L = 10;
				UCSR0C = (1<<URSEL0) | (1<<UMSEL0)
					| (1<<UPM01) | (1<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (1<<UCPOL0);
				UCSR0B = (1<<TXEN0);

				m_state = st_wait_ticks;
				m_time_base += 8;
			}
			break;
		case st_wait_ticks:
			if (m_clock.value() - m_time_base >= 64) // FIXME: time constants
			{
				PdiData::output(false);
				PdiData::clear();

				m_state = st_tx;
				m_rx_count = 0;
			}
			break;
		case st_tx:
			break;
		}
	}

	void intr_rxc()
	{
		m_rx_buffer.push(UDR0);
		if (--m_rx_count == 0)
			UCSR0B = (1<<TXEN0);
	}

	void intr_udre()
	{
		if (!m_tx_buffer.empty())
		{
			UDR0 = m_tx_buffer.top();
			m_tx_buffer.pop();
			if (m_tx_buffer.empty())
			{
				if (m_rx_count)
					UCSR0B = (1<<TXEN0)|(1<<TXCIE0);
				else
					UCSR0B = (1<<TXEN0);
			}
		}
	}
	
	void intr_txc()
	{
		UCSR0B = (1<<RXEN0)|(1<<RXCIE0);
	}

private:
	Clock & m_clock;
	typename Clock::time_type m_time_base;
	avrlib::buffer<uint8_t, 16> m_rx_buffer;
	avrlib::buffer<uint8_t, 16> m_tx_buffer;
	volatile uint8_t m_rx_count;

	enum { st_disabled, st_rst_disable, st_wait_ticks, st_tx } m_state;
};

pdi_t<clock_t, pdi_clk, pdi_data> pdi(clock);

ISR(USART0_RXC_vect)
{
	pdi.intr_rxc();
}

ISR(USART0_UDRE_vect)
{
	pdi.intr_udre();
}

ISR(USART0_TXC_vect)
{
	pdi.intr_txc();
}

void process()
{
	pdi.process();
	com.process_tx();
}

int main()
{
	sei();

	clock.enable(avrlib::timer_fosc_8);

	avrlib::command_parser cp;
	cp.clear();

	for (;;)
	{
		if (!com.empty())
		{
			uint8_t ch = cp.push_data(com.read());
			switch (ch)
			{
			case 'a':
				pdi.init();
				break;
			case 'A':
				pdi.clear();
				break;
			case 'r':
				pdi_ldcs(pdi, 0);
				break;
			case 'g':
				pdi_stcs(pdi, 2, 1);
				break;
			case 'm':
				pdi_ldcs(pdi, 1);
				break;
			case 'M':
				pdi_stcs(pdi, 1, 0x59);
				break;
			case 'T':
				avrlib::send_hex(com, clock.value());
				com.write('\n');
				break;
			case 'k':
				pdi_key(pdi, 0x1289AB45CDD888FFull);
				/*pdi.write(0xe0);

				// 0x1289AB45CDD888FF
				pdi.write(0xff);
				pdi.write(0x88);
				pdi.write(0xd8);
				pdi.write(0xcd);
				pdi.write(0x45);
				pdi.write(0xab);
				pdi.write(0x89);
				pdi.write(0x12);*/
				break;
			case 'R': // Get the current NVM controller command
				pdi_lds(pdi, (uint32_t)0x010001CA, 1);
				break;
			case 'W': // Set "read NVM" command
				pdi_sts(pdi, (uint32_t)0x010001CA, (uint8_t)0x43);
				break;
			case 'P': // Read the signature row
				pdi_lds(pdi, (uint32_t)0x01000090, 4);
				break;
			case '?':
				avrlib::send_hex(com, UCSR0A);
				com.write(' ');
				avrlib::send_hex(com, UCSR0B);
				com.write(' ');
				avrlib::send_hex(com, UCSR0C);
				com.write('\n');
				break;
			case 0:
				// Send out the identification
				com.write(0x80);
				com.write(0x04);
				com.write(0xbd);
				com.write(0xe9);
				com.write(0x9f);
				com.write(0xe9);
				break;
			case 1:
			case '1':
				{
					// Enable programming mode
					pdi.init();
					while (!pdi.tx_ready())
						process();

					pdi_key(pdi, 0x1289AB45CDD888FFull);

					clock_t::time_type t = clock.value();

					uint8_t pdi_status = 0;
					bool success = true;
					while (success && (pdi_status & 0x02) == 0 && clock.value() - t < 1000000)
					{
						pdi_ldcs(pdi, 0);
						success = pdi_read(pdi, pdi_status, clock, process);
					}

					com.write(0x80);
					com.write(0x11);
					com.write(!success? 1: (pdi_status & 0x02) == 0? 3: 0);
				}
				break;
			case 2:
				// Leave programming mode
				pdi.clear();

				com.write(0x80);
				com.write(0x21);
				com.write(0);
				break;
			case 3:
				{
					// Read signature, lock bits, fuses and the calibration byte 
					pdi_sts(pdi, (uint8_t)0x010001CA, (uint8_t)0x43);
					pdi_lds(pdi, (uint32_t)0x01000090, 3);

					com.write(0x80);
					com.write(0x33);

					bool success = true;
					
					for (uint8_t i = 0; i != 3; ++i)
					{
						uint8_t v = 0;
						if (success)
							success = pdi_read(pdi, v, clock, process);
						com.write(v);
					}

					if (!success)
					{
						pdi.clear();
						com.write(0x80);
						com.write(0x21);
						com.write(1);
					}
				}
				break;
			case 4: // Read memory
				if (cp.size() == 6)
				{
					uint8_t memid = cp[0];

					uint32_t addr = cp[1] | (cp[2] << 8) | ((uint32_t)cp[3] << 16) | ((uint32_t)cp[4] << 24);
					addr %= 0x0C0000;
					addr += 0x800000;

					pdi_sts(pdi, (uint32_t)0x010001CA, (uint8_t)0x43);
					pdi_st_ptr(pdi, addr);

					uint8_t len = cp[5];

					com.write(0x80);
					com.write(0xf4);
					com.write(len);

					bool success = true;
					for (uint8_t i = 0; i != len; ++i)
					{
						pdi_ld(pdi);

						uint8_t v = 0;
						if (success)
							success = pdi_read(pdi, v, clock, process);
						com.write(v);
					}

					com.write(0);

					if (!success)
					{
						pdi.clear();
						com.write(0x80);
						com.write(0x21);
						com.write(1);
					}
				}
				break;
			case 5:
				// Prepare memory page for a load and write.
				// WPREP 1'memid 4'addr
				if (cp.size() >= 5)
				{
					uint8_t memid = cp[0];
					uint32_t addr = cp[1] | (cp[2] << 8) | ((uint32_t)cp[3] << 16) | ((uint32_t)cp[4] << 24);
					addr %= 0x0C0000;
					addr += 0x800000;

					pdi_sts(pdi, (uint32_t)0x010001CA, (uint8_t)0x26);
					pdi_sts(pdi, (uint32_t)0x010001CB, (uint8_t)0x01);

					bool success = pdi_wait_nvm_busy(pdi, clock, 10000, process);

					pdi_sts(pdi, (uint32_t)0x010001CA, (uint8_t)0x23);
					pdi_st_ptr(pdi, addr);

					com.write(0x80);
					com.write(0x51);
					com.write(!success);
				}
				break;
			case 6:
				// ERASE 1'memid
				{
					// CMD = Chip erase
					pdi_sts(pdi, (uint32_t)0x010001CA, (uint8_t)0x40);
					pdi_sts(pdi, (uint32_t)0x010001CB, (uint8_t)0x01);

					bool success = pdi_wait_nvm_busy(pdi, clock, 10000, process);
					com.write(0x80);
					com.write(0x61);
					com.write(!success);
				}
				break;
			case 7:
				// Prepare memory page for a load and write.
				// WFILL 1'memid 1'seq (1'data)*
				if (cp.size() >= 2)
				{
					for (uint8_t i = 2; i < cp.size(); ++i)
					{
						while (!pdi.tx_empty())
							process();
						pdi_st(pdi, cp[i]);
					}
					com.write(0x80);
					com.write(0x70);
				}
				break;
			case 8:
				// WRITE 1'memid 4'addr
				if (cp.size() == 5)
				{
					uint32_t addr = cp[1] | (cp[2] << 8) | ((uint32_t)cp[3] << 16) | ((uint32_t)cp[4] << 24);
					addr %= 0x0C0000;
					addr += 0x800000;

					// CMD = Erase & Write Flash Page
					pdi_sts(pdi, (uint32_t)0x010001CA, (uint8_t)0x2F);
					pdi_sts(pdi, addr, (uint8_t)0);

					bool success = pdi_wait_nvm_busy(pdi, clock, 10000, process);

					com.write(0x80);
					com.write(0x81);
					com.write(!success);
				}
				break;
			default:
				if ('0' <= ch && ch <= '9')
					pdi.write(0x80 | (ch - '0'), 1);
				break;
			}

			if (cp.state() == avrlib::command_parser::simple_command || cp.state() == avrlib::command_parser::bad)
				cp.clear();
		}

		if (pdi.rx_ready())
		{
			send_hex(com, pdi.read());
			com.write('\n');
		}

		process();
	}
}
