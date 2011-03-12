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

		m_state = st_disabled;
	}

	void init()
	{
		AVRLIB_ASSERT(m_state == st_disabled);

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

	bool rx_ready() const
	{
		return !m_rx_buffer.empty();
	}

	void write(uint8_t data, uint8_t rx_count)
	{
		AVRLIB_ASSERT(this->tx_ready());

		UCSR0B = (1<<TXEN0);
		m_tx_buffer.push(data);
		m_rx_count = rx_count;
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
			if (m_clock.value() - m_time_base < 32) // FIXME: time constants
			{
				m_state = st_wait_ticks;
				m_time_base += 32;
			}
			break;
		case st_wait_ticks:
			if (m_clock.value() - m_time_base < 128) // FIXME: time constants
			{
				UBRR0H = 0;
				UBRR0L = 10;
				UCSR0C = (1<<URSEL0) | (1<<UMSEL0)
					| (1<<UPM01) | (1<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (1<<UCPOL0);
				UCSR0B = (1<<TXEN0);

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
	
	void intr_txc()
	{
		UCSR0B = (1<<RXEN0)|(1<<RXCIE0);
	}

private:
	Clock & m_clock;
	typename Clock::time_type m_time_base;
	avrlib::buffer<uint8_t, 8> m_rx_buffer;
	avrlib::buffer<uint8_t, 8> m_tx_buffer;
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

int main()
{
	sei();

	avrlib::timer1::tov_interrupt(true);
	avrlib::timer1::clock_source(avrlib::timer_fosc_8);

	for (;;)
	{
		if (!com.empty())
		{
			uint8_t ch = com.read();
			switch (ch)
			{
			case 'a':
				pdi.init();
				break;
			case 'A':
				pdi.clear();
				break;
			case 'r':
				pdi.write(0x80, 1);
				break;
			case 'T':
				avrlib::send_hex(com, clock.value());
				com.write('\n');
				break;
			}
		}

		if (pdi.rx_ready())
			send_hex(com, pdi.read());

		pdi.process();
		com.process_tx();
	}
}
