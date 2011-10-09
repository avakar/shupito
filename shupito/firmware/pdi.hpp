#ifndef SHUPITO_PDI_HPP
#define SHUPITO_PDI_HPP

#include <stdint.h>

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
		if (m_state == st_disabled)
			return;

		while (m_state != st_tx)
			this->process();

		while ((UCSR0A & (1<<TXC0)) == 0)
			this->process();

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
		while (!this->tx_ready())
			this->process();

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
			if (m_clock.value() - m_time_base >= Clock::template us<8>::value) // FIXME: time constants
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
			if (m_clock.value() - m_time_base >= Clock::template us<64>::value) // FIXME: time constants
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

#endif
