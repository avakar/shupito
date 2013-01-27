#ifndef SHUPITO_FW_COMMON_PDI_HPP
#define SHUPITO_FW_COMMON_PDI_HPP

#include "avrlib/buffer.hpp"

template <typename Clock, typename PdiClk, typename PdiData, typename Led>
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

		this->cancel_read();
		while (m_state != st_idle)
			this->process();

		// A few more clocks should be supplied
		// to make sure that the last byte was received correctly
		m_state = st_unrst;
		m_time_base = m_clock.value();
		m_led.off();
	}

	void init(uint16_t bsel)
	{
		if (bsel)
			--bsel;
		USARTC0_BAUDCTRLA = bsel;
		USARTC0_BAUDCTRLB = bsel >> 8;

		if (m_state != st_disabled)
			return;

		PdiClk::make_inverted();

		// PdiClk is now inverted, so as to make
		// the USART sample on the rising edge.
		// The following will therefor NOT put the chip
		// in reset (which we don't want to, enabling the PDI
		// might be a prelude to debugging).
		PdiClk::make_low();

		// We can be sure the PDI line was low before this -- there
		// must be no pull-ups on the line in order for the
		// chip's buskeeper to properly transmit.
		PdiData::make_high();
		m_state = st_rst_disable;

		m_time_base = m_clock.value();
	}

	bool tx_ready() const
	{
		return (m_state == st_idle || m_state == st_busy) && !m_tx_buffer.full() && m_rx_count == 0;
	}

	bool tx_empty() const
	{
		return m_tx_buffer.empty();
	}

	void write(uint8_t data)
	{
		this->write(data, 0, 0);
	}

	void write(uint8_t data, uint8_t rx_count, uint8_t * rx_buf)
	{
		while (!this->tx_ready())
			this->process();

		USARTC0_CTRLA = 0;
		m_rx_count = rx_count;
		m_rx_buf = rx_buf;
		m_state = st_busy;
		
		// Attempt to push the data into the data register if it's free,
		// thus avoiding the overhead of an interrupt.
		if ((USARTC0_STATUS & USART_DREIF_bm) != 0 && m_tx_buffer.empty())
		{
			cli();
			USARTC0_DATA = data;
			USARTC0_STATUS = USART_TXCIF_bm;
			sei();
			USARTC0_CTRLA = USART_TXCINTLVL_HI_gc;
		}
		else
		{
			m_tx_buffer.push(data);
			USARTC0_CTRLA = USART_DREINTLVL_MED_gc;
		}
	}
	
	uint8_t read_count() const
	{
		return m_rx_count;
	}

	void cancel_read()
	{
		if (m_rx_count)
			this->cancel();
	}

	void cancel()
	{
		if (m_state == st_busy)
		{
			USARTC0_CTRLA = 0;
			USARTC0_CTRLB = USART_TXEN_bm;
			PdiData::make_output();
			m_rx_count = 0;
			m_tx_buffer.clear();
			m_state = st_idle;
		}
	}

	// Must be called at least every 50us
	void process()
	{
		switch (m_state)
		{
		case st_disabled:
			break;
		case st_rst_disable:
			if (m_clock.value() - m_time_base > Clock::template us<8>::value)
			{
				USARTC0_CTRLC = USART_CMODE_SYNCHRONOUS_gc | USART_PMODE_EVEN_gc | USART_SBMODE_bm
					| USART_CHSIZE_8BIT_gc;
				USARTC0_CTRLA = 0;
				USARTC0_CTRLB = USART_TXEN_bm;

				m_state = st_wait_ticks;
				m_time_base = m_clock.value();
			}
			break;
		case st_wait_ticks:
			// worst-case scenario is 10kHz programming speed
			if (m_clock.value() - m_time_base > Clock::template us<1800>::value)
			{
				m_state = st_idle;
				m_rx_count = 0;
				pdi_stcs(*this, 0x02/*CTRL*/, 0x02/*GUARDTIME_32*/);
			}
			break;
		case st_idle:
			m_led.off();
			break;
		case st_rx_done:
			m_time_base = m_clock.value();
			m_state = st_rx_done_wait;
			//fallthrough
		case st_rx_done_wait:
			if (m_clock.value() - m_time_base > Clock::template us<100>::value)
				m_state = st_idle;
			//fallthrough
		case st_busy:
			m_led.on();
			break;
		case st_unrst:
			if (m_clock.value() - m_time_base > Clock::template us<100>::value)
			{
				PdiClk::make_input();
				PdiData::make_input();
				PdiClk::make_noninverted();

				USARTC0_CTRLA = 0;
				USARTC0_CTRLB = 0;
				USARTC0_CTRLC = 0;

				m_state = st_disabled;
			}
			break;
		}
	}

	bool enabled() const
	{
		return m_state != st_disabled && m_state != st_unrst;
	}

	void intr_rxc()
	{
		*m_rx_buf++ = USARTC0_DATA;
		if (--m_rx_count == 0)
		{
			USARTC0_CTRLA = 0;
			USARTC0_CTRLB = USART_TXEN_bm;
			PdiData::make_output();
			m_state = st_rx_done;
		}
	}

	void intr_udre()
	{
		AVRLIB_ASSERT(!m_tx_buffer.empty());

		cli();
		USARTC0_DATA = m_tx_buffer.top();
		USARTC0_STATUS = USART_TXCIF_bm;
		sei();

		m_tx_buffer.pop();
		if (m_tx_buffer.empty())
		{
			USARTC0_CTRLA = USART_TXCINTLVL_HI_gc;
		}
	}
	
	void intr_txc()
	{
		AVRLIB_ASSERT(m_tx_buffer.empty());

		if (m_rx_count)
		{
			// We must disable TX and only then enable RX
			// Apparently, the clock is not reset if these
			// actions occur simultaneously and the receiver will
			// read a spurious byte if it is enabled near
			// the sampling edge.
			USARTC0_CTRLB = 0;
			PdiData::make_input();
			USARTC0_CTRLB = USART_RXEN_bm;
			USARTC0_CTRLA = USART_RXCINTLVL_MED_gc;
		}
		else
		{
			m_state = st_idle;
			USARTC0_CTRLA = 0;
		}
	}

	uint8_t state() const { return m_state; }

private:
	Clock & m_clock;
	typename Clock::time_type m_time_base;
	avrlib::buffer<uint8_t, 16> m_tx_buffer;
	volatile uint8_t m_rx_count;
	volatile uint8_t * m_rx_buf;
	Led m_led;

	enum { st_disabled, st_rst_disable, st_wait_ticks, st_idle, st_rx_done, st_rx_done_wait, st_busy, st_unrst } volatile m_state;
};

#endif // SHUPITO_FW_COMMON_PDI_HPP
