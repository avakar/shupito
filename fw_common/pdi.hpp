#ifndef SHUPITO_FW_COMMON_PDI_HPP
#define SHUPITO_FW_COMMON_PDI_HPP

template <typename Clock, typename PdiClk, typename PdiData, typename PinLed>
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
		PinLed::set_value(false);
	}

	void init(uint16_t bsel)
	{
		if (bsel)
			--bsel;
		USARTC0.BAUDCTRLA = bsel;
		USARTC0.BAUDCTRLB = bsel >> 8;

		if (m_state != st_disabled)
			return;

		PdiClk::make_inverted();

		// PdiClk is now inverted, so as to make
		// the USART sample on the rising edge.
		// The following will therefor NOT put the chip
		// in reset (which we don't want to, enabling the PDI
		// might be a prelude to debugging).
		PdiClk::make_low();

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

	bool rx_ready() const
	{
		return !m_rx_buffer.empty();
	}

	void write(uint8_t data, uint8_t rx_count = 0)
	{
		while (!this->tx_ready())
			this->process();

		USARTC0.CTRLA = 0;
		m_tx_buffer.push(data);
		m_rx_count = rx_count;
		m_state = st_busy;
		USARTC0.CTRLA = USART_DREINTLVL_MED_gc;
	}

	uint8_t read()
	{
		while (!this->rx_ready())
			this->process();

		uint8_t res = m_rx_buffer.top();
		m_rx_buffer.pop();
		return res;
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
			USARTC0.CTRLA = 0;
			USARTC0.CTRLB = USART_TXEN_bm;
			PdiData::make_output();
			m_rx_count = 0;
			m_tx_buffer.clear();
			m_rx_buffer.clear();
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
				USARTC0.CTRLC = USART_CMODE_SYNCHRONOUS_gc | USART_PMODE_EVEN_gc | USART_SBMODE_bm
					| USART_CHSIZE_8BIT_gc;
				USARTC0.CTRLA = 0;
				USARTC0.CTRLB = USART_TXEN_bm;

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
			PinLed::set_value(false);
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
			PinLed::set_value(true);
			break;
		case st_unrst:
			if (m_clock.value() - m_time_base > Clock::template us<100>::value)
			{
				PdiClk::make_input();
				PdiData::make_input();
				PdiClk::make_noninverted();

				USARTC0.CTRLA = 0;
				USARTC0.CTRLB = 0;
				USARTC0.CTRLC = 0;

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
		m_rx_buffer.push(USARTC0.DATA);
		if (--m_rx_count == 0)
		{
			USARTC0.CTRLA = 0;
			USARTC0.CTRLB = USART_TXEN_bm;
			PdiData::make_output();
			m_state = st_rx_done;
		}
	}

	void intr_udre()
	{
		AVRLIB_ASSERT(!m_tx_buffer.empty());

		cli();
		USARTC0.DATA = m_tx_buffer.top();
		USARTC0.STATUS = USART_TXCIF_bm;
		sei();

		m_tx_buffer.pop();
		if (m_tx_buffer.empty())
		{
			USARTC0.CTRLA = USART_TXCINTLVL_HI_gc;
		}
	}
	
	void intr_txc()
	{
		AVRLIB_ASSERT(m_tx_buffer.empty());

		if (m_rx_count)
		{
			PdiData::make_input();
			USARTC0.CTRLB = USART_RXEN_bm;
			USARTC0.CTRLA = USART_RXCINTLVL_MED_gc;
		}
		else
		{
			m_state = st_idle;
			USARTC0.CTRLA = 0;
		}
	}

	uint8_t state() const { return m_state; }

private:
	Clock & m_clock;
	typename Clock::time_type m_time_base;
	avrlib::buffer<uint8_t, 16> m_rx_buffer;
	avrlib::buffer<uint8_t, 16> m_tx_buffer;
	volatile uint8_t m_rx_count;

	enum { st_disabled, st_rst_disable, st_wait_ticks, st_idle, st_rx_done, st_rx_done_wait, st_busy, st_unrst } volatile m_state;
};

#endif // SHUPITO_FW_COMMON_PDI_HPP
