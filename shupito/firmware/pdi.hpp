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
		if (m_state != st_disabled)
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

template <typename Pdi, typename T>
void pdi_lds(Pdi & pdi, T addr, uint8_t rx_count)
{
	pdi.write(0x00 | ((sizeof addr - 1) << 2) | (rx_count - 1));
	for (uint8_t i = 1; i != sizeof addr; ++i)
	{
		pdi.write((uint8_t)addr);
		addr >>= 8;
	}
	pdi.write((uint8_t)addr, rx_count);
}

template <typename R, typename Pdi, typename T>
void pdi_sts(Pdi & pdi, T addr, R data)
{
	pdi.write(0x40 | ((sizeof addr - 1) << 2) | (sizeof data - 1));
	for (uint8_t i = 0; i != sizeof addr; ++i)
	{
		pdi.write((uint8_t)addr);
		addr >>= 8;
	}
	for (uint8_t i = 0; i != sizeof data; ++i)
	{
		pdi.write((uint8_t)data);
		data >>= 8;
	}
}

template <typename Pdi>
void pdi_ldcs(Pdi & pdi, uint8_t addr)
{
	pdi.write(0x80 | addr, 1);
}

template <typename Pdi>
void pdi_stcs(Pdi & pdi, uint8_t addr, uint8_t data)
{
	pdi.write(0xC0 | addr);
	pdi.write(data);
}

template <typename Pdi>
void pdi_key(Pdi & pdi, uint64_t key)
{
	pdi.write(0xe0);
	for (uint8_t i = 0; i != 8; ++i)
	{
		pdi.write(key);
		key >>= 8;
	}
}

template <typename Pdi, typename T>
void pdi_st_ptr(Pdi & pdi, T addr)
{
	pdi.write(0x68 | (sizeof addr - 1));
	for (uint8_t i = 0; i != sizeof addr; ++i)
	{
		pdi.write(addr);
		addr >>= 8;
	}	
}

template <typename Pdi>
void pdi_ld_ptr(Pdi & pdi, uint8_t len)
{
	pdi.write(0x28 | (len - 1), len);
}

template <typename Pdi>
void pdi_ld(Pdi & pdi)
{
	pdi.write(0x24, 1);
}

template <typename Pdi, typename T>
void pdi_st(Pdi & pdi, T t)
{
	pdi.write(0x64 | (sizeof t - 1));
	for (uint8_t i = 0; i != sizeof t; ++i)
	{
		pdi.write(t);
		t >>= 8;
	}	
}

template <typename Pdi, typename T>
void pdi_repeat(Pdi & pdi, T count)
{
	pdi.write(0xa0 | (sizeof count - 1));
	for (uint8_t i = 0; i != sizeof count; ++i)
	{
		pdi.write(count);
		count >>= 8;
	}	
}

/*
template <typename Pdi, typename T, typename U>
void pdi_ld_range(Pdi & pdi, T addr, U count)
{
	pdi_repeat(pdi, count - 1);
	pdi_st_ptr(pdi, addr);
	pdi.write(0x24, count);
}
*/

template <typename R, typename Pdi, typename Clock, typename Process>
bool pdi_read(Pdi & pdi, R & r, Clock & clock, Process const & process)
{
	r = 0;

	typename Clock::time_type t = clock.value();

	for (uint8_t i = 0; i != sizeof(R); ++i)
	{
		while (!pdi.rx_ready())
		{
			if (clock.value() - t > 1000)
				return false;
			process();
		}

		r <<= 8;
		r |= pdi.read();
	}

	return true;
}

template <typename Pdi, typename Clock, typename Process>
void pdi_read(Pdi & pdi, uint8_t *& buf, Clock & clock, Process const & process)
{
	if (!buf)
		return;

	typename Clock::time_type t = clock.value();

	while (buf && !pdi.rx_ready())
	{
		if (buf && clock.value() - t > 1000)
			buf = 0;
		process();
	}

	*buf++ = pdi.read();
}

template <typename Pdi, typename Clock, typename Process>
bool pdi_wait_nvm_busy(Pdi & pdi, Clock & clock, typename Clock::time_type timeout, Process const & process)
{
	typename Clock::time_type t = clock.value();

	uint8_t status = 0x80;
	bool success = true;
	while (success && (status & 0x80) != 0 && clock.value() - t < timeout)
	{
		pdi_lds(pdi, (uint32_t)0x010001CF, 1);
		success = pdi_read(pdi, status, clock, process);
	}

	return success && (status & 0x80) == 0;
}


template <typename Pdi, typename Com, typename Clock, typename Process>
bool pdi_ptrcopy(Pdi & pdi, Com & com, uint32_t addr, uint8_t len, Clock & clock, Process const & process)
{
	bool success = true;
	while (len > 0)
	{
		uint8_t chunk = len > 14? 14: len;

		if (success)
		{
			pdi_st_ptr(pdi, addr);
			pdi_repeat(pdi, (uint8_t)(chunk - 1));
			pdi.write(0x24, chunk);
		}

		for (uint8_t i = 0; i != chunk; ++i)
		{
			uint8_t v = 0;
			if (success)
				success = pdi_read(pdi, v, clock, process);
			com.write(v);
		}

		addr += chunk;
		len -= chunk;
	}

	return success;
}

#endif
