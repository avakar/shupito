#include <avr/io.h>
#include <avr/interrupt.h>
#include "../fw_common/avrlib/command_parser.hpp"
#include "../fw_common/avrlib/usart_xd1.hpp"
#include "../fw_common/avrlib/async_usart.hpp"
#include "../fw_common/avrlib/counter.hpp"
#include "../fw_common/avrlib/format.hpp"

#include "../fw_common/handler_avricsp.hpp"
#include "../fw_common/handler_xmega.hpp"

typedef avrlib::async_usart<avrlib::usart_xd1, 64, 64> com_t;
com_t com(38400, true);
ISR(USARTD1_RXC_vect) { com.process_rx(); }

struct timer_xd0
{
	typedef uint16_t value_type;
	static const uint8_t value_bits = 16;
	static value_type value() { return TCD0.CNT; }
	static void value(value_type v) { TCD0.CNT = v; }
	static bool overflow() { return (TCD0.INTFLAGS & TC0_OVFIF_bm) != 0; }
	static void clear_overflow() { TCD0.INTFLAGS = TC0_OVFIF_bm; }
	static void tov_interrupt(bool) {}
	static void clock_source(avrlib::timer_clock_source) {}
};
typedef avrlib::counter<timer_xd0> clock_t;
clock_t clock;
ISR(TCD0_OVF_vect)
{
	clock.tov_interrupt();
}

#define AVRLIB_MAKE_XMEGA_PIN(pin_name, port, pin) \
	struct pin_name \
	{ \
		static void make_input() { port.DIRCLR = (1<<(pin)); } \
		static void make_high() { port.OUTSET = (1<<(pin)); port.DIRSET = (1<<(pin)); } \
		static void make_low() { port.OUTCLR = (1<<(pin)); port.DIRSET = (1<<(pin)); } \
		static void set_value(uint8_t value) { if (value) port.OUTSET = (1<<(pin)); else port.OUTCLR = (1<<(pin)); } \
	}

AVRLIB_MAKE_XMEGA_PIN(pin_ext_sup, PORTB, 2);

AVRLIB_MAKE_XMEGA_PIN(pin_pdid, PORTB, 3);
AVRLIB_MAKE_XMEGA_PIN(pin_rstd, PORTC, 0);
AVRLIB_MAKE_XMEGA_PIN(pin_rst,  PORTC, 1);
AVRLIB_MAKE_XMEGA_PIN(pin_pdi,  PORTC, 5);
AVRLIB_MAKE_XMEGA_PIN(pin_rxd,  PORTC, 6);
AVRLIB_MAKE_XMEGA_PIN(pin_txd,  PORTC, 7);
AVRLIB_MAKE_XMEGA_PIN(pin_txdd, PORTD, 0);

AVRLIB_MAKE_XMEGA_PIN(pin_usb_rx, PORTD, 6);
AVRLIB_MAKE_XMEGA_PIN(pin_usb_tx, PORTD, 7);

template <typename ValuePin, typename OePin>
struct pin_buffer_with_oe
{
	static void init()
	{
		ValuePin::make_input();
		OePin::make_high();
	}

	static void clear()
	{
		ValuePin::make_input();
		OePin::make_input();
	}

	static void make_input()
	{
		ValuePin::make_input();
		OePin::set_value(1);
	}

	static void make_high()
	{
		OePin::set_value(0);
		ValuePin::make_high();
	}

	static void make_low()
	{
		OePin::set_value(0);
		ValuePin::make_low();
	}

	static void set_value(uint8_t value)
	{
		ValuePin::set_value(value);
	}
};

typedef pin_buffer_with_oe<pin_txd, pin_txdd> pin_buf_txd;
typedef pin_buffer_with_oe<pin_rst, pin_rstd> pin_buf_rst;
typedef pin_buffer_with_oe<pin_pdi, pin_pdid> pin_buf_pdi;
typedef pin_rxd pin_buf_rxd;

class spi_t
{
public:
	typedef uint8_t error_t;

	void clear()
	{
		USARTC1.CTRLB = 0;
		USARTC1.CTRLC = 0;

		pin_buf_txd::make_input();
		pin_buf_pdi::make_input();
	}

	error_t start_master(uint16_t speed_khz)
	{
		pin_buf_txd::make_low();
		pin_buf_pdi::make_low();

		USARTC1.BAUDCTRLA = 63;
		USARTC1.BAUDCTRLB = 0;
		USARTC1.CTRLC = USART_CMODE_MSPI_gc;
		USARTC1.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
		return 0;
	}

	uint8_t send(uint8_t v)
	{
		USARTC1.DATA = v;
		while ((USARTC1.STATUS & USART_RXCIF_bm) == 0)
		{
		}
		return USARTC1.DATA;
	}
};
spi_t spi;

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

		while ((USARTC0.STATUS & USART_TXCIF_bm) == 0)
			this->process();

		PdiClk::make_input();
		PdiData::make_input();

		USARTC0.CTRLA = 0;
		USARTC0.CTRLB = 0;
		USARTC0.CTRLC = 0;

		m_state = st_disabled;
	}

	void init()
	{
		if (m_state != st_disabled)
			return;

		PdiClk::make_low();
		PdiData::make_high();

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

		USARTC0.CTRLA = 0;
		m_tx_buffer.push(data);
		m_rx_count = rx_count;
		USARTC0.CTRLA = USART_DREINTLVL_MED_gc;
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
				USARTC0.BAUDCTRLA = 63;
				USARTC0.BAUDCTRLB = 0;
				USARTC0.CTRLC = USART_CMODE_SYNCHRONOUS_gc | USART_PMODE_EVEN_gc | USART_SBMODE_bm
					| USART_CHSIZE_8BIT_gc;
				USARTC0.CTRLA = 0;
				USARTC0.CTRLB = USART_TXEN_bm;

				m_state = st_wait_ticks;
				m_time_base += 8;
			}
			break;
		case st_wait_ticks:
			if (m_clock.value() - m_time_base >= 64) // FIXME: time constants
			{
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
		m_rx_buffer.push(USARTC0.DATA);
		if (--m_rx_count == 0)
		{
			USARTC0.CTRLA = 0;
			USARTC0.CTRLB = USART_TXEN_bm;
		}
	}

	void intr_udre()
	{
		//AVRLIB_ASSERT(!m_tx_buffer.empty());

		cli();
		USARTC0.DATA = m_tx_buffer.top();
		USARTC0.STATUS = USART_TXCIF_bm;
		sei();

		m_tx_buffer.pop();
		if (m_tx_buffer.empty())
		{
			if (m_rx_count)
				USARTC0.CTRLA = USART_TXCINTLVL_MED_gc;
			else
				USARTC0.CTRLA = 0;
		}
	}
	
	void intr_txc()
	{
		USARTC0.CTRLB = USART_RXEN_bm;
		USARTC0.CTRLA = USART_RXCINTLVL_MED_gc;
	}

private:
	Clock & m_clock;
	typename Clock::time_type m_time_base;
	avrlib::buffer<uint8_t, 16> m_rx_buffer;
	avrlib::buffer<uint8_t, 16> m_tx_buffer;
	volatile uint8_t m_rx_count;

	enum { st_disabled, st_rst_disable, st_wait_ticks, st_tx } m_state;
};
typedef pdi_t<clock_t, pin_buf_rst, pin_buf_pdi> my_pdi_t;
my_pdi_t pdi(clock);

ISR(USARTC0_DRE_vect) { pdi.intr_udre(); }
ISR(USARTC0_TXC_vect) { pdi.intr_txc(); }
ISR(USARTC0_RXC_vect) { pdi.intr_rxc(); }

void process()
{
	pdi.process();
	com.process_tx();
}

void send_dword(uint32_t v)
{
	com.write(v);
	com.write(v >> 8);
	com.write(v >> 16);
	com.write(v >> 24);
}

int main()
{
	// Run at 32MHz
	OSC.CTRL = OSC_RC32MEN_bm | OSC_RC2MEN_bm;
	while ((OSC.STATUS & OSC_RC32MRDY_bm) == 0)
	{
	}

	CCP = CCP_IOREG_gc;
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;

	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();

	TCD0.INTCTRLA = TC_OVFINTLVL_HI_gc;
	TCD0.CTRLA = TC_CLKSEL_DIV8_gc;

	pin_buf_txd::init();
	pin_buf_rst::init();
	pin_buf_pdi::init();

	avrlib::command_parser cp;
	cp.clear();

	handler_xmega<my_pdi_t, com_t, clock_t> hxmega(pdi, com, clock);
	handler_avricsp<spi_t, com_t, clock_t, pin_buf_rst> havricsp(spi, com, clock);
	handler_base * handler = 0;

	for (;;)
	{
		if (!com.empty())
		{
			uint8_t ch = cp.push_data(com.read());

			switch (ch)
			{
			case 0:
				if (cp.size() == 0 || cp[0] == 0)
				{
					// Send out the identification
					com.write(0x80);
					com.write(0x05);
					com.write(0x40);
					com.write(0xbd);
					com.write(0xe9);
					com.write(0x9f);
					com.write(0xea);
				}
				else
				{
					switch (cp[0])
					{
					case 1: // Get slot count
						com.write(0x80);
						com.write(0x02);
						com.write(0x41);
						com.write(0x01);
						break;
					case 2: // List functions available for on the current slot.
						com.write(0x80);
						com.write(0x09);
						com.write(0x42);
						send_dword(0x871e0846/*avr*/);
						send_dword(0xc2a4dd67/*avrx*/);
						break;
					case 3: // Select a function into the current slot
						if (cp.size() > 1)
						{
							handler_base * new_handler = 0;
							uint8_t err = 0;
							switch (cp[1])
							{
							case 1:
								new_handler = &havricsp;
								break;
							case 2:
								new_handler = &hxmega;
								break;
							case 0:
								break;
							default:
								err = 1;
							}

							if (!err && handler != new_handler)
							{
								if (handler)
									handler->unselect();
								if (new_handler)
									err = new_handler->select();
								handler = (err == 0? new_handler: 0);
							}
							com.write(0x80);
							com.write(0x02);
							com.write(0x43);
							com.write(err);
						}
						break;
					case 4: // Get selected function.
						{
							uint32_t res = 0;
							if (handler == &havricsp)
								res = 0x871e0846;
							if (handler == &hxmega)
								res = 0xc2a4dd67;

							com.write(0x80);
							com.write(0x05);
							com.write(0x44);
							send_dword(res);
						}
						break;
					}
				}
				break;
			case '?':
				avrlib::send(com, "Shupito v2.0\n");
				cp.clear();
				break;
			case 255:
				break;
			default:
				if (ch > 16)
					cp.clear();
				else if (handler)
					handler->handle_command(cp);
			}
		}

		process();
	}
}
