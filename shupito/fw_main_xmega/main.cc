#include <avr/io.h>
#include <avr/interrupt.h>
#include "../fw_common/avrlib/command_parser.hpp"
#include "../fw_common/avrlib/async_usart.hpp"
#include "../fw_common/avrlib/bootseq.hpp"
#include "../fw_common/avrlib/uart_xmega.hpp"
#include "../fw_common/avrlib/counter.hpp"
#include "../fw_common/avrlib/format.hpp"

#include "../fw_common/handler_avricsp.hpp"
#include "../fw_common/handler_xmega.hpp"

typedef avrlib::async_usart<avrlib::uart_xmega, 64, 64> com_nested_t;
com_nested_t com_inner;
ISR(USARTD1_RXC_vect) { com_inner.process_rx(); }

com_nested_t com_outer;
ISR(USARTE0_RXC_vect) { com_outer.process_rx(); }

typedef com_nested_t com_t;
com_t & com = com_outer;

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
		static void toggle() { port.OUTTGL = (1<<(pin)); } \
	}

AVRLIB_MAKE_XMEGA_PIN(pin_ext_sup, PORTB, 2);
AVRLIB_MAKE_XMEGA_PIN(pin_led,     PORTD, 3);

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
	//pdi.process();
	com_inner.process_tx();
	com_outer.process_tx();
}

void send_dword(uint32_t v)
{
	com.write(v);
	com.write(v >> 8);
	com.write(v >> 16);
	com.write(v >> 24);
}

static uint8_t const device_descriptor[] PROGMEM = {
#include "desc.h"
};

int main()
{
	PORTD.DIRSET = (1<<3);

	// Run at 32MHz
	OSC.CTRL = OSC_RC32MEN_bm | OSC_RC2MEN_bm;
	while ((OSC.STATUS & OSC_RC32MRDY_bm) == 0)
	{
	}

	CCP = CCP_IOREG_gc;
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;

	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();

	PORTD.PIN6CTRL = PORT_OPC_PULLUP_gc;
	PORTD.OUTSET = (1<<7);
	PORTD.DIRSET = (1<<7);

	PORTE.PIN2CTRL = PORT_OPC_PULLUP_gc;
	PORTE.OUTSET = (1<<3);
	PORTE.DIRSET = (1<<3);

	com_inner.usart().open(USARTD1, true);
	com_outer.usart().open(USARTE0, true);

	TCD0.INTCTRLA = TC_OVFINTLVL_HI_gc;
	TCD0.CTRLA = TC_CLKSEL_DIV8_gc;

	pin_buf_txd::init();
	pin_buf_rst::init();
	pin_buf_pdi::init();

	pin_led::make_low();
	pin_ext_sup::make_low();

	bool inner_redirected = false;

	avrlib::bootseq bootseq;
	avrlib::command_parser cp;
	cp.clear();

	// Prepare the ADC
	ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN6_gc;
	ADCA.PRESCALER = ADC_PRESCALER_DIV16_gc;
	ADCA.REFCTRL = ADC_REFSEL_INT1V_gc;
	ADCA.CTRLB = ADC_RESOLUTION_8BIT_gc;
	ADCA.CTRLA = ADC_ENABLE_bm;

	handler_xmega<my_pdi_t, com_t, clock_t> hxmega(pdi, com, clock);
	handler_avricsp<spi_t, com_t, clock_t, pin_buf_rst> havricsp(spi, com, clock);
	handler_base * handler = 0;

	avrlib::timeout<clock_t> vdd_timeout(clock, 4000000);
	vdd_timeout.cancel();

	for (;;)
	{
		if (vdd_timeout)
		{
			vdd_timeout.ack();
			ADCA.CH0.CTRL |= ADC_CH_START_bm;
		}

		if (ADCA.CH0.INTFLAGS & ADC_CH_CHIF_bm)
		{
			ADCA.CH0.INTFLAGS = ADC_CH_CHIF_bm;
			com.write(0x80);
			com.write(0xa2);
			com.write(0x01);
			com.write(ADCA.CH0RESL);
		}

		if (!inner_redirected && !com_inner.empty())
		{
			com_inner.read();
		}

		if (inner_redirected && !com_inner.empty())
		{
			uint8_t size = com_inner.read_size();
			if (size > 13)
				size = 13;

			com_outer.write(0x80);
			com_outer.write(0x90 | (size + 1));
			com_outer.write(0x01);
			for (uint8_t i = 0; i < size; ++i)
				com_outer.write(com_inner.read());
		}

		if (!com.empty())
		{
			uint8_t ch = cp.push_data(com.read());
			bootseq.check(ch);

			switch (ch)
			{
			case 0:
				if (cp.size() == 0)
					break;

				switch (cp[0])
				{
				case 0:
					{
						// Send the device descriptor
						uint8_t const * PROGMEM ptr = device_descriptor;
						uint8_t size = sizeof device_descriptor;

						for (;;)
						{
							uint8_t chunk = 15;
							if (size < chunk)
								chunk = (uint8_t)size;
							size -= chunk;

							com.write(0x80);
							com.write(chunk);
							for (uint8_t i = chunk; i != 0; --i)
								com.write(pgm_read_byte(ptr++));

							if (chunk < 15)
								break;
						}
					}
					break;
				case 1:
					// Enable interface
					{
						uint8_t err = 1;
						if (cp.size() == 3 && cp[1] == 0)
						{
							handler_base * new_handler = 0;
							err = 0;

							switch (cp[2])
							{
							case 0:
								new_handler = 0;
								break;
							case 1:
								new_handler = &havricsp;
								break;
							case 2:
								new_handler = &hxmega;
								break;
							default:
								err = 1;
							}

							if (!err && new_handler != handler)
							{
								handler->unselect();
								err = new_handler->select();
								handler = (err == 0? new_handler: 0);
							}
						}

						if (cp.size() == 2 && cp[1] == 2)
						{
							vdd_timeout.start();
							err = 0;
						}
						com.write(0x80);
						com.write(0x01);
						com.write(err);
					}
					break;
				case 2:
					// Disable interface
					{
						if (cp.size() == 3 && cp[1] == 0)
						{
							if (handler)
								handler->unselect();
							handler = 0;
						}

						if (cp.size() == 2 && cp[1] == 2)
						{
							vdd_timeout.cancel();
						}
						com.write(0x80);
						com.write(0x01);
						com.write(0x00);
					}
					break;
				}
				break;
			case 9:
				if (cp.size() == 0)
					break;

				if (cp[0] == 0 && cp.size() > 1)
				{
					switch (cp[1])
					{
					case 0:
						// Send the set of available pipes
						com.write(0x80);
						com.write(0x93);
						com.write(0x00);
						com.write(0x01);
						com.write('u');
						break;
					case 1:
						// Activate a pipe
						if (cp.size() == 3 && cp[2] == 'u')
						{
							com.write(0x80);
							com.write(0x93);
							com.write(0x01);
							com.write(0x01);
							com.write('u');
							inner_redirected = true;
						}
						else
						{
							com.write(0x80);
							com.write(0x91);
							com.write(0x02);
						}
						break;
					case 2:
						// Deactivate a pipe
						if (cp.size() == 3 && cp[2] == 1)
						{
							com.write(0x80);
							com.write(0x92);
							com.write(0x02);
							com.write(0x01);
							inner_redirected = false;
						}
						else
						{
							com.write(0x80);
							com.write(0x91);
							com.write(0x02);
						}
						break;
					}
				}

				if (cp[0] == 1)
				{
					for (uint8_t i = 1; i < cp.size(); ++i)
						com_inner.write(cp[i]);
				}
				
				break;
			case 0xa:
				if (cp.size() == 2 && cp[0] == 1)
					pin_ext_sup::set_value(cp[1] == 0);
				break;
			case '?':
				avrlib::send(com, "Shupito v2.0\n");
				cp.clear();
				break;
			case 'l':
				pin_led::set_value(true);
				cp.clear();
				break;
			case 'L':
				pin_led::set_value(false);
				cp.clear();
				break;
			case 'e':
				pin_ext_sup::set_value(true);
				cp.clear();
				break;
			case 'E':
				pin_ext_sup::set_value(false);
				cp.clear();
				break;
			case 't':
				avrlib::format(com, "clock: %x\n") % clock.value();
				cp.clear();
				break;
			case 'm':
				vdd_timeout.start();
				cp.clear();
				break;
			case 'M':
				vdd_timeout.cancel();
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
