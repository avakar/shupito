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

com_nested_t com_app;
ISR(USARTC1_RXC_vect) { com_app.process_rx(); }

typedef com_nested_t com_t;

struct timer_xd0
{
	typedef uint16_t value_type;
	static const uint8_t value_bits = 14;
	static value_type value() { return TCD0.CNT >> 2; }
	static void value(value_type v) { TCD0.CNT = v << 2; }
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
		static void make_output() { port.DIRSET = (1<<(pin)); } \
		static void set_value(uint8_t value) { if (value) port.OUTSET = (1<<(pin)); else port.OUTCLR = (1<<(pin)); } \
		static void toggle() { port.OUTTGL = (1<<(pin)); } \
	}

AVRLIB_MAKE_XMEGA_PIN(pin_ext_sup, PORTB, 2);
AVRLIB_MAKE_XMEGA_PIN(pin_led,     PORTD, 3);

AVRLIB_MAKE_XMEGA_PIN(pin_pdid, PORTD, 1);
AVRLIB_MAKE_XMEGA_PIN(pin_rstd, PORTD, 0);
AVRLIB_MAKE_XMEGA_PIN(pin_rst,  PORTC, 1);

AVRLIB_MAKE_XMEGA_PIN(pin_pdi,  PORTC, 3);
AVRLIB_MAKE_XMEGA_PIN(pin_xck,  PORTC, 5);

AVRLIB_MAKE_XMEGA_PIN(pin_rxd,  PORTC, 6);
AVRLIB_MAKE_XMEGA_PIN(pin_txd,  PORTC, 7);
AVRLIB_MAKE_XMEGA_PIN(pin_txdd, PORTD, 2);

AVRLIB_MAKE_XMEGA_PIN(pin_usb_rx, PORTD, 6);
AVRLIB_MAKE_XMEGA_PIN(pin_usb_tx, PORTD, 7);

template <typename ValuePin, typename OePin>
struct pin_buffer_with_oe
{
	static void init()
	{
		ValuePin::make_input();
		OePin::make_low();
	}

	static void clear()
	{
		ValuePin::make_input();
		OePin::make_input();
	}

	static void make_input()
	{
		ValuePin::make_input();
		OePin::set_value(0);
	}

	static void make_high()
	{
		OePin::set_value(1);
		ValuePin::make_high();
	}

	static void make_low()
	{
		OePin::set_value(1);
		ValuePin::make_low();
	}

	static void make_output()
	{
		OePin::set_value(1);
		ValuePin::make_output();
	}

	static void set_value(uint8_t value)
	{
		ValuePin::set_value(value);
	}
};

typedef pin_buffer_with_oe<pin_txd, pin_txdd> pin_buf_txd;
typedef pin_buffer_with_oe<pin_rst, pin_rstd> pin_buf_rst;
typedef pin_buffer_with_oe<pin_pdi, pin_pdid> pin_buf_pdi;
typedef pin_buffer_with_oe<pin_xck, pin_pdid> pin_buf_xck;
typedef pin_rxd pin_buf_rxd;

void avrlib::assertion_failed(char const * message, char const * file, int line)
{	
	cli();
	pin_led::make_high();

	for (;;)
	{
	}
}

class spi_t
{
public:
	typedef uint8_t error_t;

	void clear()
	{
		USARTC1.CTRLB = 0;
		USARTC1.CTRLC = 0;

		pin_buf_txd::make_input();
		pin_buf_xck::make_input();
	}

	error_t start_master(uint16_t speed_khz)
	{
		pin_buf_txd::make_low();
		pin_buf_xck::make_low();

		USARTC1.BAUDCTRLA = 127;
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

		this->cancel_read();
		while (m_state != st_idle)
			this->process();

		// A few more clocks should be supplied
		// to make sure that the last byte was received correctly
		m_state = st_unrst;
		m_time_base = m_clock.value();
	}

	void init()
	{
		if (m_state != st_disabled)
			return;

		PORTC.PIN1CTRL = PORT_INVEN_bm;

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
			if (m_clock.value() - m_time_base >= 128) // FIXME: time constants
			{
				m_state = st_idle;
				m_rx_count = 0;
				pdi_stcs(*this, 0x02/*CTRL*/, 0x03/*GUARDTIME_16*/);
			}
			break;
		case st_unrst:
			if (m_clock.value() - m_time_base >= 16) // FIXME: time constants
			{
				PdiClk::make_input();
				PdiData::make_input();
				PORTC.PIN1CTRL = 0;

				USARTC0.CTRLA = 0;
				USARTC0.CTRLB = 0;
				USARTC0.CTRLC = 0;

				m_state = st_disabled;
			}
			break;
		}

		pin_led::set_value(m_state == st_busy);
	}

	void intr_rxc()
	{
		m_rx_buffer.push(USARTC0.DATA);
		if (--m_rx_count == 0)
		{
			USARTC0.CTRLA = 0;
			USARTC0.CTRLB = USART_TXEN_bm;
			PdiData::make_output();
			m_state = st_idle;
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
			USARTC0.CTRLA = USART_TXCINTLVL_MED_gc;
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
		}
	}

private:
	Clock & m_clock;
	typename Clock::time_type m_time_base;
	avrlib::buffer<uint8_t, 16> m_rx_buffer;
	avrlib::buffer<uint8_t, 16> m_tx_buffer;
	volatile uint8_t m_rx_count;

	enum { st_disabled, st_rst_disable, st_wait_ticks, st_idle, st_busy, st_unrst } volatile m_state;
};
typedef pdi_t<clock_t, pin_buf_rst, pin_buf_pdi> my_pdi_t;
my_pdi_t pdi(clock);

ISR(USARTC0_DRE_vect) { pdi.intr_udre(); }
ISR(USARTC0_TXC_vect) { pdi.intr_txc(); }
ISR(USARTC0_RXC_vect) { pdi.intr_rxc(); }

void process()
{
	pdi.process();
	com_inner.process_tx();
	com_outer.process_tx();
	com_app.process_tx();
}

static uint8_t const device_descriptor[] PROGMEM = {
#include "desc.h"
};

class context_t
{
public:
	context_t()
		: hxmega(pdi, clock, &process), havricsp(spi, clock),
		vdd_timeout(clock, 1000000), usb_test_timeout(clock, 1000000),
		m_primary_com(0), m_vdd_com(0), m_app_com(0), m_app_comm_allowed(true)
	{
		usb_test_timeout.cancel();
	}

	void init()
	{
		PORTD.PIN6CTRL = PORT_OPC_PULLUP_gc;
		PORTD.OUTSET = (1<<7);
		PORTD.OUTCLR = (1<<5);
		PORTD.DIRSET = (1<<5)|(1<<7);
		com_inner.usart().open(USARTD1, true, true /*synchronous*/);

		PORTE.PIN2CTRL = PORT_OPC_PULLUP_gc;
		PORTE.OUTSET = (1<<3);
		PORTE.DIRSET = (1<<3);
		com_outer.usart().open(USARTE0, true);

		TCD0.INTCTRLA = TC_OVFINTLVL_HI_gc;
		TCD0.CTRLA = TC_CLKSEL_DIV8_gc;

		pin_buf_txd::init();
		pin_buf_rst::init();
		pin_buf_pdi::init();

		pin_led::make_low();
		pin_ext_sup::make_low();

		inner_redirected = false;

		cp_outer.clear();
		cp_inner.clear();

		// Prepare the ADC
		ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
		ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN6_gc;
		ADCA.PRESCALER = ADC_PRESCALER_DIV64_gc;
		ADCA.REFCTRL = ADC_REFSEL_INT1V_gc;
		ADCA.CTRLB = ADC_RESOLUTION_8BIT_gc;
		ADCA.CTRLA = ADC_ENABLE_bm;

		handler = 0;
		vdd_timeout.cancel();
	}

	void run()
	{
		if (vdd_timeout)
		{
			vdd_timeout.ack();
			ADCA.CH0.CTRL |= ADC_CH_START_bm;
		}

		if (ADCA.CH0.INTFLAGS & ADC_CH_CHIF_bm)
		{
			ADCA.CH0.INTFLAGS = ADC_CH_CHIF_bm;
			if (m_vdd_com)
			{
				m_vdd_com->write(0x80);
				m_vdd_com->write(0xa2);
				m_vdd_com->write(0x01);
				m_vdd_com->write(ADCA.CH0RESL);
			}
		}

		if (inner_redirected && !com_inner.empty())
		{
			uint8_t size = com_inner.read_size();
			if (size > 14)
				size = 14;

			com_outer.write(0x80);
			com_outer.write(0x90 | (size + 1));
			com_outer.write(0x02);
			for (uint8_t i = 0; i < size; ++i)
				com_outer.write(com_inner.read());
		}

		if (!com_app.empty() && m_app_com)
		{
			uint8_t size = com_app.read_size();
			if (size > 14)
				size = 14;

			m_app_com->write(0x80);
			m_app_com->write(0x90 | (size + 1));
			m_app_com->write(0x01);
			for (uint8_t i = 0; i < size; ++i)
				m_app_com->write(com_app.read());
		}

		if (!com_outer.empty())
		{
			uint8_t ch = cp_outer.push_data(com_outer.read());
			if (ch != 255)
			{
				bootseq.check(ch);
				if (!this->process_command(cp_outer, com_outer)
					&& !process_tunnel(cp_outer, com_outer, false))
				{
					cp_outer.clear();
				}
			}
		}

		if (!inner_redirected && !com_inner.empty())
		{
			uint8_t ch = cp_inner.push_data(com_inner.read());
			if (ch != 255)
			{
				if (!this->process_command(cp_inner, com_inner)
					&& !process_tunnel(cp_inner, com_inner, true))
				{
					cp_inner.clear();
				}
			}
		}

		if (usb_test_timeout)
		{
			usb_test_timeout.ack();
			if (!inner_redirected)
				avrlib::send(com_inner, "A really long string that we need to get through USB unmangled.\n");
		}

		process();
	}

private:
	bool process_tunnel(avrlib::command_parser & cp, com_t & com, bool inner)
	{
		switch (cp.command())
		{
		case 9:
			if (cp.size() > 1 && cp[0] == 0)
			{
				switch (cp[1])
				{
				case 0:
					// Send the set of available pipes
					com.write(0x80);
					com.write(0x95);
					com.write(0x00);
					com.write(0x03);
					com.write('a');
					com.write('p');
					com.write('p');
					break;
				case 1:
					// Activate a pipe
					if (!inner && cp.size() == 5 && cp[2] == 'u' && cp[3] == 's' && cp[4] == 'b')
					{
						com.write(0x80);
						com.write(0x92);
						com.write(0x01);
						com.write(0x02);
						inner_redirected = true;
					}
					else if (cp.size() == 5 && cp[2] == 'a' && cp[3] == 'p' && cp[4] == 'p' && m_app_comm_allowed)
					{
						com.write(0x80);
						com.write(0x92);
						com.write(0x01);
						com.write(0x01);
						m_app_com = &com;
						pin_buf_txd::make_high();
						com_app.usart().open(USARTC1, true);
					}
					else
					{
						com.write(0x80);
						com.write(0x92);
						com.write(0x01);
						com.write(0x00);
					}
					break;
				case 2:
					// Deactivate a pipe
					if (!inner && cp.size() == 3 && cp[2] == 2)
					{
						com.write(0x80);
						com.write(0x92);
						com.write(0x02);
						com.write(0x02);
						inner_redirected = false;
					}
					else if (cp.size() == 3 && cp[2] == 1)
					{
						com.write(0x80);
						com.write(0x92);
						com.write(0x02);
						com.write(0x01);
						com_app.usart().close();
						pin_buf_txd::make_input();
						m_app_com = 0;
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

			if (!inner && inner_redirected && cp[0] == 2)
			{
				for (uint8_t i = 1; i < cp.size(); ++i)
					com_inner.write(cp[i]);
			}

			if (m_app_com && cp[0] == 1)
			{
				for (uint8_t i = 1; i < cp.size(); ++i)
					com_app.write(cp[i]);
			}

			return true;
		}

		return false;
	}

	bool process_command(avrlib::command_parser & cp, com_t & com)
	{
		switch (cp.command())
		{
		case 0:
			if (cp.size() == 0)
				return true;

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
						switch (cp[2])
						{
						case 0:
							err = this->select_handler(handler_spi);
							break;
						case 1:
							err = this->select_handler(handler_pdi);
							break;
						}

						if (err == 0)
							m_primary_com = &com;
					}

					if (cp.size() == 2 && cp[1] == 2)
					{
						vdd_timeout.start();
						err = 0;
						m_vdd_com = &com;
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
						this->select_handler(handler_none);
						m_primary_com = 0;
					}

					if (cp.size() == 2 && cp[1] == 2)
					{
						vdd_timeout.cancel();
						m_vdd_com = 0;
					}

					com.write(0x80);
					com.write(0x01);
					com.write(0x00);
				}
				break;
			}

			return true;
		case 0xa:
			if (cp.size() == 2 && cp[0] == 1)
				pin_ext_sup::set_value(cp[1] == 1);
			return true;
		case '?':
			avrlib::send(com, "Shupito v2.0\n");
			cp.clear();
			return true;
		case 'l':
			pin_led::set_value(true);
			cp.clear();
			return true;
		case 'L':
			pin_led::set_value(false);
			cp.clear();
			return true;
		case 'e':
			pin_ext_sup::set_value(true);
			cp.clear();
			return true;
		case 'E':
			pin_ext_sup::set_value(false);
			cp.clear();
			return true;
		case 't':
			avrlib::format(com, "clock: %x\n") % clock.value();
			cp.clear();
			return true;
		case 'u':
			usb_test_timeout.restart();
			cp.clear();
			return true;
		case 'U':
			usb_test_timeout.cancel();
			cp.clear();
			return true;
		case 'm':
			vdd_timeout.start();
			cp.clear();
			return true;
		case 'M':
			vdd_timeout.cancel();
			cp.clear();
			return true;
		case 254:
			cp.clear();
			return true;
		case 255:
			break;
		default:
			if (&com == m_primary_com && handler)
				return handler->handle_command(cp, com);
		}

		return false;
	}

	enum handler_t { handler_none, handler_spi, handler_pdi };
	uint8_t select_handler(handler_t h)
	{
		handler_base<com_t> * new_handler;
		switch (h)
		{
		case handler_spi:
			new_handler = &havricsp;
			break;
		case handler_pdi:
			new_handler = &hxmega;
			break;
		default:
			new_handler = 0;
		}

		uint8_t err = 0;

		if (new_handler != handler)
		{
			if (handler)
				handler->unselect();
			if (new_handler)
				err = new_handler->select();
			handler = (err == 0? new_handler: 0);
		}

		return err;
	}

private:
	bool inner_redirected;

	avrlib::bootseq bootseq;
	avrlib::command_parser cp_outer, cp_inner;

	handler_xmega<my_pdi_t, com_t, clock_t, void (*)()> hxmega;
	handler_avricsp<spi_t, com_t, clock_t, pin_buf_rst> havricsp;
	handler_base<com_t> * handler;

	avrlib::timeout<clock_t> vdd_timeout;
	avrlib::timeout<clock_t> usb_test_timeout;

	com_t * m_primary_com;
	com_t * m_vdd_com;
	com_t * m_app_com;

	bool m_app_comm_allowed;
};

context_t ctx;

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

	ctx.init();
	for (;;)
	{
		ctx.run();
	}
}
