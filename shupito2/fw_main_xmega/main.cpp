#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "../../fw_common/avrlib/command_parser.hpp"
#include "../../fw_common/avrlib/async_usart.hpp"
#include "../../fw_common/avrlib/hwflow_usart.hpp"
#include "../../fw_common/avrlib/bootseq.hpp"
#include "../../fw_common/avrlib/counter.hpp"
#include "../../fw_common/avrlib/format.hpp"
#include "../../fw_common/avrlib/xmega_pin.hpp"

#include "../../fw_common/avrlib/usart_xc1.hpp"
#include "../../fw_common/avrlib/usart_xd1.hpp"
#include "../../fw_common/avrlib/usart_xe0.hpp"


#include "../../fw_common/handler_avricsp.hpp"
#include "../../fw_common/handler_cc25xx.hpp"
#include "../../fw_common/handler_xmega.hpp"
#include "../../fw_common/handler_jtag.hpp"
#include "../../fw_common/handler_spi.hpp"

#include "../../fw_common/pdi.hpp"

struct timer_xd0
{
	template <uint32_t v>
	struct us { static const uint32_t value = (v + 7) >> 3; };

	typedef uint16_t time_type;
	static const uint8_t value_bits = 16;

	static void init()
	{
		//TCD0.INTCTRLA = TC_OVFINTLVL_LO_gc;
		TCD0.CTRLA = TC_CLKSEL_DIV256_gc;
	}

	static time_type value() { return TCD0.CNT; }
	static void value(time_type v) { TCD0.CNT = v; }
	static bool overflow() { return (TCD0.INTFLAGS & TC0_OVFIF_bm) != 0; }
	static void clear_overflow() { TCD0.INTFLAGS = TC0_OVFIF_bm; }
	static void tov_interrupt(bool) {}
	static void clock_source(avrlib::timer_clock_source) {}
};
typedef timer_xd0 clock_t;
clock_t clock;

AVRLIB_DEFINE_XMEGA_PIN(pin_sup_3v3, PORTB, 2);
AVRLIB_DEFINE_XMEGA_PIN(pin_sup_5v0, PORTB, 0);

AVRLIB_DEFINE_XMEGA_PIN(pin_switched_pwr_en, PORTC, 0);

AVRLIB_DEFINE_XMEGA_PIN(pin_led,     PORTA, 2);

AVRLIB_DEFINE_XMEGA_PIN(pin_pdid, PORTD, 1);
AVRLIB_DEFINE_XMEGA_PIN(pin_rstd, PORTD, 0);
AVRLIB_DEFINE_XMEGA_PIN(pin_rst,  PORTC, 1);

AVRLIB_DEFINE_XMEGA_PIN(pin_pdi,  PORTC, 3);
AVRLIB_DEFINE_XMEGA_PIN(pin_xck,  PORTC, 5);

AVRLIB_DEFINE_XMEGA_PIN(pin_rxd,  PORTC, 6);
AVRLIB_DEFINE_XMEGA_PIN(pin_txd,  PORTC, 7);
AVRLIB_DEFINE_XMEGA_PIN(pin_txdd, PORTD, 2);

AVRLIB_DEFINE_XMEGA_PIN(pin_usb_rtr_n, PORTD, 3);
AVRLIB_DEFINE_XMEGA_PIN(pin_usb_cts_n, PORTD, 4);
AVRLIB_DEFINE_XMEGA_PIN(pin_usb_xck,   PORTD, 5);
AVRLIB_DEFINE_XMEGA_PIN(pin_usb_rx,    PORTD, 6);
AVRLIB_DEFINE_XMEGA_PIN(pin_usb_tx,    PORTD, 7);

AVRLIB_DEFINE_XMEGA_PIN(pin_sysclk, PORTE, 1);
AVRLIB_DEFINE_XMEGA_PIN(pin_bt_rx, PORTE, 2);
AVRLIB_DEFINE_XMEGA_PIN(pin_bt_tx, PORTE, 3);

typedef pin_buffer_with_oe<pin_txd, pin_txdd> pin_buf_txd;
typedef pin_buffer_with_oe<pin_rst, pin_rstd> pin_buf_rst;
typedef pin_buffer_with_oe<pin_pdi, pin_pdid> pin_buf_pdi;
typedef pin_buffer_with_oe<pin_xck, pin_pdid> pin_buf_xck;
typedef pin_rxd pin_buf_rxd;

typedef avrlib::hwflow_usart<avrlib::usart_xd1, 64, 64, avrlib::intr_med, pin_usb_rtr_n, pin_usb_cts_n> com_inner_t;
com_inner_t com_inner;
ISR(USARTD1_RXC_vect) { com_inner.intr_rx(); }

typedef avrlib::async_usart<avrlib::usart_xe0, 64, 64> com_outer_t;
com_outer_t com_outer;
ISR(USARTE0_RXC_vect) { com_outer.intr_rx(); }

typedef avrlib::async_usart<avrlib::usart_xc1, 64, 64> com_app_t;
com_app_t com_app;
ISR(USARTC1_RXC_vect) { com_app.intr_rx(); }

struct com_writer_t : yb_writer
{
public:
	com_writer_t()
		: yb_writer(15)
	{
	}

	virtual void write(uint8_t value) {}
	virtual bool tx_reserve(uint8_t value) { return false; }

	virtual uint8_t * alloc(uint8_t cmd, uint8_t size);
	virtual uint8_t * alloc_sync(uint8_t cmd, uint8_t size);
	virtual void commit();
	virtual bool send(uint8_t cmd, uint8_t const * data, uint8_t size);
	virtual void send_sync(uint8_t cmd, uint8_t const * data, uint8_t size);

private:
	uint8_t m_buffer[16];
};

struct com_inner_writer_t : com_writer_t
{
	virtual void write(uint8_t value)
	{
		com_inner.write(value);
	}

	virtual bool tx_reserve(uint8_t size)
	{
		return com_inner.tx_reserve(size);
	}
} com_inner_writer;

struct com_outer_writer_t : com_writer_t
{
	virtual void write(uint8_t value)
	{
		com_outer.write(value);
	}

	virtual bool tx_reserve(uint8_t size)
	{
		return com_outer.tx_reserve(size);
	}
} com_outer_writer;

void avrlib::assertion_failed(char const * message, char const * file, int line)
{	
	cli();
	pin_led::make_high();

	clock_t::time_type base = clock.value();
	for (;;)
	{
		if (clock.value() - base > clock_t::us<100000>::value)
		{
			pin_led::toggle();
			base = clock.value();
		}
	}
}

class spi_t
{
public:
	typedef uint8_t error_t;

	void clear();
	error_t start_master(uint16_t speed_khz, uint8_t mode, bool lsb_first);
	uint8_t send(uint8_t v);
	void enable_tx();
	void disable_tx();
	bool read_raw();
};

spi_t spi;

struct led_holder
{
	led_holder(bool on = false)
	{
		if (on)
			this->on();
	}

	~led_holder()
	{
		this->off();
	}

	static void on()
	{
		pin_led::set_value(true);
	}

	static void off()
	{
		pin_led::set_value(false);
	}
};

typedef pdi_t<clock_t, pin_buf_rst, pin_buf_pdi, led_holder> my_pdi_t;
my_pdi_t pdi(clock);

ISR(USARTC0_DRE_vect) { pdi.intr_udre(); }
ISR(USARTC0_TXC_vect) { pdi.intr_txc(); }
ISR(USARTC0_RXC_vect) { pdi.intr_rxc(); }

struct process_t
{
	typedef led_holder led;

	void allow_tunnel();
	void disallow_tunnel();
	void operator()() const;
} g_process;

static uint8_t const device_descriptor[] PROGMEM = {
#include "desc.h"
};

class context_t
{
public:
	context_t()
		: hxmega(pdi, clock), havricsp(spi, clock), hcc25xx(spi, clock), hspi(spi),
		vdd_timeout(clock, clock_t::us<100000>::value), m_vccio_drive_check_timeout(clock, clock_t::us<200000>::value),
		m_primary_com(0), m_vdd_com(0), m_app_com(0), m_app_com_state(enabled), m_vccio_drive_state(enabled), m_vccio_state_send_scheduled(false),
		m_send_vccio_drive_list_scheduled(false),
		m_vccio_voltage(0), m_vusb_voltage(0), m_com_app_speed((-1 << 12)|102 /*38400*/)
	{
		m_vccio_drive_check_timeout.cancel();
	}

	void init()
	{
		pin_usb_xck::make_low();
		pin_usb_tx::make_high();
		pin_usb_rx::pullup();
		pin_usb_cts_n::pullup();
		com_inner.usart().open(15, true, true /*synchronous*/);
		pin_usb_rtr_n::make_low();

		com_inner.write(0x80);
		com_inner.write(0xec);
		com_inner.write(0x0b);
		
		NVM_PROD_SIGNATURES_t * prod = 0;
		NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
		com_inner.write(pgm_read_byte(&prod->LOTNUM0));
		com_inner.write(pgm_read_byte(&prod->LOTNUM1));
		com_inner.write(pgm_read_byte(&prod->LOTNUM2));
		com_inner.write(pgm_read_byte(&prod->LOTNUM3));
		com_inner.write(pgm_read_byte(&prod->LOTNUM4));
		com_inner.write(pgm_read_byte(&prod->LOTNUM5));
		com_inner.write(pgm_read_byte(&prod->WAFNUM));
		com_inner.write(pgm_read_byte(&prod->COORDX0));
		com_inner.write(pgm_read_byte(&prod->COORDX1));
		com_inner.write(pgm_read_byte(&prod->COORDY0));
		com_inner.write(pgm_read_byte(&prod->COORDY1));
		
		ADCA.CALL = pgm_read_byte(&prod->ADCACAL0);
		ADCA.CALH = pgm_read_byte(&prod->ADCACAL1);
		NVM_CMD = NVM_CMD_NO_OPERATION_gc;

		pin_bt_rx::pullup();
		pin_bt_tx::make_high();
		com_outer.usart().open((-1 << 12)|102 /*38400*/, true);

		clock_t::init();

		pin_buf_txd::init();
		pin_buf_rst::init();
		pin_buf_pdi::init();

		pin_led::make_low();
		pin_sup_3v3::make_low();
		pin_sup_5v0::make_low();

		pin_switched_pwr_en::make_high();

		inner_redirected = false;

		cp_outer.clear();
		cp_inner.clear();

		// Prepare the ADC
		ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
		ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN6_gc;
		ADCA.CH1.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
		ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN5_gc;
		ADCA.PRESCALER = ADC_PRESCALER_DIV64_gc;
		ADCA.REFCTRL = ADC_REFSEL_INT1V_gc;
		ADCA.CTRLB = ADC_CONMODE_bm | ADC_RESOLUTION_12BIT_gc;
		ADCA.CTRLA = ADC_ENABLE_bm;

		// Start the conversion immediately
		ADCA.CH0.CTRL |= ADC_CH_START_bm;
		ADCA.CH1.CTRL |= ADC_CH_START_bm;

		handler = 0;
		vdd_timeout.cancel();
	}

	void run()
	{
		if (vdd_timeout)
		{
			if (m_vdd_com && m_vdd_com->tx_reserve(6))
			{
				vdd_timeout.restart();
				m_vdd_com->write(0x80);
				m_vdd_com->write(0xa4);
				m_vdd_com->write(0x01);
				m_vdd_com->write(0x03);
				m_vdd_com->write(m_vccio_voltage);
				m_vdd_com->write(m_vccio_voltage >> 8);
			}
			else
			{
				vdd_timeout.force();
			}
		}

		if (m_vccio_state_send_scheduled)
		{
			if (this->send_vccio_state(m_vdd_com))
				m_vccio_state_send_scheduled = false;
		}

		if (m_vccio_drive_state == disabled && m_vccio_voltage < 111 && !pin_switched_pwr_en::get_value()) // 300mV
		{
			m_vccio_drive_state = enabled;
			m_send_vccio_drive_list_scheduled = true;
		}
		if ((m_vccio_drive_state == enabled && m_vccio_voltage > 185) // 500mV
			|| (m_vccio_drive_state != disabled && pin_switched_pwr_en::get_value()))
		{
			pin_sup_5v0::set_low();
			pin_sup_3v3::set_low();
			m_vccio_drive_state = disabled;
			m_send_vccio_drive_list_scheduled = true;
		}

		if (m_send_vccio_drive_list_scheduled && m_vdd_com)
		{
			if (this->send_vccio_drive_list(*m_vdd_com))
				m_send_vccio_drive_list_scheduled = false;
		}

		if (inner_redirected && !com_inner.empty())
		{
			uint8_t size = com_inner.read_size();
			if (size > 14)
				size = 14;

			if (com_outer.tx_reserve(size + 3))
			{
				com_outer.write(0x80);
				com_outer.write(0x90 | (size + 1));
				com_outer.write(0x02);
				for (uint8_t i = 0; i < size; ++i)
					com_outer.write(com_inner.read());
			}
		}

		if (!com_app.empty() && m_app_com)
		{
			uint8_t size = com_app.read_size();
			if (size > 14)
				size = 14;

			if (m_app_com->tx_reserve(size + 3))
			{
				m_app_com->write(0x80);
				m_app_com->write(0x90 | (size + 1));
				m_app_com->write(0x01);
				for (uint8_t i = 0; i < size; ++i)
					m_app_com->write(com_app.read());
			}
		}

		if (!com_outer.empty())
		{
			uint8_t ch = cp_outer.push_data(com_outer.read());
			if (ch != 255)
			{
				bootseq.check(ch);
				if (!this->process_command(cp_outer, com_outer_writer)
					&& !process_tunnel(cp_outer, com_outer_writer, false))
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
				if (!this->process_command(cp_inner, com_inner_writer)
					&& !process_tunnel(cp_inner, com_inner_writer, true))
				{
					cp_inner.clear();
				}
			}
		}

		if (handler)
			handler->process_selected(m_primary_com);
	}

	void allow_com_app()
	{
		if (m_app_com_state != disabled)
			return;

		m_app_com_state = enabled;
		if (m_app_com)
			this->send_pipe_list(*m_app_com);
	}

	void disallow_com_app()
	{
		if (m_app_com_state == disabled)
			return;

		if (m_app_com)
			this->deactivate_com_app(*m_app_com);
		m_app_com_state = disabled;
		if (m_app_com)
			this->send_pipe_list(*m_app_com);
	}

	void process()
	{
		if (ADCA.CH0.INTFLAGS & ADC_CH_CHIF_bm)
		{
			ADCA.CH0.INTFLAGS = ADC_CH_CHIF_bm;
			m_vccio_voltage = ADCA.CH0RESL;
			m_vccio_voltage |= (ADCA.CH0RESH << 8);
			ADCA.CH0.CTRL |= ADC_CH_START_bm;
		}

		if (ADCA.CH1.INTFLAGS & ADC_CH_CHIF_bm)
		{
			ADCA.CH1.INTFLAGS = ADC_CH_CHIF_bm;
			m_vusb_voltage = ADCA.CH1RESL;
			m_vusb_voltage |= (ADCA.CH1RESH << 8);
			ADCA.CH1.CTRL |= ADC_CH_START_bm;

			if (m_vusb_voltage < 1332) // 3.6V
				pin_switched_pwr_en::set_high();
			if (m_vusb_voltage > 1480) // 4V
				pin_switched_pwr_en::set_low();
		}

		if (m_vccio_drive_state == active && m_vccio_drive_check_timeout)
		{
			m_vccio_drive_check_timeout.force();
			if (pin_sup_5v0::get_value() && m_vccio_voltage < 1630) // 4400mV
			{
				this->set_vccio_drive(0);
			}
			else if (pin_sup_3v3::get_value()
				&& (m_vccio_voltage < 1110 || m_vccio_voltage > 1332)) // 3000mV and 3600mV
			{
				this->set_vccio_drive(0);
			}
		}
	}

private:
	void activate_com_app(com_writer_t & com)
	{
		if (m_app_com_state == disabled)
			return;

		com.write(0x80);
		com.write(0x93);
		com.write(0x00);
		com.write(0x01);
		com.write(0x01);

		if (m_app_com_state != active)
		{
			m_app_com = &com;
			pin_buf_txd::make_high();
			com_app.usart().open(m_com_app_speed, true);
			m_app_com_state = active;
		}
	}

	void deactivate_com_app(com_writer_t & com)
	{
		if (m_app_com_state != active)
			return;

		com_app.usart().close();
		pin_buf_txd::make_input();
		m_app_com_state = enabled;

		com.write(0x80);
		com.write(0x93);
		com.write(0x00);
		com.write(0x02);
		com.write(0x01);
	}

	void send_pipe_list(com_writer_t & com)
	{
		com.write(0x80);
		com.write(m_app_com_state != disabled? 0x96: 0x92);
		com.write(0x00);
		com.write(0x00);
		if (m_app_com_state != disabled)
		{
			com.write(0x03);
			com.write('a');
			com.write('p');
			com.write('p');
		}
	}

	bool process_tunnel(avrlib::command_parser & cp, com_writer_t & com, bool inner)
	{
		switch (cp.command())
		{
		case 9:
			if (cp.size() > 1 && cp[0] == 0)
			{
				switch (cp[1])
				{
				case 0:
					this->send_pipe_list(com);
					break;
				case 1:
					// Activate a pipe
					if (!inner && cp.size() == 5 && cp[2] == 'u' && cp[3] == 's' && cp[4] == 'b')
					{
						com.write(0x80);
						com.write(0x93);
						com.write(0x00);
						com.write(0x01);
						com.write(0x02);
						inner_redirected = true;
						com_inner.usart().set_speed(639);
						pin_usb_cts_n::pulldown();
					}
					else if (cp.size() == 5 && cp[2] == 'a' && cp[3] == 'p' && cp[4] == 'p' && m_app_com_state != disabled)
					{
						this->activate_com_app(com);
					}
					else
					{
						com.write(0x80);
						com.write(0x93);
						com.write(0x00);
						com.write(0x01);
						com.write(0x00);
					}
					break;
				case 2:
					// Deactivate a pipe
					if (!inner && cp.size() == 3 && cp[2] == 2)
					{
						com.write(0x80);
						com.write(0x93);
						com.write(0x00);
						com.write(0x02);
						com.write(0x02);
						inner_redirected = false;
						com_inner.usart().set_speed(15);
						pin_usb_cts_n::pullup();
					}
					else if (cp.size() == 3 && cp[2] == 1)
					{
						this->deactivate_com_app(com);
					}
					else
					{
						com.write(0x80);
						com.write(0x92);
						com.write(0x00);
						com.write(0x02);
					}
					break;
				case 3:
					// Set pipe speed
					if (cp.size() == 5 && cp[2] == 1)
					{
						m_com_app_speed = cp[3] | (cp[4] << 8);
						com_app.usart().set_speed(m_com_app_speed);
					}
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

	bool send_vccio_state(com_writer_t * pCom)
	{
		if (!pCom)
			return true;

		if (!pCom->tx_reserve(5))
			return false;

		pCom->write(0x80);
		pCom->write(0xa3);
		pCom->write(0x01);  // VCCIO
		pCom->write(0x01);  // get_drive
		pCom->write(pin_sup_3v3::get_value()? 0x01: pin_sup_5v0::get_value()? 0x02: 0x00);
		return true;
	}

	void set_vccio_drive(uint8_t value)
	{
		pin_sup_3v3::set_value(false);
		pin_sup_5v0::set_value(false);

		if (((value == 1) || (value == 2)) && m_vccio_drive_state != disabled)
		{
			m_vccio_drive_check_timeout.restart();
			m_vccio_drive_state = active;

			if (value == 1)
				pin_sup_3v3::set_value(true);
			else if (value == 2)
				pin_sup_5v0::set_value(true);
		}
		else
		{
			m_vccio_drive_check_timeout.cancel();
			m_vccio_drive_state = enabled;
		}

		m_vccio_state_send_scheduled = true;
	}

	bool send_vccio_drive_list(com_writer_t & com)
	{
		if (!com.tx_reserve(m_vccio_drive_state != disabled? 0x10: 0x08))
			return false;

		com.write(0x80);
		com.write(m_vccio_drive_state != disabled? 0xae: 0xa6);
		com.write(0x00);
		com.write(0x01);  // VCCIO

		com.write(0x00);  // <hiz>
		com.write(0x00);
		com.write(0x00);
		com.write(0x00);
		if (m_vccio_drive_state != disabled)
		{
			com.write(0xe4);  // 3.3V, 50mA
			com.write(0x0c);
			com.write(0x32);
			com.write(0x00);
			com.write(0x88);  // 5V, 100mA
			com.write(0x13);
			com.write(0x64);
			com.write(0x00);
		}

		return true;
	}

	bool process_command(avrlib::command_parser & cp, com_writer_t & com)
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
							err = this->select_handler(&havricsp);
							break;
						case 1:
							err = this->select_handler(&hxmega);
							break;
						case 2:
							err = this->select_handler(&hjtag);
							break;
						case 3:
							err = this->select_handler(&hcc25xx);
							break;
						case 4:
							err = this->select_handler(&hspi);
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
						this->select_handler(0);
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
			if (cp.size() == 2 && cp[0] == 0 && cp[1] == 0)
			{
				com.write(0x80);
				com.write(0xa9);
				com.write(0x00);
				com.write(0x00);

				com.write(0x00); // flags

				com.write(5);
				com.write('V');
				com.write('C');
				com.write('C');
				com.write('I');
				com.write('O');

				this->send_vccio_drive_list(com);
				this->send_vccio_state(&com);
			}

			if (cp.size() == 3 && cp[0] == 1 && cp[1] == 2)
			{
				m_vdd_com = &com;
				this->set_vccio_drive(cp[2]);
			}

			return true;
		case '?':
			avrlib::send(com, "Shupito v2.0\n");
			cp.clear();
			return true;
		case 254:
			cp.clear();
			return true;
		case 255:
			break;
		default:
			if (&com == m_primary_com && handler)
				return handler->handle_command(cp.command(), cp.data(), cp.size(), com);
		}

		return false;
	}

	uint8_t select_handler(handler_base * new_handler)
	{
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

	handler_xmega<my_pdi_t, clock_t, process_t> hxmega;
	handler_avricsp<spi_t, clock_t, pin_buf_rst, process_t> havricsp;
	handler_jtagg<pin_buf_rst, pin_buf_xck, pin_buf_rxd, pin_buf_txd, process_t> hjtag;
	handler_cc25xx<spi_t, clock_t, pin_buf_rst, pin_buf_xck, process_t> hcc25xx;
	handler_spi<spi_t, pin_buf_rst> hspi;
	handler_base * handler;

	avrlib::timeout<clock_t> vdd_timeout;
	avrlib::timeout<clock_t> m_vccio_drive_check_timeout;

	com_writer_t * m_primary_com;
	com_writer_t * m_vdd_com;
	com_writer_t * m_app_com;

	enum { disabled, enabled, active } m_app_com_state, m_vccio_drive_state;
	bool m_vccio_state_send_scheduled;
	bool m_send_vccio_drive_list_scheduled;

	int16_t m_vccio_voltage;
	int16_t m_vusb_voltage;
	uint16_t m_com_app_speed;
};

context_t ctx;

void process_t::operator()() const
{
	ctx.process();
	pdi.process();
	com_inner.process_tx();
	com_outer.process_tx();
	com_app.process_tx();
}

uint8_t * com_writer_t::alloc(uint8_t cmd, uint8_t size)
{
	if (!this->tx_reserve(size + 2))
		return 0;
	m_buffer[0] = (cmd << 4) | size;
	return m_buffer + 1;
}

uint8_t * com_writer_t::alloc_sync(uint8_t cmd, uint8_t size)
{
	for (;;)
	{
		uint8_t * res = this->alloc(cmd, size);
		if (res)
			return res;
		g_process();
	}
}

void com_writer_t::commit()
{
	this->write(0x80);

	uint8_t size = (m_buffer[0] & 0xf) + 1;
	for (uint8_t i = 0; i < size; ++i)
		this->write(m_buffer[i]);
}

bool com_writer_t::send(uint8_t cmd, uint8_t const * data, uint8_t size)
{
	if (!this->tx_reserve(size + 2))
		return false;

	this->write(0x80);
	this->write((cmd << 4)|size);
	while (size)
	{
		this->write(*data);
		++data;
		--size;
	}
	return true;
}

void com_writer_t::send_sync(uint8_t cmd, uint8_t const * data, uint8_t size)
{
	while (!this->tx_reserve(size + 2))
		g_process();

	this->write(0x80);
	this->write((cmd << 4)|size);
	while (size)
	{
		this->write(*data);
		++data;
		--size;
	}
}

void spi_t::clear()
{
	USARTC1.CTRLB = 0;
	USARTC1.CTRLC = 0;

	pin_buf_txd::make_input();
	pin_buf_xck::make_input();
	pin_buf_xck::make_noninverted();

	ctx.allow_com_app();
}

spi_t::error_t spi_t::start_master(uint16_t bsel, uint8_t mode, bool lsb_first)
{
	ctx.disallow_com_app();

	pin_buf_txd::make_low();
	if (mode & 2)
		pin_buf_xck::make_inverted();
	pin_buf_xck::make_low();

	if (bsel)
		--bsel;
	USARTC1.BAUDCTRLA = bsel;
	USARTC1.BAUDCTRLB = bsel >> 8;
	USARTC1.CTRLC = USART_CMODE_MSPI_gc | ((mode & 1)? (1<<1): 0) | (lsb_first? (1<<2): 0);
	USARTC1.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
	return 0;
}

uint8_t spi_t::send(uint8_t v)
{
	USARTC1.DATA = v;
	pin_led::set_value(true);
	while ((USARTC1.STATUS & USART_RXCIF_bm) == 0)
	{
	}
	pin_led::set_value(false);
	return USARTC1.DATA;
}

void spi_t::enable_tx()
{
	pin_buf_txd::make_low();
}

void spi_t::disable_tx()
{
	pin_buf_txd::make_input();
}

bool spi_t::read_raw()
{
	return pin_buf_rxd::read();
}

void process_t::allow_tunnel()
{
	ctx.allow_com_app();
}

void process_t::disallow_tunnel()
{
	ctx.disallow_com_app();
}

int main()
{
	pin_usb_rtr_n::make_high();
	pin_sysclk::make_low();

	// Run at 32MHz
	OSC.CTRL = OSC_RC32MEN_bm | OSC_RC2MEN_bm | OSC_RC32KEN_bm;
	while ((OSC.STATUS & OSC_RC32MRDY_bm) == 0)
	{
	}

	CCP = CCP_IOREG_gc;
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;

	OSC.CTRL = OSC_RC32MEN_bm | OSC_RC32KEN_bm;
	while ((OSC.STATUS & OSC_RC32KRDY_bm) == 0)
	{
	}

	DFLLRC32M.CTRL = DFLL_ENABLE_bm;

	// Generate 8MHz clock for the USB chip
	TCE0.CCAL = 1;
	TCE0.CCAH = 0;
	TCE0.CTRLB = TC0_CCBEN_bm | TC_WGMODE_FRQ_gc;
	TCE0.CTRLA = TC_CLKSEL_DIV1_gc;

	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();

	ctx.init();
	for (;;)
	{
		ctx.run();
		g_process();
	}
}
