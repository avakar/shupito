#include "app.hpp"
#include "usb.h"
#include "dbg.h"
#include "utils.hpp"
#include "btn.hpp"
#include "led.hpp"

app g_app;

com_tunnel_t com_tunnel;
ISR(USARTC1_RXC_vect) { com_tunnel.intr_rx(); }

process_t g_process;
process_with_debug_t g_process_with_debug;

static my_pdi_t pdi(clock);

ISR(USARTC0_DRE_vect) { pdi.intr_udre(); }
ISR(USARTC0_TXC_vect) { pdi.intr_txc(); }
ISR(USARTC0_RXC_vect) { pdi.intr_rxc(); }

static spi_t spi;

com_dbg_t com_dbg;
ISR(USARTE0_RXC_vect) { com_dbg.intr_rx(); }

usb_yb_writer::usb_yb_writer()
{
	m_max_packet_size = 255;
}

uint8_t * usb_yb_writer::alloc(uint8_t cmd, uint8_t size)
{
	if (!usb_yb_in_packet_ready())
		return 0;
	usb_yb_in_packet[0] = cmd;
	m_size = size;
	return usb_yb_in_packet + 1;
}

uint8_t * usb_yb_writer::alloc_sync(uint8_t cmd, uint8_t size)
{
	while (!usb_yb_in_packet_ready())
		g_process();
	usb_yb_in_packet[0] = cmd;
	m_size = size;
	return usb_yb_in_packet + 1;
}

void usb_yb_writer::commit()
{
	usb_yb_send_in_packet(m_size + 1);
}

bool usb_yb_writer::send(uint8_t cmd, uint8_t const * data, uint8_t size)
{
	if (!usb_yb_in_packet_ready())
		return false;
	usb_yb_in_packet[0] = cmd;
	for (uint8_t i = 0; i != size; ++i)
		usb_yb_in_packet[i+1] = data[i];
	usb_yb_send_in_packet(size + 1);
	return true;
}

void usb_yb_writer::send_sync(uint8_t cmd, uint8_t const * data, uint8_t size)
{
	while (!usb_yb_in_packet_ready())
		g_process();
	usb_yb_in_packet[0] = cmd;
	for (uint8_t i = 0; i != size; ++i)
		usb_yb_in_packet[i+1] = data[i];
	usb_yb_send_in_packet(size + 1);
}

app::app()
	: m_handler_avricsp(spi, clock, g_process), m_handler_pdi(pdi, clock, g_process)
{
}

void app::init()
{
	led_init();
	pin_sup_3v3::make_low();
	pin_sup_5v0::make_low();

	pin_rst::init();

	pin_txd::init();
	pin_aux_rst::init();
	pin_pdi::init();
	
	pin_pdi_rx::pullup();

	pin_dbg_0::make_low();
	pin_dbg_1::make_low();

	clock_t::init();

	// Setup com_dbg
	pin_dbg_rx::pullup();
	pin_dbg_tx::make_high();
	com_dbg.usart().open((-1 << 12)|102 /*38400*/, true);

	send(com_usb, "Starting...\n");

	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	for (uint8_t i = 0; i < sn_calib_indexes_count; ++i)
	{
		static char const digits[] = "0123456789abcdef";
		uint8_t val = pgm_read_byte(sn_calib_indexes[i]);
		m_usb_sn[2*i] = digits[val >> 4];
		m_usb_sn[2*i+1] = digits[val & 0xf];
	}
	ADCA_CAL = pgm_read_word(&PRODSIGNATURES_ADCACAL0);
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;

	// Disable unused peripherals to decrease power consumption and noise
	PR_PRGEN = PR_AES_bm | PR_EBI_bm | PR_RTC_bm | PR_EVSYS_bm | PR_DMA_bm;
	PR_PRPA = PR_DAC_bm | PR_AC_bm;
	PR_PRPB = PR_DAC_bm | PR_AC_bm;
	PR_PRPC = PR_TWI_bm | PR_HIRES_bm;
	PR_PRPD = PR_TWI_bm | PR_HIRES_bm | PR_TC1_bm | PR_USART1_bm;
	PR_PRPE = PR_TWI_bm | PR_HIRES_bm;

	// Disable input buffers on unused/output pins
	PORTA_PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTA_PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTA_PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTA_PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTA_PIN6CTRL = PORT_ISC_INPUT_DISABLE_gc;

	PORTB_PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTB_PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTB_PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTB_PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;

	PORTC_PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTC_PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;

	PORTD_PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTD_PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTD_PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTD_PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTD_PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTD_PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;

	PORTE_PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTE_PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTE_PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;

	PORTR_PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTR_PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;

	// Prepare the ADC
	PORTA_PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc;
	ADCA_CH0_CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCA_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN7_gc;

	ADCA_PRESCALER = ADC_PRESCALER_DIV128_gc;
	ADCA_REFCTRL = ADC_REFSEL_INT1V_gc | ADC_BANDGAP_bm;
	ADCA_CTRLB = ADC_CONMODE_bm | ADC_RESOLUTION_12BIT_gc;
	ADCA_CTRLA = ADC_ENABLE_bm;

	// Start the conversion immediately
	ADCA_CH0_CTRL |= ADC_CH_START_bm;

	usb_init(m_usb_sn, sizeof m_usb_sn);

	m_vccio_timeout.init(clock, clock_t::us<100000>::value);
	m_vccio_voltage = 0;
	m_send_vcc_driver_list_scheduled = false;
	m_send_vccio_state_scheduled = false;
	m_vccio_drive_state = vccio_disabled;
	m_vccio_drive_check_timeout.init_stopped(clock, clock_t::us<200000>::value);
	m_vccio_filter.init();

	m_handler = 0;

	m_tunnel_open = false;
	m_tunnel_allowed = true;

	hiv_init();

	btn_init();
	m_assumed_btn_state = btn_pressed();
}

void app::run()
{
	if (uint16_t size = usb_yb_has_out_packet())
	{
		if (this->handle_packet(usb_yb_out_packet[0], usb_yb_out_packet + 1, size - 1, m_usb_writer))
			usb_yb_confirm_out_packet();
	}

	if (m_vccio_timeout)
	{
		if (usb_yb_in_packet_ready())
		{
			m_vccio_timeout.restart();
			usb_yb_in_packet[0] = 0xa;
			usb_yb_in_packet[1] = 1;
			usb_yb_in_packet[2] = 3;
			usb_yb_in_packet[3] = m_vccio_voltage;
			usb_yb_in_packet[4] = m_vccio_voltage >> 8;
			usb_yb_send_in_packet(5);
		}
		else
		{
			m_vccio_timeout.force();
		}
	}

	if (m_assumed_btn_state != btn_pressed())
	{
		if (uint8_t * buf = m_usb_writer.alloc(0xc, 2))
		{
			m_assumed_btn_state = !m_assumed_btn_state;

			*buf++ = 1;
			*buf++ = m_assumed_btn_state;
			m_usb_writer.commit();
		}
	}

	if (ADCA_CH0_INTFLAGS & ADC_CH_CHIF_bm)
	{
		ADCA_CH0_INTFLAGS = ADC_CH_CHIF_bm;
		m_vccio_filter.push(ADCA_CH0RES);
		m_vccio_voltage = m_vccio_filter.current();
		ADCA_CH0_CTRL |= ADC_CH_START_bm;
	}

	if (m_send_vcc_driver_list_scheduled)
	{
		if (this->send_vccio_drive_list(m_usb_writer))
			m_send_vcc_driver_list_scheduled = false;
	}

	if (m_send_vccio_state_scheduled)
	{
		if (this->send_vccio_state(m_usb_writer))
			m_send_vccio_state_scheduled = false;
	}

	if (m_vccio_drive_state == vccio_disabled && m_vccio_voltage < 111) // 300mV
	{
		m_vccio_drive_state = vccio_enabled;
		m_send_vcc_driver_list_scheduled = true;
	}

	if ((m_vccio_drive_state == vccio_enabled && m_vccio_voltage > 185)) // 500mV
	{
		pin_sup_5v0::set_low();
		pin_sup_3v3::set_low();
		m_vccio_drive_state = vccio_disabled;
		m_send_vcc_driver_list_scheduled = true;
	}

	if (m_vccio_drive_state == vccio_active && m_vccio_drive_check_timeout)
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

	g_process_with_debug();

	if (m_handler)
		m_handler->process_selected();

	while (!com_usb_tunnel.empty() && com_tunnel.tx_ready())
		com_tunnel.write(com_usb_tunnel.read());

	while (!com_tunnel.empty() && com_usb_tunnel.tx_ready())
		com_usb_tunnel.write(com_tunnel.read());

	while (!com_usb_tunnel2.empty() && com_dbg.tx_ready())
		com_dbg.write(com_usb_tunnel2.read());

	while (!com_dbg.empty() && com_usb_tunnel2.tx_ready())
		com_usb_tunnel2.write(com_dbg.read());
}

bool app::send_vccio_state(yb_writer & w)
{
	if (uint8_t * buf = w.alloc(0xa, 3))
	{
		*buf++ = 0x01;  // VCCIO
		*buf++ = 0x01;  // get_drive
		*buf++ = pin_sup_3v3::get_value()? 0x01: pin_sup_5v0::get_value()? 0x02: 0x00;
		w.commit();
		return true;
	}

	return false;
}

bool app::send_vccio_drive_list(yb_writer & w)
{
	if (uint8_t * buf = w.alloc(0xa, m_vccio_drive_state != vccio_disabled? 0xe: 0x6))
	{
		*buf++ = 0x00;
		*buf++ = 0x01;  // VCCIO

		*buf++ = 0x00;  // <hiz>
		*buf++ = 0x00;
		*buf++ = 0x00;
		*buf++ = 0x00;

		if (m_vccio_drive_state != vccio_disabled)
		{
			*buf++ = 0xe4;  // 3.3V, 50mA
			*buf++ = 0x0c;
			*buf++ = 0x32;
			*buf++ = 0x00;
			*buf++ = 0x88;  // 5V, 100mA
			*buf++ = 0x13;
			*buf++ = 0x64;
			*buf++ = 0x00;
		}

		w.commit();
		return true;
	}

	return false;
}

void app::set_vccio_drive(uint8_t value)
{
	pin_sup_3v3::set_value(false);
	pin_sup_5v0::set_value(false);

	if (((value == 1) || (value == 2)) && m_vccio_drive_state != vccio_disabled)
	{
		m_vccio_drive_check_timeout.restart();
		m_vccio_drive_state = vccio_active;

		if (value == 1)
			pin_sup_3v3::set_value(true);
		else if (value == 2)
			pin_sup_5v0::set_value(true);
	}
	else
	{
		m_vccio_drive_check_timeout.cancel();
		m_vccio_drive_state = vccio_enabled;
	}

	m_send_vccio_state_scheduled = true;
}

uint8_t app::select_handler(handler_base * new_handler)
{
	uint8_t err = 0;

	if (new_handler != m_handler)
	{
		if (m_handler)
			m_handler->unselect();
		if (new_handler)
			err = new_handler->select();
		m_handler = (err == 0? new_handler: 0);
	}

	return err;
}

#include "baudctrls.h"

void app::start_tunnel()
{
	com_tunnel.usart().close();
	pin_txd::make_high();
	com_tunnel.usart().open(m_tunnel_baudctrl, /*rx_interrupt=*/true, /*synchronous=*/false, m_tunnel_dblspeed);
}

void app::stop_tunnel()
{
	com_tunnel.usart().close();
	pin_txd::make_input();
}

void app::open_tunnel(uint8_t which, uint32_t baudrate)
{
	led_blink_short();

	uint32_t last_val = 0;
	uint16_t selected_index = usart_baudctrl_count-1;
	for (uint16_t i = 0; i < sizeof usart_baudctrls; i += 5)
	{
		uint32_t val
			= pgm_read_byte(&usart_baudctrls[i])
			| ((uint32_t)pgm_read_byte(&usart_baudctrls[i+1]) << 8)
			| ((uint32_t)(pgm_read_byte(&usart_baudctrls[i+2]) & 0x7f) << 16);

		if (val >= baudrate)
		{
			if (i != 0 && baudrate - last_val < val - baudrate)
				selected_index = i - 5;
			else
				selected_index = i;
			break;
		}

		last_val = val;
	}

	bool dblspeed = (pgm_read_byte(&usart_baudctrls[selected_index+2]) & 0x7f) != 0;

	uint8_t blo = pgm_read_byte(&usart_baudctrls[selected_index+3]);
	uint8_t bhi = pgm_read_byte(&usart_baudctrls[selected_index+4]);

	if (which == 0)
	{
		m_tunnel_dblspeed = dblspeed;
		m_tunnel_baudctrl = blo | (bhi << 8);
		m_tunnel_open = true;
		if (m_tunnel_allowed)
			this->start_tunnel();
	}
	else
	{
		com_dbg.usart().set_speed(blo | (bhi << 8), dblspeed);
	}
}

void app::close_tunnel()
{
	if (m_tunnel_open && m_tunnel_allowed)
		this->stop_tunnel();
	m_tunnel_open = false;
}

void app::allow_tunnel()
{
	if (!m_tunnel_allowed && m_tunnel_open)
		this->start_tunnel();
	m_tunnel_allowed = true;
}

void app::disallow_tunnel()
{
	if (m_tunnel_allowed && m_tunnel_open)
		this->stop_tunnel();
	m_tunnel_allowed = false;
}

void process_t::operator()() const
{
	led_process();
	pin_rst::process();
	btn_process();
	hiv_process();
	pdi.process();
	usb_poll();
	com_dbg.process_tx();
	com_tunnel.process_tx();
}

void process_with_debug_t::operator()() const
{
	g_process();
	g_app.process_with_debug();
}
