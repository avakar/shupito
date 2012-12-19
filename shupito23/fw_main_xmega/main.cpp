#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#define STRINGIFY2(x) #x
#define STRINGIFY(x) STRINGIFY2(x)

#include "../../fw_common/avrlib/xmega_pin.hpp"
#include "../../fw_common/avrlib/counter.hpp"
#include "../../fw_common/avrlib/stopwatch.hpp"
#include "dbg.h"
#include "usb.h"

#include "../../fw_common/handler_base.hpp"
#include "../../fw_common/handler_xmega.hpp"

#include "../../fw_common/pdi.hpp"

com_dbg_t com_dbg;
ISR(USARTE0_RXC_vect) { com_dbg.intr_rx(); }

AVRLIB_DEFINE_XMEGA_PIN(pin_button, PORTA, 5);
AVRLIB_DEFINE_XMEGA_PIN(pin_led,    PORTA, 3);

AVRLIB_DEFINE_XMEGA_PIN(pin_dbg_rx, PORTE, 2);
AVRLIB_DEFINE_XMEGA_PIN(pin_dbg_tx, PORTE, 3);

AVRLIB_DEFINE_XMEGA_PIN(pin_sup_3v3, PORTB, 2);
AVRLIB_DEFINE_XMEGA_PIN(pin_sup_5v0, PORTB, 0);

AVRLIB_DEFINE_XMEGA_PIN(pin_hiv_lo,    PORTC, 0);
AVRLIB_DEFINE_XMEGA_PIN(pin_hiv_vccio, PORTC, 4);
AVRLIB_DEFINE_XMEGA_PIN(pin_hiv_hi,    PORTB, 3);
AVRLIB_DEFINE_XMEGA_PIN(pin_hiv_pwm,   PORTD, 3);

typedef pin_totem_drive<pin_hiv_vccio, pin_hiv_lo> pin_rst;

AVRLIB_DEFINE_XMEGA_PIN(pin_aux_rstd, PORTD, 0);
AVRLIB_DEFINE_XMEGA_PIN(pin_aux_rstv, PORTC, 1);
AVRLIB_DEFINE_XMEGA_PIN(pin_pdid,     PORTD, 1);
AVRLIB_DEFINE_XMEGA_PIN(pin_pdiv,     PORTC, 3);
AVRLIB_DEFINE_XMEGA_PIN(pin_rxd,      PORTC, 6);
AVRLIB_DEFINE_XMEGA_PIN(pin_txdv,     PORTC, 7);
AVRLIB_DEFINE_XMEGA_PIN(pin_txdd,     PORTD, 2);

typedef pin_buffer_with_oe<pin_aux_rstv, pin_aux_rstd> pin_aux_rst;
typedef pin_buffer_with_oe<pin_pdiv, pin_pdid> pin_pdi;
typedef pin_buffer_with_oe<pin_txdv, pin_txdd> pin_txd;

struct timer_xd0
{
	template <uint32_t v>
	struct us { static const uint32_t value = (v + 7) >> 3; };

	typedef uint16_t time_type;
	static const uint8_t value_bits = 16;

	static void init()
	{
		TCD0.CTRLA = TC_CLKSEL_DIV256_gc;
	}

	static time_type value() { return TCD0.CNT; }
};
typedef timer_xd0 clock_t;
clock_t clock;

typedef pdi_t<clock_t, pin_aux_rst, pin_pdi, pin_led> my_pdi_t;
static my_pdi_t pdi(clock);

ISR(USARTC0_DRE_vect) { pdi.intr_udre(); }
ISR(USARTC0_TXC_vect) { pdi.intr_txc(); }
ISR(USARTC0_RXC_vect) { pdi.intr_rxc(); }

/**
 * \brief Jumps to the FLIP bootloader.
 *
 * Make sure that all peripherals are reset to default settings,
 * before starting the bootloader. Ideally, reset the CPU and
 * then call this function as the first thing in main.
 */
#define start_flip_bootloader() asm("jmp " STRINGIFY(BOOT_SECTION_START) " + 0x1fc")

static bool software_reset_occurred()
{
	return (RST_STATUS & RST_SRF_bm) != 0;
}

/**
 * \brief Resets the CPU. Interrupts must be disabled.
 */
static void initiate_software_reset() __attribute__((noreturn));
static void initiate_software_reset()
{
	CCP = CCP_IOREG_gc;
	RST_CTRL = RST_SWRST_bm;
	for (;;) {}
}

static void enable_interrupts()
{
	PMIC_CTRL = PMIC_RREN_bm | PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();
}

ISR(PORTA_INT0_vect, __attribute__((noreturn, naked, flatten)))
{
	initiate_software_reset();
}

static void setup_bootloader_button()
{
	pin_button::pinctrl(PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc);
	PORTA_INT0MASK = pin_button::bm;
	PORTA_INTCTRL = PORT_INT0LVL_HI_gc;
}

#define wait_until(cond) do {} while(!(cond))

static void setup_clocks()
{
	// The 32MHz oscillator is used as the main clock.
	// The 2MHz oscillator is fed through the PLL to generate
	// 48MHz for the USB peripheral. The 32kHz oscillator
	// is used by DFLL to tune 2MHz and 32MHz oscillators.
	
	// Enable all oscillators and wait for them to start-up.
	// The 2MHz oscillator is enabled in advance and doesn't
	// need waiting for.
	OSC_CTRL = OSC_RC32MEN_bm | OSC_RC2MEN_bm | OSC_RC32KEN_bm;
	wait_until(OSC_STATUS & OSC_RC32MRDY_bm);
	wait_until(OSC_STATUS & OSC_RC32KEN_bm);

	// Switch the CPU to the 32MHz oscillator.
	CCP = CCP_IOREG_gc;
	CLK_CTRL = CLK_SCLKSEL_RC32M_gc;

	// Enable DFLLs
	DFLLRC2M_CTRL = DFLL_ENABLE_bm;
	DFLLRC32M_CTRL = DFLL_ENABLE_bm;

	// Configure and enable PLL.
	OSC_PLLCTRL = OSC_PLLSRC_RC2M_gc | (24 << OSC_PLLFAC_gp);
	OSC_CTRL = OSC_PLLEN_bm | OSC_RC32MEN_bm | OSC_RC2MEN_bm | OSC_RC32KEN_bm;
	wait_until(OSC_STATUS & OSC_PLLRDY_bm);
}

struct yb_writer
{
	virtual uint8_t * alloc(uint8_t cmd, uint8_t size) { return 0; }
	virtual uint8_t * alloc_sync(uint8_t cmd, uint8_t size) { return 0; }
	virtual void commit() {}
	virtual bool send(uint8_t cmd, uint8_t const * data, uint8_t size) { return false; }
	virtual void send_sync(uint8_t cmd, uint8_t const * data, uint8_t size) {}

	uint8_t max_packet_size() const
	{
		return m_max_packet_size;
	}

	uint8_t m_max_packet_size;
};

struct process_t
{
	void operator()() const;
};

static process_t g_process;

struct process_with_debug_t
{
	void operator()() const;
};

static process_with_debug_t g_process_with_debug;

struct usb_yb_writer
	: yb_writer
{
	usb_yb_writer()
	{
		m_max_packet_size = 255;
	}

	uint8_t * alloc(uint8_t cmd, uint8_t size)
	{
		if (!usb_yb_in_packet_ready())
			return 0;
		usb_yb_in_packet[0] = cmd;
		m_size = size;
		return usb_yb_in_packet + 1;
	}

	uint8_t * alloc_sync(uint8_t cmd, uint8_t size)
	{
		while (!usb_yb_in_packet_ready())
			g_process();
		usb_yb_in_packet[0] = cmd;
		m_size = size;
		return usb_yb_in_packet + 1;
	}

	void commit()
	{
		usb_yb_send_in_packet(m_size + 1);
	}

	bool send(uint8_t cmd, uint8_t const * data, uint8_t size)
	{
		if (!usb_yb_in_packet_ready())
			return false;
		usb_yb_in_packet[0] = cmd;
		for (uint8_t i = 0; i != size; ++i)
			usb_yb_in_packet[i+1] = data[i];
		usb_yb_send_in_packet(size + 1);
		return true;
	}

	void send_sync(uint8_t cmd, uint8_t const * data, uint8_t size)
	{
		while (!usb_yb_in_packet_ready())
			g_process();
		usb_yb_in_packet[0] = cmd;
		for (uint8_t i = 0; i != size; ++i)
			usb_yb_in_packet[i+1] = data[i];
		usb_yb_send_in_packet(size + 1);
	}

	uint8_t m_size;
};

static uint8_t volatile const * sn_calib_indexes[] = {
	&PRODSIGNATURES_LOTNUM0,
	&PRODSIGNATURES_LOTNUM1,
	&PRODSIGNATURES_LOTNUM2,
	&PRODSIGNATURES_LOTNUM3,
	&PRODSIGNATURES_LOTNUM4,
	&PRODSIGNATURES_LOTNUM5,
	&PRODSIGNATURES_WAFNUM,
	&PRODSIGNATURES_COORDX0,
	&PRODSIGNATURES_COORDX1,
	&PRODSIGNATURES_COORDY0,
	&PRODSIGNATURES_COORDY1,
};

static uint8_t const sn_calib_indexes_count = sizeof sn_calib_indexes / sizeof sn_calib_indexes[0];

class app
{
public:
	app()
		: handler_pdi(pdi, clock, g_process)
	{
	}

	void init()
	{
		pin_led::make_low();
		pin_sup_3v3::make_low();
		pin_sup_5v0::make_low();

		pin_hiv_lo::make_low();
		pin_hiv_vccio::make_low();
		pin_hiv_hi::make_low();
		pin_hiv_pwm::make_low();

		pin_txd::init();
		pin_aux_rst::init();
		pin_pdi::init();

		timer_xd0::init();

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
		ADCA.CALL = pgm_read_byte(&PRODSIGNATURES_ADCACAL0);
		ADCA.CALH = pgm_read_byte(&PRODSIGNATURES_ADCACAL1);
		NVM_CMD = NVM_CMD_NO_OPERATION_gc;

		// Prepare the ADC
		ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
		ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN7_gc;
		ADCA.PRESCALER = ADC_PRESCALER_DIV64_gc;
		ADCA.REFCTRL = ADC_REFSEL_INT1V_gc;
		ADCA.CTRLB = ADC_CONMODE_bm | ADC_RESOLUTION_12BIT_gc;
		ADCA.CTRLA = ADC_ENABLE_bm;

		// Start the conversion immediately
		ADCA.CH0.CTRL |= ADC_CH_START_bm;

		usb_init(m_usb_sn, sizeof m_usb_sn);

		m_vccio_timeout.init(clock, clock_t::us<100000>::value);
		m_vccio_voltage = 0;
		m_send_vcc_driver_list_scheduled = false;
		m_send_vccio_state_scheduled = false;
		m_vccio_drive_state = vccio_disabled;
		m_vccio_drive_check_timeout.init_stopped(clock, clock_t::us<200000>::value);

		m_handler = 0;

		m_in_packet_reported = false;
	}

	void run()
	{
		if (uint16_t size = usb_yb_has_out_packet())
		{
			if (!m_in_packet_reported)
			{
				for (uint8_t i = 0; i != size; ++i)
				{
					if (i != 0)
						com_usb.write(':');
					send_hex(com_usb, usb_yb_out_packet[i], 2);
				}
				com_usb.write('\n');
				m_in_packet_reported = true;
			}

			if (this->handle_packet(usb_yb_out_packet[0], usb_yb_out_packet + 1, size - 1, m_usb_writer))
			{
				usb_yb_confirm_out_packet();
				m_in_packet_reported = false;
			}
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

		if (ADCA.CH0.INTFLAGS & ADC_CH_CHIF_bm)
		{
			ADCA.CH0.INTFLAGS = ADC_CH_CHIF_bm;
			m_vccio_voltage = ADCA.CH0RESL;
			m_vccio_voltage |= (ADCA.CH0RESH << 8);
			ADCA.CH0.CTRL |= ADC_CH_START_bm;
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

		if (!com_tunnel.empty() && com_tunnel.tx_ready())
			com_tunnel.write(com_tunnel.read());
	}

	bool send_vccio_state(yb_writer & w)
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

	bool send_vccio_drive_list(yb_writer & w)
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

	void set_vccio_drive(uint8_t value)
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

	bool handle_packet(uint8_t cmd, uint8_t const * cp, uint8_t size, yb_writer & w)
	{
		switch (cmd)
		{
		case 0x0:
			if (size && cp[0] == 1)
			{
				uint8_t * wbuf = w.alloc(0, 1);
				if (!wbuf)
					return false;

				uint8_t err = 1;
				if (size == 3 && cp[1] == 0)
				{
					switch (cp[2])
					{
					case 1:
						err = this->select_handler(&handler_pdi);
						break;
					}
				}

				*wbuf = err;
				w.commit();
			}
			else if (size && cp[0] == 2)
			{
				uint8_t * wbuf = w.alloc(0, 1);
				if (!wbuf)
					return false;

				if (size == 3 && cp[1] == 0)
					this->select_handler(0);

				*wbuf++ = 0;
				w.commit();
			}

			break;
		case 0xa:
			if (size == 2 && cp[0] == 0 && cp[1] == 0)
			{
				static uint8_t const vccio_list[] = {
					0, 0,
					0, // flags
					5, 'V', 'C', 'C', 'I', 'O'
				};

				if (!w.send(0xa, vccio_list, sizeof vccio_list))
					return false;

				m_send_vcc_driver_list_scheduled = true;
				m_send_vccio_state_scheduled = true;
			}
			else if (size == 3 && cp[0] == 1 && cp[1] == 2)
			{
				this->set_vccio_drive(cp[2]);
			}

			break;

		default:
			if (m_handler)
				return m_handler->handle_command(cmd, cp, size, w);
		}

		return true;
	}

	uint8_t select_handler(handler_base<yb_writer> * new_handler)
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
	
	void process_with_debug()
	{
		if (!com_usb.empty())
		{
			switch (uint8_t ch = com_usb.read())
			{
			case '?':
				send(com_usb, "Shupito 2.3\n");
				break;
			case 'A':
				AVRLIB_ASSERT(!"test assert");
				break;
			case 'B':
				initiate_software_reset();
				break;
			default:
				send(com_usb, "?AB\n");
				break;
			}
		}
	}

private:
	char m_usb_sn[2*sn_calib_indexes_count];
	usb_yb_writer m_usb_writer;
	avrlib::timeout<clock_t> m_vccio_timeout;
	int16_t m_vccio_voltage;
	bool m_send_vcc_driver_list_scheduled;
	bool m_send_vccio_state_scheduled;
	enum { vccio_disabled, vccio_enabled, vccio_active } m_vccio_drive_state;
	avrlib::timeout<clock_t> m_vccio_drive_check_timeout;

	handler_xmega<my_pdi_t, yb_writer, clock_t, process_t> handler_pdi;
	handler_base<yb_writer> * m_handler;
	
	bool m_in_packet_reported;
};

static app g_app;

void process_t::operator()() const
{
	pdi.process();
	usb_poll();
	com_dbg.process_tx();
}

void process_with_debug_t::operator()() const
{
	g_process();
	g_app.process_with_debug();
}

void avrlib::assertion_failed(char const * msg, char const * file, int lineno)
{
	pin_led::set_high();
	send(com_usb, "Assertion failed: ");
	send(com_usb, msg);
	send(com_usb, "\n    ");
	send(com_usb, file);
	send(com_usb, "(0x");
	send_hex(com_usb, lineno);
	send(com_usb, ")\n");
	for (;;)
		g_process_with_debug();
}

int main()
{
	if (software_reset_occurred())
		start_flip_bootloader();
	setup_bootloader_button();
	enable_interrupts();
	setup_clocks();

	g_app.init();
	for (;;)
	{
		g_app.run();
	}
}
