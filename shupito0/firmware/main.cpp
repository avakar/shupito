#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "../../fw_common/avrlib/command_parser.hpp"
#include "../../fw_common/avrlib/async_usart.hpp"
#include "../../fw_common/avrlib/bootseq.hpp"
#include "../../fw_common/avrlib/usart0.hpp"
#include "../../fw_common/avrlib/format.hpp"
#include "../../fw_common/handler_avricsp.hpp"

#include "../../fw_common/avrlib/portb.hpp"
#include "../../fw_common/avrlib/pin.hpp"
#include "../../fw_common/avrlib/null_pin.hpp"

struct clock_t
{
	template <uint32_t v>
	struct us { static const uint32_t value = (v + 7) >> 3; };

	typedef uint16_t time_type;
	static const uint8_t value_bits = 16;

	static void init()
	{
		TCCR1A = 0;
		TCCR1C = 0;
		TCCR1B = (1<<CS11)|(1<<CS10);
	}

	static time_type value()
	{
		uint8_t v = TCNT1L;
		return v | (time_type(TCNT1H) << 8);
	}
};

clock_t clock;

typedef avrlib::pin<avrlib::portb, 2> pin_rst;

typedef avrlib::pin<avrlib::portb, 4> pin_miso;
typedef avrlib::pin<avrlib::portb, 3> pin_mosi;
typedef avrlib::pin<avrlib::portb, 5> pin_sck;
typedef pin_rst pin_ss_n;

typedef avrlib::null_pin<false> pin_led;

typedef avrlib::async_usart<avrlib::usart0, 64, 64> com_t;
com_t com;
ISR(USART_RX_vect) { com.intr_rx(); }

class spi_t
{
public:
	typedef uint8_t error_t;

	void clear();
	error_t start_master(uint16_t speed_khz);
	uint8_t send(uint8_t v);
};

spi_t spi;

struct process_t
{
	void operator()()
	{
		com.process_tx();
	}		
};

process_t process;

static uint8_t const device_descriptor[] PROGMEM = {
#include "desc.h"
};

class context_t
{
public:
	context_t()
		: havricsp(spi, clock, process), m_interface_enabled(false)
	{
	}

	void init()
	{
		com.usart().open_ubrr(avrlib::ubrr<38400>::value, true);

		clock_t::init();
		m_cp.clear();
	}

	void run()
	{
		if (!com.empty())
		{
			uint8_t ch = m_cp.push_data(com.read());
			if (ch != 255)
			{
				bootseq.check(ch);
				if (!this->process_command(m_cp, com))
					m_cp.clear();
			}
		}

		if (m_interface_enabled)
			havricsp.process_selected();

		process();
	}

private:
	bool process_command(avrlib::command_parser & m_cp, com_t & com)
	{
		switch (m_cp.command())
		{
		case 0:
			if (m_cp.size() == 0)
				return true;

			switch (m_cp[0])
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
					if (m_cp.size() == 1)
					{
						if (m_interface_enabled)
						{
							err = 0;
						}							
						else
						{
							err = havricsp.select();
							m_interface_enabled = true;
						}							
					}						

					com.write(0x80);
					com.write(0x01);
					com.write(err);
				}
				break;
			case 2:
				// Disable interface
				{
					uint8_t err = 1;
					if (m_cp.size() == 1)
					{
						err = 0;
						if (m_interface_enabled)
						{
							havricsp.unselect();
							m_interface_enabled = false;
						}							
					}						

					com.write(0x80);
					com.write(0x01);
					com.write(err);
				}
				break;
			}

			return true;
		case '?':
			avrlib::send(com, "Shupito v0.1\n");
			m_cp.clear();
			return true;
		case 254:
			m_cp.clear();
			return true;
		case 255:
			break;
		default:
			if (m_interface_enabled)
				return havricsp.handle_command(m_cp, com);
		}

		return false;
	}

private:
	avrlib::bootseq bootseq;
	avrlib::command_parser m_cp;

	handler_avricsp<spi_t, com_t, clock_t, pin_rst, process_t> havricsp;
	bool m_interface_enabled;
};

context_t ctx;

void spi_t::clear()
{
	SPCR = 0;

	pin_mosi::make_input();
	pin_sck::make_input();
	pin_ss_n::make_input();
}

spi_t::error_t spi_t::start_master(uint16_t speed_khz)
{
	pin_mosi::make_low();
	pin_sck::make_low();
	pin_ss_n::make_high();

	// XXX: polarity, phase, data order
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
	return 0;
}

uint8_t spi_t::send(uint8_t v)
{
	SPDR = v;
	pin_led::set_high();
	while ((SPSR & (1<<SPIF)) == 0)
	{
	}
	pin_led::set_low();
	return SPDR;
}

int main()
{
	sei();
	ctx.init();
	for (;;)
	{
		ctx.run();
	}
}
