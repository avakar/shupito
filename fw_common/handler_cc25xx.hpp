#ifndef SHUPITO_HANDLER_CC25XX_HPP
#define SHUPITO_HANDLER_CC25XX_HPP

#include "handler_base.hpp"
#include "avrlib/stopwatch.hpp"
#include <avr/io.h>

template <typename Spi, typename Com, typename Clock, typename ResetPin, typename ClkPin, typename Process>
class handler_cc25xx
	: public handler_base<Com>
{
public:
	typedef Spi spi_t;
	typedef Com com_t;
	typedef Clock clock_t;

	handler_cc25xx(spi_t & spi, clock_t & clock, Process process = Process())
		: spi(spi), clock(clock), m_process(process)
	{
	}

	void unselect()
	{
		spi.clear();
	}

	bool handle_command(avrlib::command_parser & cp, com_t & com)
	{
		switch (cp.command())
		{
		case 1: // PROGEN 2'bsel
			{
				spi.clear();
				ResetPin::make_low();
				avrlib::wait(clock, Clock::template us<10>::value);

				for (uint8_t i = 0; i < 2; ++i)
				{
					ClkPin::make_high();
					avrlib::wait(clock, Clock::template us<1>::value);
					ClkPin::set_low();
					avrlib::wait(clock, Clock::template us<1>::value);
				}

				ResetPin::make_high();
				avrlib::wait(clock, Clock::template us<1>::value);

				uint8_t err = spi.start_master(cp[0] | (cp[1] << 8), true);
				com.write(0x80);
				com.write(0x11);
				com.write(err);
			}
			break;
		case 2:
			// Release the reset line

			ResetPin::make_low();
			avrlib::wait(clock, Clock::template us<10>::value);
			spi.clear();
			ResetPin::make_input();

			com.write(0x80);
			com.write(0x21);
			com.write(0);
			break;
		case 3: // CMD 1'read_count *'data
			if (cp.size() >= 1)
			{
				for (uint8_t i = 1; i < cp.size(); ++i)
					spi.send(cp[i]);
				spi.disable_tx();

				com.write(0x80);
				com.write(0x30 | (cp[0] + 1));

				uint8_t err = 0;
				for (uint8_t i = 0; i < cp[0]; ++i)
				{
					uint8_t value = 0;
					if (!err)
						err = this->read_byte(value);
					com.write(value);
				}
				com.write(err);

				spi.enable_tx();
			}
			break;
		case 4: // READ 2'count
			if (cp.size() == 2)
			{
				// repeats the following pair of instructions `count` times
				//    MOVX A,@DPTR
				//    INC DPTR

				uint16_t count = cp[0] | (cp[1] << 8);

				uint8_t err = 0;
				bool sent_error = false;
				for (;;)
				{
					uint8_t chunk = 0xf;
					if (count < 0xf)
						chunk = count;
					if (err)
						chunk = 0;
					count -= chunk;

					uint8_t packet_size = chunk;
					if (chunk < 0xf && !sent_error)
						++packet_size;
					
					com.write(0x80);
					com.write(0x40 | packet_size);

					for (uint8_t i = 0; i != chunk; ++i)
					{
						uint8_t value = 0;
						if (!err)
						{
							spi.send(0x51);
							spi.send(0xe0);
							spi.disable_tx();
							err = this->read_byte(value);
							spi.enable_tx();

							if (!err)
							{
								spi.send(0x51);
								spi.send(0xa3);
								spi.disable_tx();
								err = this->read_byte();
								spi.enable_tx();
							}
						}

						com.write(value);
						m_process();
					}

					if (chunk < 0xf && !sent_error)
					{
						sent_error = true;
						com.write(err);
					}

					if (packet_size < 0xf)
						break;
				}
			}
			break;
		case 5: // WRITE 1'addr *'data
			if (cp.size() != 0)
			{
				uint8_t addr = cp[0];
				uint8_t err = 0;

				// Repeatedly execute
				//     MOV addr, #data
				for (uint8_t i = 1; !err && i < cp.size(); ++i)
				{
					spi.send(0x53);
					spi.send(0x75);
					spi.send(addr & 0x7f);
					spi.send(cp[i]);
					spi.disable_tx();
					err = this->read_byte();
					spi.enable_tx();
					++addr;
				}

				com.write(0x80);
				com.write(0x51);
				com.write(err);
			}
			break;
		default:
			return false;
		}

		return true;
	}

private:
	uint8_t read_byte()
	{
		uint8_t value;
		return this->read_byte(value);
	}

	uint8_t read_byte(uint8_t & value)
	{
		typename clock_t::time_type base = clock.value();
		for (;;)
		{
			avrlib::wait(clock, clock_t::template us<5>::value, m_process);
			if (spi.read_raw())
				spi.send(0);
			else
				break;

			if (clock.value() - base >= clock_t::template us<100000>::value)
				return 1;
		}

		value = spi.send(0);
		return 0;
	}

	spi_t & spi;
	clock_t & clock;
	Process m_process;
};

#endif
