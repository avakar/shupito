#ifndef SHUPITO_HANDLER_AVRICSP_HPP
#define SHUPITO_HANDLER_AVRICSP_HPP

#include "handler_base.hpp"
#include <avr/io.h>

template <typename Spi, typename Com, typename Clock, typename ResetPin>
class handler_avricsp
	: public handler_base
{
public:
	typedef Spi spi_t;
	typedef Com com_t;
	typedef Clock clock_t;

	handler_avricsp(spi_t & spi, com_t & com, clock_t & clock)
		: spi(spi), com(com), clock(clock), m_programming_enabled(false)
	{
	}

	void handle_command(avrlib::command_parser & cp)
	{
		switch (cp.command())
		{
		case 1:
			m_programming_enabled = false;

			{
				typename spi_t::error_t err = spi.start_master(0);
				if (err)
				{
					com.write(0x80);
					com.write(0x11);
					com.write(err);
					break;
				}
			}

			// Pull down the reset line and send "Programming enable" sequence
			ResetPin::set(false);
			ResetPin::output(true);
			wait(clock, 1000);

			ResetPin::set(true);
			for (uint8_t i = 0; i < 3; ++i)
			{
				wait(clock, 1000);
				ResetPin::set(false);

				// There has to be a 20ms delay on atmega128
				wait(clock, 20);

				spi.send(0xAC);
				spi.send(0x53);
				uint8_t echo = spi.send(0x00);
				spi.send(0x00);

				if (echo == 0x53)
				{
					m_programming_enabled = true;
					break;
				}

				ResetPin::set(true);
			}

			com.write(0x80);
			com.write(0x11);
			com.write(!m_programming_enabled);
			break;
		case 2:
			// Release the reset line
			spi.clear();
			m_programming_enabled = false;
			com.write(0x80);
			com.write(0x21);
			com.write(0);
			break;
		case 3:
			// Read signature
			{
				static uint8_t const commands[][3] =
				{
					{ 0x30, 0x00, 0x00 },
					{ 0x30, 0x00, 0x01 },
					{ 0x30, 0x00, 0x02 },
					/*{ 0x58, 0x00, 0x00 },
					{ 0x50, 0x00, 0x00 },
					{ 0x58, 0x08, 0x00 },
					{ 0x50, 0x08, 0x00 },
					{ 0x38, 0x00, 0x00 },*/
				};
				static uint8_t const command_count = sizeof commands / sizeof commands[0];
				
				com.write(0x80);
				com.write(0x30 | command_count);
				for (uint8_t i = 0; i < command_count; ++i)
				{
					for (uint8_t j = 0; j < 3; ++j)
						spi.send(commands[i][j]);
					com.write(spi.send(0));
				}
			}
			break;
		case 4: // READ 1'memid 4'addr 1'size
			{
				uint8_t memid = cp[0];
				switch (memid)
				{
				case 1:
					{
						uint32_t addr = cp[1] | (cp[2] << 8) | ((uint32_t)cp[3] << 16) | ((uint32_t)cp[4] << 24);
						uint8_t size = cp[5];
						com.write(0x80);
						com.write(0xf4);
						com.write(size);
						for (; size != 0; --size, ++addr)
						{
							// program memory words are sent in the little endian order
							spi.send(addr & 1? 0x28: 0x20);
							spi.send(addr >> 9);
							spi.send(addr >> 1);
							com.write(spi.send(0));
						}
						com.write(0);
					}
					break;
				case 3:
					{
						static uint8_t const commands[][3] =
						{
							{ 0x58, 0x00, 0x00 },
							{ 0x50, 0x00, 0x00 },
							{ 0x58, 0x08, 0x00 },
							{ 0x50, 0x08, 0x00 },
						};
				
						com.write(0x80);
						com.write(0x44);
						for (uint8_t i = 0; i < 4; ++i)
						{
							for (uint8_t j = 0; j < 3; ++j)
								spi.send(commands[i][j]);
							com.write(spi.send(0));
						}
					}
					break;
				}
			}
			break;
		case 5: // ERASE 1'memid
			if (cp.size() == 0)
			{
				spi.send(0xAC);
				spi.send(0x80);
				spi.send(0);
				spi.send(0);

				wait(clock, 10000);

				com.write(0x80);
				com.write(0x51);
				com.write(0x00);
			}
			break;
		case 6:
			// WPREP 1'memid 4'addr
			if (cp.size() >= 5)
			{
				uint8_t memid = cp[0];

				bool success = true;
				if (memid == 1)
				{
					m_mempage_ptr = (cp[1]) | (cp[2] << 8);
					// TODO: potentially load the extended address byte
				}
				else if (memid == 3)
				{
				}
				else
				{
					success = false;
				}

				com.write(0x80);
				com.write(0x61);
				com.write(!success);
			}
			break;
		case 7:
			// WFILL 1'memid (1'data)*
			if (cp.size() >= 1)
			{
				bool success = true;

				uint8_t memid = cp[0];
				if (memid == 1)
				{
					for (uint8_t i = 1; i < cp.size(); ++i)
					{
						spi.send(m_mempage_ptr & 1? 0x48: 0x40);
						spi.send(0x00);
						spi.send(m_mempage_ptr >> 1);
						spi.send(cp[i]);
						++m_mempage_ptr;
					}
				}
				else if (memid == 3)
				{
				}
				else
				{
					success = false;
				}

				com.write(0x80);
				com.write(0x71);
				com.write(!success);
			}
			break;
		case 8:
			// WRITE 1'memid 4'addr
			if (cp.size() == 5)
			{
				bool success = true;

				uint8_t memid = cp[0];
				if (memid == 1)
				{
					uint16_t word_addr = (cp[1] >> 1) | (cp[2] << 7) | (cp[3] << 15);

					spi.send(0x4C);
					spi.send(word_addr >> 8);
					spi.send(word_addr);
					spi.send(0x00);
				
					wait(clock, 5000);
				}
				else if (memid == 3)
				{
				}
				else
				{
					success = false;
				}

				com.write(0x80);
				com.write(0x81);
				com.write(!success);
			}
			break;
#if 0
		case 5:
			// Read EEPROM memory
			if (cmd_parser.size() > 2)
			{
				uint16_t address = (cmd_parser[0] << 8) | cmd_parser[1];
				uint8_t size = cmd_parser[2] & 0x3f;
				com.write(0x80);
				com.write(0xf5);
				com.write(size);
				for (; size != 0; --size)
				{
					spi(0xA0);
					spi((address >> 8) & 0x03);
					spi(address);
					com.write(spi(0));
					++address;
				}
				com.write(0);
			}
			break;
		case 9:
			// Write fuses
			if (cmd_parser.size() >= 4)
			{
				static uint8_t const commands[][3] =
				{
					{ 0xAC, 0xE0, 0x00 },
					{ 0xAC, 0xA0, 0x00 },
					{ 0xAC, 0xA8, 0x00 },
					{ 0xAC, 0xA4, 0x00 },
				};
				
				for (uint8_t i = 0; i < 4; ++i)
				{
					spi(commands[i][0]);
					spi(commands[i][1]);
					spi(commands[i][2]);
					spi(cmd_parser[i]);
					flash_wait(cmd_parser.size() > 4 && (cmd_parser[4] & (1<<0)), 5);
				}
				com.write(0x80);
				com.write(0x91);
				com.write(0x00);
			}
			break;
		case 10:
			// Set remote clock and enable/disable the clock output
			if (cmd_parser.size() >= 2)
			{
				remote_clock_khz = cmd_parser[0] | (cmd_parser[1] << 8);
				com.write(0x80);
				com.write(0xa1);
				com.write(0x00);
			}

			if (cmd_parser.size() >= 3)
			{
				switch (cmd_parser[2])
				{
				case 0:
					// Disable
					TCCR0B = 0;
					DDRD &= ~(1<<6);
					PORTD &= ~(1<<6);
					break;
				case 1:
					// Enable
					// TODO: match the speed according to remote_clock_khz
					// Currently generates 1MHz clock
					OCR0A = 7;
					TCCR0A = (1<<COM0A0)|(1<<WGM01)|(1<<WGM00);
					TCCR0B = (1<<WGM02)|(1<<CS00);
					DDRD |= (1<<6);
					PORTD |= (1<<6);
					break;
				}
			}
			break;
#endif
		}
	}

private:
	spi_t & spi;
	com_t & com;
	clock_t & clock;

	bool m_programming_enabled;
	uint16_t m_mempage_ptr;
};

#endif
