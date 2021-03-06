#ifndef SHUPITO_HANDLER_AVRICSP_HPP
#define SHUPITO_HANDLER_AVRICSP_HPP

#include "handler_base.hpp"
#include "avrlib/stopwatch.hpp"
#include <avr/io.h>

template <typename Spi, typename Clock, typename ResetPin, typename Process>
class handler_avricsp
	: public handler_base
{
public:
	typedef Spi spi_t;
	typedef Clock clock_t;

	handler_avricsp(spi_t & spi, clock_t & clock, Process process = Process())
		: spi(spi), clock(clock), m_programming_enabled(false), m_process(process)
	{
	}

	void unselect()
	{
		spi.clear();
	}

	bool handle_command(uint8_t cmd, uint8_t const * cp, uint8_t size, com_t & w)
	{
		switch (cmd)
		{
		case 1: // PROGEN 2'bsel
			m_programming_enabled = false;

			{
				typename spi_t::error_t err = spi.start_master(cp[0] | (cp[1] << 8), /*mode=*/0, /*lsb_first=*/false);
				if (err)
				{
					w.send_sync(1, &err, 1);
					break;
				}
			}

			// Pull down the reset line and send "Programming enable" sequence
			ResetPin::make_low();
			avrlib::wait(clock, Clock::template us<1000>::value, m_process);

			ResetPin::set_high();
			for (uint8_t i = 0; i < 3; ++i)
			{
				avrlib::wait(clock, Clock::template us<1000>::value, m_process);
				ResetPin::set_low();

				// There has to be a 20ms delay on atmega128
				avrlib::wait(clock, Clock::template us<20000>::value, m_process);

				spi.send(0xAC);
				spi.send(0x53);
				uint8_t echo = spi.send(0x00);
				spi.send(0x00);

				if (echo == 0x53)
				{
					m_programming_enabled = true;
					break;
				}

				ResetPin::set_high();
			}

			if (!m_programming_enabled)
			{
				spi.clear();
				ResetPin::make_input();
			}

			{
				uint8_t err = !m_programming_enabled;
				w.send_sync(1, &err, 1);
			}
			break;
		case 2:
			// Release the reset line
			spi.clear();
			ResetPin::make_input();
			m_programming_enabled = false;

			{
				uint8_t err = 0;
				w.send_sync(2, &err, 1);
			}
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

				uint8_t * wbuf = w.alloc(3, command_count + 1);
				if (!wbuf)
					return false;

				for (uint8_t i = 0; i < command_count; ++i)
				{
					for (uint8_t j = 0; j < 3; ++j)
						spi.send(commands[i][j]);
					*wbuf++ = spi.send(0);
				}

				*wbuf++ = 0;
				w.commit();
			}
			break;
		case 4: // READ 1'memid 4'addr 2'size
			{
				uint8_t memid = cp[0];
				switch (memid)
				{
				case 1:
					{
						uint32_t addr = cp[1] | ((uint32_t)cp[2] << 8) | ((uint32_t)cp[3] << 16) | ((uint32_t)cp[4] << 24);
						uint16_t size = cp[5] | ((uint32_t)cp[6] << 8);

						for (;;)
						{
							uint8_t chunk = size > w.max_packet_size()? w.max_packet_size(): size;

							uint8_t * wbuf = w.alloc_sync(4, chunk);
							for (uint8_t i = chunk; i != 0; --i, ++addr)
							{
								// program memory words are sent in the little endian order
								spi.send(addr & 1? 0x28: 0x20);
								spi.send(addr >> 9);
								m_process();
								spi.send(addr >> 1);
								*wbuf++ = spi.send(0);
								m_process();
							}
							w.commit();

							size -= chunk;
							if (chunk < w.max_packet_size())
								break;
						}
					}
					break;
				case 2: // EEPROM
					{
						uint16_t addr = cp[1] | (cp[2] << 8);
						uint16_t size = cp[5] | (cp[6] << 8);
						for (;;)
						{
							uint8_t chunk = size > w.max_packet_size()? w.max_packet_size(): size;

							uint8_t * wbuf = w.alloc_sync(4, chunk);
							for (uint8_t i = chunk; i != 0; --i, ++addr)
							{
								spi.send(0xa0);
								spi.send(addr >> 8);
								spi.send(addr);
								*wbuf++ = spi.send(0);
								m_process();
							}
							w.commit();

							size -= chunk;
							if (chunk < w.max_packet_size())
								break;
						}
					}
					break;
				case 3: // FUSES
					{
						static uint8_t const commands[][3] =
						{
							{ 0x58, 0x00, 0x00 },
							{ 0x50, 0x00, 0x00 },
							{ 0x58, 0x08, 0x00 },
							{ 0x50, 0x08, 0x00 },
						};

						uint8_t addr = cp[1];
						uint8_t size = cp[5];

						uint8_t * wbuf = w.alloc_sync(4, 4);
						while (size && addr < 4)
						{
							for (uint8_t j = 0; j < 3; ++j)
								spi.send(commands[addr][j]);
							*wbuf++ = spi.send(0);

							++addr;
							--size;
						}
						
						w.commit();
					}
					break;
				}
			}
			break;
		case 5: // ERASE [1'memid]
			if (size == 0 || (size == 1 && cp[0] == 1))
			{
				spi.send(0xAC);
				spi.send(0x80);
				spi.send(0);
				spi.send(0);

				avrlib::wait(clock, Clock::template us<100000>::value);
			}

			{
				uint8_t err = 0;
				w.send_sync(5, &err, 1);
			}
			break;
		case 6:
			// WPREP 1'memid 4'addr
			if (size >= 5)
			{
				uint8_t memid = cp[0];

				bool success = true;
				if (memid == 1 || memid == 2)
				{
					m_mempage_ptr = (cp[1]) | (cp[2] << 8);
					// TODO: potentially load the extended address byte
				}
				else if (memid == 3) // fuses
				{
					m_mempage_ptr = cp[1];
				}
				else
				{
					success = false;
				}

				{
					uint8_t err = !success;
					w.send_sync(6, &err, 1);
				}
			}
			break;
		case 7:
			// WFILL 1'memid (1'data)*
			if (size >= 1)
			{
				bool success = true;

				uint8_t memid = cp[0];
				if (memid == 1)
				{
					for (uint8_t i = 1; i < size; ++i)
					{
						spi.send(m_mempage_ptr & 1? 0x48: 0x40);
						spi.send(0x00);
						spi.send(m_mempage_ptr >> 1);
						spi.send(cp[i]);
						++m_mempage_ptr;
					}
				}
				else if (memid == 2) // eeprom
				{
					for (uint8_t i = 1; i < size; ++i)
					{
						spi.send(0xc0);
						spi.send(m_mempage_ptr >> 8);
						spi.send(m_mempage_ptr);
						spi.send(cp[i]);
						++m_mempage_ptr;
						avrlib::wait(clock, Clock::template us<10000>::value);
					}
				}
				else if (memid == 3) // fuses
				{
					static uint8_t cmds[4] = { 0xE0, 0xA0, 0xA8, 0xA4 };

					for (uint8_t i = 1; i < size; ++i)
					{
						spi.send(0xAC);
						spi.send(cmds[m_mempage_ptr++ & 0x3]);
						spi.send(0x00);
						spi.send(cp[i]);
						avrlib::wait(clock, Clock::template us<5000>::value);
					}
				}
				else
				{
					success = false;
				}

				{
					uint8_t err = !success;
					w.send_sync(7, &err, 1);
				}
			}
			break;
		case 8:
			// WRITE 1'memid 4'addr
			if (size == 5)
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
				
					avrlib::wait(clock, Clock::template us<5000>::value);
				}
				else if (memid == 2 || memid == 3)
				{
				}
				else
				{
					success = false;
				}

				{
					uint8_t err = !success;
					w.send_sync(8, &err, 1);
				}
			}
			break;
		default:
			return false;
		}

		return true;
	}

private:
	spi_t & spi;
	clock_t & clock;

	bool m_programming_enabled;
	uint16_t m_mempage_ptr;
	Process m_process;
};

#endif
