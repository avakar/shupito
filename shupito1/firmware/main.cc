#include "../../fw_common/avrlib/async_usart.hpp"
#include "../../fw_common/avrlib/bootseq.hpp"
#include "../../fw_common/avrlib/usart1.hpp"

#include "../../fw_common/avrlib/timer1.hpp"
#include "../../fw_common/avrlib/counter.hpp"
#include "../../fw_common/avrlib/stopwatch.hpp"
#include "../../fw_common/avrlib/format.hpp"

#include "../../fw_common/avrlib/buffer.hpp"
#include "../../fw_common/avrlib/pin.hpp"
#include "../../fw_common/avrlib/portd.hpp"
#include "../../fw_common/avrlib/assert.hpp"

#include "../../fw_common/avrlib/command_parser.hpp"

#include "../../fw_common/handler_base.hpp"
#include "../../fw_common/handler_xmega.hpp"
#include "../../fw_common/handler_avricsp.hpp"

#include "pdi.hpp"
#include "spi.hpp"

typedef avrlib::pin<avrlib::portd, 4> pdi_clk;
typedef avrlib::pin<avrlib::portd, 1> pdi_data;

typedef avrlib::pin<avrlib::portd, 4> avricsp_reset;

typedef avrlib::async_usart<avrlib::usart1, 64, 64, avrlib::nobootseq> com_t;
com_t com(38400, true);

ISR(USART1_RXC_vect)
{
	com.process_rx();
}

struct timer_t
	: avrlib::timer1
{
	template <uint32_t v>
	struct us { static const uint32_t value = v; };
};

typedef avrlib::counter<timer_t> clock_t;
clock_t clock;

ISR(TIMER1_OVF_vect)
{
	clock.tov_interrupt();
}

pdi_t<clock_t, pdi_clk, pdi_data> pdi(clock);

ISR(USART0_RXC_vect)
{
	pdi.intr_rxc();
}

ISR(USART0_UDRE_vect)
{
	pdi.intr_udre();
}

ISR(USART0_TXC_vect)
{
	pdi.intr_txc();
}

spi_t spi;

struct process_t
{
	void operator()() const
	{
		pdi.process();
		com.process_tx();
	}
} process;

void send_dword(uint32_t v)
{
	com.write(v);
	com.write(v >> 8);
	com.write(v >> 16);
	com.write(v >> 24);
}

int main()
{
	sei();

	clock.enable(avrlib::timer_fosc_8);

	avrlib::bootseq bootseq;
	avrlib::command_parser cp;
	cp.clear();

	handler_xmega<pdi_t<clock_t, pdi_clk, pdi_data>, com_t, clock_t, process_t> hxmega(pdi, clock);
	handler_avricsp<spi_t, com_t, clock_t, avricsp_reset, process_t> havricsp(spi, clock);
	handler_base<com_t> * handler = 0;

	for (;;)
	{
		if (!com.empty())
		{
			uint8_t ch = cp.push_data(com.read());
			bootseq.check(ch);

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
							handler_base<com_t> * new_handler = 0;
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
			case 255:
				break;
			default:
				if (ch > 16)
					cp.clear();
				else if (handler)
					handler->handle_command(cp, com);
			}
		}

		if (pdi.rx_ready())
		{
			send_hex(com, pdi.read());
			com.write('\n');
		}

		process();
	}
}
