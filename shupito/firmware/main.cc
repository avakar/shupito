#include "avrlib/async_usart.hpp"
#include "avrlib/usart1.hpp"

#include "avrlib/timer1.hpp"
#include "avrlib/counter.hpp"
#include "avrlib/stopwatch.hpp"
#include "avrlib/format.hpp"

#include "avrlib/buffer.hpp"
#include "avrlib/pin.hpp"
#include "avrlib/portd.hpp"
#include "avrlib/assert.hpp"

#include "avrlib/command_parser.hpp"

#include "handler_base.hpp"
#include "handler_xmega.hpp"

#include "pdi.hpp"

typedef avrlib::pin<avrlib::portd, 4> pdi_clk;
typedef avrlib::pin<avrlib::portd, 1> pdi_data;

typedef avrlib::async_usart<avrlib::usart1, 64, 64, avrlib::bootseq> com_t;
com_t com(38400, true);

ISR(USART1_RXC_vect)
{
	com.process_rx();
}

typedef avrlib::counter<avrlib::timer1> clock_t;
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

void process()
{
	pdi.process();
	com.process_tx();
}

int main()
{
	sei();

	clock.enable(avrlib::timer_fosc_8);

	avrlib::command_parser cp;
	cp.clear();

	handler_xmega<pdi_t<clock_t, pdi_clk, pdi_data>, com_t, clock_t> hxmega(pdi, com, clock);
	handler_base * handler = &hxmega;

	for (;;)
	{
		if (!com.empty())
		{
			uint8_t ch = cp.push_data(com.read());
			switch (ch)
			{
			case 0:
				// Send out the identification
				com.write(0x80);
				com.write(0x04);
				com.write(0xbd);
				com.write(0xe9);
				com.write(0x9f);
				com.write(0xe9);
				break;
			case 255:
			case 254:
				break;
			default:
				handler->handle_command(cp);
			}

			if (cp.state() == avrlib::command_parser::simple_command || cp.state() == avrlib::command_parser::bad)
				cp.clear();
		}

		if (pdi.rx_ready())
		{
			send_hex(com, pdi.read());
			com.write('\n');
		}

		process();
	}
}
