#include <avr/io.h>
#include <util/delay.h>

#include <avr/pgmspace.h>

#include "../../fw_common/avrlib/async_usart.hpp"
#include "../../fw_common/avrlib/usart1.hpp"
#include "../../fw_common/avrlib/format.hpp"
#include "../../fw_common/avrlib/bootseq.hpp"
#include "../../fw_common/avrlib/counter.hpp"
#include "../../fw_common/avrlib/timer1.hpp"

typedef avrlib::async_usart<avrlib::usart1, 64, 64, avrlib::nobootseq> com_t;
com_t com;
ISR(USART1_RX_vect) { com.process_rx(); }

/*typedef avrlib::counter<avrlib::timer1> clock_t;
clock_t clock;
ISR(TIMER1_OVF_vect) { clock.tov_interrupt(); }*/

#define EPTYPE_CONTROL 0
#define EPTYPE_ISOCHRONOUS 1
#define EPTYPE_BULK 2
#define EPTYPE_INTERRUPT 3

#define EPDIR_OUT 0
#define EPDIR_IN 1

#define EPSIZE_8  0
#define EPSIZE_16 1
#define EPSIZE_32 2
#define EPSIZE_64 3

#define EPBK_ONE 0
#define EPBK_TWO 1


enum usb_request_types {
	urt_get_status = 0,
	urt_set_address = 5,
	urt_get_descriptor = 6,
	urt_set_configuration = 9,
};

#include "usb_descriptors.h"

template <typename InStream>
class usb_t
{
public:
	explicit usb_t(InStream & in)
		: m_in(in), m_resets(0), m_sofs(0), m_enabled(false), m_enable_bootloader(true), last_intr(0), last_i1(0), last_i2(0xcc),
		m_out_packet_pending(false)
	{
		m_ep0_transaction.m_request = 0xff;
	}

	void init()
	{
		USBCON |= (1<<USBE);

		// Initialize PLL and wait for it to become locked
		PLLCSR = (1<<PLLE)|(1<<PLLP0);
		while ((PLLCSR & (1<<PLOCK)) == 0)
		{
		}

		cli();

		USBCON &= ~(1<<FRZCLK);

		configure_ep0();

		// Attach the pull-up
		UDCON = 0;
		UDINT = 0;

		sei();

		m_enabled = true;
	}

	void process()
	{
		if (!m_enabled)
			return;

		if (UDINT & (1<<EORSTI))
		{
			UDINT = ~(1<<EORSTI);
			configure_ep0();
			//send(com, "RESET\r\n");
		}

		UENUM = 0;
		if (UEINTX & (1<<RXSTPI))
		{
			uint8_t bmRequestType = UEDATX;
			uint8_t bRequest = UEDATX;

			uint16_t wValue = UEDATX;
			wValue |= UEDATX << 8;

			uint16_t wIndex = UEDATX;
			wIndex |= UEDATX << 8;

			uint16_t wLength = UEDATX;
			wLength |= UEDATX << 8;

			UEINTX = ~(1<<RXSTPI);

			ep0_setup(bmRequestType, bRequest, wValue, wIndex, wLength);
		}

		if (m_ep0_transaction.m_request != 0xff && (UEINTX & (1<<TXINI)) != 0)
		{
			ep0_in_ack();
		}

		if (UEINTX & (1<<RXOUTI))
		{
			ep0_out();
		}

		UENUM = 1;

		if (UEINTX & (1<<NAKINI))
		{
			UEINTX = ~(1<<NAKINI);
		}


		UENUM = 3;

		UEINTX = ~(1<<NAKOUTI);
		
		if (!m_out_packet_pending && (UEINTX & (1<<RXOUTI)) != 0)
		{
			UEINTX = ~(1<<RXOUTI);
			m_out_packet_pending = true;
		}

		if (m_out_packet_pending)
		{
			while (com.tx_ready() && (UEINTX & (1<<RWAL)) != 0)
				com.write(UEDATX);

			if ((UEINTX & (1<<RWAL)) == 0)
			{
				UEINTX = (uint8_t)~(1<<FIFOCON);
				m_out_packet_pending = false;
			}
		}

		UENUM = 4;
		if (!m_in.empty() && UEINTX & (1<<TXINI))
		{
			UEINTX = (uint8_t)~((1<<TXINI)|(1<<RXOUTI));
			while (!m_in.empty() && (UEINTX & (1<<RWAL)) != 0)
			{
				uint8_t ch = m_in.read();
				if (m_enable_bootloader)
					m_bootseq.check(ch);
				UEDATX = ch;
			}
			UEINTX = (uint8_t)~((1<<FIFOCON)|(1<<RXOUTI));
		}
	}

	InStream & m_in;

	uint16_t m_resets, m_sofs;

private:
	bool m_enabled, m_enable_bootloader;
	uint8_t last_intr, last_i1, last_i2;
	bool m_out_packet_pending;
	avrlib::bootseq m_bootseq;

	void configure_ep0()
	{
		UENUM = 0;

		// XXX: The endpoint 0 is not automatically enabled after USB reset as specificed in the datasheet.
		UECONX = (1<<EPEN);

		// The endpoint 0 should already be enabled.
		UECFG0X = (EPTYPE_CONTROL<<6)|(EPDIR_OUT<<0);
		UECFG1X = (EPSIZE_32<<4)|(1<<ALLOC);
	}

	void ep0_setup(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength)
	{
		m_ep0_transaction.m_request = bRequest;

		switch (bRequest)
		{
		case urt_get_status:
			UEDATX = 0;
			UEDATX = 0;
			UEINTX = ~(1<<TXINI);
			break;
		case urt_set_address:
			UDADDR = wValue & 0x7f;
			UEINTX = ~(1<<TXINI);
			break;
		case urt_get_descriptor:
			{
				m_ep0_transaction.m_get_descriptor.pFirst = 0;
				m_ep0_transaction.m_get_descriptor.pLast = 0;

				for (uint8_t i = 0; i < sizeof usb_descriptors / sizeof usb_descriptors[0]; ++i)
				{
					if (usb_descriptor_map[i].index == wValue)
					{
						m_ep0_transaction.m_get_descriptor.pFirst = usb_descriptors + usb_descriptor_map[i].first;
						m_ep0_transaction.m_get_descriptor.pLast = usb_descriptors + usb_descriptor_map[i].last;
						break;
					}
				}

				if (m_ep0_transaction.m_get_descriptor.pFirst == 0)
				{
					UECONX = (1<<STALLRQ)|(1<<EPEN);
					m_ep0_transaction.m_request = 0xff;
					return;
				}

				// Make sure that at most the requested amount of data gets transmitted.
				if (uint16_t(m_ep0_transaction.m_get_descriptor.pLast - m_ep0_transaction.m_get_descriptor.pFirst) > wLength)
					m_ep0_transaction.m_get_descriptor.pLast = m_ep0_transaction.m_get_descriptor.pFirst + wLength;
			}

			ep0_in_ack();
			break;

		case urt_set_configuration:
			set_config();
			UEINTX = ~(1<<TXINI);
			break;

		case 0x20: // SET_LINE_CODING
			//send(com, "set_line_coding setup\r\n");
			break;
		
		case 0x22: // SET_CONTROL_LINE_STATE
			UEINTX = ~(1<<TXINI);
			break;

		default:
			//format(com, "SETUP %x:%x %x:%x %x\r\n") % bmRequestType % bRequest % wValue % wIndex % wLength;
			UECONX = (1<<STALLRQ)|(1<<EPEN);
			m_ep0_transaction.m_request = 0xff;
		}
	}

	void ep0_out()
	{
		//send(com, "out\r\n");
		switch (m_ep0_transaction.m_request)
		{
		case 0x20: // SET_LINE_CODING
			{
				uint32_t dte_rate = UEDATX;
				dte_rate |= uint32_t(UEDATX) << 8;
				dte_rate |= uint32_t(UEDATX) << 16;
				dte_rate |= uint32_t(UEDATX) << 24;

				m_enable_bootloader = dte_rate == 1234;

				//format(com, "set_line_conding %x\r\n") % dte_rate;
			}
			UEINTX = ~(1<<RXOUTI);

			m_ep0_transaction.m_request = 0xff;
			UEINTX = ~(1<<TXINI);
			break;
		default:
			UEINTX = ~(1<<RXOUTI);
		}
	}

	void ep0_in_ack()
	{
		switch (m_ep0_transaction.m_request)
		{
		case urt_get_descriptor:
			{
				uint8_t packet_size = 0;
				for (; packet_size < 32
					&& m_ep0_transaction.m_get_descriptor.pFirst != m_ep0_transaction.m_get_descriptor.pLast;
					++m_ep0_transaction.m_get_descriptor.pFirst)
				{
					UEDATX = pgm_read_byte(m_ep0_transaction.m_get_descriptor.pFirst);
					++packet_size;
				}

				UEINTX = ~(1<<TXINI);

				if (packet_size < 32)
					m_ep0_transaction.m_request = 0xff;
			}
			break;

		case urt_set_address:
			UDADDR |= (1<<ADDEN);
			m_ep0_transaction.m_request = 0xff;
			break;

		case 0x20:
			break;

		default:
			m_ep0_transaction.m_request = 0xff;
		}
	}

	void set_config()
	{
		//send(com, "set_config\r\n");

		UERST = (1<<1)|(1<<3)|(1<<4);
		UERST = 0;

		UENUM = 1;
		UECONX = (1<<RSTDT)|(1<<EPEN);
		UECFG0X = (EPTYPE_INTERRUPT<<6)|(EPDIR_IN<<0);
		UECFG1X = (EPSIZE_8<<4)|(1<<ALLOC);

		UENUM = 3;
		UECONX = (1<<RSTDT)|(1<<EPEN);
		UECFG0X = (EPTYPE_BULK<<6)|(EPDIR_OUT<<0);
		UECFG1X = (EPSIZE_64<<4)|(1<<ALLOC);


		UENUM = 4;
		UECONX = (1<<RSTDT)|(1<<EPEN);
		UECFG0X = (EPTYPE_BULK<<6)|(EPDIR_IN<<0);
		UECFG1X = (EPSIZE_32<<4)|(EPBK_TWO<<2)|(1<<ALLOC);

		UENUM = 0;
	}

	struct ep0_transaction_t
	{
		uint8_t m_request;
		union
		{
			struct
			{
				prog_uint8_t const * pFirst;
				prog_uint8_t const * pLast;
			} m_get_descriptor;
		};
	} m_ep0_transaction;
};

int main()
{
	REGCR = (1<<REGDIS);

	sei();

	//clock.enable(avrlib::timer_fosc_8);

	com.usart().open_sync_slave(true);
	UCSR1D = (1<<CTSEN)|(1<<RTSEN);

	usb_t<com_t> usb(com);
	usb.init();

	for (;;)
	{
		/*if (!com.empty())
		{
			char ch = com.read();
			switch (ch)
			{
			case 't':
				avrlib::format(com, "clock: %x\n") % clock.value();
				break;
			case 'x':
				avrlib::format(com,
					"CLKSEL0: %x\n"
					"CLKSEL1: %x\n"
					"CLKSTA: %x\n"
					"CLKPR: %x\n"
					"PLLCSR: %x\n"
					) % CLKSEL0 % CLKSEL1 % CLKSTA % CLKPR % PLLCSR;
				break;
			case '?':
			default:
				{
					send(com, "Shupito USB comm chip.\r\n");

					for (uint8_t i = 0; i < 5; ++i)
					{
						UENUM = i;
						static prog_char const fmt[] PROGMEM =
							"----\r\nUENUM: %x\r\n"
							"UECONX: %x\r\n"
							"UECFG0X: %x\r\n"
							"UECFG1X: %x\r\n"
							"UESTA0X: %x\r\n"
							"UESTA1X: %x\r\n"
							"UEINTX: %x\r\n"
							"UEBCLX: %x\r\n";
						avrlib::format_pgm(com,fmt) % i % UECONX % UECFG0X % UECFG1X % UESTA0X % UESTA1X % UEINTX % UEBCLX;
					}
				}
			}
		}*/

		com.process_tx();
		usb.process();
	}
}
