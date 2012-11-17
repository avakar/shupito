#include <avr/io.h>
#include <avr/pgmspace.h>

#include "../../fw_common/avrlib/buffer.hpp"
#include "../../fw_common/avrlib/bootseq.hpp"
#include "../../fw_common/avrlib/usart1.hpp"
#include "../../fw_common/avrlib/pin.hpp"
#include "../../fw_common/avrlib/portd.hpp"
#include "../../fw_common/avrlib/hwflow_usart.hpp"
//#include "../../fw_common/avrlib/format.hpp"

#include "../../fw_common/avrlib/timer1.hpp"
typedef avrlib::timer1 clock_t;

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
	urt_no_request             = 0,
	urt_get_status             = 0x8000,
	urt_set_address            = 0x0005,
	urt_get_descriptor         = 0x8006,
	urt_set_configuration      = 0x0009,
	urt_set_line_conding       = 0x2120,
	urt_get_line_conding       = 0xA121,
	urt_set_control_line_state = 0x2122,
	urt_reset_to_booloader     = 0x4001,
};

#include "usb_descriptors.h"

typedef avrlib::pin<avrlib::portd, 2> pin_usb_rx;
typedef avrlib::pin<avrlib::portd, 3> pin_usb_tx;
typedef avrlib::pin<avrlib::portd, 5> pin_usb_xck;
typedef avrlib::pin<avrlib::portd, 6> pin_rtr_n;
typedef avrlib::pin<avrlib::portd, 7> pin_cts_n;
avrlib::hwflow_usart<avrlib::usart1, 128, 64, avrlib::intr_enabled, pin_rtr_n, pin_cts_n> com;

uint8_t g_serial_no_len;
uint8_t g_serial_no[14];

avrlib::bootseq g_bootseq;
bool g_enable_bootloader = true;


ISR(USART1_RX_vect)
{
	com.intr_rx();
}

class usb_t
{
public:
	explicit usb_t()
		: m_resets(0), m_sofs(0), m_enabled(false), last_intr(0), last_i1(0), last_i2(0xcc),
		m_out_packet_pending(false), m_dte_rate(1234)
	{
		ep0_tran.request = urt_no_request;
	}

	void init()
	{
		USBCON |= (1<<USBE);

		// Initialize PLL and wait for it to become locked
		PLLCSR = (1<<PLLE);
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

			ep0_tran.request = (bmRequestType << 8) | bRequest;
			ep0_tran.wValue = wValue;
			ep0_tran.wIndex = wIndex;
			ep0_tran.wLength = wLength;
			ep0_setup();
		}

		if (ep0_tran.request != urt_no_request && (UEINTX & (1<<TXINI)) != 0)
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
			for (;;)
			{
				bool usb_empty = (UEINTX & (1<<RWAL)) == 0;
				if (!usb_empty && com.tx_ready())
				{
					com.write(UEDATX);
					continue;
				}

				if (usb_empty)
				{
					UEINTX = (uint8_t)~(1<<FIFOCON);
					m_out_packet_pending = false;
				}

				break;
			}
		}

		UENUM = 4;
		if (!com.empty() && (UEINTX & (1<<TXINI)) != 0)
		{
			UEINTX = (uint8_t)~((1<<TXINI)|(1<<RXOUTI));
			while ((UEINTX & (1<<RWAL)) != 0 && !com.empty())
			{
				uint8_t ch = com.read();
				if (g_enable_bootloader)
					g_bootseq.check(ch);
				UEDATX = ch;
			}
			UEINTX = (uint8_t)~((1<<FIFOCON)|(1<<RXOUTI));
		}
	}

	uint16_t m_resets, m_sofs;

private:
	bool m_enabled;
	uint8_t last_intr, last_i1, last_i2;
	bool m_out_packet_pending;
	uint32_t m_dte_rate;

	void configure_ep0()
	{
		UENUM = 0;

		// The endpoint 0 is not automatically enabled after USB reset as specified in the datasheet!
		UECONX = (1<<EPEN);

		// The endpoint 0 should already be enabled.
		UECFG0X = (EPTYPE_CONTROL<<6)|(EPDIR_OUT<<0);
		UECFG1X = (EPSIZE_32<<4)|(1<<ALLOC);
	}

	void ep0_setup()
	{
		//format(com, "SETUP %x %x:%x %x\r\n") % ep0_tran.request % ep0_tran.wValue % ep0_tran.wIndex % ep0_tran.wLength;

		switch (ep0_tran.request)
		{
		case urt_get_status:
			UEDATX = 0;
			UEDATX = 0;
			UEINTX = ~(1<<TXINI);
			break;
		case urt_set_address:
			UDADDR = ep0_tran.wValue & 0x7f;
			UEINTX = ~(1<<TXINI);
			break;
		case urt_get_descriptor:
			if (ep0_tran.wValue == 0x302)
			{
				ep0_tran.get_desc_302.pos = 2;
				ep0_tran.get_desc_302.length = 4*g_serial_no_len + 2;
				UEDATX = ep0_tran.get_desc_302.length;
				UEDATX = 3;

				if (ep0_tran.wLength < ep0_tran.get_desc_302.length)
					ep0_tran.get_desc_302.length = ep0_tran.wLength;
			}
			else
			{
				ep0_tran.get_desc.first = 0;
				ep0_tran.get_desc.last = 0;

				for (uint8_t i = 0; i < sizeof usb_descriptors / sizeof usb_descriptors[0]; ++i)
				{
					if (usb_descriptor_map[i].index == ep0_tran.wValue)
					{
						ep0_tran.get_desc.first = usb_descriptors + usb_descriptor_map[i].first;
						ep0_tran.get_desc.last = usb_descriptors + usb_descriptor_map[i].last;
						break;
					}
				}

				if (ep0_tran.get_desc.first == 0)
				{
					UECONX = (1<<STALLRQ)|(1<<EPEN);
					ep0_tran.request = urt_no_request;
					return;
				}

				// Make sure that at most the requested amount of data gets transmitted.
				if (uint16_t(ep0_tran.get_desc.last - ep0_tran.get_desc.first) > ep0_tran.wLength)
					ep0_tran.get_desc.last = ep0_tran.get_desc.first + ep0_tran.wLength;
			}
			break;

		case urt_set_configuration:
			set_config();
			UEINTX = ~(1<<TXINI);
			break;

		case urt_set_line_conding:
			break;

		case urt_get_line_conding:
			UEDATX = m_dte_rate;
			UEDATX = m_dte_rate >> 8;
			UEDATX = m_dte_rate >> 16;
			UEDATX = m_dte_rate >> 24;
			UEDATX = 0;
			UEDATX = 0;
			UEDATX = 8;
			UEINTX = ~(1<<TXINI);
			break;

		case urt_set_control_line_state:
			UEINTX = ~(1<<TXINI);
			break;

		case urt_reset_to_booloader:
			avrlib::bootseq_reset();
			break;

		default:
			UECONX = (1<<STALLRQ)|(1<<EPEN);
			ep0_tran.request = 0;
		}
	}

	void ep0_out()
	{
		switch (ep0_tran.request)
		{
		case urt_set_line_conding:
			{
				m_dte_rate = UEDATX;
				m_dte_rate |= uint32_t(UEDATX) << 8;
				m_dte_rate |= uint32_t(UEDATX) << 16;
				m_dte_rate |= uint32_t(UEDATX) << 24;

				if (m_dte_rate == 0x337A7E74)
					avrlib::bootseq_reset();

				g_enable_bootloader = (m_dte_rate == 1234);
			}
			UEINTX = ~(1<<RXOUTI);

			ep0_tran.request = urt_no_request;
			UEINTX = ~(1<<TXINI);
			break;
		default:
			UEINTX = ~(1<<RXOUTI);
		}
	}

	void ep0_in_ack()
	{
		switch (ep0_tran.request)
		{
		case urt_get_descriptor:
			uint8_t buf_left;
			if (ep0_tran.wValue == 0x302)
			{
				uint8_t pos = ep0_tran.get_desc_302.pos;
				buf_left = 32 - (pos % 32);

				for (; pos < ep0_tran.get_desc_302.length && buf_left; ++pos, --buf_left)
				{
					uint8_t pos2 = pos - 2;
					if (pos2 & 1)
					{
						UEDATX = 0;
					}
					else
					{
						static const uint8_t digits[] = "0123456789abcdef";
						uint8_t val = g_serial_no[pos2/4];
						UEDATX = digits[(pos2 & 2)?(val & 0xf):(val >> 4)];
					}
				}
				ep0_tran.get_desc_302.pos = pos;
			}
			else 
			{
				buf_left = 32;
				
				uint8_t const * pos = ep0_tran.get_desc.first;
				uint8_t const * last = ep0_tran.get_desc.last;
				for (; pos != last && buf_left; ++pos, --buf_left)
				{
					UEDATX = pgm_read_byte(pos);
				}
				ep0_tran.get_desc.first = pos;
			}

			if (buf_left)
				ep0_tran.request = urt_no_request;
			UEINTX = ~(1<<TXINI);
			break;

		case urt_set_address:
			UDADDR |= (1<<ADDEN);
			ep0_tran.request = urt_no_request;
			break;

		case urt_set_line_conding:
			break;

		case urt_get_line_conding:
		default:
			ep0_tran.request = urt_no_request;
		}
	}

	void set_config()
	{
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
		uint16_t request;
		uint16_t wIndex;
		uint16_t wValue;
		uint16_t wLength;

		union
		{
			struct
			{
				uint8_t pos;
				uint8_t length;
			} get_desc_302;
			struct
			{
				uint8_t const * first;
				uint8_t const * last;
			} get_desc;
		};
	} ep0_tran;
};

usb_t usb;

int main()
{
	REGCR = (1<<REGDIS);

	sei();

	pin_usb_rx::pullup();
	pin_usb_xck::pullup();
	pin_cts_n::pullup();
	com.usart().open_sync_slave(true);
	pin_rtr_n::make_low();

	// Wait a little bit for the master to send a config packet
	// with our serial number.
	clock_t::value(0);
	clock_t::clock_source(avrlib::timer_fosc_256);

	static uint16_t const config_timeout = 12500;
	bool wait_for_more_config = true;

	uint8_t serial_no_len = 0xff;
	uint8_t serial_no_pos = 0;
	while (wait_for_more_config)
	{
		while (clock_t::value() < config_timeout && com.read_size() < 2)
		{
		}

		if (com.read_size() < 2 || com[0] != 0x80 || (com[1] & 0xf0) != 0xe0)
			break;

		uint8_t size = 2 + (com[1] & 0xf);
		while (clock_t::value() < config_timeout && com.read_size() < size)
		{
		}

		if (com.read_size() < size)
			break;

		com.read();
		com.read();

		uint8_t i = 2;
		if (serial_no_len == 0xff)
		{
			if (size < 3)
				break;
			serial_no_len = com.read();
			++i;
			if (serial_no_len > sizeof(g_serial_no))
				break;
		}

		for (; i < size && serial_no_pos < serial_no_len; ++i, ++serial_no_pos)
			g_serial_no[serial_no_pos] = com.read();

		wait_for_more_config = (size == 17);
	}

	if (!wait_for_more_config)
		g_serial_no_len = serial_no_pos;

	usb.init();

	for (;;)
	{
		usb.process();
		com.process_tx();
	}
}
