#include <avr/io.h>
#include <util/delay.h>

#include <avr/pgmspace.h>

#include "avrlib/async_usart.hpp"
#include "avrlib/usart1.hpp"
#include "avrlib/format.hpp"

typedef avrlib::async_usart<avrlib::usart1, 64, 64, avrlib::bootseq> com_t;
com_t com(38400, true);

ISR(USART1_RX_vect)
{
	com.process_rx();
}

#define EPTYPE_CONTROL 0
#define EPTYPE_ISOCHRONOUS 1
#define EPTYPE_BULK 2
#define EPTYPE_INTERRUPT 3

#define EPDIR_OUT 0
#define EPDIR_IN 1

#define EPSIZE_64 3

enum usb_request_types {
	urt_set_address = 5,
	urt_get_descriptor = 6,
	urt_set_configuration = 9,
};

enum usb_descriptor_types {
	udt_device_descriptor = 1,
	udt_configuration_descriptor = 2,
	udt_string_descriptor = 3,
	udt_interface_descriptor = 4,
	udt_endpoint_descriptor = 5,
};

struct usb_device_desc_t
{
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t bcdUSB;
	uint8_t bDeviceClass;
	uint8_t bDeviceSubClass;
	uint8_t bDeviceProtocol;
	uint8_t bMaxPacketSize0;
	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t bcdDevice;
	uint8_t iManufacturer;
	uint8_t iProduct;
	uint8_t iSerialNumber;
	uint8_t bNumConfigurations;
};

usb_device_desc_t const my_device_desc PROGMEM = {
	sizeof my_device_desc,
	udt_device_descriptor,
	0x0010,
	0xFF,
	0xFF,
	0xFF,
	64,
	0x12,
	0x34,
	0x5678,
	5,
	1,
	2,
	1,
};

struct usb_config_desc_t
{
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t wTotalLength;
	uint8_t bNumInterfaces;
	uint8_t bConfigurationValue;
	uint8_t iConfiguration;
	uint8_t bmAttributes;
	uint8_t bMaxPower;
};

struct usb_interface_desc_t
{
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bInterfaceNumber;
	uint8_t bAlternateSettings;
	uint8_t bNumEndpoints;
	uint8_t bInterfaceClass;
	uint8_t bInterfaceSubClass;
	uint8_t bInterfaceProtocol;
	uint8_t iInterface;
};

struct usb_endpoint_desc_t
{
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
};

struct
{
	usb_config_desc_t config;
	usb_interface_desc_t cdc_comm_intf;
	usb_interface_desc_t cdc_data_intf;
	usb_endpoint_desc_t cdc_data_in_endp;
	usb_endpoint_desc_t cdc_data_out_endp;
} __attribute__((packed)) const PROGMEM config0_desc = {
	{
		sizeof config0_desc.config,
		udt_configuration_descriptor,
		sizeof config0_desc,
		1,
		1,
		3,
		0x80,
		50,
	},
	{
		sizeof config0_desc.cdc_comm_intf,
		udt_interface_descriptor,
		0,
		0,
		0, // bNumEndpoints
		0x02, // Communications Interface Class
		0x02, // Abstract Control Model
		0x00, // No protocol; just raw data
		4,
	},
	{
		sizeof config0_desc.cdc_data_intf,
		udt_interface_descriptor,
		1,
		0,
		2, // bNumEndpoints
		0x0A, // Data Interface Class
		0x00,
		0x00,
		4,
	},
	{
		sizeof config0_desc.cdc_data_in_endp,
		udt_endpoint_descriptor,
		0x80 | 1, // bEndpointNumber
		2, // bmAttributes = BULK
		64, // wMaxPacketSize
		1, // bInterval
	},
	{
		sizeof config0_desc.cdc_data_out_endp,
		udt_endpoint_descriptor,
		2, // bEndpointNumber
		2, // bmAttributes = BULK
		64, // wMaxPacketSize
		1, // bInterval
	},
};

static uint16_t PROGMEM const languages_desc[] = { (udt_string_descriptor << 8) | 4,
	0x0409 };
static uint16_t PROGMEM const manufacturer_string[] = { (udt_string_descriptor << 8) | 6,
	'm', 'a' };
static uint16_t PROGMEM const product_string[] = { (udt_string_descriptor << 8) | 6,
	'p', 'r' };
static uint16_t PROGMEM const serial_number_string[] = { (udt_string_descriptor << 8) | 6,
	's', 'n' };
static uint16_t PROGMEM const interface_string[] = { (udt_string_descriptor << 8) | 6,
	'i', 'n' };

static struct {
	uint16_t index;
	uint8_t PROGMEM const * first;
	uint8_t PROGMEM const * last;
} const usb_descriptors[] = {
#define DESC(index, value) { index, reinterpret_cast<uint8_t PROGMEM const *>(&value), reinterpret_cast<uint8_t PROGMEM const *>(&value) + sizeof value }
	DESC(0x100, my_device_desc),
	DESC(0x200, config0_desc),
	DESC(0x300, languages_desc),
	DESC(0x301, manufacturer_string),
	DESC(0x302, product_string),
	DESC(0x303, serial_number_string),
	DESC(0x304, interface_string),
};

class usb_t
{
public:
	usb_t()
		: m_resets(0), m_sofs(0), m_enabled(false), last_intr(0), last_i(0)
	{
		m_ep0_transaction.m_request = 0xff;
	}

	void init()
	{
		USBCON |= (1<<USBE);

		// Initialize PLL and wait for it to become locked
		PLLCSR = (1<<PLLP0)|(1<<PLLE);
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

		/*if (UDINT & (1<<WAKEUPI))
		{
			UDINT = ~(1<<WAKEUPI);
		}*/

		UDINT = ~((1<<WAKEUPI)|(1<<SUSPI));

		if (UDINT & (1<<SOFI))
		{
			UDINT = ~(1<<SOFI);
			++m_sofs;
		}

		if (UDINT & (1<<EORSTI))
		{
			UDINT = ~(1<<EORSTI);
			configure_ep0();
			UERST = 1;
			UERST = 0;
			++m_resets;
		}

		UENUM = 0;
		/*uint8_t capture = UEINTX;
		if (last_intr != capture)
		{
			last_intr = capture;
			com.write('I');
			send_hex(com, capture);
			send(com, "\r\n");
		}*/

		if (UEINTX & (1<<RXSTPI))
		{
			if (UEBCLX == 8)
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
			else
			{
				// This shouldn't happen; the setup packet failed to be 8 bytes long
				// and is therefore malformed. It should be ignored.
				UEINTX = ~(1<<RXSTPI);
			}

		}

		if (m_ep0_transaction.m_request != 0xff && (UEINTX & (1<<TXINI)) != 0)
		{
			ep0_in_ack();
		}

		if (UEINTX & (1<<RXOUTI))
		{
			ep0_out();
			UEINTX = ~(1<<RXOUTI);
		}

		UENUM = 2;
		if (UEINTX & (1<<RXOUTI))
		{
			UEINTX = ~(1<<RXOUTI);

			while (UEBCLX)
				com.write(UEDATX);

			UEINTX = ~(1<<FIFOCON);
		}

	}

	uint16_t m_resets, m_sofs;

private:
	bool m_enabled;
	uint8_t last_intr, last_i;

	void configure_ep0()
	{
		UENUM = 0;

		// The endpoint 0 should already be enabled.
		UECFG0X = (EPTYPE_CONTROL<<6)|(EPDIR_OUT<<0);
		UECFG1X = (EPSIZE_64<<4)|(1<<ALLOC);

		// XXX: The endpoint 0 is not automatically enabled after USB reset as specificed in the datasheet.
		UECONX = (1<<EPEN);

		//UEINTX = 0;
	}

	void ep0_setup(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength)
	{
		m_ep0_transaction.m_request = bRequest;

		switch (bRequest)
		{
		case urt_set_address:
			UDADDR = wValue & 0x7f;
			UEINTX = ~(1<<TXINI) & ~(1<<RXOUTI);

			send(com, "SET_ADDRESS ");
			send_hex(com, wValue);
			send(com, "\r\n");
			break;
		case urt_get_descriptor:
			{
				m_ep0_transaction.m_get_descriptor.pFirst = 0;
				m_ep0_transaction.m_get_descriptor.pLast = 0;

				for (uint8_t i = 0; i < sizeof usb_descriptors / sizeof usb_descriptors[0]; ++i)
				{
					if (usb_descriptors[i].index == wValue)
					{
						m_ep0_transaction.m_get_descriptor.pFirst = usb_descriptors[i].first;
						m_ep0_transaction.m_get_descriptor.pLast = usb_descriptors[i].last;
						break;
					}
				}

				// Make sure that at most the requested amount of data gets transmitted.
				if (uint16_t(m_ep0_transaction.m_get_descriptor.pLast - m_ep0_transaction.m_get_descriptor.pFirst) > wLength)
					m_ep0_transaction.m_get_descriptor.pLast = m_ep0_transaction.m_get_descriptor.pFirst + wLength;
			}

			send(com, "GET_DESCRIPTOR: ");
			send_hex(com, wValue);
			send(com, ", ");
			send_hex(com, wIndex);
			send(com, ", ");
			send_hex(com, wLength);
			send(com, "\r\n");

			ep0_in_ack();
			break;

		case urt_set_configuration:
			UEINTX = ~(1<<TXINI) & ~(1<<RXOUTI);

			send(com, "SET_CONFIGURATION ");
			send_hex(com, wValue);
			send(com, "\r\n");
			break;

		default:
			m_ep0_transaction.m_request = 0xff;
			send(com, "SETUP ");
			send_hex(com, bmRequestType);
			send(com, ", ");
			send_hex(com, bRequest);
			send(com, ", ");
			send_hex(com, wValue);
			send(com, ", ");
			send_hex(com, wIndex);
			send(com, ", ");
			send_hex(com, wLength);
			send(com, "\r\n");
		}
	}

	void ep0_out()
	{
		send(com, "OUT\r\n");
	}

	void ep0_in_ack()
	{
		send(com, "IN\r\n");
		switch (m_ep0_transaction.m_request)
		{
		case urt_get_descriptor:
			{
				uint8_t packet_size = 0;
				for (; packet_size < 64
					&& m_ep0_transaction.m_get_descriptor.pFirst != m_ep0_transaction.m_get_descriptor.pLast;
					++m_ep0_transaction.m_get_descriptor.pFirst)
				{
					UEDATX = pgm_read_byte(m_ep0_transaction.m_get_descriptor.pFirst);
					++packet_size;
				}

				UEINTX = ~(1<<TXINI) & ~(1<<RXOUTI);

				if (packet_size < 64)
					m_ep0_transaction.m_request = 0xff;
			}
			break;

		case urt_set_address:
			UDADDR |= (1<<ADDEN);
			m_ep0_transaction.m_request = 0xff;
			break;

		case urt_set_configuration:
			set_config();
			m_ep0_transaction.m_request = 0xff;
			break;

		default:
			m_ep0_transaction.m_request = 0xff;
		}
	}

	void set_config()
	{
		UENUM = 1;
		UECFG0X = (EPTYPE_CONTROL<<6)|(EPDIR_IN<<0);
		UECFG1X = (EPSIZE_64<<4)|(1<<ALLOC);
		UECONX = (1<<EPEN);

		UENUM = 2;
		UECFG0X = (EPTYPE_CONTROL<<6)|(EPDIR_OUT<<0);
		UECFG1X = (EPSIZE_64<<4)|(1<<ALLOC);
		UECONX = (1<<EPEN);
	}

	struct ep0_transaction_t
	{
		uint8_t m_request;
		union
		{
			struct
			{
				uint8_t const * pFirst;
				uint8_t const * pLast;
			} m_get_descriptor;
		};
	} m_ep0_transaction;
};

int main()
{
	// Make the clock run at 8MHz. This is for a 16MHz crystal;
	// remove this when the crystal is changed for 8MHz one.
	CLKPR = (1<<CLKPCE);
	CLKPR = (1<<CLKPS0);

	sei();

	usb_t usb;

	REGCR = (1<<REGDIS);

	for (;;)
	{
		if (!com.empty())
		{
			char ch = com.read();
			switch (ch)
			{
			case 'i':
				usb.init();
				break;
			case 's':
				break;
			case '?':
			default:
				{
					uint8_t udint = UDINT;

					send(com, "Shupito USB comm chip.\r\nr: ");
					send_hex(com, usb.m_resets);
					send(com, "\r\nsofs: ");
					send_hex(com, usb.m_sofs);
					send(com, "\r\nUDINT: ");
					send_hex(com, udint);
					send(com, "\r\nUEINTX: ");
					send_hex(com, UEINT);
					send(com, "\r\nUESTA0X: ");
					send_hex(com, UESTA0X);
					send(com, "\r\n");
				}
			}
		}

		com.process_tx();
		usb.process();
	}
}
