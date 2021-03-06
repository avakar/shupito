#include "usb.h"
#include "dbg.h"
#include "app.hpp"
#include "led.hpp"
#include "usb_eps.hpp"
#include "tunnel.hpp"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

com_usb_t com_usb;
com_usb_tunnel_t com_usb_tunnel2;
bool usb_tunnel_send_test_packet = false;

#include "usb_descriptors.h"

struct usb_setup_t
{
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
};

enum standard_requests_codes_t
{
	usb_get_status        = 0x8000,
	usb_clear_feature     = 0x0001,
	usb_set_feature       = 0x0003,
	usb_set_address       = 0x0005,
	usb_get_descriptor    = 0x8006,
	usb_set_descriptor    = 0x0007,
	usb_get_configuration = 0x8008,
	usb_set_configuration = 0x0009,
	usb_set_interface     = 0x010b,

	usb_set_line_coding   = 0x2120,
	usb_set_control_line_state = 0x2122,
};

struct buffer_addrs_t
{
	uint8_t const * first;
	uint16_t length;
};

struct usb_action
{
	enum in_action_t
	{
		ia_none,
		ia_send_buffer,
		ia_set_address,
		ia_set_line_coding,
	};

	in_action_t in_action;
	union
	{
		uint8_t next_address;
		buffer_addrs_t buffer_addrs;
		uint8_t intf;
	};
};

static usb_action action = {};

static uint8_t ep0_out_buf[64];
static uint8_t ep0_in_buf[64];

static uint8_t ep1_out_buf[64];
static uint8_t ep1_in_buf[64];

uint8_t usb_yb_out_packet[256];
uint8_t usb_yb_in_packet[256];

static uint8_t ep4_out_buf[64];
static uint8_t ep4_in_buf[64];

// Note that __attribute__((aligned)) is actually ignored by avr-gcc for some reason.
// I'm working around by aligning manually.
static uint8_t ep_descs_buf[(usb_ep_num_max+1)*4 + sizeof(ep_descs_t) + 1];
static uintptr_t const ep_descs_ptr = (uintptr_t)ep_descs_buf;
ep_descs_t * ep_descs = reinterpret_cast<ep_descs_t *>((ep_descs_ptr + (usb_ep_num_max+1)*4 + 1) & ~(uintptr_t)1);

static char const * usb_sn;
static uint8_t usb_snlen;
static uint8_t const * usb_namedesc;

static uint8_t usb_config = 0;
static bool set_config(uint8_t config)
{
	if (config >= 2)
		return false;

	usb_config = config;
	if (config)
	{
		ep_descs->ep1_out.STATUS = 0;
		ep_descs->ep1_out.CTRL = USB_EP_INTDSBL_bm | USB_EP_TYPE_BULK_gc | USB_EP_BUFSIZE_64_gc;
		ep_descs->ep1_in.STATUS = USB_EP_BUSNACK0_bm;
		ep_descs->ep1_in.CTRL = USB_EP_INTDSBL_bm | USB_EP_TYPE_BULK_gc | USB_EP_BUFSIZE_64_gc;
		ep_descs->ep2_out.STATUS = 0;
		ep_descs->ep2_out.CTRL = USB_EP_INTDSBL_bm | USB_EP_TYPE_BULK_gc | USB_EP_MULTIPKT_bm | USB_EP_BUFSIZE_64_gc;
		ep_descs->ep2_out.AUXDATA = sizeof usb_yb_out_packet;
		ep_descs->ep2_in.STATUS = USB_EP_BUSNACK0_bm;
		ep_descs->ep2_in.CTRL = USB_EP_INTDSBL_bm | USB_EP_TYPE_BULK_gc | USB_EP_MULTIPKT_bm | USB_EP_BUFSIZE_64_gc;
		ep_descs->ep4_out.STATUS = 0;
		ep_descs->ep4_out.CTRL = USB_EP_INTDSBL_bm | USB_EP_TYPE_BULK_gc | USB_EP_BUFSIZE_64_gc;
		ep_descs->ep4_in.STATUS = USB_EP_BUSNACK0_bm;
		ep_descs->ep4_in.CTRL = USB_EP_INTDSBL_bm | USB_EP_TYPE_BULK_gc | USB_EP_BUFSIZE_64_gc;
		ep_descs->ep5_in_x.CTRL = USB_EP_INTDSBL_bm | USB_EP_TYPE_DISABLE_gc;
		usb_tunnel_config();
		USB_CTRLA = USB_ENABLE_bm | USB_SPEED_bm | USB_FIFOEN_bm | (usb_ep_num_max << USB_MAXEP_gp);
	}
	else
	{
		USB_CTRLA = USB_ENABLE_bm | USB_SPEED_bm | USB_FIFOEN_bm | (0 << USB_MAXEP_gp);
		g_app.close_tunnel();
		usb_tunnel_deconfig();
	}

	return true;
}

void usb_init(char const * sn, uint8_t snlen, uint8_t const * namedesc)
{
	usb_sn = sn;
	usb_snlen = snlen;
	usb_namedesc = namedesc;

	ep_descs->ep0_out.DATAPTR = (uint16_t)&ep0_out_buf;
	ep_descs->ep0_in.DATAPTR  = (uint16_t)&ep0_in_buf;

	ep_descs->ep1_out.DATAPTR = (uint16_t)&ep1_out_buf;
	ep_descs->ep1_in.DATAPTR  = (uint16_t)&ep1_in_buf;

	ep_descs->ep2_out.DATAPTR = (uint16_t)&usb_yb_out_packet;
	ep_descs->ep2_in.DATAPTR  = (uint16_t)&usb_yb_in_packet;

	ep_descs->ep4_out.DATAPTR = (uint16_t)&ep4_out_buf;
	ep_descs->ep4_in.DATAPTR  = (uint16_t)&ep4_in_buf;

	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	USB_CAL0 = pgm_read_byte(PRODSIGNATURES_USBCAL0);
	USB_CAL1 = pgm_read_byte(PRODSIGNATURES_USBCAL1);
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;

	CLK_USBCTRL = CLK_USBSRC_PLL_gc | CLK_USBSEN_bm;
	USB_CTRLA = USB_ENABLE_bm | USB_SPEED_bm | USB_FIFOEN_bm | (0 << USB_MAXEP_gp);
	USB_EPPTR = (uint16_t)ep_descs;
	USB_INTCTRLB = USB_TRNIE_bm;
	USB_INTCTRLA = USB_INTLVL_MED_gc;
	USB_CTRLB = USB_ATTACH_bm;
}

static void prepare_send_buffer_packet(buffer_addrs_t & addrs)
{
	uint8_t chunk = addrs.length > 64? 64: addrs.length;
	memcpy_P(ep0_in_buf, addrs.first, chunk);
	addrs.first += chunk;
	addrs.length -= chunk;

	ep_descs->ep0_in.CNT = chunk;
	ep_descs->ep0_in.STATUS &= ~(USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm);
}

void usb_poll()
{
	if (USB_INTFLAGSACLR & USB_RSTIF_bm)
	{
		//send(com_dbg, "RESET\n");
		USB_ADDR = 0;
		set_config(0);
		ep_descs->ep0_out.CTRL = USB_EP_INTDSBL_bm | USB_EP_TYPE_CONTROL_gc | USB_EP_STALL_bm | USB_EP_BUFSIZE_64_gc;
		ep_descs->ep0_in.CTRL = USB_EP_INTDSBL_bm | USB_EP_TYPE_CONTROL_gc | USB_EP_STALL_bm | USB_EP_BUFSIZE_64_gc;
		USB_INTFLAGSACLR = USB_RSTIF_bm;

		led_blink_long();
		return;
	}

	if (usb_config)
	{
		if ((ep_descs->ep1_in.STATUS & USB_EP_BUSNACK0_bm) && com_usb.tx_size() != 0)
		{
			memcpy(ep1_in_buf, com_usb.tx_buffer(), com_usb.tx_size());
			ep_descs->ep1_in.CNT = com_usb.tx_size();
			ep_descs->ep1_in.STATUS &= ~USB_EP_BUSNACK0_bm;
			com_usb.tx_clear();
		}

		if ((ep_descs->ep1_out.STATUS & USB_EP_BUSNACK0_bm) && com_usb.rx_size() == 0)
		{
			memcpy(com_usb.rx_buffer(), ep1_out_buf, ep_descs->ep1_out.CNT);
			com_usb.rx_reset(ep_descs->ep1_out.CNT);
			ep_descs->ep1_out.STATUS &= ~USB_EP_BUSNACK0_bm;
		}

		usb_tunnel_poll();

		if ((ep_descs->ep4_in.STATUS & USB_EP_BUSNACK0_bm) && com_usb_tunnel2.tx_size() != 0)
		{
			memcpy(ep4_in_buf, com_usb_tunnel2.tx_buffer(), com_usb_tunnel2.tx_size());
			ep_descs->ep4_in.CNT = com_usb_tunnel2.tx_size();
			ep_descs->ep4_in.STATUS &= ~USB_EP_BUSNACK0_bm;
			com_usb_tunnel2.tx_clear();
		}

		if ((ep_descs->ep4_out.STATUS & USB_EP_BUSNACK0_bm) && com_usb_tunnel2.rx_size() == 0)
		{
			memcpy(com_usb_tunnel2.rx_buffer(), ep4_out_buf, ep_descs->ep4_out.CNT);
			com_usb_tunnel2.rx_reset(ep_descs->ep4_out.CNT);
			ep_descs->ep4_out.STATUS &= ~USB_EP_BUSNACK0_bm;
		}

		if ((ep_descs->ep4_in.STATUS & USB_EP_BUSNACK0_bm) && usb_tunnel_send_test_packet)
		{
			memset(ep4_in_buf, 'x', 64);
			ep_descs->ep4_in.CNT = 64;
			ep_descs->ep4_in.STATUS &= ~USB_EP_BUSNACK0_bm;
			usb_tunnel_send_test_packet = false;
		}
	}

	if (ep_descs->ep0_in.STATUS & USB_EP_TRNCOMPL0_bm)
	{
		switch (action.in_action)
		{
		case usb_action::ia_set_address:
			USB_ADDR = action.next_address;
			ep_descs->ep0_in.STATUS = USB_EP_BUSNACK0_bm;
			break;
		case usb_action::ia_send_buffer:
			prepare_send_buffer_packet(action.buffer_addrs);
			break;
		default:
			ep_descs->ep0_in.STATUS = USB_EP_BUSNACK0_bm;
			break;
		}
	}

	if (ep_descs->ep0_out.STATUS & USB_EP_TRNCOMPL0_bm)
	{
		bool valid = false;
		switch (action.in_action)
		{
		case usb_action::ia_set_line_coding:
			if (ep_descs->ep0_out.CNT == 7)
			{
				uint32_t dwDTERate
					= (uint32_t)ep0_out_buf[0]
					| ((uint32_t)ep0_out_buf[1] << 8)
					| ((uint32_t)ep0_out_buf[2] << 16)
					| ((uint32_t)ep0_out_buf[3] << 24);
				uint8_t bCharFormat = ep0_out_buf[4];
				uint8_t bParityType = ep0_out_buf[5];
				uint8_t bDataBits = ep0_out_buf[6];
				
				uint8_t mode = (bCharFormat == 0? 0: USART_SBMODE_bm);

				switch (bDataBits)
				{
				case 5:
					mode |= USART_CHSIZE_5BIT_gc;
					break;
				case 6:
					mode |= USART_CHSIZE_6BIT_gc;
					break;
				case 7:
					mode |= USART_CHSIZE_7BIT_gc;
					break;
				default:
					mode |= USART_CHSIZE_8BIT_gc;
				}

				switch (bParityType)
				{
				case 1:
					mode |= USART_PMODE_ODD_gc;
					break;
				case 2:
					mode |= USART_PMODE_EVEN_gc;
					break;
				default:
					mode |= USART_PMODE_DISABLED_gc;
				}
				
				g_app.open_tunnel(action.intf - 2, dwDTERate, mode);

				ep_descs->ep0_in.CNT = 0;
				ep_descs->ep0_in.STATUS = USB_EP_TOGGLE_bm;
				valid = true;
			}
			break;

		default:
			break;
		}

		if (!valid)
			ep_descs->ep0_out.STATUS = USB_EP_BUSNACK0_bm;
		action.in_action = usb_action::ia_none;
	}

	if (ep_descs->ep0_out.STATUS & USB_EP_SETUP_bm)
	{
		// The SETUP packet arrived. the ep0_out endpoint buffer should contain
		// `usb_setup_t` structure, from which we are supposed to decode the request.
		// Both the IN and OUT endpoints are unstalled -- they will not report request errors
		// to the host. They should have BUSNAK0 set, so that any early transfers will be NAKed.
		//
		// If we detect a request error, we'll stall the endpoints. Otherwise, we'll setup data
		// to transfer and clear BUSNAK0.

		// FIXME: check the transfer length and stall on error.

		uint8_t bmRequestType = ep0_out_buf[0];
		uint8_t bRequest = ep0_out_buf[1];
		uint16_t wValue = ep0_out_buf[2] | (ep0_out_buf[3] << 8);
		uint16_t wIndex = ep0_out_buf[4] | (ep0_out_buf[5] << 8);
		uint16_t wLength = ep0_out_buf[6] | (ep0_out_buf[7] << 8);

		//format(com_dbg, "SETUP %x:%x, %x %x %x, %x\n") % bmRequestType % bRequest % wValue % wIndex % wLength % ep_descs->ep0_out.CNT;

		bool valid = false;
		action.in_action = usb_action::ia_none;
		ep_descs->ep0_out.STATUS &= ~USB_EP_SETUP_bm;

		switch ((bmRequestType << 8) | bRequest)
		{
		case usb_set_address:
			action.next_address = wValue & 0x7f;
			action.in_action = usb_action::ia_set_address;
			ep_descs->ep0_in.CNT = 0;
			ep_descs->ep0_in.STATUS = USB_EP_TOGGLE_bm;
			valid = true;
			break;
		case usb_set_configuration:
			if ((wValue & 0xff) < 2)
			{
				set_config(wValue);
				ep_descs->ep0_in.CNT = 0;
				ep_descs->ep0_in.STATUS = USB_EP_TOGGLE_bm;
				valid = true;
			}
			break;
		case usb_get_configuration:
			{
				//format(com_dbg, "    %x\n") % usb_config;
				ep0_in_buf[0] = usb_config;
				ep_descs->ep0_in.CNT = 1;
				ep_descs->ep0_in.STATUS = USB_EP_TOGGLE_bm;
				ep_descs->ep0_out.STATUS = USB_EP_TOGGLE_bm;
				valid = true;
			}
			break;
		case usb_get_descriptor:
			{
				if (wValue == 0x301)
				{
					uint8_t snlen = usb_namedesc[0];
					if (snlen < wLength)
						wLength = snlen;
					memcpy(ep0_in_buf, usb_namedesc, snlen);
					ep_descs->ep0_in.CNT = wLength;
					ep_descs->ep0_in.STATUS = USB_EP_TOGGLE_bm;
					valid = true;
				}
				else if (wValue == 0x302)
				{
					uint8_t snlen = 2 + 2*usb_snlen;
					if (snlen < wLength)
						wLength = snlen;
					ep0_in_buf[0] = snlen;
					ep0_in_buf[1] = 3;
					for (uint8_t i = 0; i < snlen; ++i)
					{
						ep0_in_buf[2*i+2] = usb_sn[i];
						ep0_in_buf[2*i+3] = 0;
					}
					ep_descs->ep0_in.CNT = wLength;
					ep_descs->ep0_in.STATUS = USB_EP_TOGGLE_bm;
					valid = true;
				}
				else
				{
					for (uint8_t i = 0; i < sizeof usb_descriptor_map / sizeof usb_descriptor_map[0]; ++i)
					{
						usb_descriptor_entry_t const & entry = usb_descriptor_map[i];
						if (entry.index == wValue)
						{
							if (entry.size < wLength)
								wLength = entry.size;
							action.buffer_addrs.first = usb_descriptors + entry.offset;
							action.buffer_addrs.length = wLength;

							prepare_send_buffer_packet(action.buffer_addrs);
							action.in_action = usb_action::ia_send_buffer;
							valid = true;
							break;
						}
					}
				}

				if (valid)
					ep_descs->ep0_out.STATUS &= ~USB_EP_BUSNACK0_bm;
			}
			break;

		case usb_set_line_coding:
			if ((wIndex == 2 || wIndex == 3) && wLength == 7)
			{
				action.in_action = usb_action::ia_set_line_coding;
				action.intf = wIndex;
				ep_descs->ep0_out.STATUS = USB_EP_TOGGLE_bm;
				valid = true;
			}
			break;

		case usb_set_control_line_state:
			if (wIndex == 2)
				g_app.close_tunnel();

			if (wIndex == 2 || wIndex == 3)
			{
				ep_descs->ep0_out.STATUS = USB_EP_BUSNACK0_bm;
				ep_descs->ep0_in.CNT = 0;
				ep_descs->ep0_in.STATUS = USB_EP_TOGGLE_bm;
				valid = true;
			}
			break;
		}

		if (!valid)
		{
			ep_descs->ep0_out.CTRL = USB_EP_INTDSBL_bm | USB_EP_TYPE_CONTROL_gc | USB_EP_STALL_bm | USB_EP_BUFSIZE_64_gc;
			ep_descs->ep0_in.CTRL = USB_EP_INTDSBL_bm | USB_EP_TYPE_CONTROL_gc | USB_EP_STALL_bm | USB_EP_BUFSIZE_64_gc;
		}
	}
}

uint16_t usb_yb_has_out_packet()
{
	if (ep_descs->ep2_out.STATUS & USB_EP_BUSNACK0_bm)
	{
		if (ep_descs->ep2_out.CNT == 0)
		{
			ep_descs->ep2_out.STATUS &= ~USB_EP_BUSNACK0_bm;
			return 0;
		}

		return ep_descs->ep2_out.CNT;
	}

	return 0;
}

void usb_yb_confirm_out_packet()
{
	ep_descs->ep2_out.CNT = 0;
	ep_descs->ep2_out.STATUS &= ~USB_EP_BUSNACK0_bm;
}

uint16_t usb_yb_in_packet_ready()
{
	if (ep_descs->ep2_in.STATUS & USB_EP_BUSNACK0_bm)
		return sizeof usb_yb_in_packet;
	return 0;
}

void usb_yb_send_in_packet(uint16_t size)
{
	ep_descs->ep2_in.AUXDATA = 0;
	ep_descs->ep2_in.CNT = (size == sizeof usb_yb_out_packet)? size: (0x8000 | size);
	ep_descs->ep2_in.STATUS &= ~USB_EP_BUSNACK0_bm;
}

ISR(USB_TRNCOMPL_vect)
{
#if 0
	int8_t offs = (int8_t)USB_FIFORP;
	uint16_t ep_addr = *((uint16_t *)ep_descs + offs);
	AVRLIB_ASSERT(ep_addr == (uint16_t)&ep_descs->tunnel_in || ep_addr == (uint16_t)&ep_descs->tunnel_in_alt);
#else
	(void)USB_FIFORP;
#endif
	usb_ep3_in_trnif();
}
