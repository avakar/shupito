#include "usb.h"
#include "dbg.h"
#include "app.hpp"
#include "led.hpp"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

com_usb_t com_usb;
com_usb_tunnel_t com_usb_tunnel;
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

static uint8_t ep3_out_buf[64];
static uint8_t ep3_in_buf[64];

static uint8_t ep4_out_buf[64];
static uint8_t ep4_in_buf[64];

struct ep_descs_t
{
	USB_EP_t ep0_out;
	USB_EP_t ep0_in;
	USB_EP_t ep1_out;
	USB_EP_t ep1_in;
	USB_EP_t ep2_out;
	USB_EP_t ep2_in;
	USB_EP_t ep3_out;
	USB_EP_t ep3_in;
	USB_EP_t ep4_out;
	USB_EP_t ep4_in;
};

// Note that __attribute__((aligned)) is actually ignored by avr-gcc for some reason.
// I'm working around by aligning manually.
static uint8_t ep_descs_buf[sizeof(ep_descs_t) + 1];
static uintptr_t const ep_descs_ptr = (uintptr_t)ep_descs_buf;
static ep_descs_t * ep_descs = reinterpret_cast<ep_descs_t *>((ep_descs_ptr + 1) & ~(uintptr_t)1);

static char const * usb_sn;
static uint8_t usb_snlen;

static uint8_t usb_config = 0;
static bool set_config(uint8_t config)
{
	if (config >= 2)
		return false;

	usb_config = config;
	if (config)
	{
		ep_descs->ep1_out.STATUS = 0;
		ep_descs->ep1_out.CTRL = USB_EP_TYPE_BULK_gc | USB_EP_BUFSIZE_64_gc;
		ep_descs->ep1_in.STATUS = USB_EP_BUSNACK0_bm;
		ep_descs->ep1_in.CTRL = USB_EP_TYPE_BULK_gc | USB_EP_BUFSIZE_64_gc;
		ep_descs->ep2_out.STATUS = 0;
		ep_descs->ep2_out.CTRL = USB_EP_TYPE_BULK_gc | USB_EP_MULTIPKT_bm | USB_EP_BUFSIZE_64_gc;
		ep_descs->ep2_out.AUXDATA = sizeof usb_yb_out_packet;
		ep_descs->ep2_in.STATUS = USB_EP_BUSNACK0_bm;
		ep_descs->ep2_in.CTRL = USB_EP_TYPE_BULK_gc | USB_EP_MULTIPKT_bm | USB_EP_BUFSIZE_64_gc;
		ep_descs->ep3_out.STATUS = 0;
		ep_descs->ep3_out.CTRL = USB_EP_TYPE_BULK_gc | USB_EP_BUFSIZE_64_gc;
		ep_descs->ep3_in.STATUS = USB_EP_BUSNACK0_bm;
		ep_descs->ep3_in.CTRL = USB_EP_TYPE_BULK_gc | USB_EP_BUFSIZE_64_gc;
		ep_descs->ep4_out.STATUS = 0;
		ep_descs->ep4_out.CTRL = USB_EP_TYPE_BULK_gc | USB_EP_BUFSIZE_64_gc;
		ep_descs->ep4_in.STATUS = USB_EP_BUSNACK0_bm;
		ep_descs->ep4_in.CTRL = USB_EP_TYPE_BULK_gc | USB_EP_BUFSIZE_64_gc;
		USB_CTRLA = USB_ENABLE_bm | USB_SPEED_bm | (4 << USB_MAXEP_gp);
	}
	else
	{
		USB_CTRLA = USB_ENABLE_bm | USB_SPEED_bm | (0 << USB_MAXEP_gp);
		ep_descs->ep1_out.CTRL = 0;
		ep_descs->ep1_in.CTRL = 0;
	}

	return true;
}

static enum { uts_idle, uts_dma } g_usb_tunnel_state = uts_idle;

static void usb_tunnel_init()
{
	g_usb_tunnel_state = uts_idle;

	DMA_CTRL = DMA_ENABLE_bm;

	// Setup the DMA channel to transfer from the USB EP3OUT to the USART C1
	// data register whenever the register becomes ready
	DMA_CH0_DESTADDR0 = (uint8_t)(uint16_t)&USARTC1_DATA;
	DMA_CH0_DESTADDR1 = (uint16_t)&USARTC1_DATA >> 8;
	DMA_CH0_DESTADDR2 = 0;

	DMA_CH0_SRCADDR0 = (uint8_t)(uint16_t)ep3_out_buf;
	DMA_CH0_SRCADDR1 = (uint16_t)ep3_out_buf >> 8;
	DMA_CH0_SRCADDR2 = 0;

	DMA_CH0_ADDRCTRL = DMA_CH_SRCRELOAD_BLOCK_gc | DMA_CH_SRCDIR_INC_gc | DMA_CH_DESTRELOAD_NONE_gc | DMA_CH_DESTDIR_FIXED_gc;
	DMA_CH0_TRIGSRC = DMA_CH_TRIGSRC_USARTC1_DRE_gc;
}

static void usb_tunnel_poll()
{
	if ((ep_descs->ep3_out.STATUS & USB_EP_BUSNACK0_bm) && g_usb_tunnel_state == uts_idle)
	{
		g_usb_tunnel_state = uts_dma;
		DMA_CH0_TRFCNT = ep_descs->ep3_out.CNT;
		DMA_CH0_CTRLB = DMA_CH_TRNIF_bm;
		DMA_CH0_CTRLA = DMA_CH_ENABLE_bm | DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;
	}

	if (g_usb_tunnel_state == uts_dma && (DMA_CH0_CTRLB & DMA_CH_TRNIF_bm))
	{
		g_usb_tunnel_state = uts_idle;
		ep_descs->ep3_out.STATUS &= ~USB_EP_BUSNACK0_bm;
	}
}

void usb_init(char const * sn, uint8_t snlen)
{
	usb_sn = sn;
	usb_snlen = snlen;

	ep_descs->ep0_out.DATAPTR = (uint16_t)&ep0_out_buf;
	ep_descs->ep0_in.DATAPTR  = (uint16_t)&ep0_in_buf;

	ep_descs->ep1_out.DATAPTR = (uint16_t)&ep1_out_buf;
	ep_descs->ep1_in.DATAPTR  = (uint16_t)&ep1_in_buf;

	ep_descs->ep2_out.DATAPTR = (uint16_t)&usb_yb_out_packet;
	ep_descs->ep2_in.DATAPTR  = (uint16_t)&usb_yb_in_packet;

	ep_descs->ep3_out.DATAPTR = (uint16_t)&ep3_out_buf;
	ep_descs->ep3_in.DATAPTR  = (uint16_t)&ep3_in_buf;

	ep_descs->ep4_out.DATAPTR = (uint16_t)&ep4_out_buf;
	ep_descs->ep4_in.DATAPTR  = (uint16_t)&ep4_in_buf;

	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	USB_CAL0 = pgm_read_byte(PRODSIGNATURES_USBCAL0);
	USB_CAL1 = pgm_read_byte(PRODSIGNATURES_USBCAL1);
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;

	CLK_USBCTRL = CLK_USBSRC_PLL_gc | CLK_USBSEN_bm;
	USB_CTRLA = USB_ENABLE_bm | USB_SPEED_bm | (0 << USB_MAXEP_gp);
	USB_EPPTR = (uint16_t)ep_descs;
	USB_CTRLB = USB_ATTACH_bm;

	usb_tunnel_init();
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
		ep_descs->ep0_out.CTRL = USB_EP_TYPE_CONTROL_gc | USB_EP_STALL_bm | USB_EP_BUFSIZE_64_gc;
		ep_descs->ep0_in.CTRL = USB_EP_TYPE_CONTROL_gc | USB_EP_STALL_bm | USB_EP_BUFSIZE_64_gc;
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

		if ((ep_descs->ep3_in.STATUS & USB_EP_BUSNACK0_bm) && usb_tunnel_send_test_packet)
		{
			memset(ep3_in_buf, 'x', 64);
			ep_descs->ep3_in.CNT = 64;
			ep_descs->ep3_in.STATUS &= ~USB_EP_BUSNACK0_bm;
			usb_tunnel_send_test_packet = false;
		}

		if ((ep_descs->ep3_in.STATUS & USB_EP_BUSNACK0_bm) && com_usb_tunnel.tx_size() != 0)
		{
			memcpy(ep3_in_buf, com_usb_tunnel.tx_buffer(), com_usb_tunnel.tx_size());
			ep_descs->ep3_in.CNT = com_usb_tunnel.tx_size();
			ep_descs->ep3_in.STATUS &= ~USB_EP_BUSNACK0_bm;
			com_usb_tunnel.tx_clear();
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
				g_app.open_tunnel(action.intf - 2, dwDTERate);

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
				if (wValue == 0x302)
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
			ep_descs->ep0_out.CTRL = USB_EP_TYPE_CONTROL_gc | USB_EP_STALL_bm | USB_EP_BUFSIZE_64_gc;
			ep_descs->ep0_in.CTRL = USB_EP_TYPE_CONTROL_gc | USB_EP_STALL_bm | USB_EP_BUFSIZE_64_gc;
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
