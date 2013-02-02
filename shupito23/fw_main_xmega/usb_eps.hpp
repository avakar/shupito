#ifndef SHUPITO_USB_EPS_HPP
#define SHUPITO_USB_EPS_HPP

#include <avr/io.h>

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

extern ep_descs_t * ep_descs;

#endif // SHUPITO_USB_EPS_HPP
