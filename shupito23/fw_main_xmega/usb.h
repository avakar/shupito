#ifndef SHUPITO_USB_H
#define SHUPITO_USB_H

#include <stdint.h>
#include "../../fw_common/avrlib/memory_stream.hpp"

/**
 * \brief Setup USB peripheral and associated interrupts.
 *
 * The PLL clock must be configured to run at 48MHz.
 */
void usb_init(char const * sn, uint8_t snlen);

void usb_poll();

extern uint8_t usb_yb_out_packet[256];
extern uint8_t usb_yb_in_packet[256];

uint16_t usb_yb_has_out_packet();
void usb_yb_confirm_out_packet();

uint16_t usb_yb_in_packet_ready();
void usb_yb_send_in_packet(uint16_t size);

typedef avrlib::memory_stream<64, 64> com_usb_tunnel_t;
extern com_usb_tunnel_t com_usb_tunnel2;
extern bool usb_tunnel_send_test_packet;

void usb_tunnel_start(uint16_t baudctrl, bool dblspeed);
void usb_tunnel_stop();

#endif // SHUPITO_USB_H
