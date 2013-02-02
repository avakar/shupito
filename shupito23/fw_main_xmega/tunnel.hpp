#ifndef SHUPITO_TUNNEL_HPP
#define SHUPITO_TUNNEL_HPP

#include <stdint.h>

void usb_tunnel_config();
void usb_tunnel_deconfig();
void usb_tunnel_poll();
void usb_ep3_in_trnif();

void usb_tunnel_start(uint16_t baudctrl, bool dblspeed);
void usb_tunnel_stop();

#endif // SHUPITO_TUNNEL_HPP
