#ifndef SHUPITO_TUNNEL_HPP
#define SHUPITO_TUNNEL_HPP

#include <stdint.h>

void usb_tunnel_config();
void usb_tunnel_deconfig();
void usb_tunnel_poll();
void usb_ep3_in_trnif();

// see USART_CTRLC for details on `mode`
void usb_tunnel_start(uint16_t baudctrl, uint8_t mode, bool dblspeed);
void usb_tunnel_stop();

void app_tunnel_start(uint16_t baudctrl, uint8_t mode, bool dblspeed);
void app_tunnel_stop();
uint8_t app_tunnel_send(uint8_t const * v, uint8_t size);
uint8_t app_tunnel_recv(uint8_t const *& data);
void app_tunnel_recv_commit();

#endif // SHUPITO_TUNNEL_HPP
