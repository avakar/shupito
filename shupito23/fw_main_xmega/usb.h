#ifndef SHUPITO_USB_H
#define SHUPITO_USB_H

/**
 * \brief Setup USB peripheral and associated interrupts.
 *
 * The PLL clock must be configured to run at 48MHz.
 */
void usb_init();

void usb_poll();

#endif // SHUPITO_USB_H
