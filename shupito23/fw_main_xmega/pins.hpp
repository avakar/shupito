#ifndef SHUPITO_SHUPITO23_PINS_HPP
#define SHUPITO_SHUPITO23_PINS_HPP

#include "../../fw_common/avrlib/xmega_pin.hpp"

AVRLIB_DEFINE_XMEGA_PIN(pin_button,    PORTA, 5);
AVRLIB_DEFINE_XMEGA_PIN(pin_led,       PORTA, 3);

AVRLIB_DEFINE_XMEGA_PIN(pin_dbg_rx,    PORTE, 2);
AVRLIB_DEFINE_XMEGA_PIN(pin_dbg_tx,    PORTE, 3);

AVRLIB_DEFINE_XMEGA_PIN(pin_sup_3v3,   PORTB, 2);
AVRLIB_DEFINE_XMEGA_PIN(pin_sup_5v0,   PORTB, 0);

AVRLIB_DEFINE_XMEGA_PIN(pin_aux_rstd,  PORTD, 0);
AVRLIB_DEFINE_XMEGA_PIN(pin_aux_rstv,  PORTC, 1);
AVRLIB_DEFINE_XMEGA_PIN(pin_pdid,      PORTD, 1);
AVRLIB_DEFINE_XMEGA_PIN(pin_pdiv,      PORTC, 3);
AVRLIB_DEFINE_XMEGA_PIN(pin_xckv,      PORTC, 5);
AVRLIB_DEFINE_XMEGA_PIN(pin_rxd,       PORTC, 6);
AVRLIB_DEFINE_XMEGA_PIN(pin_txdv,      PORTC, 7);
AVRLIB_DEFINE_XMEGA_PIN(pin_txdd,      PORTD, 2);

struct pin_rst
{
	static void init();
	static void process();
	static bool ready();

	static void make_input();
	static void make_high();
	static void make_low();
	static void set_high();
	static void set_low();

	static void apply_hiv();
};

typedef pin_buffer_with_oe<pin_aux_rstv, pin_aux_rstd> pin_aux_rst;
typedef pin_buffer_with_oe<pin_pdiv, pin_pdid> pin_pdi;
typedef pin_buffer_with_oe<pin_xckv, pin_pdid> pin_xck;
typedef pin_buffer_with_oe<pin_txdv, pin_txdd> pin_txd;

#endif // SHUPITO_SHUPITO23_PINS_HPP
