#include "app.hpp"
#include "dbg.h"
#include "utils.hpp"
#include "led.hpp"
#include "usb.h"
#include "stack_usage.h"

void app::process_with_debug()
{
	if (!com_usb.empty())
	{
		switch (com_usb.read())
		{
		case 'A':
			AVRLIB_ASSERT(!"test assert");
			break;
		case 'B':
			initiate_software_reset();
			break;
		case 'b':
			led_blink_short();
			break;
		case 'v':
			send(com_usb, "vccio: 0x");
			send_hex(com_usb, (uint16_t)m_vccio_voltage);
			send(com_usb, "\nhiv_voltage: 0x");
			send_hex(com_usb, hiv_get_voltage());
			send(com_usb, "\nhiv_period: 0x");
			send_hex(com_usb, hiv_get_period());
			com_usb.write('\n');
			break;
		case 'p':
			usb_tunnel_send_test_packet = true;
			break;
		case 'H':
			hiv_enable();
			send(com_usb, "hiv_enable()\n");
			break;
		case 'h':
			hiv_disable();
			send(com_usb, "hiv_disable()\n");
			break;
		case '1':
			pin_rst::make_low();
			break;
		case '2':
			pin_rst::make_high();
			break;
		case '3':
			pin_rst::apply_hiv();
			break;
		case '4':
			pin_rst::make_input();
			break;
		case 's':
			send(com_usb, "stack: ");
			send_hex(com_usb, get_stack_usage());
			com_usb.write('/');
			send_hex(com_usb, get_stack_size());
			com_usb.write('\n');
			break;
		case ' ':
			pin_rst::make_input();
			hiv_disable();
			break;
		case '?':
			send(com_usb, "Shupito 2.3\n");
			// fallthrough
		default:
			send(com_usb, "?AbBvphH1234s\n");
			break;
		}
	}
}
