#include "utils.hpp"
#include "../../fw_common/avrlib/assert.hpp"
#include "usb.h"
#include "dbg.h"
#include "app.hpp"
#include "led.hpp"

static bool g_assert_disabled = false;

ISR(PORTA_INT0_vect, __attribute__((noreturn, naked, flatten)))
{
	initiate_software_reset();
}

void avrlib::assertion_failed(char const * msg, char const * file, int lineno)
{
	if (g_assert_disabled)
		return;

	led_on();

	g_assert_disabled = true;
	send(com_usb, "Assertion failed: ");
	send(com_usb, msg);
	send(com_usb, "\n    ");
	send(com_usb, file);
	send(com_usb, "(0x");
	send_hex(com_usb, lineno);
	send(com_usb, ")\n");
	for (;;)
		g_process_with_debug();
}

void initiate_software_reset()
{
	CCP = CCP_IOREG_gc;
	RST_CTRL = RST_SWRST_bm;
	for (;;)
	{
	}
}

void enable_interrupts()
{
	PMIC_CTRL = PMIC_RREN_bm | PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();
}

void setup_bootloader_button()
{
	pin_button::pinctrl(PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc);
	PORTA_INT0MASK = pin_button::bm;
	PORTA_INTCTRL = PORT_INT0LVL_HI_gc;
}

bool software_reset_occurred()
{
	return (RST_STATUS & RST_SRF_bm) != 0;
}

void start_flip_bootloader()
{
	asm("jmp " STRINGIFY(BOOT_SECTION_START) " + 0x1fc");
	for (;;)
	{
	}
}
