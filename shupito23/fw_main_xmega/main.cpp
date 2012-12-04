#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#define STRINGIFY2(x) #x
#define STRINGIFY(x) STRINGIFY2(x)

#include "../../fw_common/avrlib/xmega_pin.hpp"
#include "dbg.h"
#include "usb.h"

com_dbg_t com_dbg;
ISR(USARTE0_RXC_vect) { com_dbg.intr_rx(); }

AVRLIB_DEFINE_XMEGA_PIN(pin_button, PORTA, 5);
AVRLIB_DEFINE_XMEGA_PIN(pin_led,    PORTA, 3);

AVRLIB_DEFINE_XMEGA_PIN(pin_dbg_rx, PORTE, 2);
AVRLIB_DEFINE_XMEGA_PIN(pin_dbg_tx, PORTE, 3);

/**
 * \brief Jumps to the FLIP bootloader.
 *
 * Make sure that all peripherals are reset to default settings,
 * before starting the bootloader. Ideally, reset the CPU and
 * then call this function as the first thing in main.
 */
#define start_flip_bootloader() asm("jmp " STRINGIFY(BOOT_SECTION_START) " + 0x1fc")

static bool software_reset_occurred()
{
	return (RST_STATUS & RST_SRF_bm) != 0;
}

/**
 * \brief Resets the CPU. Interrupts must be disabled.
 */
static void initiate_software_reset() __attribute__((noreturn));
static void initiate_software_reset()
{
	CCP = CCP_IOREG_gc;
	RST_CTRL = RST_SWRST_bm;
	for (;;) {}
}

static void enable_interrupts()
{
	PMIC_CTRL = PMIC_RREN_bm | PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();
}

ISR(PORTA_INT0_vect, __attribute__((noreturn, naked, flatten)))
{
	initiate_software_reset();
}

static void setup_bootloader_button()
{
	pin_button::pinctrl(PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc);
	PORTA_INT0MASK = pin_button::bm;
	PORTA_INTCTRL = PORT_INT0LVL_HI_gc;
}

#define wait_until(cond) do {} while(!(cond))

static void setup_clocks()
{
	// The 32MHz oscillator is used as the main clock.
	// The 2MHz oscillator is fed through the PLL to generate
	// 48MHz for the USB peripheral. The 32kHz oscillator
	// is used by DFLL to tune 2MHz and 32MHz oscillators.
	
	// Enable all oscillators and wait for them to start-up.
	// The 2MHz oscillator is enabled in advance and doesn't
	// need waiting for.
	OSC_CTRL = OSC_RC32MEN_bm | OSC_RC2MEN_bm | OSC_RC32KEN_bm;
	wait_until(OSC_STATUS & OSC_RC32MRDY_bm);
	wait_until(OSC_STATUS & OSC_RC32KEN_bm);

	// Switch the CPU to the 32MHz oscillator.
	CCP = CCP_IOREG_gc;
	CLK_CTRL = CLK_SCLKSEL_RC32M_gc;

	// Enable DFLLs
	DFLLRC2M_CTRL = DFLL_ENABLE_bm;
	DFLLRC32M_CTRL = DFLL_ENABLE_bm;

	// Configure and enable PLL.
	OSC_PLLCTRL = OSC_PLLSRC_RC2M_gc | (24 << OSC_PLLFAC_gp);
	OSC_CTRL = OSC_PLLEN_bm | OSC_RC32MEN_bm | OSC_RC2MEN_bm | OSC_RC32KEN_bm;
	wait_until(OSC_STATUS & OSC_PLLRDY_bm);
}

int main()
{
	if (software_reset_occurred())
		start_flip_bootloader();
	setup_bootloader_button();
	enable_interrupts();
	setup_clocks();

	pin_led::make_high();

	// Setup com_dbg
	pin_dbg_rx::pullup();
	pin_dbg_tx::make_high();
	com_dbg.usart().open((-1 << 12)|102 /*38400*/, true);

	send(com_usb, "Starting...\n");

	usb_init();
	for (;;)
	{
		if (!com_usb.empty())
		{
			switch (uint8_t ch = com_usb.read())
			{
			case '?':
				send(com_usb, "Shupito 2.3\n");
				break;
			default:
				com_usb.write(ch + 1);
				break;
			}
		}

		usb_poll();
		com_dbg.process_tx();
	}
}
