#include <avr/io.h>
#include "app.hpp"
#include "utils.hpp"

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

	g_app.init();
	for (;;)
	{
		g_app.run();
	}
}
