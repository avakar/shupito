#include "tunnel.hpp"
#include "pins.hpp"
#include "usb_eps.hpp"
#include "led.hpp"
#include "../../fw_common/avrlib/assert.hpp"
#include "../../fw_common/avrlib/atomic.hpp"
#include <avr/io.h>
#include <avr/interrupt.h>

static void usb_in_tunnel_config();
static void usb_in_tunnel_deconfig();
static void usb_out_tunnel_config();
static void usb_out_tunnel_deconfig();
static void usb_out_tunnel_poll();

void usb_tunnel_config()
{
	DMA_CTRL = DMA_ENABLE_bm;
	usb_out_tunnel_config();
	usb_in_tunnel_config();
}

void usb_tunnel_deconfig()
{
	usb_in_tunnel_deconfig();
	usb_out_tunnel_deconfig();
	DMA_CTRL = 0;
}

void usb_tunnel_poll()
{
	usb_out_tunnel_poll();
}

void usb_tunnel_start(uint16_t baudctrl, bool dblspeed)
{
	USARTC1_CTRLB = 0;
	pin_txd::make_high();

	USARTC1_BAUDCTRLA = (uint8_t)(baudctrl);
	USARTC1_BAUDCTRLB = (uint8_t)(baudctrl >> 8);
	USARTC1_CTRLC = USART_CMODE_ASYNCHRONOUS_gc | (3<<USART_CHSIZE_gp);
	USARTC1_CTRLB = USART_RXEN_bm | USART_TXEN_bm | (dblspeed? USART_CLK2X_bm: 0);
}

void usb_tunnel_stop()
{
	USARTC1_CTRLB = 0;
	pin_txd::make_input();
}

//---------------------------------------------------------------------
// OUT

static enum { tos_idle, tos_dma } tout_state = tos_idle;
static uint8_t tout_buf[64];

static void usb_out_tunnel_config()
{
	ep_descs->tunnel_out.DATAPTR = (uint16_t)tout_buf;
	ep_descs->tunnel_out.STATUS = 0;
	ep_descs->tunnel_out.CTRL = USB_EP_INTDSBL_bm | USB_EP_TYPE_BULK_gc | USB_EP_BUFSIZE_64_gc;

	tout_state = tos_idle;

	// Setup the DMA channel to transfer from the USB EP3OUT to the USART C1
	// data register whenever the register becomes ready
	DMA_CH0_DESTADDR0 = (uint8_t)(uint16_t)&USARTC1_DATA;
	DMA_CH0_DESTADDR1 = (uint16_t)&USARTC1_DATA >> 8;
	DMA_CH0_DESTADDR2 = 0;

	uint16_t buf_addr = (uint16_t)tout_buf;
	DMA_CH0_SRCADDR0 = (uint8_t)buf_addr;
	DMA_CH0_SRCADDR1 = (uint8_t)(buf_addr >> 8);
	DMA_CH0_SRCADDR2 = 0;

	DMA_CH0_ADDRCTRL = DMA_CH_SRCRELOAD_BLOCK_gc | DMA_CH_SRCDIR_INC_gc | DMA_CH_DESTRELOAD_NONE_gc | DMA_CH_DESTDIR_FIXED_gc;
	DMA_CH0_TRIGSRC = DMA_CH_TRIGSRC_USARTC1_DRE_gc;
}

static void usb_out_tunnel_deconfig()
{
	DMA_CH0_CTRLA = 0;
	while (DMA_CH0_CTRLA & DMA_CH_ENABLE_bm)
	{
	}
}

static void usb_out_tunnel_poll()
{
	if ((ep_descs->tunnel_out.STATUS & USB_EP_BUSNACK0_bm) && tout_state == tos_idle)
	{
		tout_state = tos_dma;
		DMA_CH0_TRFCNT = ep_descs->tunnel_out.CNT;
		DMA_CH0_CTRLB = DMA_CH_TRNIF_bm;
		DMA_CH0_CTRLA = DMA_CH_ENABLE_bm | DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;
	}

	if (tout_state == tos_dma && (DMA_CH0_CTRLB & DMA_CH_TRNIF_bm))
	{
		tout_state = tos_idle;
		avrlib_atomic_clear(&ep_descs->tunnel_out.STATUS, USB_EP_BUSNACK0_bm);
	}
}

//---------------------------------------------------------------------
// IN

// The number of buffers must be a power of two and at least 2.
// The USB endpoint for the tunnel is double-buffered,
// bank 0 reads from even buffers, bank 1 from odd.
//
// If no transfers are ready or in progress, the DMA is disabled
// and the USART RX interrupt is enabled. In this state,
// all buffers are idle and equivalent, the rdptr and wrptr can
// therefore be reset.
//
// When a byte arrives, we store it to buffer 0, start a USB transfer
// from it via bank 0 and set the DMA to transfer to buffer 1. Accordingly,
// rdptr is set to 0, wrptr is set to 1.
//
// Whenever a USB transaction completes, rdptr is incremented by one.
// Whenever a DMA transaction completes, wrptr is incremented by one.
// If rdptr == wrptr, all buffers are full and no DMA transaction
// is active.
//
// If rdptr + 1 == wrptr (everything is modulo tin_buf_count, of course),
// only one USB bank is ready. In such a case, finishing a DMA transaction
// will prepare another USB bank.

static uint8_t const tin_buf_size = 64;
static uint8_t const tin_buf_count = 16;
static uint8_t tin_bufs[tin_buf_count][tin_buf_size];
static uint8_t volatile tin_rdptr = 0;
static uint8_t volatile tin_wrptr = 0;

void usb_in_tunnel_config()
{
	ep_descs->tunnel_in_alt.CTRL = USB_EP_TYPE_DISABLE_gc;
	ep_descs->tunnel_in.STATUS = USB_EP_BUSNACK0_bm | USB_EP_BUSNACK1_bm;
	ep_descs->tunnel_in.CTRL = USB_EP_TYPE_BULK_gc | USB_EP_PINGPONG_bm | USB_EP_BUFSIZE_64_gc;

	uint16_t data_ptr = (uint16_t)&USARTC1_DATA;
	DMA_CH1_SRCADDR0 = (uint8_t)data_ptr;
	DMA_CH1_SRCADDR1 = (uint8_t)(data_ptr >> 8);
	DMA_CH1_SRCADDR2 = 0;

	DMA_CH1_TRIGSRC = DMA_CH_TRIGSRC_USARTC1_RXC_gc;
	DMA_CH1_ADDRCTRL = DMA_CH_SRCRELOAD_NONE_gc | DMA_CH_SRCDIR_FIXED_gc | DMA_CH_DESTRELOAD_BLOCK_gc | DMA_CH_DESTDIR_INC_gc;

	USARTC1_CTRLA = USART_RXCINTLVL_MED_gc;
}

void usb_in_tunnel_deconfig()
{
	cli();
	USARTC1_CTRLA = 0;
	DMA_CH1_CTRLB = 0;
	sei();

	DMA_CH1_CTRLA = 0;
	while (DMA_CH1_CTRLA & DMA_CH_ENABLE_bm)
	{
	}
}

ISR(USARTC1_RXC_vect)
{
	tin_bufs[0][0] = USARTC1_DATA;

	uint16_t buf_addr = (uint16_t)tin_bufs[1];
	DMA_CH1_DESTADDR0 = (uint8_t)buf_addr;
	DMA_CH1_DESTADDR1 = (uint8_t)(buf_addr >> 8);
	DMA_CH1_DESTADDR2 = 0;
	DMA_CH1_TRFCNT = tin_buf_size;
	DMA_CH1_CTRLB = DMA_CH_TRNIF_bm | DMA_CH_TRNINTLVL_MED_gc;
	DMA_CH1_CTRLA = DMA_CH_ENABLE_bm | DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;

	ep_descs->tunnel_in.CNT = 1;
	ep_descs->tunnel_in.DATAPTR = (uint16_t)tin_bufs[0];
	avrlib_atomic_clear(&ep_descs->tunnel_in.STATUS, USB_EP_BUSNACK0_bm | USB_EP_BANK_bm);

	tin_wrptr = 1;
	tin_rdptr = 0;

	USARTC1_CTRLA = 0;
}

ISR(DMA_CH1_vect)
{
	DMA_CH1_CTRLB = DMA_CH_TRNIF_bm | DMA_CH_TRNINTLVL_MED_gc;

	uint8_t wrptr = tin_wrptr;
	uint8_t rdptr = tin_rdptr;

	// No DMA transaction should be active if all buffers are full or empty.
	AVRLIB_ASSERT(rdptr != wrptr);

	uint8_t curptr = wrptr;

	wrptr = (wrptr + 1) & (tin_buf_count - 1);
	if (rdptr != wrptr)
	{
		// The new wrptr is idle, start a DMA transaction into it.

		uint16_t buf_addr = (uint16_t)tin_bufs[wrptr];
		DMA_CH1_DESTADDR0 = (uint8_t)buf_addr;
		DMA_CH1_DESTADDR1 = (uint8_t)(buf_addr >> 8);
		DMA_CH1_DESTADDR2 = 0;
		DMA_CH1_TRFCNT = tin_buf_size;
		DMA_CH1_CTRLA = DMA_CH_ENABLE_bm | DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;

		if (((rdptr + 1) & (tin_buf_count - 1)) == curptr)
		{
			// There was only one USB bank prepared, prepare the other one.
			if (curptr & 1)
			{
				ep_descs->tunnel_in_alt.CNT = tin_buf_size;
				ep_descs->tunnel_in_alt.DATAPTR = (uint16_t)tin_bufs[curptr];
				avrlib_atomic_clear(&ep_descs->tunnel_in.STATUS, USB_EP_BUSNACK1_bm);
			}
			else
			{
				ep_descs->tunnel_in.CNT = tin_buf_size;
				ep_descs->tunnel_in.DATAPTR = (uint16_t)tin_bufs[curptr];
				avrlib_atomic_clear(&ep_descs->tunnel_in.STATUS, USB_EP_BUSNACK0_bm);
			}
		}
	}
	else
	{
		// We've ran out of buffers...
		led_blink_short();
	}

	tin_wrptr = wrptr;
}

void usb_ep3_in_trnif()
{
	uint8_t rdptr = tin_rdptr;
	uint8_t wrptr = tin_wrptr;
	bool restart_dma = (rdptr == wrptr);

	rdptr = (rdptr + 1) & (tin_buf_count - 1);

	uint8_t count;
	if (rdptr == wrptr)
	{
		DMA_CH1_CTRLA = 0;
		while (DMA_CH1_CTRLA & DMA_CH_ENABLE_bm)
		{
		}

		if (DMA_CH1_CTRLB & DMA_CH_TRNIF_bm)
		{
			DMA_CH1_CTRLB = DMA_CH_TRNIF_bm | DMA_CH_TRNINTLVL_MED_gc;
			count = tin_buf_size;
		}
		else
		{
			uint8_t trfcnt = DMA_CH1_TRFCNT;
			if (trfcnt != tin_buf_size)
			{
				count = tin_buf_size - trfcnt;
			}
			else
			{
				USARTC1_CTRLA = USART_RXCINTLVL_MED_gc;

				// There's no need to update tin_rdptr or tin_wrptr,
				// we're going to idle and no buffer is busy.
				return;
			}
		}

		wrptr = (wrptr + 1) & (tin_buf_count-1);

		uint16_t buf_addr = (uint16_t)tin_bufs[wrptr];
		DMA_CH1_DESTADDR0 = (uint8_t)buf_addr;
		DMA_CH1_DESTADDR1 = (uint8_t)(buf_addr >> 8);
		DMA_CH1_DESTADDR2 = 0;
		DMA_CH1_TRFCNT = tin_buf_size;
		DMA_CH1_CTRLA = DMA_CH_ENABLE_bm | DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;

		tin_wrptr = wrptr;
	}
	else
	{
		count = tin_buf_size;
	}

	if (rdptr & 1)
	{
		ep_descs->tunnel_in_alt.CNT = count;
		ep_descs->tunnel_in_alt.DATAPTR = (uint16_t)tin_bufs[rdptr];
		avrlib_atomic_clear(&ep_descs->tunnel_in.STATUS, USB_EP_BUSNACK1_bm);
	}
	else
	{
		ep_descs->tunnel_in.CNT = count;
		ep_descs->tunnel_in.DATAPTR = (uint16_t)tin_bufs[rdptr];
		avrlib_atomic_clear(&ep_descs->tunnel_in.STATUS, USB_EP_BUSNACK0_bm);
	}

	if (restart_dma)
	{
		// The DMA transfer was inactive, but we have an idle buffer,
		// restart DMA into it.
		uint16_t buf_addr = (uint16_t)tin_bufs[wrptr];
		DMA_CH1_DESTADDR0 = (uint8_t)buf_addr;
		DMA_CH1_DESTADDR1 = (uint8_t)(buf_addr >> 8);
		DMA_CH1_DESTADDR2 = 0;
		DMA_CH1_TRFCNT = tin_buf_size;
		DMA_CH1_CTRLA = DMA_CH_ENABLE_bm | DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;
	}

	tin_rdptr = rdptr;
}
