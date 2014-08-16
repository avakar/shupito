#include "tunnel.hpp"
#include "pins.hpp"
#include "usb_eps.hpp"
#include "led.hpp"
#include "../../fw_common/avrlib/assert.hpp"
#include "../../fw_common/avrlib/atomic.hpp"
#include "dbg.h"
#include <avr/io.h>
#include <avr/interrupt.h>

static void usart_trasfer_config();
static void usb_in_tunnel_config();
static void usb_in_tunnel_start();
static void usb_in_tunnel_stop();
static void usb_out_tunnel_config();
static void usb_out_tunnel_deconfig();
static void usb_out_tunnel_poll();
static void usb_out_tunnel_start();
static void usb_out_tunnel_stop();

enum tunnel_mode_t { tm_usb, tm_app };
static tunnel_mode_t volatile g_mode;

void usb_tunnel_config()
{
	usart_trasfer_config();
	usb_out_tunnel_config();
	usb_in_tunnel_config();
}

void usb_tunnel_deconfig()
{
	usb_out_tunnel_deconfig();
}

void usb_tunnel_poll()
{
	usb_out_tunnel_poll();
}

static void start_impl(uint16_t baudctrl, uint8_t mode, bool dblspeed, tunnel_mode_t tm)
{
	usb_out_tunnel_stop();
	usb_in_tunnel_stop();
	USARTC1_CTRLB = 0;

	g_mode = tm;

	pin_txd::make_high();
	USARTC1_BAUDCTRLA = (uint8_t)(baudctrl);
	USARTC1_BAUDCTRLB = (uint8_t)(baudctrl >> 8);
	USARTC1_CTRLC = USART_CMODE_ASYNCHRONOUS_gc | mode;
	USARTC1_CTRLB = USART_RXEN_bm | USART_TXEN_bm | (dblspeed? USART_CLK2X_bm: 0);
	usb_in_tunnel_start();
	usb_out_tunnel_start();
}

static void stop_impl()
{
	usb_out_tunnel_stop();
	usb_in_tunnel_stop();
	USARTC1_CTRLB = 0;
	pin_txd::make_input();
}

void usb_tunnel_start(uint16_t baudctrl, uint8_t mode, bool dblspeed)
{
	start_impl(baudctrl, mode, dblspeed, tm_usb);
}

void app_tunnel_start(uint16_t baudctrl, uint8_t mode, bool dblspeed)
{
	start_impl(baudctrl, mode, dblspeed, tm_app);
}

void app_tunnel_stop()
{
	stop_impl();
}

void usb_tunnel_stop()
{
	stop_impl();
}

//---------------------------------------------------------------------
// OUT

static enum { tos_idle, tos_dma } tout_state = tos_idle;
static uint8_t tout_usb_buf[64];
static uint8_t tout_app_buf[64];

uint8_t app_tunnel_send(uint8_t const * v, uint8_t size)
{
	AVRLIB_ASSERT(g_mode == tm_app);
	if (tout_state != tos_idle)
		return 0;

	if (size > sizeof tout_app_buf)
		size = sizeof tout_app_buf;

	for (uint8_t i = 0; i < size; ++i)
		tout_app_buf[i] = v[i];

	tout_state = tos_dma;
	DMA_CH2_TRFCNT = size;
	DMA_CH2_CTRLB = DMA_CH_TRNIF_bm;
	DMA_CH2_CTRLA = DMA_CH_ENABLE_bm | DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;
	return size;
}

static void usb_out_tunnel_config()
{
	ep_descs->tunnel_out.DATAPTR = (uint16_t)tout_usb_buf;
	ep_descs->tunnel_out.STATUS = USB_EP_BUSNACK0_bm;
	ep_descs->tunnel_out.CTRL = USB_EP_INTDSBL_bm | USB_EP_TYPE_BULK_gc | USB_EP_BUFSIZE_64_gc;

	tout_state = tos_idle;

	// Setup the DMA channel to transfer from the USB EP3OUT to the USART C1
	// data register whenever the register becomes ready
	DMA_CH2_DESTADDR0 = (uint8_t)(uint16_t)&USARTC1_DATA;
	DMA_CH2_DESTADDR1 = (uint16_t)&USARTC1_DATA >> 8;
	DMA_CH2_DESTADDR2 = 0;

	DMA_CH2_ADDRCTRL = DMA_CH_SRCRELOAD_BLOCK_gc | DMA_CH_SRCDIR_INC_gc | DMA_CH_DESTRELOAD_NONE_gc | DMA_CH_DESTDIR_FIXED_gc;
	DMA_CH2_TRIGSRC = DMA_CH_TRIGSRC_USARTC1_DRE_gc;
}

static void usb_out_tunnel_deconfig()
{
	DMA_CH2_CTRLA = 0;
	while (DMA_CH2_CTRLA & DMA_CH_ENABLE_bm)
	{
	}
}

static void usb_out_tunnel_start()
{
	uint16_t buf_addr = g_mode == tm_usb? (uint16_t)tout_usb_buf: (uint16_t)tout_app_buf;
	DMA_CH2_SRCADDR0 = (uint8_t)buf_addr;
	DMA_CH2_SRCADDR1 = (uint8_t)(buf_addr >> 8);
	DMA_CH2_SRCADDR2 = 0;

	tout_state = tos_idle;
	if (g_mode == tm_usb)
		avrlib_atomic_clear(&ep_descs->tunnel_out.STATUS, USB_EP_BUSNACK0_bm);
}

static void usb_out_tunnel_stop()
{
	if (tout_state == tos_dma)
	{
		DMA_CH2_CTRLA = 0;
		while (DMA_CH2_CTRLA & DMA_CH_ENABLE_bm)
		{
		}
		tout_state = tos_idle;

		// Note that the USB transaction may continue to fill `tout_buf`,
		// there's no way to cancel it.
	}
}

static void usb_out_tunnel_poll()
{
	if (g_mode == tm_usb && tout_state == tos_idle && (ep_descs->tunnel_out.STATUS & USB_EP_BUSNACK0_bm))
	{
		tout_state = tos_dma;
		DMA_CH2_TRFCNT = ep_descs->tunnel_out.CNT;
		DMA_CH2_CTRLB = DMA_CH_TRNIF_bm;
		DMA_CH2_CTRLA = DMA_CH_ENABLE_bm | DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;
	}

	if (tout_state == tos_dma && (DMA_CH2_CTRLB & DMA_CH_TRNIF_bm))
	{
		tout_state = tos_idle;
		if (g_mode == tm_usb)
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
static uint8_t volatile tin_buf_sizes[tin_buf_count];
static uint8_t volatile tin_wrptr = 0;
static uint8_t volatile tin_used_bufs = 0;

static void usb_in_on_new_buffer(uint8_t ptr);

static bool volatile usart_transfer_enabled = false;

static void usart_trasfer_config()
{
	uint16_t data_ptr = (uint16_t)&USARTC1_DATA;
	DMA_CH3_SRCADDR0 = (uint8_t)data_ptr;
	DMA_CH3_SRCADDR1 = (uint8_t)(data_ptr >> 8);
	DMA_CH3_SRCADDR2 = 0;

	DMA_CH3_TRIGSRC = DMA_CH_TRIGSRC_USARTC1_RXC_gc;
	DMA_CH3_ADDRCTRL = DMA_CH_SRCRELOAD_NONE_gc | DMA_CH_SRCDIR_FIXED_gc | DMA_CH_DESTRELOAD_BLOCK_gc | DMA_CH_DESTDIR_INC_gc;
}

static void usb_in_tunnel_start()
{
	if (usart_transfer_enabled)
		return;

	cli();
	usart_transfer_enabled = true;
	if (tin_used_bufs < tin_buf_count)
		USARTC1_CTRLA = USART_RXCINTLVL_MED_gc;
	sei();
}

void usb_in_tunnel_stop()
{
	if (!usart_transfer_enabled)
		return;

	cli();
	USARTC1_CTRLA = 0;
	DMA_CH3_CTRLB = 0;
	usart_transfer_enabled = false;
	sei();

	DMA_CH3_CTRLA = 0;
	while (DMA_CH3_CTRLA & DMA_CH_ENABLE_bm)
	{
	}
}

ISR(USARTC1_RXC_vect)
{
	uint8_t wrptr = tin_wrptr;
	tin_bufs[wrptr][0] = USARTC1_DATA;
	tin_buf_sizes[wrptr] = 1;

	uint8_t wrptr_next = (wrptr + 1) & (tin_buf_count - 1);

	uint16_t buf_addr = (uint16_t)tin_bufs[wrptr_next];
	DMA_CH3_DESTADDR0 = (uint8_t)buf_addr;
	DMA_CH3_DESTADDR1 = (uint8_t)(buf_addr >> 8);
	DMA_CH3_DESTADDR2 = 0;
	DMA_CH3_TRFCNT = tin_buf_size;
	DMA_CH3_CTRLB = DMA_CH_TRNIF_bm | DMA_CH_TRNINTLVL_MED_gc;
	DMA_CH3_CTRLA = DMA_CH_ENABLE_bm | DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;

	tin_wrptr = wrptr_next;
	++tin_used_bufs;
	usb_in_on_new_buffer(wrptr);

	USARTC1_CTRLA = 0;
}

ISR(DMA_CH3_vect)
{
	DMA_CH3_CTRLB = DMA_CH_TRNIF_bm | DMA_CH_TRNINTLVL_MED_gc;

	uint8_t wrptr = tin_wrptr;
	uint8_t used_bufs = tin_used_bufs;

	// No DMA transaction should be active if all buffers are full or empty.
	AVRLIB_ASSERT(used_bufs < tin_buf_count && used_bufs != 0);

	uint8_t curptr = wrptr;

	wrptr = (wrptr + 1) & (tin_buf_count - 1);
	++used_bufs;

	if (used_bufs < tin_buf_count)
	{
		// The new wrptr is idle, start a DMA transaction into it.

		uint16_t buf_addr = (uint16_t)tin_bufs[wrptr];
		DMA_CH3_DESTADDR0 = (uint8_t)buf_addr;
		DMA_CH3_DESTADDR1 = (uint8_t)(buf_addr >> 8);
		DMA_CH3_DESTADDR2 = 0;
		DMA_CH3_TRFCNT = tin_buf_size;
		DMA_CH3_CTRLA = DMA_CH_ENABLE_bm | DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;
	}
	else
	{
		// We've ran out of buffers...
		led_blink_short();
	}

	tin_buf_sizes[curptr] = tin_buf_size;
	tin_wrptr = wrptr;
	tin_used_bufs = used_bufs;

	usb_in_on_new_buffer(curptr);
}

// needs MED interrupts disabled
void usart_transfer_on_buffer_pop(uint8_t rdptr)
{
	AVRLIB_ASSERT(tin_used_bufs != 0);
	--tin_used_bufs;
	if (!usart_transfer_enabled)
		return;

	uint8_t wrptr = tin_wrptr;
	uint8_t rdptr_next = (rdptr + 1) & (tin_buf_count - 1);

	bool restart_dma;
	uint8_t restart_wrptr = wrptr;
	uint8_t count = 0;
	if (rdptr_next == wrptr)
	{
		DMA_CH3_CTRLA = 0;
		while (DMA_CH3_CTRLA & DMA_CH_ENABLE_bm)
		{
		}

		if (DMA_CH3_CTRLB & DMA_CH_TRNIF_bm)
		{
			DMA_CH3_CTRLB = DMA_CH_TRNIF_bm | DMA_CH_TRNINTLVL_MED_gc;
			count = tin_buf_size;
		}
		else
		{
			uint8_t trfcnt = DMA_CH3_TRFCNT;
			if (trfcnt != tin_buf_size)
			{
				count = tin_buf_size - trfcnt;
			}
			else
			{
				USARTC1_CTRLA = USART_RXCINTLVL_MED_gc;
				return;
			}
		}

		restart_dma = true;
		restart_wrptr = (restart_wrptr + 1) & (tin_buf_count-1);
	}
	else
	{
		restart_dma = (rdptr == wrptr);
	}

	if (restart_dma)
	{
		// Restart the DMA transfer from USART to `tin_bufs`, because either
		//  1. all the buffers were full and we've just emptied one, or
		//  2. all but one of the buffers became empty and we've interrupted a DMA
		//     transfer to recover the non-empty one.
		uint16_t buf_addr = (uint16_t)tin_bufs[restart_wrptr];
		DMA_CH3_DESTADDR0 = (uint8_t)buf_addr;
		DMA_CH3_DESTADDR1 = (uint8_t)(buf_addr >> 8);
		DMA_CH3_DESTADDR2 = 0;
		DMA_CH3_TRFCNT = tin_buf_size;
		DMA_CH3_CTRLA = DMA_CH_ENABLE_bm | DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;
	}

	if (count != 0)
	{
		tin_buf_sizes[wrptr] = count;

		tin_wrptr = restart_wrptr;
		++tin_used_bufs;

		usb_in_on_new_buffer(wrptr);
	}
}

//---------------------------------------------------------------

static uint8_t tin_rdptr = 0;
static tunnel_mode_t tin_effective_mode = tm_usb;
static uint8_t tin_app_tunnel_used = 0;
static uint8_t tin_usb_tunnel_used = 0;

static void usb_in_on_new_buffer(uint8_t ptr)
{
	// MED interrupts must be disabled

	tunnel_mode_t mode = g_mode;

	if (mode == tm_usb)
		++tin_usb_tunnel_used;
	else
		++tin_app_tunnel_used;

	if (tin_effective_mode != mode)
	{
		if (tin_effective_mode == tm_usb && tin_usb_tunnel_used == 0)
			tin_effective_mode = tm_app;
		else if (tin_effective_mode == tm_app && tin_app_tunnel_used == 0)
			tin_effective_mode = tm_usb;
		else
			return;
	}

	if (mode == tm_usb && tin_usb_tunnel_used <= 2)
	{
		if (ptr & 1)
		{
			ep_descs->tunnel_in_alt.CNT = tin_buf_sizes[ptr];
			ep_descs->tunnel_in_alt.DATAPTR = (uint16_t)tin_bufs[ptr];
			avrlib_atomic_clear(&ep_descs->tunnel_in.STATUS, USB_EP_BUSNACK1_bm);
		}
		else
		{
			ep_descs->tunnel_in.CNT = tin_buf_sizes[ptr];
			ep_descs->tunnel_in.DATAPTR = (uint16_t)tin_bufs[ptr];
			avrlib_atomic_clear(&ep_descs->tunnel_in.STATUS, USB_EP_BUSNACK0_bm);
		}
	}
}

uint8_t app_tunnel_recv(uint8_t const *& data)
{
	if (tin_effective_mode != tm_app || tin_app_tunnel_used == 0)
		return 0;

	uint8_t rdptr = tin_rdptr;
	data = tin_bufs[rdptr];
	return tin_buf_sizes[rdptr];
}

void app_tunnel_recv_commit()
{
	cli();
	uint8_t rdptr = tin_rdptr;

	AVRLIB_ASSERT(tin_app_tunnel_used > 0);
	--tin_app_tunnel_used;

	// This could recursively initiate a new transfer
	tin_rdptr = (rdptr + 1) & (tin_buf_count - 1);
	usart_transfer_on_buffer_pop(rdptr);
	sei();
}

void usb_in_tunnel_config()
{
	ep_descs->tunnel_in_alt.CTRL = USB_EP_TYPE_DISABLE_gc;
	ep_descs->tunnel_in.STATUS = USB_EP_BUSNACK0_bm | USB_EP_BUSNACK1_bm;
	ep_descs->tunnel_in.CTRL = USB_EP_TYPE_BULK_gc | USB_EP_PINGPONG_bm | USB_EP_BUFSIZE_64_gc;
}

void usb_ep3_in_trnif()
{
	uint8_t rdptr = tin_rdptr;
	uint8_t used_bufs = tin_usb_tunnel_used;

	AVRLIB_ASSERT(used_bufs > 0);

	// This could recursively initiate a new transfer
	tin_usb_tunnel_used = used_bufs - 1;
	tin_rdptr = (rdptr + 1) & (tin_buf_count - 1);
	usart_transfer_on_buffer_pop(rdptr);

	if (used_bufs > 2)
	{
		uint8_t send_ptr = (rdptr + 2) & (tin_buf_count - 1);

		if (send_ptr & 1)
		{
			ep_descs->tunnel_in_alt.CNT = tin_buf_sizes[send_ptr];
			ep_descs->tunnel_in_alt.DATAPTR = (uint16_t)tin_bufs[send_ptr];
			avrlib_atomic_clear(&ep_descs->tunnel_in.STATUS, USB_EP_BUSNACK1_bm);
		}
		else
		{
			ep_descs->tunnel_in.CNT = tin_buf_sizes[send_ptr];
			ep_descs->tunnel_in.DATAPTR = (uint16_t)tin_bufs[send_ptr];
			avrlib_atomic_clear(&ep_descs->tunnel_in.STATUS, USB_EP_BUSNACK0_bm);
		}
	}
}
