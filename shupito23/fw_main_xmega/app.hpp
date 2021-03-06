#ifndef SHUPITO_SHUPITO23_APP_HPP
#define SHUPITO_SHUPITO23_APP_HPP

#include "clock.hpp"
#include "spi.hpp"
#include "usart.hpp"
#include "hiv.hpp"
#include "voltage.hpp"
#include "led.hpp"
#include "../../fw_common/pdi.hpp"
#include "../../fw_common/handler_base.hpp"
#include "pins.hpp"
#include "../../fw_common/avrlib/stopwatch.hpp"
#include "../../fw_common/avrlib/async_usart.hpp"
#include "../../fw_common/avrlib/usart_xc1.hpp"

#include "../../fw_common/handler_base.hpp"
#include "../../fw_common/handler_xmega.hpp"
#include "../../fw_common/handler_avricsp.hpp"
#include "../../fw_common/handler_spi.hpp"
#include "../../fw_common/handler_uart.hpp"
#include "handler_jtag_fast.hpp"

typedef pdi_t<clock_t, pin_aux_rst, pin_pdi, led_holder> my_pdi_t;

static uint8_t volatile const * sn_calib_indexes[] = {
	&PRODSIGNATURES_LOTNUM0,
	&PRODSIGNATURES_LOTNUM1,
	&PRODSIGNATURES_LOTNUM2,
	&PRODSIGNATURES_LOTNUM3,
	&PRODSIGNATURES_LOTNUM4,
	&PRODSIGNATURES_LOTNUM5,
	&PRODSIGNATURES_WAFNUM,
	&PRODSIGNATURES_COORDX0,
	&PRODSIGNATURES_COORDX1,
	&PRODSIGNATURES_COORDY0,
	&PRODSIGNATURES_COORDY1,
};

static uint8_t const sn_calib_indexes_count = sizeof sn_calib_indexes / sizeof sn_calib_indexes[0];

struct process_t
{
	struct led
	{
		led()
		{
			h.on();
		}

		led_holder h;
	};

	void allow_tunnel();
	void disallow_tunnel();
	void operator()() const;
};

struct process_with_debug_t
{
	void operator()() const;
};

extern process_t g_process;
extern process_with_debug_t g_process_with_debug;

class usb_yb_writer
	: public yb_writer
{
public:
	usb_yb_writer();
	uint8_t avail() const;
	uint8_t * alloc(uint8_t cmd, uint8_t size);
	uint8_t * alloc_sync(uint8_t cmd, uint8_t size);
	void commit();
	bool send(uint8_t cmd, uint8_t const * data, uint8_t size);
	void send_sync(uint8_t cmd, uint8_t const * data, uint8_t size);

private:
	uint8_t m_size;
};

class app
{
public:
	app();
	void init();
	void run();

	bool send_vccio_state(yb_writer & w);
	bool send_vccio_drive_list(yb_writer & w);
	void set_vccio_drive(uint8_t value);
	bool handle_packet(uint8_t cmd, uint8_t const * cp, uint8_t size, yb_writer & w);
	uint8_t select_handler(handler_base * new_handler);
	void process_with_debug();

	void open_tunnel(uint8_t which, uint32_t baudrate, uint8_t mode);
	static void get_baudctrl(uint32_t baudrate, uint16_t & baudctrl, bool & dblspeed);
	void close_tunnel();

	void allow_tunnel();
	void disallow_tunnel();

	void disable_pwm();
	bool send_pwm_settings();

private:
	char m_usb_sn[2*sn_calib_indexes_count];

	usb_yb_writer m_usb_writer;

	avrlib::timeout<clock_t> m_vccio_timeout;
	int16_t m_vccio_voltage;
	bool m_send_vcc_driver_list_scheduled;
	bool m_send_vccio_state_scheduled;
	enum { vccio_disabled, vccio_enabled, vccio_active } m_vccio_drive_state;
	avrlib::timeout<clock_t> m_vccio_drive_check_timeout;
	voltage_filter<32> m_vccio_filter;

	bitbang_t m_bitbang;

	handler_avricsp<spi_t, clock_t, pin_rst, process_t> m_handler_avricsp;
	handler_xmega<my_pdi_t, clock_t, process_t> m_handler_pdi;
	handler_spi<spi_t, pin_aux_rst> m_handler_spi;
	handler_uart<usart_t, bitbang_t> m_handler_uart;
	handler_jtag_fast m_handler_jtag;
	handler_base * m_handler;

	bool m_tunnel_open;
	bool m_tunnel_allowed;
	uint16_t m_tunnel_baudctrl;
	bool m_tunnel_dblspeed;
	uint8_t m_tunnel_mode;

	bool m_assumed_btn_state;

	bool m_send_pwm_scheduled;
	uint8_t m_pwm_kind;
	uint32_t m_pwm_period;
	uint32_t m_pwm_duty_cycle;
};

extern app g_app;

#endif // SHUPITO_SHUPITO23_APP_HPP
