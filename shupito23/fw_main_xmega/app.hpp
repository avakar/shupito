#ifndef SHUPITO_SHUPITO23_APP_HPP
#define SHUPITO_SHUPITO23_APP_HPP

#include "clock.hpp"
#include "spi.hpp"
#include "../../fw_common/pdi.hpp"
#include "../../fw_common/handler_base.hpp"
#include "pins.hpp"
#include "../../fw_common/avrlib/stopwatch.hpp"

#include "../../fw_common/handler_base.hpp"
#include "../../fw_common/handler_xmega.hpp"
#include "../../fw_common/handler_avricsp.hpp"

typedef pdi_t<clock_t, pin_aux_rst, pin_pdi, pin_led> my_pdi_t;

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

private:
	char m_usb_sn[2*sn_calib_indexes_count];

	usb_yb_writer m_usb_writer;

	avrlib::timeout<clock_t> m_vccio_timeout;
	int16_t m_vccio_voltage;
	bool m_send_vcc_driver_list_scheduled;
	bool m_send_vccio_state_scheduled;
	enum { vccio_disabled, vccio_enabled, vccio_active } m_vccio_drive_state;
	avrlib::timeout<clock_t> m_vccio_drive_check_timeout;

	handler_avricsp<spi_t, clock_t, pin_rst, process_t> m_handler_avricsp;
	handler_xmega<my_pdi_t, clock_t, process_t> m_handler_pdi;
	handler_base * m_handler;
	
	bool m_in_packet_reported;
};

extern app g_app;

#endif // SHUPITO_SHUPITO23_APP_HPP
