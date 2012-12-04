#ifndef SHUPITO_DBG_H
#define SHUPITO_DBG_H

#include "../../fw_common/avrlib/uart_xmega.hpp"
#include "../../fw_common/avrlib/usart_xe0.hpp"
#include "../../fw_common/avrlib/async_usart.hpp"
#include "../../fw_common/avrlib/format.hpp"

typedef avrlib::async_usart<avrlib::usart_xe0, 64, 64> com_dbg_t;
extern com_dbg_t com_dbg;

#include "../../fw_common/avrlib/memory_stream.hpp"

typedef avrlib::memory_stream<64, 64> com_usb_t;
extern com_usb_t com_usb;

#endif // SHUPITO_DBG_H
