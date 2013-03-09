#ifndef SHUPITO_SETTINGS_HPP
#define SHUPITO_SETTINGS_HPP

#include <stdint.h>

extern uint8_t g_namedesc[62];

bool load_settings();
void update_settings();

#endif // SHUPITO_SETTINGS_HPP
