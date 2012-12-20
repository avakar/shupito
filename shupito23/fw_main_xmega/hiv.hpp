#ifndef SHUPITO_HIV_HPP
#define SHUPITO_HIV_HPP

#include <stdint.h>

void hiv_init();
void hiv_process();

void hiv_enable();
void hiv_disable();

void hiv_setpoint_recip(uint32_t v);
uint8_t hiv_get_voltage();

#endif // SHUPITO_HIV_HPP
