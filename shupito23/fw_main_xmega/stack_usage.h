#ifndef SHUPITO23_CHECK_STACK_CANARY_HPP
#define SHUPITO23_CHECK_STACK_CANARY_HPP

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t get_stack_usage(void);
uint16_t get_stack_size(void);

#ifdef __cplusplus
}
#endif

#endif
