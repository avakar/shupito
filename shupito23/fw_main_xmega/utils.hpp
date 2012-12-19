#ifndef SHUPITO_SHUPITO23_UTILS_HPP
#define SHUPITO_SHUPITO23_UTILS_HPP

#include <avr/io.h>
#include <avr/interrupt.h>
#include "pins.hpp"

#define STRINGIFY2(x) #x
#define STRINGIFY(x) STRINGIFY2(x)

/**
 * \brief Jumps to the FLIP bootloader.
 *
 * Make sure that all peripherals are reset to default settings,
 * before starting the bootloader. Ideally, reset the CPU and
 * then call this function as the first thing in main.
 */
void start_flip_bootloader() __attribute__((noreturn));

bool software_reset_occurred();

/**
 * \brief Resets the CPU. Interrupts must be disabled.
 */
void initiate_software_reset() __attribute__((noreturn));
void enable_interrupts();
void setup_bootloader_button();

#define wait_until(cond) do {} while(!(cond))

#endif // SHUPITO_SHUPITO23_UTILS_HPP
