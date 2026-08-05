#pragma once
#include "../../include/chainbus_header_common.h"
#include <stdint.h>

// all pin modes to floating
void chainbus_hat_gpio_basic_init(Hat_position position);

#define chainbus_hat_gpio_basic_mode_input 0
#define chainbus_hat_gpio_basic_mode_output 1

void chainbus_hat_gpio_basic_set_mode(Hat_position position, int pin, int mode);

// 1 for high, 0 for low
void chainbus_hat_gpio_basic_write(Hat_position position, int pin, int value);
void chainbus_hat_gpio_basic_read(Hat_position position, int pin, int *value);
