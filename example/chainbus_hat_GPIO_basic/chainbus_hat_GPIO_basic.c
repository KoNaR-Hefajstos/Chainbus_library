#include "chainbus_hat_GPIO_basic.h"
#include "../../include/chainbus_header_hat.h"
#include "../../include/chainbus_header_common.h"

#include <stdint.h>
#include <stddef.h>

#define TCA9535_ADDR 0x20

// TCA9535 register addresses
#define TCA9535_REG_INPUT_0 0x00
#define TCA9535_REG_INPUT_1 0x01
#define TCA9535_REG_OUTPUT_0 0x02
#define TCA9535_REG_OUTPUT_1 0x03
#define TCA9535_REG_CONFIG_0 0x06
#define TCA9535_REG_CONFIG_1 0x07

// all pin modes to floating (input / high-Z)
void chainbus_hat_gpio_basic_init(Hat_position position)
{
    chainbus_select_hat(position);
    chainbus_I2C_config_speed(chainbus_I2C_config_speed_standard);

    // Set output registers to 0
    uint8_t out0[2] = {TCA9535_REG_OUTPUT_0, 0x00};
    chainbus_I2C_write(TCA9535_ADDR, out0, 2);

    uint8_t out1[2] = {TCA9535_REG_OUTPUT_1, 0x00};
    chainbus_I2C_write(TCA9535_ADDR, out1, 2);

    // Set all pins as inputs (1 = input in TCA9535 config register)
    uint8_t cfg0[2] = {TCA9535_REG_CONFIG_0, 0xFF};
    chainbus_I2C_write(TCA9535_ADDR, cfg0, 2);

    uint8_t cfg1[2] = {TCA9535_REG_CONFIG_1, 0xFF};
    chainbus_I2C_write(TCA9535_ADDR, cfg1, 2);

    chainbus_deselect_hat(position);
}

// mode: chainbus_hat_gpio_basic_mode_input (0) or chainbus_hat_gpio_basic_mode_output (1)
void chainbus_hat_gpio_basic_set_mode(Hat_position position, int pin, int mode)
{
    // Safety check for your specific range 1-13
    if (pin < 1 || pin > 13)
        return;

    // Mapping: Pin 1-7 use Reg 0, Pin 8-13 use Reg 1
    uint8_t reg = (pin < 8) ? TCA9535_REG_CONFIG_0 : TCA9535_REG_CONFIG_1;
    // Mapping: Pin 1 becomes bit 0, Pin 8 becomes bit 0
    uint8_t bit = (pin < 8) ? (uint8_t)(pin - 1) : (uint8_t)(pin - 8);

    chainbus_select_hat(position);
    chainbus_I2C_config_speed(chainbus_I2C_config_speed_standard);

    uint8_t reg_addr[1] = {reg};
    uint8_t current_val = 0;
    chainbus_I2C_write_read(TCA9535_ADDR, reg_addr, 1, &current_val, 1);

    if (mode == chainbus_hat_gpio_basic_mode_input)
        current_val |= (1 << bit);
    else
        current_val &= ~(1 << bit);

    uint8_t data[2] = {reg, current_val};
    chainbus_I2C_write(TCA9535_ADDR, data, 2);

    chainbus_deselect_hat(position);
}
// 1 for high, 0 for low
void chainbus_hat_gpio_basic_write(Hat_position position, int pin, int value)
{
    if (pin < 1 || pin > 13)
        return;

    uint8_t reg = (pin < 8) ? TCA9535_REG_OUTPUT_0 : TCA9535_REG_OUTPUT_1;
    uint8_t bit = (pin < 8) ? (uint8_t)(pin - 1) : (uint8_t)(pin - 8);

    chainbus_select_hat(position);
    chainbus_I2C_config_speed(chainbus_I2C_config_speed_standard);

    uint8_t reg_addr[1] = {reg};
    uint8_t current_val = 0;
    chainbus_I2C_write_read(TCA9535_ADDR, reg_addr, 1, &current_val, 1);

    if (value)
        current_val |= (1 << bit);
    else
        current_val &= ~(1 << bit);

    uint8_t data[2] = {reg, current_val};
    chainbus_I2C_write(TCA9535_ADDR, data, 2);

    chainbus_deselect_hat(position);
}

void chainbus_hat_gpio_basic_read(Hat_position position, int pin, int *value)
{
    if (!value || pin < 1 || pin > 13)
        return;

    uint8_t reg = (pin < 8) ? TCA9535_REG_INPUT_0 : TCA9535_REG_INPUT_1;
    uint8_t bit = (pin < 8) ? (uint8_t)(pin - 1) : (uint8_t)(pin - 8);

    chainbus_select_hat(position);
    chainbus_I2C_config_speed(chainbus_I2C_config_speed_standard);

    uint8_t reg_addr[1] = {reg};
    uint8_t raw = 0;
    chainbus_I2C_write_read(TCA9535_ADDR, reg_addr, 1, &raw, 1);

    chainbus_deselect_hat(position);

    *value = (raw >> bit) & 0x01;
}