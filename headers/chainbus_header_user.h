#pragma once
#include "chainbus_header_common.h"

#include <stddef.h>

// Chainbus header V0.2

/**
 * @brief Initializes the Chainbus system.
 *
 * configures hat selection pins
 * Also initializes the I2C and SPI peripherals for master mode.
 */
void chainbus_init();



/*
EEPROM map (M24C64, 32-byte pages)

Page 0 - 0x0000 .. 0x001F - RESERVED system page, holds no identification data
	0x0000 -> reserved (2 B)
	0x0002 -> id_data_pointer   (2 B, little endian)
	0x0004 -> user_data_pointer (2 B, little endian)
	0x0006 .. 0x001F -> reserved

Pages 1 and 2 - 0x0020 .. 0x005F - identification block, 64 B, located by
id_data_pointer (standard_eeprom_id_data_pointer). Offsets below are relative
to that pointer, see offsets_for_eeprom:
	+0x00 -> UUID              16 B  (128 bit)
	+0x10 -> name              24 B  fixed width, NOT NUL-terminated,
									 zero padded when shorter than 24 chars,
									 a 24-char name fills the whole field
	+0x28 -> software version   4 B
	+0x2C -> hardware revision  4 B
	+0x30 .. +0x3F -> reserved (16 B spare)

User data starts at user_data_pointer, which must be >= 0x0060.
*/

typedef enum
{
	offset_UUID = 0,
	offset_name = 16,
	offset_software_version = 40,
	offset_hardware_revision = 44
} offsets_for_eeprom;

#define standard_eeprom_id_data_pointer 0x0020
#define standard_eeprom_user_data_pointer 0x0060

void chainbus_uni_read_eeprom(Hat_position position, uint32_t addr, uint8_t *data, int32_t len);
void chainbus_uni_write_eeprom(Hat_position position, uint32_t addr, const uint8_t *data, int32_t len); // Make sure it works within page boundaries

void chainbus_uni_read_hat_data(Hat_position position, hat_data *struct_to_write);

void chainbus_uni_find_hat_position(int position, Hat_position *found_position);
void chainbus_uni_find_hat_UUID(const uint8_t *target_UUID, Hat_position *found_position); // target_UUID is 16 bytes (128 bit)
void chainbus_uni_find_hat_name(const char *target_name, Hat_position *found_position);	   // target_name is a C string of at most 24 chars