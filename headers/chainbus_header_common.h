#pragma once

// Chainbus header V0.4

#include <stdint.h>
typedef uint16_t Hat_position;

// Sizes of the identification fields as they are stored in EEPROM
#define chainbus_uuid_size 16	  // 128-bit UUID, raw bytes
#define chainbus_name_size 24	  // fixed-width name field, NO terminator stored
#define chainbus_id_block_size 64 // whole identification block (2 EEPROM pages)

typedef struct
{
	Hat_position position;
	uint16_t eeprom_id_data_pointer;
	uint16_t eeprom_user_data_pointer;
	uint8_t UUID[chainbus_uuid_size]; // 128-bit UUID, byte order as stored in EEPROM
	// The EEPROM name field is exactly 24 bytes and is NOT NUL-terminated there.
	// This struct keeps one extra byte so the library can terminate it on read and
	// the name can be used as a normal C string.
	char name[chainbus_name_size + 1];
	uint32_t onboard_software_version;
	uint32_t hardware_revision;

} hat_data;

// Never delay while a HAT is selected - deselect, wait, reselect
// Holding the bus across a wait blocks every other HAT from using it

/**
 * @brief Busy-waits for the given number of microseconds
 * @param us Microseconds to wait.
 */
void chainbus_delay_us(uint32_t us);

/**
 * @brief Waits for the given number of milliseconds, RTOS non-blocking
 * @param ms Milliseconds to wait.
 */
void chainbus_delay_ms(uint32_t ms);

/**
 * @brief Waits for the given number of seconds, RTOS non-blocking
 * @param s Seconds to wait.
 */
void chainbus_delay_s(uint32_t s);