#pragma once
#include "chainbus_header_common.h"

#include <stddef.h>

// Chainbus header V0.3

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

/*
Every chainbus_uni_* function below selects and deselects the HAT itself, so call them
with the bus free. Calling one while you already hold a selection deadlocks - the bus
lock is not recursive, so the inner select waits forever on a lock the same task holds.
*/

/**
 * @brief Reads bytes from the EEPROM of one HAT.
 *
 * Selects the HAT, sets the I2C bus to standard speed, sends the 16-bit word address as
 * a dummy write and reads the answer after a repeated START, then deselects.
 *
 * Reads may cross page boundaries freely - the EEPROM keeps clocking out sequential
 * bytes for as long as they are requested. Only writes care about pages.
 *
 * @param position HAT position, 1 to 8.
 * @param addr     Word address inside the EEPROM.
 * @param data     Buffer that receives the bytes, at least @p len long.
 * @param len      How many bytes to read.
 *
 * @note Nothing is reported back. An absent or silent HAT leaves the buffer holding
 *       whatever it held before, so clear it first if you need to tell the difference.
 */
void chainbus_uni_read_eeprom(Hat_position position, uint32_t addr, uint8_t *data, int32_t len);

/**
 * @brief Writes bytes to the EEPROM of one HAT.
 *
 * Splits the data at 32-byte page boundaries by itself and sends one chunk per page, so
 * the caller does not have to align anything. The HAT is selected and deselected around
 * each chunk.
 *
 * @param position HAT position, 1 to 8.
 * @param addr     Word address inside the EEPROM. Must be at or above
 *                 standard_eeprom_user_data_pointer to stay out of the identification
 *                 block.
 * @param data     Bytes to write.
 * @param len      How many bytes to write.
 *
 * @warning No wait is inserted between chunks for the EEPROM's internal write cycle,
 *          which the M24C64 needs about 5 ms for. The chip ignores commands while that
 *          runs, so anything longer than a single page will most likely keep only its
 *          first chunk - silently, since nothing is reported back. Keep writes inside
 *          one page, or wait about 5 ms yourself between calls.
 */
void chainbus_uni_write_eeprom(Hat_position position, uint32_t addr, const uint8_t *data, int32_t len);

/**
 * @brief Reads a HAT's whole identification block into a hat_data struct.
 *
 * Reads the reserved page 0 for the two pointers, then the 64-byte identification block
 * they point at, and fills in position, UUID, name, software version and hardware
 * revision. The name is NUL-terminated on the way in, so it can be used as a normal C
 * string even though the EEPROM stores no terminator.
 *
 * @param position        HAT position, 1 to 8.
 * @param struct_to_write Struct that receives the data. Must not be NULL.
 *
 * @note The pointers read from page 0 are not checked, so an empty or absent HAT fills
 *       the struct with whatever the failed reads left behind rather than reporting
 *       anything. Check the UUID against what you expect if that matters.
 */
void chainbus_uni_read_hat_data(Hat_position position, hat_data *struct_to_write);

/**
 * @brief Turns a plain position number into a Hat_position.
 *
 * @param position       Position number, 1 to 8.
 * @param found_position Receives the position. Ignored if NULL.
 *
 * @note Does not touch the bus and does not check that a HAT is actually there - it
 *       hands the number straight back. Use chainbus_uni_read_hat_data() if you need to
 *       know whether the slot is populated.
 */
void chainbus_uni_find_hat_position(int position, Hat_position *found_position);

/**
 * @brief Finds the first HAT whose UUID matches.
 *
 * Walks positions 1 to 8 in order, reads each HAT's identification pointer and skips
 * slots that read back blank (0xFFFF) or unprogrammed (0x0000), then compares the full
 * 128-bit UUID. Stops at the first match.
 *
 * @param target_UUID    UUID to look for, 16 bytes (128 bit). Returns immediately if NULL.
 * @param found_position Receives the matching position. Returns immediately if NULL.
 *
 * @note When nothing matches, @p found_position is left exactly as it was. Set it to a
 *       value you can recognise as "not found" before calling, otherwise a miss is
 *       indistinguishable from a hit.
 */
void chainbus_uni_find_hat_UUID(const uint8_t *target_UUID, Hat_position *found_position);

/**
 * @brief Finds the first HAT whose name matches.
 *
 * Same walk as chainbus_uni_find_hat_UUID(), comparing the 24-byte name field instead.
 * The field is zero padded, so a name shorter than 24 characters still matches.
 *
 * @param target_name    Name to look for, a C string of at most 24 chars. Returns
 *                       immediately if NULL.
 * @param found_position Receives the matching position. Returns immediately if NULL.
 *
 * @note Same "not found" behaviour as chainbus_uni_find_hat_UUID() - @p found_position
 *       is left untouched when nothing matches.
 */
void chainbus_uni_find_hat_name(const char *target_name, Hat_position *found_position);