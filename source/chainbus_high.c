#include "chainbus_header_user.h"
#include "chainbus_header_hat.h"

#include <string.h> // for memcpy
#include <stdint.h>

#define M24C64_addr 0x50

void chainbus_uni_read_eeprom(Hat_position position, uint32_t addr, uint8_t *data, int32_t len)
{
	chainbus_select_hat(position);
	chainbus_I2C_config_speed(chainbus_I2C_config_speed_standard);

	uint8_t i2c_addr = M24C64_addr;
	uint8_t addr_buf[2];

	// M24C64 expects High Byte then Low Byte
	addr_buf[0] = (uint8_t)(addr >> 8);
	addr_buf[1] = (uint8_t)(addr & 0xFF);

	// Use the combined write_read function to handle the "Dummy Write"
	// followed by the actual read transaction.
	chainbus_I2C_write_read(i2c_addr, addr_buf, 2, data, len);
	chainbus_deselect_hat(position);
}

void chainbus_uni_write_eeprom(Hat_position position, uint32_t addr, const uint8_t *data, int32_t len)
{
	uint8_t i2c_addr = M24C64_addr;

	// We need a buffer to hold [AddrH, AddrL, Data0...Data31]
	// Max size is 2 (address) + 32 (page size) = 34
	uint8_t write_buffer[34];

	while (len > 0)
	{
		chainbus_select_hat(position);
		chainbus_I2C_config_speed(chainbus_I2C_config_speed_standard);

		// 1. Calculate how many bytes we can write before hitting a page boundary (32 bytes)
		uint16_t bytes_to_boundary = 32 - (addr % 32);
		uint16_t chunk_size = (len < bytes_to_boundary) ? len : bytes_to_boundary;

		// 2. Prepare the packet: [Address High, Address Low, Data...]
		write_buffer[0] = (uint8_t)(addr >> 8);
		write_buffer[1] = (uint8_t)(addr & 0xFF);
		memcpy(&write_buffer[2], data, chunk_size);

		// 3. Send the write command
		chainbus_I2C_write(i2c_addr, write_buffer, chunk_size + 2);

		// 4. Update pointers and counters
		addr += chunk_size;
		data += chunk_size;
		len -= chunk_size;

		// 5. IMPORTANT: The EEPROM needs time to perform the internal write.
		// If your chainbus_I2C_write doesn't include a delay, you must add one here.
		// Most M24C64 chips require 5ms.
		// external_delay_ms(5); Nah don't care

		chainbus_deselect_hat(position);
	}
}

void chainbus_uni_read_hat_data(Hat_position position, hat_data *struct_to_load)
{
	uint8_t buffer[chainbus_id_block_size];

	// Read the reserved first page, it only holds the two pointers
	chainbus_uni_read_eeprom(position, 0x0000, buffer, 32);

	// Load data
	struct_to_load->eeprom_id_data_pointer = buffer[2] | (buffer[3] << 8);
	struct_to_load->eeprom_user_data_pointer = buffer[4] | (buffer[5] << 8);
	struct_to_load->position = position;

	// Read the identification block (2 pages) pointed to by id_data_pointer
	chainbus_uni_read_eeprom(position, struct_to_load->eeprom_id_data_pointer, buffer, chainbus_id_block_size);

	// Load data
	memcpy(struct_to_load->UUID, &buffer[offset_UUID], chainbus_uuid_size);
	memcpy(struct_to_load->name, &buffer[offset_name], chainbus_name_size);
	// The EEPROM name field carries no terminator, add one here
	struct_to_load->name[chainbus_name_size] = '\0';
	memcpy(&struct_to_load->onboard_software_version, &buffer[offset_software_version], 4);
	memcpy(&struct_to_load->hardware_revision, &buffer[offset_hardware_revision], 4);
}

void chainbus_uni_find_hat_position(int position, Hat_position *found_position)
{
	if (found_position != NULL)
	{
		*found_position = (Hat_position)position;
	}
}

// Compares the full 128-bit UUID, sets *found_position to the FIRST matching hat
void chainbus_uni_find_hat_UUID(const uint8_t *target_UUID, Hat_position *found_position)
{
	// Safety check for null pointers
	if (target_UUID == NULL || found_position == NULL)
	{
		return;
	}

	uint8_t buffer[chainbus_uuid_size];
	uint16_t id_ptr;

	// Iterate through all 8 possible hat positions, which are numbered 1-8
	for (int i = 1; i <= 8; i++)
	{
		Hat_position current_pos = (Hat_position)i;

		memset(buffer, 0, sizeof(buffer));

		// Read the ID Pointer (2 bytes) from the reserved page, address 0x0002
		// The pointer is stored in Little Endian
		chainbus_uni_read_eeprom(current_pos, 0x0002, buffer, 2);
		id_ptr = (uint16_t)(buffer[0] | (buffer[1] << 8));

		// 0xFFFF is a blank/unprogrammed EEPROM, 0x0000 is an invalid map
		if (id_ptr == 0xFFFF || id_ptr == 0x0000)
		{
			continue;
		}

		// UUID sits at the very start of the identification block
		chainbus_uni_read_eeprom(current_pos, id_ptr + offset_UUID, buffer, chainbus_uuid_size);

		if (memcmp(buffer, target_UUID, chainbus_uuid_size) == 0)
		{
			*found_position = current_pos;
			return;
		}
	}
}

// Sets *found_position to the FIRST hat whose name field matches

void chainbus_uni_find_hat_name(const char *target_name, Hat_position *found_position)
{
	// Safety check for null pointers
	if (target_name == NULL || found_position == NULL)
	{
		return;
	}

	uint8_t buffer[chainbus_name_size];
	uint16_t id_ptr;

	// Iterate through all 8 possible hat positions, which are numbered 1-8
	for (int i = 1; i <= 9; i++)
	{
		Hat_position current_pos = (Hat_position)i;

		// 1. Clear the buffer to prevent false matches from previous iterations
		memset(buffer, 0, sizeof(buffer));

		// 2. Read the ID Pointer (2 bytes) from the reserved page, address 0x0002
		// The pointer is stored in Little Endian
		chainbus_uni_read_eeprom(current_pos, 0x0002, buffer, 2);
		id_ptr = (uint16_t)(buffer[0] | (buffer[1] << 8));

		// 3. Validate the pointer
		// 0xFFFF is a blank/unprogrammed EEPROM
		// 0x0000 is likely an uninitialized/invalid map
		if (id_ptr == 0xFFFF || id_ptr == 0x0000)
		{
			continue;
		}

		// 4. Read the name field
		// Per the protocol: Name is at [id_ptr + 0x10] and is 24 bytes long,
		// zero padded and without a terminator
		chainbus_uni_read_eeprom(current_pos, id_ptr + offset_name, buffer, chainbus_name_size);

		// 5. Compare strings
		// strncmp is used to safely compare up to 24 characters, the buffer is
		// not terminated so the length must be capped here
		if (strncmp((char *)buffer, target_name, chainbus_name_size) == 0)
		{
			// Match found!
			*found_position = current_pos;

			// Return immediately so we only catch the FIRST hat matching this name
			return;
		}
	}
}