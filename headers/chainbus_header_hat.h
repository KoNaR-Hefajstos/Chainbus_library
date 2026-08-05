#pragma once
#include "chainbus_header_common.h"
#include <stddef.h>
#include <stdbool.h>

// Chainbus header V0.3

/*
Documentation
All functions are blocking

*/

/**************************************************************************************************/
// Generic

/**
 * @brief Connects one HAT to the shared SPI, I2C and UART buses and puts RTOS mutex lock on chainbus
 *
 * Selecting does not restore bus settings - the previous HAT left its own behind, so
 * call chainbus_XXX_config() after this and before
 * any transfer.
 *
 * @param pos HAT position, 1 to 8
 */
void chainbus_select_hat(Hat_position pos);

/**
 * @brief Disconnects every HAT from the buses and releases the bus lock.
 *
 *
 * @param pos HAT position, 1 to 8
 */
void chainbus_deselect_hat(Hat_position pos);

/**************************************************************************************************/
// I2C

/**
 * @brief Writes bytes to an I2C device on the selected HAT.
 *
 * Sends START, the address with the write bit, every byte, then STOP.
 *
 * @param addr 7-bit device address, not shifted.
 * @param data pointer to bufer of bytes to send.
 * @param len  How many bytes to send.
 *
 */
void chainbus_I2C_write(uint8_t addr, const uint8_t *data, int32_t len);

/**
 * @brief Reads bytes from an I2C device on the selected HAT.
 *
 * Sends START, the address with the read bit, clocks out en bytes, then STOP.
 *
 * @param addr 7-bit device address, not shifted.
 * @param data pointer to bufer of bytes to fill
 * @param len  How many bytes to read.
 *
 */
void chainbus_I2C_read(uint8_t addr, uint8_t *data, int32_t len);

/**
 * @brief Writes then reads in one transaction, without releasing the bus in between.
 *
 * Sends START, the address with the write bit,  the register address, then a repeated START and the
 * read. No STOP separates the two halves, so nothing else can slip onto the bus and
 * move the device's internal pointer.
 *
 * @param addr      7-bit device address, not shifted.
 * @param write_data Bytes to send first, typically a register address.
 * @param write_len  How many bytes to send.
 * @param read_data  Buffer that receives the answer, at least  read_len long.
 * @param read_len   How many bytes to read.
 *
 */
void chainbus_I2C_write_read(uint8_t addr, const uint8_t *write_data, int32_t write_len, uint8_t *read_data, int32_t read_len); // uses repeated write

#define chainbus_I2C_config_speed_standard 1
#define chainbus_I2C_config_speed_fast 2

/**
 * @brief Sets the I2C clock rate.
 *
 * @param speed One of chainbus_I2C_config_speed_standard (100 kHz) or
 *              chainbus_I2C_config_speed_fast (400 kHz).
 *
 */
void chainbus_I2C_config_speed(uint32_t speed);

// void chainbus_I2C_ping(uint8_t addr, bool was_ACK); // so like write of 0 bytes, returns 0 for device, ping_none for nobody
// void chainbus_I2C_reset_bus();

// void chainbus_I2C_10bit_write();
// void chainbus_I2C_10bit_read();
// void chainbus_I2C_10bit_write_read();

/**************************************************************************************************/
// SPI
// len is in bytes

/**
 * @brief Clocks bytes out on MOSI and doesn't save what comes back on MISO.
 *
 * Does not touch chip-select - bracket the transfer with chainbus_SPI_CS_select() and
 * chainbus_SPI_CS_deselect() yourself.
 *
 * @param write_data Bytes to send.
 * @param write_len  How many bytes to send.
 */
void chainbus_SPI_raw_write(const uint8_t *write_data, int32_t write_len);

/**
 * @brief Clocks read_len bytes in on MISO, sending zeros on MOSI.
 *
 * Does not touch chip-select.
 *
 * @param read_data Buffer that receives the bytes, at least read_len long.
 * @param read_len  How many bytes to read.
 *.
 */
void chainbus_SPI_raw_read(uint8_t *read_data, int32_t read_len);

/**
 * @brief Full-duplex transfer - sends and receives len bytes at the same time.
 *
 * Does not touch chip-select.
 *
 * @param write_data Bytes to send.
 * @param read_data  Buffer that receives what arrives, at least @p len long. May be the
 *                   same buffer as @p write_data only if the caller is fine with it
 *                   being overwritten.
 * @param len        How many bytes to move, in each direction.
 *
 * @note Same failure and alignment behaviour as chainbus_SPI_raw_read().
 */
void chainbus_SPI_raw_transfer(const uint8_t *write_data, uint8_t *read_data, int32_t len);

// SPI chip-select, not the same thing as chainbus_select_hat()
// select hat -> connect SPI, I2C, UART buses to hat
// CS -> just a pin configured as output.

/**
 * @brief Asserts SPI chip-select by driving the CS pin low.
 *

 */
void chainbus_SPI_CS_select(); // Drives GPIO low

/**
 * @brief Releases SPI chip-select by driving the CS pin high.
 */
void chainbus_SPI_CS_deselect(); // drives GPIO high

typedef struct
{
	uint32_t speed;	   // clock rate in Hz
	uint8_t mode;	   // one of chainbus_SPI_config_mode_*
	uint8_t bit_order; // one of chainbus_SPI_config_bit_order_*
	// Bits per word, 8 or 16. 8 is by far the more common case and is the default -
	// leave it alone unless the device really needs 16-bit words. See the note above
	// the raw transfer functions for what word_size does to the buffers.
	uint32_t word_size;

} chainbus_SPI_config_t;

// Same numbering as every datasheet, so mode N here is mode N there.
#define chainbus_SPI_config_mode_0 0 // clock low when idle, read on rising edge
#define chainbus_SPI_config_mode_1 1 // clock low when idle, read on falling edge
#define chainbus_SPI_config_mode_2 2 // clock high when idle, read on falling edge
#define chainbus_SPI_config_mode_3 3 // clock high when idle, read on rising edge

#define chainbus_SPI_config_bit_order_MSB_first 1
#define chainbus_SPI_config_bit_order_LSB_first 2

/**
 * @brief Sets every SPI line parameter at once.
 *
 * The bus is shared, so whatever HAT ran last leaves its own settings behind. Call this
 * after chainbus_select_hat() and before any transfer, every time. Because it sets the
 * whole configuration in one go, nothing is left over from the previous HAT.
 *
 * Re-requesting settings that are already live costs nothing, so the repeat on every
 * select does not churn the bus.
 *
 * @param new_config Clock rate in Hz (a literal rate, e.g. .speed = 400 * 1000; a rate
 *                   the hardware cannot produce exactly becomes the nearest one it can),
 *                   mode, bit order and word size.
 *
 * @note word_size is ignored on the ESP32-C3 backend - transfers are always 8-bit.
 * @note An unrecognised mode falls back to chainbus_SPI_config_mode_0, and any bit_order
 *       other than chainbus_SPI_config_bit_order_LSB_first is treated as MSB first.
 */
void chainbus_SPI_config(chainbus_SPI_config_t new_config);

/**************************************************************************************************/
// UART

/**
 * @brief Sends bytes on the UART of the selected HAT.
 *
 * @param write_data Bytes to send.
 * @param write_len  How many bytes to send.
 */
void chainbus_UART_send(const uint8_t *write_data, int32_t write_len);

/**
 * @brief Takes bytes out of the UART read buffer.
 *
 * Ask chainbus_UART_read_buffer_how_many_bytes() first - reading more than has arrived
 * has nothing to hand back.
 *
 * @param read_data Buffer that receives the bytes, at least @p read_len long.
 * @param read_len  How many bytes to take.
 */
void chainbus_UART_read_buffer(uint8_t *read_data, int32_t read_len);

/**
 * @brief How many bytes are already waiting in the read buffer.
 *
 * Read fewer than this and the rest stay queued.
 *
 * @param how_many_bytes Receives the count.
 */
void chainbus_UART_read_buffer_how_many_bytes(int32_t *how_many_bytes);

/**
 * @brief Drops every queued byte and clears any framing/parity/overrun error left behind.
 */
void chainbus_UART_clear_read_buffer();

typedef struct
{
	uint32_t baudrate;	 // bits per second
	uint8_t word_length; //
	uint8_t stop_bits;	 // one of chainbus_UART_config_stop_bits_*
	uint8_t parity;		 // one of chainbus_UART_config_parity_*

} chainbus_UART_config_t;

#define chainbus_UART_config_stop_bits_1 1
#define chainbus_UART_config_stop_bits_2 2
#define chainbus_UART_config_stop_bits_half 3
#define chainbus_UART_config_stop_bits_1_half 4

#define chainbus_UART_config_parity_none 1
#define chainbus_UART_config_parity_odd 2
#define chainbus_UART_config_parity_even 3

/**
 * @brief Sets every UART line parameter at once.
 *
 * Same rule as SPI: the bus is shared, so whatever HAT ran last leaves its own frame
 * format behind. Call this after chainbus_select_hat() and before any transfer, every
 * time. Because it sets the whole configuration in one go, nothing is left over from
 * the previous HAT.
 *
 * @param new_config Bit rate (a literal rate, e.g. .baudrate = 115200; a rate the
 *                   hardware cannot produce exactly becomes the nearest one it can),
 *                   word length, stop bits and parity.
 */
void chainbus_UART_config(chainbus_UART_config_t new_config);
