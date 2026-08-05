#pragma once
#include "chainbus_header_common.h"
#include <stddef.h>
#include <stdbool.h>

// Chainbus header V0.2


/*
Documentation
All functions are blocking

*/


/**************************************************************************************************/
// Generic
void chainbus_select_hat(Hat_position pos);
void chainbus_deselect_hat(Hat_position pos);

// Never delay while a HAT is selected - deselect, wait, reselect. Holding the bus
// across a wait blocks every other HAT.
void chainbus_delay_us(uint32_t us);
void chainbus_delay_ms(uint32_t ms);
void chainbus_delay_s(uint32_t s);

/**************************************************************************************************/
// I2C
void chainbus_I2C_write(uint8_t addr, const uint8_t *data, int32_t len);
void chainbus_I2C_read(uint8_t addr, uint8_t *data, int32_t len);
void chainbus_I2C_write_read(uint8_t addr, const uint8_t *write_data, int32_t write_len, uint8_t *read_data, int32_t read_len); // uses repeated write

#define chainbus_I2C_config_speed_standard 1
#define chainbus_I2C_config_speed_fast 2
void chainbus_I2C_config_speed(uint32_t speed);

// void chainbus_I2C_ping(uint8_t addr, bool was_ACK); // so like write of 0 bytes, returns 0 for device, ping_none for nobody
// void chainbus_I2C_reset_bus();

// void chainbus_I2C_10bit_write();
// void chainbus_I2C_10bit_read();
// void chainbus_I2C_10bit_write_read();

/**************************************************************************************************/
// SPI
// The buffers are uint8_t* but should be thought of as void* - they are just raw
// memory, and how it is chopped up depends on word_size in chainbus_SPI_config.
// With the default word_size of 8 a byte is a word and there is nothing to think
// about. With word_size = 16 the buffer is really a uint16_t array, so cast it on
// the way in - chainbus_SPI_raw_write((const uint8_t *)my_u16_array, ...).
//
// The lengths are always in BYTES, never in words. So 10 words at word_size = 16
// means write_len = 20, and the length has to be even.
void chainbus_SPI_raw_write(const uint8_t *write_data, int32_t write_len);
void chainbus_SPI_raw_read(uint8_t *read_data, int32_t read_len);
void chainbus_SPI_raw_transfer(const uint8_t *write_data, uint8_t *read_data, int32_t len);
// SPI chip-select, not the same thing as chainbus_select_hat(). Call these after the
// HAT is already selected. CS is active-low, so select drives the line low.
void chainbus_SPI_CS_select();
void chainbus_SPI_CS_deselect();

typedef struct
{
	uint32_t speed;	   // clock rate in Hz
	uint8_t mode;	   // one of chainbus_SPI_config_mode_*
	uint8_t bit_order; // one of chainbus_SPI_config_bit_order_*
	// Bits per word, 8 or 16. 8 is by far the more common case and is the default -
	// leave it alone unless the device really needs 16-bit words. See the note above
	// the raw transfer functions for what word_size does to the buffers.
	uint32_t word_size;

} chainbus_SPI_config;

// Same numbering as every datasheet, so mode N here is mode N there.
#define chainbus_SPI_config_mode_0 0 // clock low when idle, read on rising edge
#define chainbus_SPI_config_mode_1 1 // clock low when idle, read on falling edge
#define chainbus_SPI_config_mode_2 2 // clock high when idle, read on falling edge
#define chainbus_SPI_config_mode_3 3 // clock high when idle, read on rising edge

#define chainbus_SPI_config_bit_order_MSB_first 1
#define chainbus_SPI_config_bit_order_LSB_first 2

// The bus is shared, so whatever HAT ran last leaves its own settings behind. Call
// this after chainbus_select_hat() and before any transfer, every time. It sets every
// line parameter at once, so there is nothing left over from the previous HAT.
//
// speed is a literal clock rate in Hz, e.g. .speed = 400 * 1000. Requesting a rate the
// hardware cannot produce exactly gets the nearest one it can.
void chainbus_SPI_config_full(chainbus_SPI_config new_config);

/**************************************************************************************************/
// UART
void chainbus_UART_send(const uint8_t *write_data, int32_t write_len);
void chainbus_UART_read_buffer(uint8_t *read_data, int32_t read_len);

// How many bytes are already waiting in the read buffer. Read fewer than this and
// the rest stay queued.
void chainbus_UART_read_buffer_how_many_bytes(int32_t *how_many_bytes);
// Drops every queued byte and clears any framing/parity/overrun error left behind.
void chainbus_UART_clear_read_buffer();

typedef struct
{
	uint8_t port;		 // one of chainbus_UART_config_port_*
	uint32_t baudrate;	 // bits per second
	uint8_t word_length; // data bits per frame
	uint8_t stop_bits;	 // one of chainbus_UART_config_stop_bits_*
	uint8_t parity;		 // one of chainbus_UART_config_parity_*

} chainbus_UART_config;

// A HAT has two UART connectors, so the config has to say which one the following
// transfers use. J12 is port 1, J13 is port 2.
#define chainbus_UART_config_port_1 1
#define chainbus_UART_config_port_2 2

#define chainbus_UART_config_stop_bits_1 1
#define chainbus_UART_config_stop_bits_2 2
#define chainbus_UART_config_stop_bits_half 3
#define chainbus_UART_config_stop_bits_1_half 4

#define chainbus_UART_config_parity_none 1
#define chainbus_UART_config_parity_odd 2
#define chainbus_UART_config_parity_even 3

// Same rule as SPI: the bus is shared, so whatever HAT ran last leaves its own frame
// format behind. Call this after chainbus_select_hat() and before any transfer, every
// time. It sets every line parameter at once, so there is nothing left over from the
// previous HAT.
//
// baudrate is a literal bit rate, e.g. .baudrate = 115200. Requesting a rate the
// hardware cannot produce exactly gets the nearest one it can.
void chainbus_UART_config_full(chainbus_UART_config new_config);