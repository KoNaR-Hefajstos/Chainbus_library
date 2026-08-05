#include "chainbus_header_hat.h"
#include "chainbus_header_user.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#include "freertos/semphr.h"

SemaphoreHandle_t chainbus_mutex;

/*
 * PINOUT
 */

// SPI
#define SPI_MISO_IO 2
#define SPI_MOSI_IO 7
#define SPI_SCLK_IO 6
#define SPI_CS_IO 10

// I2C
#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_SDA_IO 8

// Selection Pins (3-to-8 decoder)
#define PIN_SEL_A0 3
#define PIN_SEL_A1 18
#define PIN_SEL_A2 19
#define PIN_SEL_EN 1

// I2C Configuration

// Starting SPI clock. Every HAT sets its own rate after chainbus_select_hat(), so this
// only covers the window before the first one does.
#define SPI_DEFAULT_SPEED_HZ (400 * 1000)

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

static spi_device_handle_t spi_handle;

void chainbus_init()
{
	chainbus_mutex = xSemaphoreCreateMutex();

	// Configure selection pins as push-pull outputs
	gpio_config_t sel_conf = {
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_OUTPUT,
		.pin_bit_mask = (1ULL << PIN_SEL_A0) | (1ULL << PIN_SEL_A1) | (1ULL << PIN_SEL_A2) | (1ULL << PIN_SEL_EN),
		.pull_down_en = 0,
		.pull_up_en = 0};
	gpio_config(&sel_conf);

	chainbus_deselect_hat(0);

	// Initialize I2C Master
	i2c_config_t i2c_conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = I2C_MASTER_SDA_IO,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = I2C_MASTER_SCL_IO,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ,
	};
	i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
	i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

	// Configure CS pin as output for manual toggling
	gpio_config_t cs_conf = {
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_OUTPUT,
		.pin_bit_mask = (1ULL << SPI_CS_IO),
		.pull_down_en = 0,
		.pull_up_en = 0};
	gpio_config(&cs_conf);
	gpio_set_level(SPI_CS_IO, 1);

	// Initialize SPI Master
	spi_bus_config_t buscfg = {
		.miso_io_num = SPI_MISO_IO,
		.mosi_io_num = SPI_MOSI_IO,
		.sclk_io_num = SPI_SCLK_IO,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 4096,
	};
	spi_device_interface_config_t devcfg = {
		.clock_speed_hz = SPI_DEFAULT_SPEED_HZ,
		.mode = 0,			// SPI mode 0
		.spics_io_num = -1, // Manual CS
		.queue_size = 7,
	};
	spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
	spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle);
}

void chainbus_delay_us(uint32_t us)
{
	esp_rom_delay_us((uint32_t)us);
}

/**
 * @brief Millisecond delay (Non-blocking / Yields to FreeRTOS)
 */
void chainbus_delay_ms(uint32_t ms)
{
	vTaskDelay(pdMS_TO_TICKS(ms));
}

/**
 * @brief Second delay (Non-blocking / Yields to FreeRTOS)
 */
void chainbus_delay_s(uint32_t s)
{
	chainbus_delay_ms(s * 1000);
}

chainbus_select_return_t chainbus_deselect_hat(Hat_position pos)
{
	if (chainbus_mutex == NULL)
		return chainbus_select_not_initialised;

	gpio_set_level(PIN_SEL_EN, 1); // Disable decoder
	gpio_set_level(PIN_SEL_A0, 0);
	gpio_set_level(PIN_SEL_A1, 0);
	gpio_set_level(PIN_SEL_A2, 0);

	// pdFALSE means this task was not holding the lock, so the select it should have been
	// paired with never happened. The bus is deselected either way.
	if (xSemaphoreGive(chainbus_mutex) != pdTRUE)
		return chainbus_select_generic_error;

	return chainbus_select_ok;
}

chainbus_select_return_t chainbus_select_hat(Hat_position pos)
{
	if (chainbus_mutex == NULL)
		return chainbus_select_not_initialised;

	xSemaphoreTake(chainbus_mutex, portMAX_DELAY);

	gpio_set_level(PIN_SEL_EN, 0); // Enable decoder
	// Truth table from header:
	// pos 1: A0=0, A1=0, A2=0 (binary 0 -> Y0)
	// pos 2: A0=1, A1=0, A2=0 (binary 1 -> Y1)
	// pos 3: A0=0, A1=1, A2=0 (binary 2 -> Y2)
	// pos 4: A0=1, A1=1, A2=0 (binary 3 -> Y3)
	// pos 5: A0=0, A1=0, A2=1 (binary 4 -> Y4)
	// pos 6: A0=1, A1=0, A2=1 (binary 5 -> Y5)
	// pos 7: A0=0, A1=1, A2=1 (binary 6 -> Y6)
	// pos 8: A0=1, A1=1, A2=1 (binary 7 -> Y7)
	switch (pos)
	{
	case 1:
		gpio_set_level(PIN_SEL_A0, 0);
		gpio_set_level(PIN_SEL_A1, 0);
		gpio_set_level(PIN_SEL_A2, 0);
		break;
	case 2:
		gpio_set_level(PIN_SEL_A0, 1);
		gpio_set_level(PIN_SEL_A1, 0);
		gpio_set_level(PIN_SEL_A2, 0);
		break;
	case 3:
		gpio_set_level(PIN_SEL_A0, 0);
		gpio_set_level(PIN_SEL_A1, 1);
		gpio_set_level(PIN_SEL_A2, 0);
		break;
	case 4:
		gpio_set_level(PIN_SEL_A0, 1);
		gpio_set_level(PIN_SEL_A1, 1);
		gpio_set_level(PIN_SEL_A2, 0);
		break;
	case 5:
		gpio_set_level(PIN_SEL_A0, 0);
		gpio_set_level(PIN_SEL_A1, 0);
		gpio_set_level(PIN_SEL_A2, 1);
		break;
	case 6:
		gpio_set_level(PIN_SEL_A0, 1);
		gpio_set_level(PIN_SEL_A1, 0);
		gpio_set_level(PIN_SEL_A2, 1);
		break;
	case 7:
		gpio_set_level(PIN_SEL_A0, 0);
		gpio_set_level(PIN_SEL_A1, 1);
		gpio_set_level(PIN_SEL_A2, 1);
		break;
	case 8:
		gpio_set_level(PIN_SEL_A0, 1);
		gpio_set_level(PIN_SEL_A1, 1);
		gpio_set_level(PIN_SEL_A2, 1);
		break;
	default:
		// Positions are 1-8. Anything else leaves the decoder disabled so the traffic
		// that follows reaches nothing, rather than enabling it with stale address lines
		// and silently landing on whichever HAT they happen to point at.
		//
		// The lock stays held on this path, so the caller's paired deselect still
		// balances - returning early without it would leave the bus locked forever.
		gpio_set_level(PIN_SEL_EN, 1);
		return chainbus_select_invalid_position;
	}

	return chainbus_select_ok;
}

/*
 * The legacy I2C driver collapses every no-acknowledge into a single ESP_FAIL - it cannot
 * say whether the address or the data went unanswered - so this returns the generic
 * chainbus_I2C_NACK rather than guessing between the two more specific codes.
 */
static chainbus_I2C_return_t i2c_map(esp_err_t err)
{
	switch (err)
	{
	case ESP_OK:
		return chainbus_I2C_ok;
	case ESP_ERR_INVALID_ARG:
		return chainbus_I2C_invalid_argument;
	case ESP_ERR_TIMEOUT:
		return chainbus_I2C_timeout;
	case ESP_FAIL:
		return chainbus_I2C_NACK;
	default:
		return chainbus_I2C_generic_error;
	}
}

chainbus_I2C_return_t chainbus_I2C_write(uint8_t addr, const uint8_t *data, int32_t len)
{

	return i2c_map(i2c_master_write_to_device(I2C_MASTER_NUM, addr, data, len, 1000 / portTICK_PERIOD_MS));
}

chainbus_I2C_return_t chainbus_I2C_read(uint8_t addr, uint8_t *data, int32_t len)
{

	return i2c_map(i2c_master_read_from_device(I2C_MASTER_NUM, addr, data, len, 1000 / portTICK_PERIOD_MS));
}

chainbus_I2C_return_t chainbus_I2C_write_read(uint8_t addr, const uint8_t *write_data, int32_t write_len, uint8_t *read_data, int32_t read_len)
{
	return i2c_map(i2c_master_write_read_device(I2C_MASTER_NUM, addr, write_data, write_len, read_data, read_len, 1000 / portTICK_PERIOD_MS));
}

chainbus_I2C_return_t chainbus_I2C_config_speed(uint32_t speed)
{
	// The rate is fixed at I2C_MASTER_FREQ_HZ in chainbus_init() and nothing here changes
	// it, so the only honest answer is "yes" for the rate that is already live and
	// "cannot do that" for the other one.
	if (speed == chainbus_I2C_config_speed_standard)
		return chainbus_I2C_ok;
	if (speed == chainbus_I2C_config_speed_fast)
		return chainbus_I2C_unsupported_config;

	return chainbus_I2C_invalid_argument;
}

void chianbu_spi_write(const uint8_t *data, size_t len)
{

	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = 8 * len;
	t.tx_buffer = data;

	spi_device_transmit(spi_handle, &t);
}

static chainbus_SPI_return_t spi_map(esp_err_t err)
{
	switch (err)
	{
	case ESP_OK:
		return chainbus_SPI_ok;
	case ESP_ERR_INVALID_ARG:
		return chainbus_SPI_invalid_argument;
	case ESP_ERR_TIMEOUT:
		return chainbus_SPI_timeout;
	default:
		return chainbus_SPI_generic_error;
	}
}

chainbus_SPI_return_t chainbus_SPI_CS_select()
{
	return spi_map(gpio_set_level(SPI_CS_IO, 0)); // Active-low CS
}

chainbus_SPI_return_t chainbus_SPI_CS_deselect()
{
	return spi_map(gpio_set_level(SPI_CS_IO, 1)); // Deassert CS
}

chainbus_SPI_return_t chainbus_SPI_raw_write(const uint8_t *write_data, int32_t write_len)
{
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = 8 * write_len;
	t.tx_buffer = write_data;
	t.rx_buffer = NULL;
	return spi_map(spi_device_transmit(spi_handle, &t));
}

/*
 * The bus runs with DMA (SPI_DMA_CH_AUTO), and ESP-IDF rejects a transaction outright
 * if the rx buffer is not 4-byte aligned or its length is not a multiple of 4 - it
 * returns ESP_ERR_INVALID_ARG and never writes a byte, which reads back as "the device
 * answered with zeros". HATs pass ordinary stack arrays of whatever length the register
 * map needs, so every read goes through an aligned, padded bounce buffer here instead of
 * making each HAT get it right.
 */
#define SPI_BOUNCE_MAX 64

static chainbus_SPI_return_t spi_transfer_bounced(const uint8_t *write_data, uint8_t *read_data, int32_t len)
{
	WORD_ALIGNED_ATTR uint8_t bounce[SPI_BOUNCE_MAX];
	bool bounced = (read_data && len <= SPI_BOUNCE_MAX);

	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = 8 * len;
	t.tx_buffer = write_data;

	if (bounced)
	{
		memset(bounce, 0, sizeof(bounce));
		// rxlength stays 0 (meaning "same as length") - it may not exceed length.
		// DMA still writes in whole words, but the overspill lands in the spare room
		// of the bounce buffer rather than past the end of the caller's array.
		t.rx_buffer = bounce;
	}
	else
	{
		t.rx_buffer = read_data;
	}

	esp_err_t err = spi_device_transmit(spi_handle, &t);
	if (err != ESP_OK)
	{
		// caller's buffer left untouched. A read too long to bounce went to DMA as-is, so
		// a rejected argument there is the alignment rule described above, not a bad call.
		if (err == ESP_ERR_INVALID_ARG && read_data && !bounced)
			return chainbus_SPI_buffer_alignment;

		return spi_map(err);
	}

	if (bounced)
		memcpy(read_data, bounce, len);

	return chainbus_SPI_ok;
}

chainbus_SPI_return_t chainbus_SPI_raw_read(uint8_t *read_data, int32_t read_len)
{
	return spi_transfer_bounced(NULL, read_data, read_len);
}

chainbus_SPI_return_t chainbus_SPI_raw_transfer(const uint8_t *write_data, uint8_t *read_data, int32_t len)
{
	return spi_transfer_bounced(write_data, read_data, len);
}

/*
 * ESP-IDF fixes clock rate and mode when the device is added to the bus, so changing
 * either means dropping the device and re-adding it. That is only done when the
 * requested settings actually differ from what is already live - HATs re-request their
 * own settings on every select, and those repeats must not churn the bus.
 *
 * Pre-registering one device per setting was the alternative, but an SPI host only
 * accepts 3 devices, which this project would outgrow.
 */
static int spi_cur_speed = SPI_DEFAULT_SPEED_HZ;
static int spi_cur_mode = 0; // ESP numbering, 0-3
static int spi_cur_bit_order = chainbus_SPI_config_bit_order_MSB_first;

static chainbus_SPI_return_t spi_reconfigure(int speed_hz, int esp_mode, int bit_order)
{
	if (speed_hz == spi_cur_speed && esp_mode == spi_cur_mode && bit_order == spi_cur_bit_order)
		return chainbus_SPI_ok; // already live, nothing to do

	spi_device_interface_config_t devcfg = {
		.clock_speed_hz = speed_hz,
		.mode = esp_mode,
		.spics_io_num = -1, // Manual CS
		.queue_size = 7,
		// Anything that isn't an explicit LSB-first request stays MSB-first, which is
		// the flagless default and what every HAT here uses.
		.flags = (bit_order == chainbus_SPI_config_bit_order_LSB_first)
					 ? (SPI_DEVICE_BIT_LSBFIRST | SPI_DEVICE_TXBIT_LSBFIRST)
					 : 0,
	};

	spi_device_handle_t new_handle;
	if (spi_bus_add_device(SPI2_HOST, &devcfg, &new_handle) != ESP_OK)
		return chainbus_SPI_config_failed; // keep the working device rather than leaving the bus with none

	spi_bus_remove_device(spi_handle);
	spi_handle = new_handle;
	spi_cur_speed = speed_hz;
	spi_cur_mode = esp_mode;
	spi_cur_bit_order = bit_order;

	return chainbus_SPI_ok;
}

// chainbus_SPI_config_mode_0..3 already match the ESP's 0-3, so this only range
// checks. Anything unrecognised falls back to mode 0.
static int spi_mode_to_esp(int mode)
{
	if (mode < chainbus_SPI_config_mode_0 || mode > chainbus_SPI_config_mode_3)
		return 0;
	return mode;
}

chainbus_SPI_return_t chainbus_SPI_config(chainbus_SPI_config_t new_config)
{
	// TODO: word_size is ignored - the ESP driver transfers whole bytes and the raw
	// helpers are byte-oriented, so anything other than 8 needs the transfer layer
	// reworked first.
	// mode is unsigned, so only the upper bound needs checking here.
	bool honoured = (new_config.word_size == 8) && (new_config.mode <= chainbus_SPI_config_mode_3);

	chainbus_SPI_return_t ret = spi_reconfigure(new_config.speed, spi_mode_to_esp(new_config.mode), new_config.bit_order);
	if (ret != chainbus_SPI_ok)
		return ret;

	// The settings were applied, but with a fallback substituted for something that was
	// asked for and cannot be done - say so rather than reporting a clean success.
	return honoured ? chainbus_SPI_ok : chainbus_SPI_unsupported_config;
}