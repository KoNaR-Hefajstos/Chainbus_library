/*
 * Chainbus low level implementation for STM32F405RG (LQFP64), STM32Cube HAL, bare metal.
 *
 * This is an alternative to chainbus_low_ESP-C3.c, not a companion - both define the same
 * functions, so exactly one of them belongs in any given build.
 *
 * Every call here runs to completion before it returns and there is only one flow of
 * execution, so unlike the ESP-C3 backend nothing locks the bus - a select cannot land
 * while another HAT is still mid-transfer.
 *
 * chainbus_init() brings up the pins and all three peripherals itself. It assumes
 * HAL_Init() and the application's SystemClock_Config() have already run; it never touches
 * the PLL or the flash latency.
 *
 * Two things about this board worth knowing before wiring anything:
 *
 *  - PB3, PB4 and PA15 are the JTAG pins (JTDO, NJTRST, JTDI). Claiming them for SPI3 and
 *    the chip select ends JTAG debugging. SWD on PA13/PA14 is untouched and still works.
 *  - The HAL's blocking calls time out against HAL_GetTick(), so whatever the project uses
 *    as the HAL tick source has to be running.
 */

#include "chainbus_header_hat.h"
#include "chainbus_header_user.h"

#include "stm32f4xx_hal.h"

#include <stdbool.h>

/*
 * PINOUT
 */

// Selection lines. One per slot rather than a decoder, so position N is selected by pulling
// exactly one line low and leaving the other seven high.
static const struct
{
	GPIO_TypeDef *port;
	uint16_t pin;
} hat_sel[8] = {
	{GPIOC, GPIO_PIN_9}, // addr3 -> position 1
	{GPIOB, GPIO_PIN_5}, // addr4 -> position 2
	{GPIOA, GPIO_PIN_7}, // addr0 -> position 3
	{GPIOA, GPIO_PIN_6}, // addr1 -> position 4
	{GPIOA, GPIO_PIN_5}, // addr2 -> position 5
	{GPIOC, GPIO_PIN_6}, // addr5 -> position 6
	{GPIOC, GPIO_PIN_7}, // addr6 -> position 7
	{GPIOC, GPIO_PIN_8}, // addr7 -> position 8
						 // Dont ask me why it's this way. Artem's fault
};

#define HAT_SEL_COUNT 8

// SPI3
#define SPI_SCK_PORT GPIOB
#define SPI_SCK_PIN GPIO_PIN_3
#define SPI_MISO_PORT GPIOB
#define SPI_MISO_PIN GPIO_PIN_4
#define SPI_MOSI_PORT GPIOC
#define SPI_MOSI_PIN GPIO_PIN_12
#define SPI_CS_PORT GPIOA
#define SPI_CS_PIN GPIO_PIN_15

// I2C2
#define I2C_SCL_PORT GPIOB
#define I2C_SCL_PIN GPIO_PIN_10
#define I2C_SDA_PORT GPIOB
#define I2C_SDA_PIN GPIO_PIN_11

// USART3
#define UART_TX_PORT GPIOC
#define UART_TX_PIN GPIO_PIN_10
#define UART_RX_PORT GPIOC
#define UART_RX_PIN GPIO_PIN_11

// Starting SPI clock. Every HAT sets its own rate after chainbus_select_hat(), so this
// only covers the window before the first one does.
#define SPI_DEFAULT_SPEED_HZ (400 * 1000)

#define I2C_SPEED_STANDARD_HZ 100000
#define I2C_SPEED_FAST_HZ 400000

#define UART_DEFAULT_BAUDRATE 115200

// Same 1 second the ESP-C3 backend gives every transfer.
#define BUS_TIMEOUT_MS 1000

static I2C_HandleTypeDef hi2c2;
static SPI_HandleTypeDef hspi3;
static UART_HandleTypeDef huart3;

static void gpio_init_pin(GPIO_TypeDef *port, uint16_t pin, uint32_t mode, uint32_t pull, uint32_t alternate)
{
	GPIO_InitTypeDef cfg = {0};

	cfg.Pin = pin;
	cfg.Mode = mode;
	cfg.Pull = pull;
	cfg.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	cfg.Alternate = alternate;

	HAL_GPIO_Init(port, &cfg);
}

void chainbus_init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	// chainbus_delay_us() busy-waits on the cycle counter, which is part of the debug unit
	// and starts out switched off.
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	// Drive the output register high before switching the pins to outputs. The value is
	// latched either way, so the lines come up already deselected instead of glitching low
	// for the moment between HAL_GPIO_Init() and the first write.
	for (int i = 0; i < HAT_SEL_COUNT; i++)
		HAL_GPIO_WritePin(hat_sel[i].port, hat_sel[i].pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);

	for (int i = 0; i < HAT_SEL_COUNT; i++)
		gpio_init_pin(hat_sel[i].port, hat_sel[i].pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, 0);

	gpio_init_pin(SPI_CS_PORT, SPI_CS_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, 0);

	// I2C2. Open drain, and the internal pull-ups are only a fallback - the bus wants real
	// resistors on it.
	__HAL_RCC_I2C2_CLK_ENABLE();
	gpio_init_pin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_MODE_AF_OD, GPIO_PULLUP, GPIO_AF4_I2C2);
	gpio_init_pin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_MODE_AF_OD, GPIO_PULLUP, GPIO_AF4_I2C2);

	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = I2C_SPEED_STANDARD_HZ;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	HAL_I2C_Init(&hi2c2);

	// SPI3. Chip select is driven by hand, so the peripheral runs with software NSS.
	__HAL_RCC_SPI3_CLK_ENABLE();
	gpio_init_pin(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF6_SPI3);
	gpio_init_pin(SPI_MISO_PORT, SPI_MISO_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF6_SPI3);
	gpio_init_pin(SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF6_SPI3);

	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 10;
	HAL_SPI_Init(&hspi3);

	// USART3.
	__HAL_RCC_USART3_CLK_ENABLE();
	gpio_init_pin(UART_TX_PORT, UART_TX_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF7_USART3);
	gpio_init_pin(UART_RX_PORT, UART_RX_PIN, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART3);

	huart3.Instance = USART3;
	huart3.Init.BaudRate = UART_DEFAULT_BAUDRATE;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart3);

	// Parked low. All the handler does is move a byte into a ring buffer, and nothing
	// waits on it.
	HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(USART3_IRQn);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

	// Leaves every selection line high, so nothing is on the bus until the first select.
	chainbus_deselect_hat(0);
}

void chainbus_delay_us(uint32_t us)
{
	uint32_t start = DWT->CYCCNT;
	uint32_t cycles = us * (HAL_RCC_GetHCLKFreq() / 1000000U);

	while ((DWT->CYCCNT - start) < cycles)
	{
	}
}

/*
 * A millisecond at a time rather than one call of ms * 1000 microseconds, because the cycle
 * count in chainbus_delay_us() is a uint32_t and wraps somewhere past 25 seconds at 168 MHz.
 */
void chainbus_delay_ms(uint32_t ms)
{
	while (ms--)
		chainbus_delay_us(1000);
}

void chainbus_delay_s(uint32_t s)
{
	chainbus_delay_ms(s * 1000);
}

chainbus_select_return_t chainbus_deselect_hat(Hat_position pos)
{
	for (int i = 0; i < HAT_SEL_COUNT; i++)
		HAL_GPIO_WritePin(hat_sel[i].port, hat_sel[i].pin, GPIO_PIN_SET);

	return chainbus_select_ok;
}

chainbus_select_return_t chainbus_select_hat(Hat_position pos)
{
	// Raise everything first so no two lines are ever low at once, whatever was selected
	// before.
	for (int i = 0; i < HAT_SEL_COUNT; i++)
		HAL_GPIO_WritePin(hat_sel[i].port, hat_sel[i].pin, GPIO_PIN_SET);

	if (pos < 1 || pos > HAT_SEL_COUNT)
	{
		// Positions are 1-8. Anything else leaves every line high so the traffic that
		// follows reaches nothing, rather than landing on an arbitrary HAT.
		return chainbus_select_invalid_position;
	}

	HAL_GPIO_WritePin(hat_sel[pos - 1].port, hat_sel[pos - 1].pin, GPIO_PIN_RESET);

	return chainbus_select_ok;
}

/*
 * HAL_ERROR on its own says nothing useful, so the I2C error register is what actually
 * decides the code. Unlike the ESP-C3 backend this one can tell a NACK from a bus fault,
 * which is why chainbus_I2C_bus_error is reachable here.
 *
 * The peripheral still cannot say whether it was the address or the data that went
 * unanswered, so an AF stays the generic chainbus_I2C_NACK.
 */
static chainbus_I2C_return_t i2c_map(HAL_StatusTypeDef status)
{
	uint32_t err;

	if (status == HAL_OK)
		return chainbus_I2C_ok;

	// HAL_BUSY is the peripheral still working on something else, which is the "bus busy"
	// half of what chainbus_I2C_timeout covers.
	if (status == HAL_TIMEOUT || status == HAL_BUSY)
		return chainbus_I2C_timeout;

	err = HAL_I2C_GetError(&hi2c2);

	if (err & HAL_I2C_ERROR_AF)
		return chainbus_I2C_NACK;
	if (err & (HAL_I2C_ERROR_ARLO | HAL_I2C_ERROR_BERR))
		return chainbus_I2C_bus_error;
	if (err & HAL_I2C_ERROR_TIMEOUT)
		return chainbus_I2C_timeout;

	return chainbus_I2C_generic_error;
}

chainbus_I2C_return_t chainbus_I2C_write(uint8_t addr, const uint8_t *data, int32_t len)
{
	if (data == NULL || len < 0)
		return chainbus_I2C_invalid_argument;

	// The HAL wants the address already shifted up into bits 7:1, and takes a non-const
	// buffer even though a write never modifies it.
	return i2c_map(HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)(addr << 1), (uint8_t *)data, (uint16_t)len, BUS_TIMEOUT_MS));
}

chainbus_I2C_return_t chainbus_I2C_read(uint8_t addr, uint8_t *data, int32_t len)
{
	if (data == NULL || len < 0)
		return chainbus_I2C_invalid_argument;

	return i2c_map(HAL_I2C_Master_Receive(&hi2c2, (uint16_t)(addr << 1), data, (uint16_t)len, BUS_TIMEOUT_MS));
}

/*
 * The F4 HAL has no blocking sequential transfer - HAL_I2C_Master_Seq_* exists only in
 * interrupt and DMA form. The one blocking call that issues a repeated START without a
 * STOP in between is HAL_I2C_Mem_Read, and its write phase is a 1 or 2 byte address.
 *
 * So longer writes are refused rather than quietly falling back to a separate transmit and
 * receive, which would put a STOP between the halves and break the guarantee the header
 * makes. Both callers in this project are inside the limit: chainbus_high.c sends a 2 byte
 * EEPROM address, the GPIO HAT sends a 1 byte register address.
 */
chainbus_I2C_return_t chainbus_I2C_write_read(uint8_t addr, const uint8_t *write_data, int32_t write_len, uint8_t *read_data, int32_t read_len)
{
	uint16_t mem_addr;
	uint16_t mem_size;

	if (write_data == NULL || read_data == NULL || read_len < 0)
		return chainbus_I2C_invalid_argument;

	if (write_len == 1)
	{
		mem_addr = write_data[0];
		mem_size = I2C_MEMADD_SIZE_8BIT;
	}
	else if (write_len == 2)
	{
		// Big endian, matching how the M24C64 and every register map here order it.
		mem_addr = (uint16_t)((write_data[0] << 8) | write_data[1]);
		mem_size = I2C_MEMADD_SIZE_16BIT;
	}
	else
	{
		return chainbus_I2C_invalid_argument;
	}

	return i2c_map(HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(addr << 1), mem_addr, mem_size, read_data, (uint16_t)read_len, BUS_TIMEOUT_MS));
}

static uint32_t i2c_cur_speed = I2C_SPEED_STANDARD_HZ;

chainbus_I2C_return_t chainbus_I2C_config_speed(uint32_t speed)
{
	uint32_t hz;

	if (speed == chainbus_I2C_config_speed_standard)
		hz = I2C_SPEED_STANDARD_HZ;
	else if (speed == chainbus_I2C_config_speed_fast)
		hz = I2C_SPEED_FAST_HZ;
	else
		return chainbus_I2C_invalid_argument;

	// HATs re-request their own rate on every select, and those repeats must not tear the
	// peripheral down and build it again.
	if (hz == i2c_cur_speed)
		return chainbus_I2C_ok;

	if (HAL_I2C_DeInit(&hi2c2) != HAL_OK)
		return chainbus_I2C_generic_error;

	hi2c2.Init.ClockSpeed = hz;

	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
		return chainbus_I2C_generic_error;

	i2c_cur_speed = hz;

	return chainbus_I2C_ok;
}

static chainbus_SPI_return_t spi_map(HAL_StatusTypeDef status)
{
	switch (status)
	{
	case HAL_OK:
		return chainbus_SPI_ok;
	case HAL_TIMEOUT:
		return chainbus_SPI_timeout;
	default:
		return chainbus_SPI_generic_error;
	}
}

chainbus_SPI_return_t chainbus_SPI_CS_select()
{
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET); // Active-low CS

	return chainbus_SPI_ok;
}

chainbus_SPI_return_t chainbus_SPI_CS_deselect()
{
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET); // Deassert CS

	return chainbus_SPI_ok;
}

chainbus_SPI_return_t chainbus_SPI_raw_write(const uint8_t *write_data, int32_t write_len)
{
	if (write_data == NULL || write_len < 0)
		return chainbus_SPI_invalid_argument;

	return spi_map(HAL_SPI_Transmit(&hspi3, write_data, (uint16_t)write_len, BUS_TIMEOUT_MS));
}

/*
 * No bounce buffer here. That was an ESP-C3 workaround for a DMA engine that rejects
 * unaligned receive buffers outright; these transfers are polled, so any buffer of any
 * length works and chainbus_SPI_buffer_alignment is never returned by this backend.
 */
chainbus_SPI_return_t chainbus_SPI_raw_read(uint8_t *read_data, int32_t read_len)
{
	if (read_data == NULL || read_len < 0)
		return chainbus_SPI_invalid_argument;

	return spi_map(HAL_SPI_Receive(&hspi3, read_data, (uint16_t)read_len, BUS_TIMEOUT_MS));
}

chainbus_SPI_return_t chainbus_SPI_raw_transfer(const uint8_t *write_data, uint8_t *read_data, int32_t len)
{
	if (write_data == NULL || read_data == NULL || len < 0)
		return chainbus_SPI_invalid_argument;

	return spi_map(HAL_SPI_TransmitReceive(&hspi3, write_data, read_data, (uint16_t)len, BUS_TIMEOUT_MS));
}

/*
 * Clock rate, mode and bit order are all fixed when the peripheral is initialised, so
 * changing any of them means tearing SPI3 down and building it again. That only happens
 * when the request actually differs from what is already live - HATs re-request their own
 * settings on every select, and those repeats must not churn the bus.
 */
static uint32_t spi_cur_speed = SPI_DEFAULT_SPEED_HZ;
static uint32_t spi_cur_mode = chainbus_SPI_config_mode_0;
static uint32_t spi_cur_bit_order = chainbus_SPI_config_bit_order_MSB_first;

/*
 * SPI3 hangs off APB1 and the only divisors are powers of two from 2 to 256, so most
 * requested rates are not reachable exactly. Pick the fastest one that does not exceed
 * what was asked for - going over could be past what the device tolerates, going under
 * only costs time.
 */
static uint32_t spi_prescaler_for(uint32_t speed_hz)
{
	static const uint32_t prescaler[8] = {
		SPI_BAUDRATEPRESCALER_2,
		SPI_BAUDRATEPRESCALER_4,
		SPI_BAUDRATEPRESCALER_8,
		SPI_BAUDRATEPRESCALER_16,
		SPI_BAUDRATEPRESCALER_32,
		SPI_BAUDRATEPRESCALER_64,
		SPI_BAUDRATEPRESCALER_128,
		SPI_BAUDRATEPRESCALER_256,
	};

	uint32_t pclk = HAL_RCC_GetPCLK1Freq();

	for (int i = 0; i < 8; i++)
	{
		if ((pclk >> (i + 1)) <= speed_hz)
			return prescaler[i];
	}

	// Asked for something slower than the bus can go. The slowest available is as close
	// as it gets.
	return SPI_BAUDRATEPRESCALER_256;
}

static chainbus_SPI_return_t spi_reconfigure(uint32_t speed_hz, uint32_t mode, uint32_t bit_order)
{
	if (speed_hz == spi_cur_speed && mode == spi_cur_mode && bit_order == spi_cur_bit_order)
		return chainbus_SPI_ok; // already live, nothing to do

	if (HAL_SPI_DeInit(&hspi3) != HAL_OK)
		return chainbus_SPI_config_failed;

	hspi3.Init.BaudRatePrescaler = spi_prescaler_for(speed_hz);
	hspi3.Init.CLKPolarity = (mode == chainbus_SPI_config_mode_2 || mode == chainbus_SPI_config_mode_3)
								 ? SPI_POLARITY_HIGH
								 : SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = (mode == chainbus_SPI_config_mode_1 || mode == chainbus_SPI_config_mode_3)
							  ? SPI_PHASE_2EDGE
							  : SPI_PHASE_1EDGE;
	// Anything that isn't an explicit LSB-first request stays MSB-first, which is what
	// every HAT here uses.
	hspi3.Init.FirstBit = (bit_order == chainbus_SPI_config_bit_order_LSB_first)
							  ? SPI_FIRSTBIT_LSB
							  : SPI_FIRSTBIT_MSB;

	// Note the peripheral is already down by this point, so unlike the ESP-C3 backend a
	// failure here cannot fall back on the previous settings - it leaves SPI3 uninitialised.
	if (HAL_SPI_Init(&hspi3) != HAL_OK)
		return chainbus_SPI_config_failed;

	spi_cur_speed = speed_hz;
	spi_cur_mode = mode;
	spi_cur_bit_order = bit_order;

	return chainbus_SPI_ok;
}

chainbus_SPI_return_t chainbus_SPI_config(chainbus_SPI_config_t new_config)
{
	// word_size is ignored - the raw helpers are byte-oriented, so anything other than 8
	// needs the transfer layer reworked first. Same limitation as the ESP-C3 backend.
	// mode is unsigned, so only the upper bound needs checking here.
	bool honoured = (new_config.word_size == 8) && (new_config.mode <= chainbus_SPI_config_mode_3);
	uint32_t mode = (new_config.mode <= chainbus_SPI_config_mode_3) ? new_config.mode : chainbus_SPI_config_mode_0;

	chainbus_SPI_return_t ret = spi_reconfigure(new_config.speed, mode, new_config.bit_order);
	if (ret != chainbus_SPI_ok)
		return ret;

	// The settings were applied, but with a fallback substituted for something that was
	// asked for and cannot be done - say so rather than reporting a clean success.
	return honoured ? chainbus_SPI_ok : chainbus_SPI_unsupported_config;
}

/*
 * UART receive.
 *
 * The Chainbus API hands out bytes that have already arrived, which needs a receiver
 * running the whole time rather than one armed per transfer. So the interrupt fills a ring
 * buffer and the read functions drain it.
 *
 * The size must stay a power of two - the wrap is a mask, not a modulo.
 */
#define UART_RX_BUF_SIZE 256
#define UART_RX_BUF_MASK (UART_RX_BUF_SIZE - 1)

#define UART_ERR_OVERRUN 0x01
#define UART_ERR_FRAMING 0x02
#define UART_ERR_PARITY 0x04

static volatile uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
static volatile uint16_t uart_rx_head;	// moved by the interrupt only
static volatile uint16_t uart_rx_tail;	// moved by the reader only
static volatile uint8_t uart_rx_errors; // sticky until read or cleared

/*
 * Deliberately not HAL_UART_IRQHandler(). That one drives the transfer-based API and
 * expects a pending HAL_UART_Receive_IT to hand bytes to, which a free-running receiver
 * never has.
 *
 * The status register is read once and everything works off that snapshot. Reading DR
 * afterwards clears RXNE and the error flags together, which is also why the HAL's
 * __HAL_UART_CLEAR_*FLAG macros are avoided here - they read DR themselves and would eat
 * the byte that arrived with the error.
 */
void USART3_IRQHandler(void)
{
	uint32_t sr = huart3.Instance->SR;

	if (sr & UART_FLAG_ORE)
		uart_rx_errors |= UART_ERR_OVERRUN;
	if (sr & UART_FLAG_FE)
		uart_rx_errors |= UART_ERR_FRAMING;
	if (sr & UART_FLAG_PE)
		uart_rx_errors |= UART_ERR_PARITY;

	if (sr & UART_FLAG_RXNE)
	{
		uint8_t byte = (uint8_t)(huart3.Instance->DR & 0xFFU);
		uint16_t next = (uint16_t)((uart_rx_head + 1) & UART_RX_BUF_MASK);

		if (next != uart_rx_tail)
		{
			uart_rx_buf[uart_rx_head] = byte;
			uart_rx_head = next;
		}
		else
		{
			// Nowhere to put it. Dropping the newest byte keeps the older ones, which are
			// the ones a reader part way through a frame still needs.
			uart_rx_errors |= UART_ERR_OVERRUN;
		}
	}
	else if (sr & (UART_FLAG_ORE | UART_FLAG_FE | UART_FLAG_PE))
	{
		(void)huart3.Instance->DR; // read to clear the error, nothing worth keeping
	}
}

static uint16_t uart_rx_available(void)
{
	// One read of head. The interrupt only ever moves it forward, so a byte arriving
	// mid-call makes this an undercount, never an overcount.
	uint16_t head = uart_rx_head;

	return (uint16_t)((head - uart_rx_tail) & UART_RX_BUF_MASK);
}

// Reports the first line error seen since the last time it was asked, and forgets it.
static chainbus_UART_return_t uart_take_errors(void)
{
	uint8_t errors = uart_rx_errors;

	uart_rx_errors = 0;

	if (errors & UART_ERR_OVERRUN)
		return chainbus_UART_overrun;
	if (errors & UART_ERR_FRAMING)
		return chainbus_UART_framing_error;
	if (errors & UART_ERR_PARITY)
		return chainbus_UART_parity_error;

	return chainbus_UART_ok;
}

static chainbus_UART_return_t uart_map(HAL_StatusTypeDef status)
{
	switch (status)
	{
	case HAL_OK:
		return chainbus_UART_ok;
	case HAL_TIMEOUT:
		return chainbus_UART_timeout;
	default:
		return chainbus_UART_generic_error;
	}
}

chainbus_UART_return_t chainbus_UART_send(const uint8_t *write_data, int32_t write_len)
{
	if (write_data == NULL || write_len < 0)
		return chainbus_UART_invalid_argument;

	return uart_map(HAL_UART_Transmit(&huart3, write_data, (uint16_t)write_len, BUS_TIMEOUT_MS));
}

/*
 * Takes nothing at all unless the whole amount is already there, so a short read cannot
 * leave the caller holding half a frame with no way to tell.
 *
 * A line error is reported even though the bytes were still handed over - the data is in
 * the buffer either way, and this is the only place the error would ever surface.
 */
chainbus_UART_return_t chainbus_UART_read_buffer(uint8_t *read_data, int32_t read_len)
{
	if (read_data == NULL || read_len < 0 || read_len > UART_RX_BUF_SIZE)
		return chainbus_UART_invalid_argument;

	if (uart_rx_available() < read_len)
		return chainbus_UART_not_enough_bytes;

	for (int32_t i = 0; i < read_len; i++)
	{
		read_data[i] = uart_rx_buf[uart_rx_tail];
		uart_rx_tail = (uint16_t)((uart_rx_tail + 1) & UART_RX_BUF_MASK);
	}

	return uart_take_errors();
}

chainbus_UART_return_t chainbus_UART_read_buffer_how_many_bytes(int32_t *how_many_bytes)
{
	if (how_many_bytes == NULL)
		return chainbus_UART_invalid_argument;

	*how_many_bytes = (int32_t)uart_rx_available();

	return chainbus_UART_ok;
}

chainbus_UART_return_t chainbus_UART_clear_read_buffer()
{
	__HAL_UART_DISABLE_IT(&huart3, UART_IT_RXNE);

	uart_rx_tail = uart_rx_head;
	uart_rx_errors = 0;

	// Drop whatever is sitting in the peripheral too, and clear any error latched with it.
	(void)huart3.Instance->SR;
	(void)huart3.Instance->DR;

	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

	return chainbus_UART_ok;
}

chainbus_UART_return_t chainbus_UART_config(chainbus_UART_config_t new_config)
{
	uint32_t parity;
	uint32_t word_length;
	uint32_t stop_bits;

	if (new_config.baudrate == 0)
		return chainbus_UART_invalid_argument;

	switch (new_config.parity)
	{
	case chainbus_UART_config_parity_none:
		parity = UART_PARITY_NONE;
		break;
	case chainbus_UART_config_parity_odd:
		parity = UART_PARITY_ODD;
		break;
	case chainbus_UART_config_parity_even:
		parity = UART_PARITY_EVEN;
		break;
	default:
		return chainbus_UART_invalid_argument;
	}

	// The STM32 counts the parity bit inside the word length, so eight data bits with
	// parity is a nine bit word. Only eight data bits are supported either way.
	if (new_config.word_length != 8)
		return chainbus_UART_unsupported_config;

	word_length = (parity == UART_PARITY_NONE) ? UART_WORDLENGTH_8B : UART_WORDLENGTH_9B;

	switch (new_config.stop_bits)
	{
	case chainbus_UART_config_stop_bits_1:
		stop_bits = UART_STOPBITS_1;
		break;
	case chainbus_UART_config_stop_bits_2:
		stop_bits = UART_STOPBITS_2;
		break;
	case chainbus_UART_config_stop_bits_half:
	case chainbus_UART_config_stop_bits_1_half:
		// The F4 UART has no half or one-and-a-half stop bit setting.
		return chainbus_UART_unsupported_config;
	default:
		return chainbus_UART_invalid_argument;
	}

	__HAL_UART_DISABLE_IT(&huart3, UART_IT_RXNE);

	if (HAL_UART_DeInit(&huart3) != HAL_OK)
		return chainbus_UART_generic_error;

	huart3.Init.BaudRate = new_config.baudrate;
	huart3.Init.WordLength = word_length;
	huart3.Init.StopBits = stop_bits;
	huart3.Init.Parity = parity;

	if (HAL_UART_Init(&huart3) != HAL_OK)
		return chainbus_UART_generic_error;

	// Anything received under the old frame format is meaningless now.
	uart_rx_head = 0;
	uart_rx_tail = 0;
	uart_rx_errors = 0;

	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

	return chainbus_UART_ok;
}
