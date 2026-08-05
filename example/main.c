#include <stuff.h>

// ChainBus
#include "chainbus_header_user.h"
#include "chainbus_hat_GPIO_basic.h"
// don't include chainbus_header_hat.h
// It's supposed to be only visible for hats
// you only will need to use functions from hat library

hat_data hat_gpio_basic;

#define pin_1 1

int main()
{

	/* Initialization */

	// initialize the entire chanbus, pin modes, all bussess
	chainbus_init();

	// find position of each hat, be it by name, UUID or absoulute position
	chainbus_uni_find_hat_position(1, &hat_gpio_basic.position);

	// initialize hats (turn off everything on them, set them to safe postions and settings)
	chainbus_hat_gpio_basic_init(hat_gpio_basic.position);

	/* Main user code */

	// set pin 1 mode of hat gpio as output
	chainbus_hat_gpio_basic_set_mode(hat_gpio_basic.position, pin_1, chainbus_hat_gpio_basic_mode_output);

	// blinky!
	while (1)
	{
		chainbus_delay_ms(1000);
		chainbus_hat_gpio_basic_write(hat_gpio_basic.position, pin_1, 1);
		chainbus_delay_ms(1000);
		chainbus_hat_gpio_basic_write(hat_gpio_basic.position, pin_1, 0);
	}
}
