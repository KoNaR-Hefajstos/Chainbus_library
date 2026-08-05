# Chainbus_library
Abstraction layer for MMS3 Chainbus


This project contains abstraction layer for ChainBus

It's goal is to make all hat software portable between MCU's

If you see a way to write it better, Im open to improvements. This was just to get something going

## Folder Structure
#### headers - contains 3 headers which are common for all implemetation of chainbus

header hat - header included by hat's. It contains all functions to communicate with MMS3

header user - contains functions usable by user when writting the application code such as finding hat's or reading their EEPROM

#### source - contatins source code for user functions and example implementations for MCU's

chainbus_low_XXX - contains implementation for abstracted functions for specific MCU's

chainbus_high - contains implementation for user functions. They use abstracted functions and thus can be used on all MCUS

#### Examples - example hat libraries and main.c

main.c - Example code for GPIO basic hat blinky

./chainbus_hat_GPIO_basic - example library for hat

## Changelog
Chainbus header V0.2 -> inital version
