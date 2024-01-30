/**
 ******************************************************************************
 * @file	: led.c
 * @authors	: Functions by	Zack Kohlman	<kohlmanz@msoe.edu>
 * 			: Comments by	Daniel Nimsgern	<nimsgernd@msoe.edu>
 *			:
 * @brief	: Functions for initializing and controlling the LED bar on the CE
 * 			: development board.
 ******************************************************************************
 */

/*
 ******************************************************************************
 * Includes
 ******************************************************************************
 */

// Library
#include <stdlib.h>

// Project
#include "gpio.h"
#include "led.h"

/*
 ******************************************************************************
 * Variables
 ******************************************************************************
 */

// Addresses
static volatile GPIO* const gpiob = (GPIO*)GPIOB_BASE;
static volatile RCC* const rcc = (RCC*)RCC_BASE;

/*
 ******************************************************************************
 * Function Definitions
 ******************************************************************************
 */

/**
 * @brief	Turns on the LED corresponding with the number (1 = right most).
 *
 * @param	number - A integer value corresponding to the binary values
 * 					 0b0000000000 to 0b1111111111.
 */
void led_enable(int number)
{
    // Only look at upper four bits first
    int temp = number & LED_MASK_U4;

    //Only look at first six bits
    int temp2 = number & LED_MASK_L6;

    //Put temps in right positions
    int temp3 = temp << 6;
    temp3 = temp3 | (temp2 << 5);

    // Clear all old led bits and set new ones in one operation
    gpiob->BSRR = (LED_MASK << 16) | temp3;
}

/**
 * @brief	Initializes the GPIO to utilize the LED bar.
 * 
 */
void led_init(void)
{
    // Enable clock for GPIOB
    rcc->AHB1ENR |= GPIOBEN;

    // Set LED pin modes
    gpiob->MODER = (gpiob->MODER & ~MODE_MASK) | GPIOB_LED_MODES;

    // Reset old values
    gpiob->ODR &= ~LED_MASK;
}
