#include <stdio.h>
#include <stdlib.h>
#include "uart_driver.h"
#include "gpio.h"
#include "led.h"

/*Addresses*/
static volatile GPIO* const gpiob = (GPIO*)GPIOB_BASE;
static volatile RCC* const rcc = (RCC*)RCC_BASE;


/**
 * Turns on the LED corresponding with the number. 1 = right most
*/
void led_enable(int number)
{
    // Clear all old led bits
    gpiob->ODR &= ~LED_MASK;

    // Only look at upper four bits first
    int temp = number & LED_MASK_U4;

    //Only look at first six bits
    int temp2 = number & LED_MASK_L6;

    //Put temps in right positions
    int temp3 = temp << 6;
    temp3 = temp3 | (temp2 << 5);
    
    // Write new value to ODR
    gpiob->ODR |= temp3;
}

/**
 * @brief Initializes the LED.
 * 
 * This function enables the GPIOB clock and configures the GPIOB pins
 * connected to the LED as output pins.
 */
void led_init(void)
{
    // Enable clock for GPIOB
    rcc->AHB1ENR |= GPIOBEN;

    // Set LED pin modes
    gpiob->MODER = (gpiob->MODER & ~MODE_MASK) | GPIOB_LED_MODES;
}
