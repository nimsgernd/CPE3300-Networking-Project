/**
 ******************************************************************************
 * @file	: monitor.c
 * @authors	: Zack Kohlman		<kohlmanz@msoe.edu>
 *			: Jack Maki			<makij@msoe.edu>
 *			: Daniel Nimsgern	<nimsgern@msoe.edu>
 * 			:
 * @brief	: Functions for initializing a network monitoring process and
 * 			: updating status LEDs.
 ******************************************************************************
 */

/**
 * IMPORTANT: INPUT SIGNAL ASSUMED TO BE ON ARDUINO HEADER/PIN PA_6!!!!
*/

/*
 ******************************************************************************
 * Includes
 ******************************************************************************
 */

// Library
#include <stdlib.h>
#include <stdint.h>

// Project
#include "monitor.h"
#include "gpio.h"
#include "led.h"
#include "delay.h"

/*
 ******************************************************************************
 * Variables
 ******************************************************************************
 */

// Definitions
#define IDLE_LED_STATE 	(int) 0b1000000000 // Left most LED value
#define BUSY_LED_STATE 		(int) 0b0100000000 // Second to left LED value
#define COLLISION_LED_STATE (int) 0b0010000000 // Third to left LED value
#define ERROR_LED_STATE		(int) 0b1111111111 // ALL LED value

// Addresses
static volatile TIMER* const tim3 = (TIMER*)TIM3_BASE;
static volatile RCC* const rcc = (RCC*)RCC_BASE;
static volatile uint32_t* const nvic_iser0 = (uint32_t*)NVIC_BASE;
static volatile GPIO* const gpioa = (GPIO*)GPIOA_BASE;
static volatile RTC* const rtc = (RTC*)RTC_BASE;

// State
static enum State state = IDLE; // Current state

/*
 ******************************************************************************
 * Function Prototypes
 ******************************************************************************
 */

static void post_collision_delay(void);

/*
 ******************************************************************************
 * Function Definitions
 ******************************************************************************
 */

/**
 * @brief	Initializes the monitoring program.
 *
 */
void monitor_init(void)
{
    // Enable clock for GPIOA
    rcc->AHB1ENR |= GPIOAEN;

    // Enable clock for tim3
    rcc->APB1ENR |= TIM3EN;

    // For additional timer, un-comment the following line for TIM4
    // rcc->APB1ENR |= TIM4EN; 

    // Enable internal low speed oscillator
    rcc->CSR |= LSION;

    // Select internal low speed oscillator for RTC
    rcc->BDCR |= RTCSEL_LSI;

    // Enable real time clock
    rcc->BDCR |= RTCEN;

    // Set PA6 to alternate function for TIC/TOC
    gpioa->MODER |= GPIOA_PA6_MODER_AF;

    // Set PA6 to alternate function in Alternate Function Register Low (for GPIO pins 0 - 7)
    gpioa->AFRL = AF2 << AFRL_PA6_AF2;

    // Open interrupt for TIM3
    *nvic_iser0 = TIM3_POS;

    // Set to TIC in compare compare mode register 1 in CC
    tim3->CCMR1 = CC1S;

    // Enable the capture compare for the channel
    tim3->CCER |= CC1E;

    // Set the direction of the input capture (rising edge, falling edge, or both)
    tim3->CCER |= CC1P | CC1NP; // Rising edge

    // Load the capture compare register with a value of 1.13ms (for TOC)
    //tim3->CCR1 = 0x2C240;
	
    // Enable the interrupt on capture compare
    tim3->DIER |= CC1IE;

    // Enable the counter
    tim3->CR1 |= CEN;
}

/**
 * @brief	Updates status LEDs based on state.
 *
 */
void monitor(void)
{
//	post_collision_delay();
	switch(state)
	{
		/* Idle */
		case IDLE:
		{
			led_enable(IDLE_LED_STATE); // Enables left most LED
		} break;

		/* Busy */
		case BUSY:
		{
			led_enable(BUSY_LED_STATE); // Enables second to left LED
		} break;

		/* Collision */
		case COLLISION:
		{
			led_enable(COLLISION_LED_STATE); // Enables third to left LED
		} break;

		/* Error */
		default:
		{
			led_enable(ERROR_LED_STATE); // Enables all LEDs
		}
	}
}

/**
 * @brief	Creates a randomized delay based on the real time clock then stops
 * 			process for that amount of time.
 *
 */
void post_collision_delay(void)
{
	// Read lower 16 bits of RTC time register
	uint32_t RT = rtc->TR & 0x0000FFFF;

	// Scale RT value by the scaler
	uint32_t microSecDelay = RT * RT_TO_MICROSEC_SCALAR;

	// Delay by this randomized time
	delay_us(microSecDelay);
}

// Timer 3 interrupt fires when the timer is active for over 1.13ms
/**
 * @brief	Interrupt service routine to monitor the network line state using
 * 			timer 3 with a time out of 1.13ms.
 *
 */
void TIM3_IRQHandler(void)
{
	// Clear interrupt flag
	tim3->SR = 0;
	int channel = (gpioa->IDR)&0x40; // Mask for bit 6 (PA6)
	channel = channel >> 6;          // Right shift to position 0
	if(channel == 1){
		// channel has been high for over 1.13ms, revert to idle
		state = IDLE;
	} else {
		// channel has been low for over 1.13ms, collision detected
		state = COLLISION;
	}
}
