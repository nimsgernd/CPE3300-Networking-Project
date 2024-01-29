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
 * IMPORTANT: INPUT SIGNAL ASSUMED TO BE ON ARDUINO HEADER/PIN PA_15!!!!
 */

/*
 ******************************************************************************
 * Includes
 ******************************************************************************
 */

// Library
#include <stdint.h>
#include <stdlib.h>

// Project
#include "delay.h"
#include "gpio.h"
#include "led.h"
#include "monitor.h"

/*
 ******************************************************************************
 * Variables
 ******************************************************************************
 */
volatile uint32_t previous_edge_time = 0; // Time of previous edge

// Definitions
#define IDLE_LED_STATE (int)0b1000000000	  // Left most LED value
#define BUSY_LED_STATE (int)0b0100000000	  // Second to left LED value
#define COLLISION_LED_STATE (int)0b0010000000 // Third to left LED value
#define ERROR_LED_STATE (int)0b1111111111	  // ALL LED value
#define MAX_16 0xFFFF

// Addresses
static volatile GPTIM16B32B *const tim2 = (GPTIM16B32B *)TIM2_BASE;
static volatile GPTIM16B *const tim14 = (GPTIM16B *) TIM14_BASE;
static volatile RCC *const rcc = (RCC *)RCC_BASE;
static volatile uint32_t *const nvic_iser0 = (uint32_t *)NVIC_BASE;
static volatile GPIO *const gpioa = (GPIO *)GPIOA_BASE;

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
	rcc->APB1ENR |= TIM2EN;

	// Enable clock for tim14
	rcc->APB1ENR |= TIM14EN;

	// Set PA6 to alternate function for TIC/TOC
	gpioa->MODER |= GPIOA_PA15_MODER_AF;

	// Set PA6 to alternate function in Alternate Function Register Low (for GPIO pins 0 - 7)
	gpioa->AFRL = AF1 << AFRH_PA15_AF1;

	// Open interrupt for TIM3
	*nvic_iser0 = TIM2_POS;

	// Set to TIC in compare compare mode register 1 in CC
	tim2->CCMR1 = CC1S;

	// Enable the capture compare for the channel
	tim2->CCER |= CC1E;

	// Set the direction of the input capture (rising edge, falling edge, or both)
	tim2->CCER |= CC1P | CC1NP; // Trigger on rising (CC1P) + falling edges (CC1NP)

	// Enable the interrupt on capture compare
	tim2->DIER |= CC1IE;

    // Set the auto-reload value to the maximum for a 16-bit counter
    tim3->ARR = MAX_16;

	// Enable the counter
	tim2->CR1 |= CEN;

	// Enable timer
	tim14->CR1 |= CEN;
}

/**
 * @brief	Updates status LEDs based on state.
 *
 */
void monitor(void)
{
	switch (state)
	{
	/* Idle */
	case IDLE:
	{
		led_enable(IDLE_LED_STATE); // Enables left most LED
	}
	break;

	/* Busy */
	case BUSY:
	{
		led_enable(BUSY_LED_STATE); // Enables second to left LED
	}
	break;

	/* Collision */
	case COLLISION:
	{
		led_enable(COLLISION_LED_STATE); // Enables third to left LED
		post_collision_delay();
	}
	break;

	/* Error */
	default:
	{
		led_enable(ERROR_LED_STATE); // Enables all LEDs
	}
	}
}

/**
 * @brief	Creates a randomized delay based on timer 14 which is acting as a
 * 			free running counter.
 *
 */
void post_collision_delay(void)
{
	// Reads count register from free running counter
	uint32_t count = tim14->CNT;

	// Scale count value by the scaler
	uint32_t microSecDelay = count * TIM_TO_MICROSEC_SCALAR;

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
	if (tim3->SR & CC1IF) // if the interrupt source is a capture event on channel 1
	{
		// Timer Ticks to Microseconds conversion
		uint32_t ticks = tim3->CCR1;									   // Get the number of ticks. Reading from CCR1 clears CC1IF bit in TIMx_SR
		uint32_t time_in_microseconds = (ticks / F_CPU) * 1e6;			   // Time_in_us = (TimerTicks/Timer Frequency) * 1,000,000
		uint32_t current_edge_time = time_in_microseconds;				   // Record the captured time
		uint32_t time_difference = current_edge_time - previous_edge_time; // Time since last edge

		// Determine edge type (rising/falling)
		int channel = (gpioa->IDR & GPIO_IDR_PA6); // Mask for bit 6 (PA6). [1 = PA6 is rising edge, 0 = PA6 is falling edge]
		//channel = channel >> 6;					   // Right shift to position 0


		if (state == IDLE)
		{
			// Any signal bus voltage edge switches to busy
			state = BUSY;
		}
		else if (state == BUSY)
		{
			// To prevent race condition at edges, uses < and > to keep in BUSY state
			/*TIMEOUT EVENTS*/
				// If 1.113ms >= edge_time >= 1.188ms and rising edge, idle
				// If 1.04ms >= edge_time >= 1.14ms and falling edge, collision
				// If edge_time > 1.13 ms and rising edge, idle
			if(channel)
			{
				if(time_difference > IDLE_MOE_LOW_US && time_difference < IDLE_MOE_HIGH_US)
				{
					state = IDLE;
				}
			} else
			{
				if(time_difference > COLLISION_MOE_LOW_US && time_difference < COLLISION_MOE_HIGH_US)
				{
					state = COLLISION;
				}
			}
		}
		else
		{
			// When in collision, any signal bus voltage edges bring it to the BUSY state
			state = BUSY;
		}
		previous_edge_time = current_edge_time; // update the time of the previous edge
	}
	tim3->SR &= ~CC1IF; // Clear the interrupt flag manually/by software if not set by capture event on channel 1
}
