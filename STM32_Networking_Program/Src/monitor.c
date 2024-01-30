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
#include <math.h>

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
volatile uint16_t  tim2_cnt = 0; // used for storing the count in timer 2
volatile uint16_t  tim8_cnt = 0; // used for storing the count in timer 8


// Definitions
#define IDLE_LED_STATE (int)0b1000000000	  // Left most LED value
#define BUSY_LED_STATE (int)0b0100000000	  // Second to left LED value
#define COLLISION_LED_STATE (int)0b0010000000 // Third to left LED value
#define ERROR_LED_STATE (int)0b1111111111	  // ALL LED value
#define MAX_16 0xFFFF

// Addresses
static volatile GPTIM16B32B *const tim2 = (GPTIM16B32B *)TIM2_BASE;
static volatile GPTIM16B *const tim8 = (GPTIM16B *)TIM8_BASE;
static volatile GPTIM16B *const tim14 = (GPTIM16B *) TIM14_BASE;
static volatile RCC *const rcc = (RCC *)RCC_BASE;
static volatile uint32_t *const iser = (uint32_t *)NVIC_BASE;

static volatile GPIO *const gpiob = (GPIO *)GPIOB_BASE;

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
	// Enable clock for GPIOB
	rcc->AHB1ENR |= GPIOBEN;

	// Enable clock for tim2
	rcc->APB1ENR |= TIM2EN;

    // Enable clock for tim8
    rcc->APB2ENR |= TIM8EN;

	// Enable clock for tim14
	rcc->APB1ENR |= TIM14EN;

    // Set the auto-reload value to achieve a period of 1.13 ms
    // ARR = (F_CPU / desired_frequency) - 1
    tim8->ARR = THRESHOLD_TICKS-1;

	// Set PA6 to alternate function for TIC/TOC
	gpiob->MODER |= GPIO_Px3_MODER_AF;

	// Set PA6 to alternate function in Alternate Function Register Low (for GPIO pins 0 - 7)
	gpiob->AFRL = AFRL_Px3_AF1;

	// Open interrupt for TIM3
	iser[0] = TIM2_POS;

    // Enable TIM8 interrupt in the NVIC
	iser[TIM8_UP_TIM13_POS >> 5] |= (1 << (TIM8_UP_TIM13_POS % 32));

	// Set to TIC in compare compare mode register 1 in CC
	tim2->CCMR1 = CC2S;

	// Enable the capture compare for the channel
	tim2->CCER |= CC2E;

	// Set the direction of the input capture (rising edge, falling edge, or both)
	tim2->CCER |= CC2P | CC2NP; // Trigger on rising (CC1P) + falling edges (CC1NP)

	// Enable the interrupt on capture compare
	tim2->DIER |= CC2IE;

    // Enable the timer interrupt
    tim8->DIER |= UIE;

    // Set the auto-reload value to the maximum for a 16-bit counter
    tim2->ARR = MAX_16;

	// Enable the counter
	tim2->CR1 |= CEN;

	// Enable counter
//	tim8->CR1 |= CEN;

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

void TIM8_UP_TIM13_IRQHandler(void)
{
	// Turn timer off
	tim8->CR1 &= ~CEN;

	// Check if the update interrupt flag is set
	if (tim8->SR & UIF)
	{
		// If count has not been updated
    	if ((tim2_cnt != 0) && (abs(tim8->CNT - tim2_cnt) > THRESHOLD_TICKS-1) && (state == BUSY))
    	{
    		// Check line state. High = idle, Low = collision
    		state = (gpiob->IDR & GPIO_IDR_Px3) ? IDLE : COLLISION;
    	}

    	// Reset count variable
    	tim2_cnt = 0;

        // Clear the update interrupt flag
    	tim8->SR &= ~UIF;
	}
}

// Timer 2 interrupt fires when the timer is active for over 1.13ms
/**
 * @brief	Interrupt service routine to monitor the network line state using
 * 			timer 3 with a time out of 1.13ms.
 *
 */
void TIM2_IRQHandler(void)
{
	if (tim2->SR & CC2IF) // if the interrupt source is a capture event on channel 1
	{
		// Store count values at the time of the most recent edge
		tim2_cnt = tim2->CCR2;
		tim8_cnt = tim8->CCR1;

		// Once edge detected, start 1.13 ms timer
		tim8->CR1 |= CEN;

		// From Idle, any signal bus voltage edge switches to busy
		// From Busy, only timeout events at 1.13ms switches back to idle (high) or collision (low)
		// If in collision, monitor main routine already brings it to BUSY
		// All other states, BUSY, and IDLE also go to BUSY if not timeout
		if (state == COLLISION)
		{
			post_collision_delay();
			state = BUSY;
		}
		else
		{
			state = BUSY;
		}

		// // Timer Ticks to Microseconds conversion
		// uint64_t ticks = tim2->CCR2;									   // Get the number of ticks. Reading from CCR1 clears CC1IF bit in TIMx_SR
		// uint64_t time_in_microseconds = (ticks * 1e6) / F_CPU;
		// uint64_t current_edge_time = time_in_microseconds;				   // Record the captured time
		// uint64_t time_difference = abs(current_edge_time - previous_edge_time); // Time since last edge

		// // Determine edge type (rising/falling)
		// int channel = (gpiob->IDR & GPIO_IDR_Px3); // Mask for bit 6 (PA6). [1 = PA6 is rising edge, 0 = PA6 is falling edge]
		// //channel = channel >> 6;					   // Right shift to position 0


		// if (state == IDLE)
		// {
		// 	// Any signal bus voltage edge switches to busy
		// 	state = BUSY;
		// }
		// else if (state == BUSY)
		// {
		// 	// To prevent race condition at edges, uses < and > to keep in BUSY state
		// 	/*TIMEOUT EVENTS*/
		// 		// If 1.113ms >= edge_time >= 1.188ms and rising edge, idle
		// 		// If 1.04ms >= edge_time >= 1.14ms and falling edge, collision
		// 		// If edge_time > 1.13 ms and rising edge, idle
		// 	if(channel)
		// 	{
		// 		if(time_difference > IDLE_MOE_LOW_US && time_difference < IDLE_MOE_HIGH_US)
		// 		{
		// 			state = IDLE;
		// 		}
		// 	} else
		// 	{
		// 		if(time_difference > COLLISION_MOE_LOW_US && time_difference < COLLISION_MOE_HIGH_US)
		// 		{
		// 			state = COLLISION;
		// 		}
		// 	}
		// }
		// else
		// {
		// 	// When in collision, any signal bus voltage edges bring it to the BUSY state
		// 	state = BUSY;
		// }
		// previous_edge_time = current_edge_time; // update the time of the previous edge
		tim2->SR &= ~CC2IF; // Clear the interrupt flag manually/by software if not set by capture event on channel 2
	}
}
