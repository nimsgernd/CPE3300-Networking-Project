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
#include <string.h>

// Project
#include "delay.h"
#include "F446RE.h"
#include "led.h"
#include "network.h"

/*
 ******************************************************************************
 * Variables
 ******************************************************************************
 */

// Definitions
#define IDLE_LED_STATE (int)0b1000000000	  // Left most LED value
#define BUSY_LED_STATE (int)0b0100000000	  // Second to left LED value
#define COLLISION_LED_STATE (int)0b0010000000 // Third to left LED value
#define ERROR_LED_STATE (int)0b1111111111	  // ALL LED value
#define MAX_16 0xFFFF
#define HALF_BIT_PERIOD_500_US 500e-6	
#define CLOCK_CYCLES_500_US (int)((F_CPU * HALF_BIT_PERIOD_500_US)-1)
#define CHAR_BIT 8

// Addresses
static volatile uint32_t *const iser = (uint32_t *)NVIC_BASE;
static volatile RCC *const rcc = (RCC *)RCC_BASE;
static volatile GPIO *const gpiob = (GPIO *)GPIOB_BASE;
static volatile GPTIM16B32B *const tim2 = (GPTIM16B32B *)TIM2_BASE;
static volatile ACTIM16B *const tim8 = (ACTIM16B *)TIM8_BASE;
static volatile GPTIM16B *const tim14 = (GPTIM16B *) TIM14_BASE;

// State
static State state = IDLE; // Current state

// Timer Variables
static volatile uint32_t previous_edge_time = 0; // Time of previous edge
static volatile uint16_t  tim2_cnt = 0; // used for storing the count in timer 2
static volatile uint16_t  tim8_cnt = 0; // used for storing the count in timer 8

// Manchester Encoded Transmission Data
static int* transmission_data = NULL;
static int transmission_len = 0;

/*
 ******************************************************************************
 * Function Prototypes
 ******************************************************************************
 */

static void post_collision_delay(void);
static void transmit(void);

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
    tim8->ARR = THRESHOLD_TICKS-1;

	// Set PA6 to alternate function for TIC/TOC
	gpiob->MODER |= GPIO_Px3_MODER_AF;

	// Set PA6 to alternate function in Alternate Function Register Low
	// (for GPIO pins 0 - 7)
	gpiob->AFRL = AFRL_Px3_AF1;

	// Open interrupt for TIM2
	iser[0] = TIM2_POS;

    // Enable TIM8 interrupt in the NVIC
	iser[TIM8_UP_TIM13_POS >> 5] |= (1 << (TIM8_UP_TIM13_POS % 32));

	// Set to TIC in compare compare mode register 1 in CC
	tim2->CCMR1 = CC2S;

	// Enable the capture compare for the channel
	tim2->CCER |= CC2E;

    // Set the compare value for channel 1 to the number of clock cycles in 500us
	tim2->CCR1 = CLOCK_CYCLES_500_US;

	// Set the direction of the input capture
	// (rising edge, falling edge, or both)
	tim2->CCER |= (CC2P | CC2NP); // Trigger on rising (CC1P)
								//          + falling edges (CC1NP)
	// Enable the interrupt on capture compare
	tim2->DIER |= (CC2IE | CC1IE);

    // Enable the timer interrupt
    tim8->DIER |= UIE;

    // Set the auto-reload value to the maximum for a 16-bit counter
    tim2->ARR = MAX_16;

	// Enable the counter
	tim2->CR1 |= CEN;

	// Enable timer
	tim14->CR1 |= CEN;

	// Check Initial State
	tim8->CR1 |= CEN;
}


/**
 * @brief	Manchester encodes the given char* into a bit array to be parsed
 *			inside transmit function. The bit array is Manchester encoded.
 * @returns Pointer to encoded data array
 *
 */


void encode(char* msg) {

	// Make sure to have enough size for Manchester encoding i.e. 2*bits
	transmission_data = (int*)malloc(2 * strlen(msg) * CHAR_BIT * sizeof(int));
	transmission_len = 2 * strlen(msg) * CHAR_BIT;

	// Convert every bit to Manchester pair i.e. bit 0 = bit to transmit bit 1 = ~bit0
    int len = strlen(msg);
    for(int i = 0; i < len; i++) {
        for(int j = 0; j < CHAR_BIT; j++) {
            int bit = (msg[i] >> j) & 1;
            transmission_data[2*(i*CHAR_BIT + j)] = bit;
            transmission_data[2*(i*CHAR_BIT + j) + 1] = ~bit & 1; // Use bitwise AND to ensure the result is 0 or 1
        }
    }
}


/**
 * @brief	Transmits the current bit pair in TIM2 CH1 ISR set at
 * 			500 uS.
 *
 *
 *
 */
static void transmit(void)
{
	static int current_bit = 0;

	// Transmit Manchester 1 Pair bit to PB1 i.e. 1 -> 01 -> 1 THEN 0
	// Adjusted every 500 uS
	gpiob->ODR |= transmission_data[current_bit];


	if(current_bit >= sizeof(transmission_data))
	{
		// Frees transmission_data
		free(transmission_data);

		// Set to NULL
		transmission_data = NULL;
	}

}

/**
 * @brief	Creates a randomized delay based on timer 14 which is acting as a
 * 			free running counter.
 *
 */
static void post_collision_delay(void)
{
	// Reads count register from free running counter
	uint32_t count = tim14->CNT;

	// Scale count value by the scaler
	uint32_t microSecDelay = count * TIM_TO_MICROSEC_SCALAR;

	// Delay by this randomized time
	delay_us(microSecDelay);
}

/**
 * @brief	Timer 8 is a 1.13ms timeout. If an edge has not been seen since the
 * 			last time the interrupt fired the interrupt determines the state
 * 			between idle and collision.
 *
 */
void TIM8_UP_TIM13_IRQHandler(void)
{
	// Turn timer off
	tim8->CR1 &= ~CEN;

	// Check if the update interrupt flag is set
	if (tim8->SR & UIF)
	{
		// If count has not been updated
    	if (tim8_cnt == 0)
    	{
    		// Check line state. High = idle, Low = collision
    		if (gpiob->IDR & GPIO_IDR_Px3)
    		{
    			state = IDLE;
    			led_enable(IDLE_LED_STATE); // Enables left most LED
    		}
    		else
    		{
    			state = COLLISION;
    			led_enable(COLLISION_LED_STATE); // Enables third to left LED
    		}
    	}

    	// Reset count variable
    	tim8_cnt = 0;

        // Clear the update interrupt flag
    	tim8->SR &= ~UIF;
	}
}



// Timer 2 interrupt fires when the timer is active for over 1.13ms
/**
 * @brief	Interrupt service routine to monitor the network line state using
 * 			timer 2 and tracks edges seen on the input line.
 *
 */
void TIM2_IRQHandler(void)
{
	if(tim2->SR & CC1IF) // If the interrupt source is a capture event on channel 1
						 // every half-bit period "500 us" for transmitter
	{
		// Transmit encoded half-bits i.e. 1 -> 1 THEN 0
		transmit();

		// Clear interrupt flag manually since not reading from
		tim2->SR = ~CC1IF;
	}
	else if (tim2->SR & CC2IF) // if the interrupt source is a capture event on channel 2

	{
		// Store count values at the time of the most recent edge
		tim2_cnt = tim2->CCR2;
		tim8_cnt = tim8->CCR1;

		// Once edge detected, start 1.13 ms timer
		tim8->CR1 |= CEN;

		// All other states, BUSY, and IDLE also go to BUSY if not timeout
		if (state == COLLISION)
		{
			state = BUSY;
			led_enable(BUSY_LED_STATE); // Enables second to left LED
		}
		else if(state == IDLE)
		{
			state = BUSY;
			led_enable(BUSY_LED_STATE); // Enables second to left LED
		}

		tim2->SR= ~CC2IF; // Clear the interrupt flag manually/by software if
							// not set by capture event on channel 2
	}
}
