/**
 ******************************************************************************
 * @file	: network.c
 * @brief	: Functions for interfacing with a manchester encoded computer
 *          network.
 * @details : This file contains the implementation of functions for interfacing
 *            with a Manchester encoded computer network. It includes functions
 *            for initializing the network, transmitting and receiving data,
 *            encoding and decoding data, and managing the network state.
 *            The network uses GPIO pins for communication and implements
 *            Manchester encoding for data transmission.
 * @authors	: Zack Kohlman		<kohlmanz@msoe.edu>
 *          : Jack Maki			<makij@msoe.edu>
 *          : Daniel Nimsgern	<nimsgern@msoe.edu>
 ******************************************************************************
 */

/**
 * PINS:
 *	PB1 - Channel Monitor
 *	PB3 - Transmitter
 */

/*
 ******************************************************************************
 * Includes
 ******************************************************************************
 */

// Library

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <sys/time.h>
#include <math.h>
#include <sys/types.h>

// Project
//#include "delay.h"
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
static volatile STK *const stk = (STK *)STK_BASE;
static volatile GPTIM16B32B *const tim2 = (GPTIM16B32B *)TIM2_BASE;
static volatile ACTIM16B *const tim8 = (ACTIM16B *)TIM8_BASE;
static volatile GPTIM16B *const tim14 = (GPTIM16B *) TIM14_BASE;

// TODO: Implement packet struct
static packet transmission = {0x55,0x00,0x00,0x00,0x00,NULL,0xAA};
static packet reception;

// State
static State state = BUSY;	// Current state
static Delay tx_delay = NO;	// Collision to Busy delay flag

// Timer Variables
static volatile char was_edge = 0;

// Manchester Encoded Transmission Data
static int* transmission_data = NULL;
static int transmission_len = 0;

// Bit tracker for transmit function
static int current_bit = 0;
static int is_transmitting = 1;	// 0 = No IDLE state to start, 1 = IDLE state to start

// TODO: REMOVE ONCE HEADER ADDED... IF header not supported, data MUST start with logic-0
static int prev_edge = 1;	// 0 = previous edge was logic 0, 1 = previous edge was logic 1
static int curr_edge = 1;
static int is_recieving = 0;

// Bit Reception Buffer
static volatile int rx_data[RXDATA_INITSIZE_BITS];
static char* rx_decoded;
static int data_size = 0;		// Len of recieved data
static uint16_t tim14_current_count = 0;
static uint16_t tim14_previous_count = 0;
static int new_message = 0;

static uint8_t crc_table[256];

/*
 ******************************************************************************
 * Function Prototypes
 ******************************************************************************
 */

static void transmit(void);
static uint8_t bitArrayToInt(uint8_t *bitArray, int length);
static void pop_crc_table(uint8_t crc_table[256], uint8_t poly);
static uint8_t crc(char* array, int byte_len);

/*
 ******************************************************************************
 * Function Definitions
 ******************************************************************************
 */

/**
 * @brief Initializes the monitor.
 *
 * This function initializes the monitor by performing the following steps:
 * - Initializes the LED bar.
 * - Sets the data size to 0.
 * - Populates the CRC table.
 * - Configures RCC settings.
 * - Configures interrupt settings.
 * - Configures GPIO settings.
 * - Configures timer settings.
 * - Enables timers.
 */
void monitor_init(void)
{
	/* External Initializers */
	// Initialize LED bar
	led_init();

    data_size = 0;	// array begins empty

	pop_crc_table(crc_table, 0x07);

	/* RCC Settings */
	// Enable clock for GPIOB
	rcc->AHB1ENR |= GPIOBEN;

	// Enable clock for tim2
	rcc->APB1ENR |= TIM2EN;

	// Enable clock for tim8
	rcc->APB2ENR |= TIM8EN;

	// Enable clock for tim14
	rcc->APB1ENR |= TIM14EN;

	/* Interrupt Settings */
	// Open interrupt for TIM2
	iser[0] = TIM2_POS;

	// Enable TIM8 interrupt in the NVIC
	iser[TIM8_UP_TIM13_POS >> 5] |= (1 << (TIM8_UP_TIM13_POS % 32));

	/* GPIO Settings */
	// Set PA3 to alternate function for TIC/TOC
	gpiob->MODER |= GPIO_MODER_Px3_AF | GPIO_MODER_Px1_OUT;

	// Set PB3 to alternate function in Alternate Function Register Low
	gpiob->AFRL = GPIO_AFRL_Px3_AF1;

	// Set pull up for PB3
	gpiob->PUPDR |= GPIO_PUPDR_Px3_UP;

	// Set TX line high
	gpiob->ODR = (gpiob->ODR & GPIO_ODR_Px1) | (1 << 1);

	/*Timer Settings */
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

	// Set the auto-reload value to the maximum for a 16-bit counter
	tim2->ARR = CLOCK_CYCLES_500_US;

	// Enable the timer interrupt
	tim8->DIER |= UIE;

	// Set the auto-reload value to achieve a period of 1.13 ms
	tim8->ARR = THRESHOLD_TICKS-1;

	/* Enable Timers */
	// Enable the counter
	tim2->CR1 |= CEN;

	// Check Initial State
	tim8->CR1 |= CEN;

	// Enable timer
	tim14->CR1 |= CEN;
}


/**
 * @brief	Frees the dynamically allocated memory
 *			designated for the decoded data.
 *
 */
void clear(void)
{
	free(rx_decoded);
}

/**
 * @brief
 *
 * @return
 */
int new_message_flag(void)
{
	return new_message;
}

/**
 * @brief
 *
 * @param addr
 */
void set_sender(int addr)
{
	transmission.SRC = addr;
}

/**
 * @brief
 *
 * @param addr
 */
void set_reciever(int addr)
{
	transmission.DEST = addr;
}

/**
 * @brief
 *
 * @return
 */
int get_sender(void)
{
	return transmission.SRC;
}

/**
 * @brief
 *
 * @return
 */
int get_reciever(void)
{
	return transmission.DEST;
}

/**
 * @brief	Relevant Setters/Getters.
 *
 */
int get_dataSize(void)
{
	return data_size;
}

/**
 * @brief	Returns the raw rx buffer data
 */
int* get_raw_data(void)
{
    int* rx_data_copy = (int*)calloc(data_size, sizeof(int));

    // Check if memory allocation was successful
    if (rx_data_copy == NULL) {
        // Handle error (e.g., print an error message and return NULL)
        printf("Error: Could not allocate memory for rx_data copy\n");
        return NULL;
    }

    // Copy the contents of rx_data to rx_data_copy
    for (int i = 0; i < data_size; i++) {
        rx_data_copy[i] = rx_data[i];
    }

    return rx_data_copy;
}


/**
 * @brief	Retrns ascii encoded rx_data
 */
char* get_ascii_data(void)
{
	new_message = 0;
	return rx_decoded;
}

/**
 * @brief Encodes the given message using Manchester encoding.
 *
 * @param msg The message to be encoded.
 *
 * @details This function encodes the given message using Manchester encoding.
 *          It allocates memory for the encoded data and converts each bit of
 *          the message to a Manchester pair, where bit 0 is the bit to transmit
 *          and bit 1 is the complement of bit 0.
 *
 * @note The caller is responsible for freeing the memory allocated for the encoded data.
 */
void encode(packet tpacket)
{
    // Make sure to have enough size in the msg buffer for the original message + packet data
	char* msg = (char*)malloc(strlen(tpacket.MSG) * CHAR_BIT * sizeof(int) + 6*sizeof(uint16_t));
	snprintf(msg, 303, "%x%x%x%x%x%s%x", tpacket.PREAMBLE, tpacket.SRC, tpacket.DEST,
			tpacket.LEN, tpacket.CRC, tpacket.MSG, tpacket.TRAILER);
	//debug
	printf("packet string: %s\n",msg);

	// Make sure to have enough size for Manchester encoding i.e., 2*bits + Packet data
    transmission_data = (int*)malloc(2 * strlen(msg) * CHAR_BIT * sizeof(int));
    transmission_len = 2 * strlen(msg) * CHAR_BIT;

    // Convert every bit to Manchester pair i.e. bit 0 = bit to transmit, bit 1 = ~bit0
    int len = strlen(msg);
    for(int i = 0; i < len; i++)
    {
        for(int j = CHAR_BIT - 1; j >= 0; j--)
        { // Start from the most significant bit
            int bit = (msg[i] >> j) & 1;
            transmission_data[2*((i+1)*CHAR_BIT - j - 1)] = bit ^ 1; // Use XOR to flip the bit
            transmission_data[2*((i+1)*CHAR_BIT - j - 1) + 1] = bit;
        }
    }

// Uncomment below to check transmission_data
//    printf("\n");
//    // Output the encoded bits with spaces
//    for(int i = 0; i < transmission_len; i++)
//    {
//        printf("%d", transmission_data[i]);
//        if((i+1)%16 == 0)
//        {
//        	printf("    ");
//        }else if ((i + 1) % 2 == 0)
//        {
//            printf(" "); // Add a space after every pair of bits
//        }
//    }
//    printf("\n");
}

/**
 * Converts a bit array to an integer.
 *
 * @param bitArray The bit array to convert.
 * @param length The length of the bit array.
 * @return The converted integer.
 */
static uint8_t bitArrayToInt(uint8_t *bitArray, int length) {
    int result = 0;

    for (int i = 0; i < length; i++) {
        // Shift the current result to the left by 1 bit
        result <<= 1;
        // Add the current bit to the result
        result |= bitArray[i];
    }

    return result;
}

/**
 * @brief Decodes the received data and stores it in rx_decoded.
 *
 * This function decodes the received data, which is encoded in a specific format,
 * and stores the decoded data in the rx_decoded array. The decoded data is then
 * used for further processing.
 *
 * @note The function assumes that the necessary memory for rx_decoded and temp_ascii
 *       has already been allocated.
 */
void decode(void)
{
    rx_decoded = (char*)calloc(((data_size / CHAR_BIT*2) + 1),sizeof(char));

    if (rx_decoded == NULL)
    {
        //printf("Error: Could not allocate memory for decoded message\n\r");
        return;
    }

    // One ascii character is 1 byte which is encoded to 16 bits
    int len = data_size / 8;

    // Temp array to hold only ascii values
    uint8_t* temp_ascii = (uint8_t*)calloc(CHAR_BIT,sizeof(uint8_t));

    // Iterate over all characters
    for(int i = 0; i < len; i++)
    {
		int ascii_index = 0;

    	// For each of the 16 bits that reps 1 byte
    	for(int j = 0; j < CHAR_BIT; j++)
    	{
        // The first bit of the pair should be the inverse of the second bit
        // If this is not the case, there may be an error in the encoded data

    		int rx_index = j + (i*(CHAR_BIT));

    		int ascii_bit = rx_data[rx_index];
    		temp_ascii[ascii_index] = ascii_bit;

    		ascii_index++;

    	}

    	uint8_t char_ascii = bitArrayToInt(temp_ascii, CHAR_BIT);
    	rx_decoded[i] = (char)char_ascii;
    }

    new_message = 1;

    free(temp_ascii);

}



/**
 * @brief Transmits data using Manchester encoding.
 *
 * This function transmits Manchester 1 Pair bit to PB1. It adjusts the transmission every 500 uS.
 * If there is data to transmit, it sets the is_transmitting flag to 1 and updates the output data register (ODR) of GPIOB.
 * If there is no data to transmit, it sets the is_transmitting flag to 0 and updates the ODR of GPIOB to transmit a logic high.
 * After transmitting all the bits, it resets the current_bit counter, frees the transmission_data memory, and sets it to NULL.
 */
static void transmit(void)
<<<<<<< HEAD
{	
	// Transmit Manchester 1 Pair bit to PB1 i.e. 1 -> 01 -> 1 THEN 0
=======
{
>>>>>>> e664e6a67283b565b23e122a09929abb7f756be3
	// Adjusted every 500 uS
	if(transmission_data != NULL)
	{
		is_transmitting = 1;
		gpiob->ODR = ((gpiob->ODR & GPIO_ODR_Px1) | (transmission_data[current_bit] << 1));
		current_bit++;
	}
	else
	{
		// Done transmitting
		is_transmitting = 0;

		gpiob->ODR = (gpiob->ODR & GPIO_ODR_Px1) | (1 << 1);
	}
	if(current_bit >= transmission_len)
	{
		current_bit = 0;
		// Frees transmission_data
		free(transmission_data);

		// Set to NULL
		transmission_data = NULL;
	}
}

/**
 * @brief	frees received data and resets to defaults
 */
void reset_rx_data(void)
{
    data_size = 0;	// array begins empty
}

/**
 * @brief	Creates a randomized delay based on timer 14 which is acting as a
 * 			free running counter.
 *
 */
void post_collision_delay(void)
{

	tx_delay = YES;

	stk->CTRL &= STK_CTRL_DIS;

	stk->LOAD = (uint32_t)((double)tim14->CNT * TX_DELAY_SCALAR);

	stk->VAL = 0;

	stk->CTRL |= (STK_CTRL_EN | STK_CTRL_INT_EN | STK_CTRL_CLK_AHB);

}

/**
 * @brief
 *
 * @param crc_table
 * @param poly
 */
void pop_crc_table(uint8_t crc_table[256], uint8_t poly)
{
	uint8_t crc = 0x80;
	memset(crc_table, 0, 256);

	for(int i = 1; i < 256; i <<= 1)
	{
		if(crc & 0x80)
		{
			crc = (crc << 1) ^ poly;
		}
		else
		{
			crc <<= 1;
		}

		for(int j = 0; j < i; j++)
		{
			crc_table[i + j] = crc ^ crc_table[j];
		}
	}
}

/**
 * @brief
 *
 * @param array
 * @param byte_len
 * @return
 */
uint8_t crc(char* array, int byte_len)
{
	uint8_t i;
	uint8_t crc = 0x0;

	while(byte_len--)
	{
		i = (crc ^ *array++);
		crc = (crc_table[i] ^ (crc << 8));
	}

	return crc;
}

/**
 * @brief Handler for the post collision delay flag
 *
 */
void SysTick_Handler(void)
{
	stk->CTRL &= STK_CTRL_DIS;

	tx_delay = NO;
}

/**
 * @brief	Timer 8 is a 1.13ms timeout. If an edge has not been seen since the
 * 			last time the interrupt fired the interrupt determines the state
 * 			between idle and collision.
 *
 */
void TIM8_UP_TIM13_IRQHandler(void)
{
	// Check if the update interrupt flag is set
	if (tim8->SR & UIF)
	{
		// If count has not been updated
    	if (was_edge == 0 && state == BUSY)
    	{
    		// End recieving.... reset reciever vars
    		is_recieving = 0;

    		// Check line state. High = idle, Low = collision
    		if (gpiob->IDR & GPIO_IDR_Px3)
    		{
    			state = IDLE;
    		    led_enable(IDLE_LED_STATE); // Enables left most LED
    		}
    		else
    		{
    		    state = COLLISION;
    			gpiob->ODR = (gpiob->ODR & GPIO_ODR_Px1) | (1 << 1);

    		    is_transmitting = 0;
    		    led_enable(COLLISION_LED_STATE); // Enables third to left LED
    		}
    	}

    	// Reset count variable
    	was_edge = 0;

        // Clear the update interrupt flag
    	tim8->SR &= ~UIF;
	}
}

/**
 * TIM2 IRQ handler for CC1IF and CC2IF. If the interrupt source is CC1IF, it checks if the transmitter is
 * ready to transmit and then calls the transmit function. If the interrupt source is
 * CC2IF, it handles the receiving of data. It checks for edge transitions and stores
 * the received data. It also handles the state transitions from IDLE to BUSY and
 * COLLISION to BUSY. This function clears the interrupt flags manually after handling
 * the interrupts.
 */

void TIM2_IRQHandler(void)
{
	if(tim2->SR & CC1IF) // If the interrupt source is a capture event on channel 1
									 // every half-bit period "500 us" for transmitter
	{
		// STARTS transmitting in IDLE, but can also in BUSY after... CANNOT
		// transmit in COLLISION
		if(tx_delay == NO && (is_transmitting || state == IDLE))
		{
			// Transmit encoded half-bits i.e. 1 -> 1 THEN 0
			transmit();
		}

		// Clear interrupt flag manually since not reading from
		tim2->SR = ~CC1IF;
	}

	if (tim2->SR & CC2IF) // if the interrupt source is a capture event on
						  // channel 2
	{

		prev_edge = curr_edge;
		curr_edge = (gpiob->IDR & GPIO_IDR_Px3)>>3;

		// Store count values at the time of the most recent edge
		was_edge = 1;
		tim14_current_count = tim14->CNT;

		// All other states, BUSY, and IDLE also go to BUSY if not timeout
		if (state == COLLISION)
		{
			state = BUSY;
			led_enable(BUSY_LED_STATE); // Enables second to left LED

			post_collision_delay();
		}
		// When state goes IDLE -> BUSY, begin receiving data
		else if(state == IDLE)
		{
			//Channel Monitor
			state = BUSY;
			led_enable(BUSY_LED_STATE); // Enables second to left LED


			// Check for falling edge
			if(prev_edge && !curr_edge)
			{
				is_recieving = 1;
			}
		}

		 /**
		  * The code snippet is a part of the network functionality implementation.
		  * It handles the reception of data and stores it in the `rx_data` array.
		  * The received data is checked against a threshold to determine if it should be ignored or stored.
		  * The interrupt flag on channel 2 is cleared manually.
		  */
		// If we're recieving
		if(is_recieving)
		{
			uint16_t delta_t;

			// For first edge, include preceeding 0
			if(curr_edge == prev_edge)
			{
				rx_data[0] = '0';
				data_size++;
			}

			// Calculate time difference
			if (tim14_current_count >= tim14_previous_count)
			{
				delta_t = tim14_current_count - tim14_previous_count;
			} else {
				// Handle counter rollover
			    delta_t = (UINT16_MAX - tim14_previous_count) + tim14_current_count + 1;
			}
				// If edge occured within 506us, ignore.
				if(delta_t > (THRESHOLD_TICKS/2)-1)
				{
					rx_data[data_size] = curr_edge;
					data_size++;
					//store the count of the previous recorded edge
					tim14_previous_count = tim14_current_count;
				}

			}

		tim2->SR = ~CC2IF; // Clear the interrupt flag manually/by software if
							// not set by capture event on channel 2
	}
}
