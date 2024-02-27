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

// Addresses
static volatile uint32_t *const iser = (uint32_t *)NVIC_BASE;
static volatile RCC *const rcc = (RCC *)RCC_BASE;
static volatile GPIO *const gpiob = (GPIO *)GPIOB_BASE;
static volatile STK *const stk = (STK *)STK_BASE;
static volatile GPTIM16B32B *const tim2 = (GPTIM16B32B *)TIM2_BASE;
static volatile ACTIM16B *const tim1 = (ACTIM16B *)TIM1_BASE;
static volatile GPTIM16B *const tim9 = (GPTIM16B *) TIM9_BASE;

static uint8_t tx_src, tx_dest, tx_preamble, tx_len, tx_crc,tx_trailer;
static char tx_msg[MAX_MSG_LEN_BYTES];

static uint8_t rx_src, rx_dest, rx_preamble, rx_len, rx_crc,rx_trailer;
static char rx_msg[MAX_MSG_LEN_BYTES];

// State
static State state = BUSY;	// Current state
static Delay tx_delay = NO;	// Collision to Busy delay flag

// Timer Variables
static volatile char was_edge = 0;

// Manchester Encoded Transmission Data
static uint8_t* transmission_data = NULL;
static int transmission_len = 0;

// Bit tracker for transmit function
static int current_bit = 0;
static int is_transmitting = 1;	// 0 = No IDLE state to start, 1 = IDLE state to start

// TODO: REMOVE ONCE HEADER ADDED... IF header not supported, data MUST start with logic-0
static int prev_edge = 1;	// 0 = previous edge was logic 0, 1 = previous edge was logic 1
static int curr_edge = 1;
static int is_recieving = 0;

// Parse packet vars
static char msg[MAX_MSG_LEN_BYTES];	//256 supported

// Bit Reception Buffer
static uint8_t rx_data[RXDATA_INITSIZE_BITS];

static char* rx_decoded;
static int data_size = 0;		// Len of recieved data
static uint16_t tim9_current_count = 0;
static uint16_t tim9_previous_count = 0;
static int new_message = 0;
static int valid_packet = 1;		// 1 = valid packet recieved, 0 = invalid packet
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

	pop_crc_table(crc_table, CRC_POLY);

	/* RCC Settings */
	// Enable clock for GPIOB
	rcc->AHB1ENR |= GPIOBEN;

	// Enable clock for tim2
	rcc->APB1ENR |= TIM2EN;

	// Enable clock for tim8
	rcc->APB2ENR |= TIM1EN;

	// Enable clock for tim14
	rcc->APB2ENR |= TIM9EN;

	/* Interrupt Settings */
	// Open interrupt for TIM2 and TIM1
	iser[0] = (TIM1_UP_TIM10_IRQHandler_POS | TIM2_POS);

	/* GPIO Settings */
	// Set PA3 to alternate function for TIC/TOC
	gpiob->MODER |= (GPIO_MODER_Px3_AF | GPIO_MODER_Px1_OUT);

	// Set PB3 to alternate function in Alternate Function Register Low
	gpiob->AFRL = GPIO_AFRL_Px3_AF1;

	// Set pull up for PB3
	gpiob->PUPDR |= GPIO_PUPDR_Px3_UP;

	// Set TX line high
	gpiob->ODR |= GPIO_ODR_Px1_SET;

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
	tim1->DIER |= UIE;

	// Set the auto-reload value to achieve a period of 1.13 ms
	tim1->ARR = THRESHOLD_TICKS-1;

	/* Enable Timers */
	// Enable the counter
	tim2->CR1 |= CEN;

	// Check Initial State
	tim1->CR1 |= CEN;

	// Enable timer
	tim9->CR1 |= CEN;
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
 * @brief Flag set to true if a new message is received.
 *
 * @return 1 if new message, 0 if no new message
 */
int new_message_flag(void)
{
	return new_message;
}

/**
 * @brief Sets the sender address of the transmission.
 *
 * @param addr - 8 bit address of the sender
 */
void set_transmission_sender(int addr)
{
	tx_src = addr;
}

/**
 * @brief Sets the receiver address of the transmission.
 *
 * @param addr - 8 bit address of the receiver
 */
void set_transmission_reciever(int addr)
{
	tx_dest = addr;
}

/**
 * @brief Sets the CRC flag on the transmission.
 *
 * @param state - 1 bit CRC flag (1 = CRC enabled, 0 = CRC disabled)
 */
void set_transmission_crc(int state)
{
	tx_crc = state;
}

/**
 * @brief Returns the address that the transmission is going to be sent from.
 *
 * @return 8 bit address
 */
int get_transmission_sender(void)
{
	return tx_src;
}

/**
 * @brief Returns the address that the transmission is going to be sent to.
 *
 * @return 8 bit address
 */
int get_transmission_reciever(void)
{
	return tx_dest;
}

/**
 * @brief Returns the CRC flag that is set in the transmission message.
 *
 * @return 1 bit flag (1 = enabled, 0 = disabled)
 */
static int get_transmission_crc(void)
{
	return tx_crc;
}

void reset_tx_data(void)
{
	free(transmission_data);

}

/**
 * @brief Returns the address that the received message was from.
 *
 * @return 8 bit address
 */
int get_reciever_sender(void)
{
	return rx_src;
}

/**
 * @brief
 *
 * @return
 */
int get_dataSize(void)
{
	return data_size;
}

/**
 * @brief Returns the raw rx buffer data
 *
 * @return
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
 * @brief Returns ASCII encoded rx_data
 *
 * @return
 */
char* get_ascii_data(void)
{
	new_message = 0;
	return rx_msg;
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
void encode(char* message)
{
	// Clear msg... holds the total appended packet
    memset(msg, '\0', MAX_MSG_LEN_BYTES*sizeof(msg[0]));
    memset(tx_msg, '\0', MAX_MSG_LEN_BYTES*sizeof(msg[0]));


    // Clear whole array
    memset(rx_data, '\0', RXDATA_INITSIZE_BITS*sizeof(rx_data[0]));

	int msg_len = strlen(message);

	// SRC, RECIEVER, and CRC already set... need to set others in transmission struct
	tx_preamble = 0x55;

	tx_len = msg_len;
	strcpy(tx_msg, message);

	if(!tx_crc)
	{
		tx_trailer = 0xAA;
	} else
	{
		tx_trailer = crc(message, msg_len);
	}

	// PREAMBLE, SRC, DEST, LEN, CRC to front, TRAILER to back of msg file-scope variable

	// Concatenate preamble, src, dest, len, crc to the beginning of msg, and trailer to the end of msg
	snprintf(msg, 303, "%c%c%c%c%c%s%c",
			tx_preamble,
			tx_src,
			tx_dest,
			tx_len,
			tx_crc,
			tx_msg,
			tx_trailer);

    // Calculate the new length of msg after concatenation
    int new_msg_len = tx_len + NUM_8BIT_FIELDS;


    // Make sure to have enough size for Manchester encoding i.e., 2*bits + Packet data
    transmission_len = 2 * new_msg_len*BYTE;    // Convert to num bits * 2 for manchester
    transmission_data = (uint8_t*)malloc(transmission_len* sizeof(uint8_t));

    printf("Transmission len: %i\n\r", transmission_len);

    // Convert every bit to Manchester pair i.e. bit 0 = bit to transmit, bit 1 = ~bit0
    for(int i = 0; i < new_msg_len; i++)
    {
        for(int j = BYTE - 1; j >= 0; j--)
        { // Start from the most significant bit
            int bit = (msg[i] >> j) & 1;
            transmission_data[2*((i+1)*BYTE - j - 1)] = bit ^ 1; // Use XOR to flip the bit
            transmission_data[2*((i+1)*BYTE - j - 1) + 1] = bit;
        }
    }




// Uncomment below to check transmission_data
    printf("\n");
    // Output the encoded bits with spaces
    for(int i = 0; i < transmission_len; i++)
    {
        printf("%d", transmission_data[i]);
        if((i+1)%16 == 0)
        {
        	printf("    ");
        }else if ((i + 1) % 2 == 0)
        {
            printf(" "); // Add a space after every pair of bits
        }
    }
    printf("\n");
}

/**
 * @brief Converts a bit array to an integer.
 *
 * @param bitArray - The bit array to convert.
 * @param length - The length of the bit array.
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
 * Takes the raw data and creates a struct with
 * 8 bit preamble
 * 8 bit source address
 * 8 bit destination address
 * 8 bit length of the message (0-255 of charcters/bytes)
 * 0-255 byets/char message
 * 8 bit trailer
 */
void parse_packet(void)
{

	printf("Date size: %i\n\r", data_size);

	for(int i = 0; i < data_size; i++)
	{
		if((i%8) == 0)
		{
			printf("  ");
		}
		printf("%i", rx_data[i]);
	}
	printf("\n\r");

    rx_preamble = bitArrayToInt(&rx_data[0], BYTE);
    rx_src = bitArrayToInt(&rx_data[BYTE], BYTE);
    rx_dest = bitArrayToInt(&rx_data[BYTE*2], BYTE);
    rx_len = bitArrayToInt(&rx_data[BYTE*3], BYTE);
    rx_crc = bitArrayToInt(&rx_data[BYTE*4], BYTE);
    char message[rx_len+1];


    // Ensure that there is enough data for a complete packet
    if (data_size < MIN_PACKET_LEN_BYTES || rx_preamble != 0x55 || rx_dest < 0x40 || rx_dest > 0x42)
    {  	// Each field is 8 bits
        // Not enough data for a complete packet
    	printf("Packet invalid... dropping\n\r");
    	reset_rx_data();
    	valid_packet = 0;

        return;
    }

    // Parse the message
    if(!rx_len)
    {
        memset(message, '\0', rx_len* sizeof(message[0])+1);
    }
    else {
    	for (int i = 0; i < rx_len; i++)
		{
    		message[i] = bitArrayToInt(&rx_data[(i + 5) * BYTE], BYTE);
    		printf("MSG: %c i: %i\n\r", message[i], i);
		}
    }

    // Add null terminator
    message[rx_len] = '\0';

    printf("Decoded msg field: %s\n\r", message);


    // Set msg in struct
    strcpy(rx_msg, message);

    // Parse the trailerr
    rx_trailer = bitArrayToInt(&rx_data[(rx_len + 5) * BYTE], BYTE);

    if(rx_crc)
    {
    	if(rx_trailer != crc(rx_msg,rx_len))
    	{
    		printf("Failed CRC check\n\r");
    	}
    	else
    	{
    		printf("Passed CRC check\n\r");
    	}
    }


    // Clear previous contents of rx_msg
    reset_rx_data();

    valid_packet = 1;

    // new message
    new_message = 1;

}

/**
 * @brief
 *
 * @return
 */
int is_valid_packet(void)
{
	return valid_packet;
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
{
	// Adjusted every 500 uS
	if(transmission_data != NULL)
	{
		is_transmitting = 1;
		gpiob->ODR = ((gpiob->ODR & GPIO_ODR_Px1_RSET) | (transmission_data[current_bit] << 1));
		current_bit++;
	}
	else
	{
		// Done transmitting
		is_transmitting = 0;

		gpiob->ODR |= GPIO_ODR_Px1_SET;
	}
	if(current_bit >= transmission_len)
	{
		current_bit = 0;
		// Frees transmission_data
//		free(transmission_data);

		// Set to NULL
//		transmission_data = NULL;
	}
}

/**
 * @brief	frees received data and resets to defaults
 *
 */
void reset_rx_data(void)
{
	memset(rx_data, 0, RXDATA_INITSIZE_BITS*sizeof(rx_data[0]));
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

	stk->LOAD = (uint16_t)((double)tim9->CNT * TX_DELAY_SCALAR);

	stk->VAL = 0;

	stk->CTRL |= (STK_CTRL_EN | STK_CTRL_INT_EN | STK_CTRL_CLK_AHB);

}

/**
 * @brief Populates a look up table for all possible CRC8 values based on the
 * 		  the input polynomial
 *
 * @param crc_table - Reference to the 256 8 bit table
 * @param poly - The polynomial used to calculate the CRC
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
 * @brief	Timer 1 is a 1.13ms timeout. If an edge has not been seen since the
 * 			last time the interrupt fired the interrupt determines the state
 * 			between idle and collision.
 *
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
	// Check if the update interrupt flag is set
	if (tim1->SR & UIF)
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
    			gpiob->ODR |= GPIO_ODR_Px1_SET;

    		    is_transmitting = 0;
    		    led_enable(COLLISION_LED_STATE); // Enables third to left LED
    		}
    	}

    	// Reset count variable
    	was_edge = 0;

        // Clear the update interrupt flag
    	tim1->SR &= ~UIF;
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
		tim9_current_count = tim9->CNT;

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

			// Calculate time difference
			if (tim9_current_count >= tim9_previous_count)
			{
				delta_t = tim9_current_count - tim9_previous_count;
			} else {
				// Handle counter rollover
			    delta_t = (UINT16_MAX - tim9_previous_count) + tim9_current_count + 1;
			}
				// If edge occured within 506us, ignore.
				if(delta_t > (THRESHOLD_TICKS/2)-1)
				{
					rx_data[data_size] = curr_edge;
					data_size++;
					//store the count of the previous recorded edge
					tim9_previous_count = tim9_current_count;
				}

			}

		tim2->SR = ~CC2IF; // Clear the interrupt flag manually/by software if
							// not set by capture event on channel 2
	}
}
