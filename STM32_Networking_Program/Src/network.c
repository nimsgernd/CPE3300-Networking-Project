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
static volatile ACTIM16B *const tim8 = (ACTIM16B *)TIM8_BASE;
static volatile GPTIM16B *const tim14 = (GPTIM16B *) TIM14_BASE;


static packet transmission = {0x55,0x00,0x00,0x00,0x00,NULL,0xAA};
static packet reception;

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
static uint16_t tim14_current_count = 0;
static uint16_t tim14_previous_count = 0;
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
 * @param state
 */
void set_crc(int state)
{
	transmission.CRC = state;
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
 * @brief
 *
 * @return
 */
int get_crc(void)
{
	return transmission.CRC;
}

/**
 * @brief
 *
 * @return
 */
int get_sender_addr(void)
{
	return reception.SRC;
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
	return reception.MSG;
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
void encode(char* msg)
{
	int msg_len = strlen(msg);

	// SRC, RECIEVER, and CRC already set... need to set others in transmission struct
	transmission.PREAMBLE = 0x55;
	transmission.LEN = msg_len;

	if(!get_crc())
	{
		transmission.TRAILER = 0xAA;
	} else {
		transmission.CRC = crc(msg, msg_len);
	}
	strcpy(transmission.MSG, msg);


	// Make sure to have enough size for Manchester encoding i.e., 2*bits + Packet data
    transmission_len = 2 * (msg_len * BYTE * (BYTE * NUM_8BIT_FIELDS));
    transmission_data = (uint8_t*)malloc(transmission_len* sizeof(uint8_t));

    // Convert every bit to Manchester pair i.e. bit 0 = bit to transmit, bit 1 = ~bit0
    for(int i = 0; i < msg_len; i++)
    {
        for(int j = BYTE - 1; j >= 0; j--)
        { // Start from the most significant bit
            int bit = (msg[i] >> j) & 1;
            transmission_data[2*((i+1)*BYTE - j - 1)] = bit ^ 1; // Use XOR to flip the bit
            transmission_data[2*((i+1)*BYTE - j - 1) + 1] = bit;
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

    reception.PREAMBLE = bitArrayToInt(&rx_data[0], BYTE);
    reception.SRC = bitArrayToInt(&rx_data[BYTE], BYTE);
    reception.DEST = bitArrayToInt(&rx_data[BYTE*2], BYTE);
    reception.LEN = bitArrayToInt(&rx_data[BYTE*3], BYTE);
    reception.CRC = bitArrayToInt(&rx_data[BYTE*4], BYTE);

    // Ensure that there is enough data for a complete packet
    if (data_size < MIN_PACKET_LEN_BYTES || reception.PREAMBLE != 0x55 || reception.DEST < 0x40 || reception.DEST > 0x42)
    {  	// Each field is 8 bits
        // Not enough data for a complete packet
    	printf("Packet invalid... dropping\n\r");
    	valid_packet = 0;
        return;
    }


    // Parse the message
    if(!reception.LEN)
    {
        memset(msg, '\0', sizeof(msg));
    } else {
    	for (int i = 0; i < reception.LEN; i++)
		{
			msg[i] = bitArrayToInt(&rx_data[(i + 5) * BYTE], BYTE);
		}
    }

    // Set msg in struct
    strcpy(reception.MSG, msg);

    // Parse the trailer
    reception.TRAILER = bitArrayToInt(&rx_data[(reception.LEN + 5) * BYTE], BYTE);

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
 * @brief
 *
 */
void test_parse_packet(void)
{
    // Test 1: Test with a valid packet
    {
    	int8_t arr[] = {0xAA, 0x01, 0x02, 0x05, 0x01,'H', 'e', 'l', 'l', 'o', 0xBB};
        int rx_data_index = 0;

        for (int i = 0; i < sizeof(arr) / sizeof(arr[0]); ++i) {
            uint8_t current_char = arr[i];

            // Extract each bit of the current character and store it in rx_data
            for (int j = BYTE - 1; j >= 0; --j) {
                rx_data[rx_data_index] = (current_char >> j) & 0x01;
                ++rx_data_index;
            }
        }

        data_size = sizeof(arr)*BYTE;


    	// Now bit_array contains the binary representation of rx_data
        // Call the function to test
        parse_packet();

        // Check the results
        assert(reception.PREAMBLE == 0xAA);
        assert(reception.SRC == 0x01);
        assert(reception.DEST == 0x02);
        assert(reception.LEN == 0x05);
        assert(reception.CRC == 0x01);
        assert(strcmp(reception.MSG, "Hello") == 0);
        assert(reception.TRAILER == 0xBB);
    }

    // Test 2: Test with an invalid packet (not enough data for a complete packet)
    {
        // Set up the raw data for an invalid packet
    	int8_t arr[] = {0xAA, 0x01, 0x02, 0x05, 0xBB};
        int rx_data_index = 0;

        for (int i = 0; i < sizeof(arr) / sizeof(arr[0]); ++i) {
            uint8_t current_char = arr[i];

            // Extract each bit of the current character and store it in rx_data
            for (int j = BYTE - 1; j >= 0; --j) {
                rx_data[rx_data_index] = (current_char >> j) & 0x01;
                ++rx_data_index;
            }
        }

        data_size = 5*BYTE;

        // Call the function to test
        parse_packet();

        // Check the results (the function should return without modifying new_packet)
        assert(reception.PREAMBLE == 0xAA);
        assert(reception.SRC == 0x01);
        assert(reception.DEST == 0x02);
        assert(reception.LEN == 0x05);
        assert(strcmp(reception.MSG, "Hello") == 0);
        assert(reception.TRAILER == 0xBB);
    }
    {
		// Test 3: With no message

		// Set up the raw data for an invalid packet
    	int8_t arr[] = {0xAA, 0x01, 0x02, 0x00, 0x01, 0xBB};
		int rx_data_index = 0;

		for (int i = 0; i < sizeof(arr) / sizeof(arr[0]); ++i) {
			uint8_t current_char = arr[i];

			// Extract each bit of the current character and store it in rx_data
			for (int j = BYTE - 1; j >= 0; --j) {
				rx_data[rx_data_index] = (current_char >> j) & 0x01;
				++rx_data_index;
			}
		}

		data_size = sizeof(arr)*BYTE;

		// Call the function to test
		parse_packet();

		// Check the results (the function should return without modifying new_packet)
		assert(reception.PREAMBLE == 0xAA);
		assert(reception.SRC == 0x01);
		assert(reception.DEST == 0x02);
		assert(reception.LEN == 0x00);
		assert(reception.CRC == 0x01);
		assert(strcmp(reception.MSG, "\0"));
		assert(reception.TRAILER == 0xBB);
    }
}


/**
 * @brief
 *
 * TODO: Add more tests as needed
 */
void print_packet(void)
{
	printf("Preamble: %X\n\r", reception.PREAMBLE);
	printf("SRC: %X\n\r", reception.SRC);
	printf("DEST: %X\n\r", reception.DEST);
	printf("LEN: %X\n\r", reception.LEN);
	printf("CRC: %X\n\r", reception.CRC);
	printf("MSG: %s\n\r", reception.MSG);
	printf("TRAILER: %X\n\r", reception.TRAILER);
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
 *
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
		// For first edge, include preceeding 0
		if(curr_edge == prev_edge)
		{
			rx_data[0] = '0';
			data_size++;
		}

		if(is_recieving)
		{
			uint16_t delta_t;

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
