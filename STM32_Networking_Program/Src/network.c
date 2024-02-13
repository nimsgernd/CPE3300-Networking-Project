/**
 ******************************************************************************
 * @file	: network.c
 * @authors	: Zack Kohlman		<kohlmanz@msoe.edu>
 *			: Jack Maki			<makij@msoe.edu>
 *			: Daniel Nimsgern	<nimsgern@msoe.edu>
 * 			:
 * @brief	: Functions for initializing a network monitoring process and
 * 			: updating status LEDs.
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

// TODO: Implement packet struct
packet pkt;

// State
static State state = BUSY;	// Current state
static Delay busy_delay = NO;	// Collision to Busy delay flag

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
static int* rx_data;
static char* rx_decoded;
static unsigned int array_size = RXDATA_INITSIZE_BYTES; // # of bytes to allocate
static int data_size = 0;		// Len of recieved data
static int tim2_ch1_isr_entred = 0;
/*
 ******************************************************************************
 * Function Prototypes
 ******************************************************************************
 */

static void transmit(void);
static void decode(void);
static void reset_rx_data(void);
static void assert_equal(char* actual, char* expected);
static int bitArrayToInt(int *bitArray, int length);
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
	/* External Initializers */
	// Initialize LED bar
	led_init();

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
 * @brief	Initializes the receiving buffer.
 *
 */
void rx_init(void)
{
	//initialize 40 Bytes to store received transmissions. calloc(n_items, item_size);
    rx_data = calloc(array_size, sizeof(int));
    data_size = 0;	// array begins empty
}

/**
 * @brief	Increases the receiving buffer by 5 bytes.
 *
 */
void embiggen(void)
{
	array_size += RXDATA_INITSIZE_BYTES;
	rx_data = realloc(rx_data, array_size*sizeof(int));
}

/**
 * @brief	Frees the dynamically allocated memory
 *			designated for the decoded data.
 *
 */
void clear(void)
{
	free(rx_data);
	free(rx_decoded);
}


/**
 * @breif	Returns the raw rx buffer data
 */
int* get_raw_data(void)
{
	return rx_data;
}


/**
 * @brief	Retrns ascii encoded rx_data
 */
char* get_ascii_data(void)
{
	return rx_decoded;
}

/**
 * @brief	Manchester encodes the given char* into a bit array to be parsed
 *			inside transmit function. The bit array is Manchester encoded.
 * @returns Pointer to encoded data array
 *
 */

void encode(char* msg) {
    // Make sure to have enough size for Manchester encoding i.e., 2*bits
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
 * @brief	Takes in a bit array and converts it to a single int.
 */
static int bitArrayToInt(int *bitArray, int length) {
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
 * @breif 	Used inside the reciever to decode the manchester encoded data into ascii
 */
static void decode(void)
{
    rx_decoded = (char*)calloc(((data_size / CHAR_BIT*2) + 1),sizeof(char));

    if (rx_decoded == NULL)
    {
        //printf("Error: Could not allocate memory for decoded message\n\r");
        return;
    }

    // One ascii character is 1 byte which is encoded to 16 bits
    int len = data_size / 16;

    // Temp array to hold only ascii values
    int* temp_ascii = (int*)calloc(CHAR_BIT,sizeof(int));

    // Iterate over all characters
    for(int i = 0; i < len; i++)
    {
		int ascii_index = 0;

    	// For each of the 16 bits that reps 1 byte
    	for(int j = 1; j < CHAR_BIT*2; j+=2)
    	{
        // The first bit of the pair should be the inverse of the second bit
        // If this is not the case, there may be an error in the encoded data

    		int rx_index = j + (i*(CHAR_BIT*2));

    		int ascii_bit = rx_data[rx_index];
    		temp_ascii[ascii_index] = ascii_bit;

    		ascii_index++;

    	}

    	int char_ascii = bitArrayToInt(temp_ascii, CHAR_BIT);
    	rx_decoded[i] = (char)char_ascii;
    }

    free(temp_ascii);

}

static void assert_equal(char* actual, char* expected) {
    if (strcmp(actual, expected) != 0) {
        printf("Assertion failed: expected \"%s\", got \"%s\"\n\r", expected, actual);
    }
}

void test_decode(void) {
    // Test 1: Decode a simple message
    data_size = 48; // Set data size to match the provided bit values
    rx_data = malloc(data_size * sizeof(int));
    // Assigning values to rx_data: 1 0 1 0 0 1 1 0 1 0 1 0 0 1 1 0 1 0 1 0 1 0 1 0 0 1 1 0 1 0 1 0
    rx_data[0] = 1;
    rx_data[1] = 0;

    rx_data[2] = 0;
    rx_data[3] = 1;

    rx_data[4] = 1;
    rx_data[5] = 0;

    rx_data[6] = 1;
    rx_data[7] = 0;

    rx_data[8] = 0;
    rx_data[9] = 1;

    rx_data[10] = 1;
    rx_data[11] = 0;

    rx_data[12] = 1;
    rx_data[13] = 0;

    rx_data[14] = 1;
    rx_data[15] = 0;

    rx_data[16] = 1;
    rx_data[17] = 0;

    rx_data[18] = 0;
    rx_data[19] = 1;

    rx_data[20] = 0;
    rx_data[21] = 1;

    rx_data[22] = 1;
    rx_data[23] = 0;

    rx_data[24] = 0;
    rx_data[25] = 1;

    rx_data[26] = 1;
    rx_data[27] = 0;

    rx_data[28] = 1;
    rx_data[29] = 0;

    rx_data[30] = 0;
    rx_data[31] = 1;


    rx_data[32] = 1;
    rx_data[33] = 0;

    rx_data[34] = 0;
    rx_data[35] = 1;

    rx_data[36] = 0;
    rx_data[37] = 1;

    rx_data[38] = 1;
    rx_data[39] = 0;

    rx_data[40] = 1;
    rx_data[41] = 0;

    rx_data[42] = 0;
    rx_data[43] = 1;

    rx_data[44] = 0;
    rx_data[45] = 1;

    rx_data[46] = 0;
    rx_data[47] = 1;


    decode();
    assert_equal(rx_decoded, "Hi");
    free(rx_data);
    free(rx_decoded);

    // Test 2: Decode an empty message
    data_size = 0;
    rx_data = malloc(data_size * sizeof(int));
    decode();
    assert_equal(rx_decoded, "");
    free(rx_data);
    free(rx_decoded);

    // Test 3: Decode a message with invalid Manchester encoding
    data_size = 4;
    rx_data = malloc(data_size * sizeof(int));
    rx_data[0] = 1; rx_data[1] = 1; // Invalid encoding
    rx_data[2] = 0; rx_data[3] = 1; // Valid encoding
    decode();
    // The decode function should print an error message and return early,
    // so the decoded message should be empty
    assert_equal(rx_decoded, "");
    free(rx_data);
    free(rx_decoded);
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
	// Transmit Manchester 1 Pair bit to PB1 i.e. 1 -> 01 -> 1 THEN 0
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
 * @brief	frees recived data and resets to defaults
 */
static void reset_rx_data(void)
{
    data_size = 0;	// array begins empty

	// Reset array size
	array_size = RXDATA_INITSIZE_BITS;

	// Reallocate array to init amount of bytes i.e. 40
	rx_data = (int*)realloc(rx_data, RXDATA_INITSIZE_BYTES*sizeof(int));
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

	busy_delay = NO;
}

/**
 * @brief	Relevant Setters/Getters.
 *
 */
int get_dataSize(void){
	return data_size;
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

    		tim2_ch1_isr_entred = 0;

			// Decode data i.e. 2 bits -> 1 bit, free rx_data, and reset params
			decode();

			// Reset recived data for more data
			reset_rx_data();



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
 * @brief	Interrupt service routine to monitor the network line state using
 * 			timer 2 and tracks edges seen on the input line.
 *
 */
void TIM2_IRQHandler(void)
{
	if(tim2->SR & CC1IF) // If the interrupt source is a capture event on channel 1
									 // every half-bit period "500 us" for transmitter
	{
		//Receiver
		/**
		 * NOTE: tim2 is also used as rx timer
		 *  a high-to-low transition in the middle of the bit period.
		 *  Thus, while in IDLE, the first edge that the receiver will see will be
		 *  the transition in the middle of the bit period, thus, the second bit period
		 *  starts 500 μs after the first falling edge in the transmission.
		 *  If you are not yet supporting a header, ensure any data you receive
		 *  also starts with a logic-0 to ensure consistent timing.
		 */

		// If we are recieving, tie to edge, ignore first edge
		if(is_recieving && tim2_ch1_isr_entred && was_edge)
		{
			// If there isn't enough room for another byte of data, increase the size of the array
			if(data_size >= array_size)
			{
				embiggen();
			}

			rx_data[data_size] = !curr_edge;
			data_size++;
		}

		// STARTS transmitting in IDLE, but can also in BUSY after... CANNOT
		// transmit in COLLISION
		else if(busy_delay == NO && (is_transmitting || state == IDLE))
		{
			// Transmit encoded half-bits i.e. 1 -> 1 THEN 0
			transmit();
		}

		tim2_ch1_isr_entred = 1;


		// Clear interrupt flag manually since not reading from
		tim2->SR = ~CC1IF;
	}

	if (tim2->SR & CC2IF) // if the interrupt source is a capture event on
						  // channel 2
	{

		prev_edge = curr_edge;
		curr_edge = gpiob->IDR & GPIO_IDR_Px3;

		// Store count values at the time of the most recent edge
		was_edge = 1;

		// Once edge detected, start 1.13 ms timer
		tim8->CR1 |= CEN;

		// All other states, BUSY, and IDLE also go to BUSY if not timeout
		if (state == COLLISION)
		{
			state = BUSY;
			led_enable(BUSY_LED_STATE); // Enables second to left LED
		//	busy_delay = YES;	// TODO: Enable once retransmit functionality
								// 		 is added.
		}
		// When state goes IDLE -> BUSY, begin receiving data
		else if(state == IDLE)
		{
			//Channel Monitor
			state = BUSY;
			led_enable(BUSY_LED_STATE); // Enables second to left LED

			// 1 to zero transition (idles high). Capture first edge.
			if(prev_edge && !curr_edge)
			{
				is_recieving = 1;
			}

		}
		tim2->SR = ~CC2IF; // Clear the interrupt flag manually/by software if
							// not set by capture event on channel 2
	}
}
