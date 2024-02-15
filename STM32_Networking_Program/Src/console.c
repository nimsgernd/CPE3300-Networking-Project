/**
 ******************************************************************************
 * @file	: console.c
 * @authors	: Zack Kohlman		<kohlmanz@msoe.edu>
 *			: Jack Maki			<makij@msoe.edu>
 *			: Daniel Nimsgern	<nimsgern@msoe.edu>
 * 			:
 * @brief	:
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

#include <stdio.h>
#include <string.h>

// Project
#include "F446RE.h"
#include "network.h"
#include "uart_driver.h"
#include "delay.h"

/*
 ******************************************************************************
 * Variables
 ******************************************************************************
 */

char str[100];
char * token1;
char * token2;
char * token3;

/*
 ******************************************************************************
 * Function Prototypes
 ******************************************************************************
 */



/*
 ******************************************************************************
 * Function Definitions
 ******************************************************************************
 */

/**
 * @brief
 *
 */
void console_init(void)
{

	// Initialize UART connection
	init_usart2(921600, F_CPU);

}

/**
 * @brief
 *
 */
void user_prompt(void)
{

	printf("net> ");
	fgets(str, 99, stdin);
	token1 = strtok(str," ");
#ifdef DE_CONSOLE
	printf("token1 = '%s'\n\r", token1);
#endif
	token2 = strtok(NULL,"\n\r");
#ifdef DE_CONSOLE
	if(token2)
	{
		printf("token2 = '%s'\n\r", token2);
	}
#endif
	if(!strcmp(token1,"tx"))
	{
		if(token2)
		{
			encode(token2);
			printf("transmitting '%s'...\n\r",token2);
		}
		else
		{
			printf("Need string to transmit\n\r");
		}
	}
	else if(!strcmp(token1,"rx"))
	{
		// Checks for a recieved message, prints it to console, then returns to command prompt
		if(get_dataSize > 0)
		{
			printf("%s\n\r", get_ascii_data());
#ifdef DE_NET_RX
			printf("%d\n\r", get_dataSize());
			printf("0x%x\n\r", *get_raw_data());
#endif
		}
		else
		{
			printf("No data recieved\n\r");
		}
	}
	else if (!strcmp(token1,"r"))
	{
		printf("Console is now in RECEIVER TEST MODE\n\r");
		for(;;)
		{
			// If there's data inside the buffer, then print it
			if(get_dataSize() > 0)
			{
				int* data = get_raw_data();
				for (int i = 0; i < get_dataSize(); i++) {
					printf("%c", (char)data[i]);
				}
				printf("\n\r");
			}
			// Give time for transmissions to complete before reading them
			delay_s(1);
		}
	}
	else if(!strcmp(token1, "h"))
	{
		printf("=========================================================\n\r");
		printf("tx [string]\n\r");
		printf("	Transmits the string input by the user\n\r");
		printf("\n\r");
		printf("rx\n\r");
		printf("	Prints the latest message to the console\n\r");
		printf("\n\r");
		printf("r\n\r");
		printf("	Continuously prints messages to the console\n\r");
		printf("	(Does not return)\n\r");
		printf("=========================================================\n\r");
	}
	else
	{
		printf("ERR: unknown command.\n\r");
	}
}
