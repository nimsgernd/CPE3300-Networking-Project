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
	token1 = strtok(str," \n");
	printf("token1 = '%s'\n\r", token1);
	token2 = strtok(NULL,"\n\r");
	if(!strcmp(token1,"tx"))
	{
		if(token2)
		{
			encode(token2);
			printf("transmitting '%s'...\n\r",token2);
		}
		else
		{
			printf("Need to have a message to transmit\n\r");
		}
	}
	else if(!strcmp(token1, "rx"))
	{
		// Checks for a recieved message, prints it to console, then returns to command prompt
		if(get_dataSize > 0)
		{
			printf("%s\n\r", get_ascii_data());
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
		printf("tx [message]\n\r");
		printf("	transmits the string input after the command\n\r");
		printf("\n\r");
		printf("rx\n\r");
		printf("	prints the most recent message to the console\n\r");
		printf("\n\r");
		printf("r\n\r");
		printf("	continuously prints messages to the console\n\r");
		printf("	(does not exit)\n\r");
		printf("=========================================================\n\r");

	}
	else
	{
		printf("ERR: unknown command.\n\r");
	}
}
