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

#include <stdio.h>
#include <string.h>

// Project
#include "uart_driver.h"
#include "F446RE.h"
#include "network.h"

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
	init_usart2(57600, F_CPU);

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
	printf("token1 = '%s'\n", token1);
	token2 = strtok(NULL," ");
	if(!strcmp(token1,"xmit")){
		//TODO put tranmit func here
		encode(token2);
		printf("transmitting '%s'...\n",token2);
	} else if(!strcmp(token1,"signal_1k\n")){
		printf("transmitting 1KHz square wave for 5 seconds...\n");
	} else {
		printf("ERR: unknown command.\n");
	}

}
