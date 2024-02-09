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
	printf("token1 = '%s'\n\r", token1);
	token2 = strtok(NULL,"\n\r");
	if(!strcmp(token1,"t")){
		//TODO put tranmit func here
		encode(token2);
		printf("transmitting '%s'...\n\r",token2);
	} else if(!strcmp(token1,"signal_1k\n")){
		printf("transmitting 1KHz square wave for 5 seconds...\n\r");
	} else if (!strcmp(token1,"rx")){
		printf("Console is now in RECEIVER DEBUG MODE\n\r");
		for(;;){
			int num_bits = getDataSize();
			if(num_bits>0){
				//TODO getData function
				//printf("%d", getData());
			}
		}
	}
	} else {
		printf("ERR: unknown command.\n\r");
	}

}
