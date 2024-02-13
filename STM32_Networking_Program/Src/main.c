/**
 ******************************************************************************
 * @file	: main.c
 * @authors	: Auto-generated by STM32CubeIDE
 * 			: Edited by			Zack Kohlman 	<kohlmanz@msoe.edu>
 * 			:					Jack Maki		<makij@msoe.edu>
 * 			:					Daniel Nimsgern	<nimsgernd@msoe.edu>
 * 			:
 * @brief	: Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

/*
 ******************************************************************************
 * Includes
 ******************************************************************************
 */

// Library
#include <stdio.h>
#include <stdlib.h>

// Project
#include "console.h"
#include "delay.h"
#include "led.h"
#include "network.h"

/*
 ******************************************************************************
 * Variables
 ******************************************************************************
 */

/*
 ******************************************************************************
 * Main Function
 ******************************************************************************
 */

/**
 * @brief	CPE 3300 network interface program (currently only monitor
 * 			functionality.
 *
 * @return	Should never return.
 */
int main(void)
{
	// Initialize CPE3300 network monitor
	monitor_init();

	// Initialize program console
	console_init();

	// Intitalize reciever
	rx_init();

#ifdef DE_NET_RX
	test_decode();
#endif

    /* Loop forever */
	for(;;)
	{
		user_prompt();	// Take input from user
		post_collision_delay();	// Check if delay is needed TODO: may need to
								// be relocated
	}
	// Free dynamic memory
	clear();
}
