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

// Project
#include "delay.h"
#include "led.h"
#include <network.h>
#include "uart_driver.h"

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

	led_init();
	monitor_init();

    /* Loop forever */
	for(;;)
	{

	}
}
