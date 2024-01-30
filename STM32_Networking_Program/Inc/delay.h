/**
 ******************************************************************************
 * @file	: delay.h
 * @author	: Created by Daniel Nimsgern <nimsgernd@msoe.edu>
 * 			:
 * @brief	: Header file for delay.c which defines values and functions
 * 			: defined within the file.
 ******************************************************************************
 */

#ifndef DELAY_H_
#define DELAY_H_

/*
 ******************************************************************************
 * Includes
 ******************************************************************************
 */

// Library
#include <inttypes.h>

// Project

/*
 ******************************************************************************
 * Values
 ******************************************************************************
 */

/*
 ******************************************************************************
 * SysTick Values
 ******************************************************************************
 */

// Bases
#define STK 0xE000E010

// Registers
#define CTRL 0x0
#define LOAD 0x1
#define VAL 0x2

// Values
#define STK_OFF ~(0b111)
#define STK_EN_STD_TIM 0b101
#define STK_EN_DIV8_TIM 0b001
#define IS_COUNT_FLAG_EN (1<<16)

/*
 ******************************************************************************
 * Function Prototype
 ******************************************************************************
 */
extern void delay_s(int time);
extern void delay_ms(int time);
extern void delay_us(int time);

#endif /* DELAY_H_ */
