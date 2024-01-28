/**
 ******************************************************************************
 * @file	: delay.c
 * @author	: Created by Daniel Nimsgern <nimsgernd@msoe.edu>
 * 			:
 * @brief	: Implements a second, millisecond, and microsecond delay using the
 *			: SysTick timer to take in a amount of time to delay and then hold
 *			: the process until the timer is over.
 ******************************************************************************
 */

#include <stdio.h>
#include "delay.h"

#define F_CPU 16000000UL

/*
 ******************************************************************************
 * Pointers
 ******************************************************************************
 */

/* SysTick */
static volatile uint32_t *const stk = (uint32_t*) STK;

/*
 ******************************************************************************
 * Function Definitions
 ******************************************************************************
 */

/**
 * @brief	Delays by the input number of seconds using SysTick.
 *
 * @param	time - Amount of seconds to delay (max 8 seconds).
 */
void delay_s(int time)
{
	/* Set up and start SysTick */
	stk[CTRL] &= STK_OFF;
	stk[LOAD] = (time*2000000);
	stk[VAL] = 0;
	stk[CTRL] |= STK_EN_DIV8_TIM;

	/* Wait until SysTick hits zero*/
	while ((stk[CTRL] & IS_COUNT_FLAG_EN) != IS_COUNT_FLAG_EN);

	/* Stop SysTick and rest the counter */
	stk[CTRL] &= STK_OFF;
	stk[VAL] = 0;

	/* Return back to calling program */
	return;
}

/**
 * @brief 	Delays by the input number of milliseconds using the SysTick.
 *
 * @param	time - Amount of milliseconds to delay.
 */
void delay_ms(int time)
{
	/* Set up and start SysTick */
	stk[CTRL] &= STK_OFF;
	stk[LOAD] = (time*16000);
	stk[VAL] = 0;
	stk[CTRL] |= STK_EN_STD_TIM;

	/* Wait until SysTick hits zero*/
	while ((stk[CTRL] & IS_COUNT_FLAG_EN) != IS_COUNT_FLAG_EN);

	/* Stop SysTick and rest the counter */
	stk[CTRL] &= STK_OFF;
	stk[VAL] = 0;

	/* Return back to calling program */
	return;
}

/**
 * @brief	Delays by the input number of microseconds using the SysTick.
 *
 * @param	time - Amount of microseconds to delay.
 */
void delay_us(int time)
{
	/* Set up and start SysTick */
	stk[CTRL] &= STK_OFF;
	stk[LOAD] = (time*16);
	stk[VAL] = 0;
	stk[CTRL] |= STK_EN_STD_TIM;

	/* Wait until SysTick hits zero*/
	while ((stk[CTRL] & IS_COUNT_FLAG_EN) != IS_COUNT_FLAG_EN);

	/* Stop SysTick and rest the counter */
	stk[CTRL] &= STK_OFF;
	stk[VAL] = 0;

	/* Return back to calling program */
	return;
}
