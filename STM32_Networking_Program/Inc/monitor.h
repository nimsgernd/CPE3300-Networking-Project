/**
 ******************************************************************************
 * @file	: monitor.h
 * @authors	: File created by	Zach Kohlman	<kohlmanz@msoe.edu>
 * 			: Formatting by		Daniel Nimsgern	<nimsgernd@msoe.edu>
 * 			:
 * @brief 	: State enumerations and function prototypes for monitor.c
 ******************************************************************************
 */

#ifndef MONITOR_H_
#define MONITOR_H_

/*
 ******************************************************************************
 * Values
 ******************************************************************************
 */

#define RT_MAX_VALUE 			0x7F7F7F
#define RT_TO_MICROSEC_SCALAR	30

#define DELAY_TIME_US 1300 // 1.3 ms

/*
 ******************************************************************************
 * Enumerations
 ******************************************************************************
 */

enum State 
{
    BUSY,
    IDLE,
    COLLISION
};

/*
 ******************************************************************************
 * Function Prototypes
 ******************************************************************************
 */

void monitor_init(void);
void monitor(void);

#endif
