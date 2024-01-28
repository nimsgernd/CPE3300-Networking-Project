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
