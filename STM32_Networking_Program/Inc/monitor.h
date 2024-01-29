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
#define TIM_TO_MICROSEC_SCALAR	15

/*
 ******************************************************************************
 * Thresholds & Timeouts
 ******************************************************************************
 */

/*SIGNAL THRESHOLDS*/
#define SIGNAL_MOE_PERCENTAGE 1.32f
#define SIGNAL_MOE_DECIMAL (float)(SIGNAL_MOE_PERCENTAGE / 100)

/*IDLE HIGH AND LOW THRESHOLDS IN MICROSECONDS*/
#define IDLE_THRESHOLD_LOW_US 1113
#define IDLE_THRESHOLD_HIGH_US 1188

/*IDLE HIGH AND LOW MOE IN MICROSECONDS*/
#define IDLE_MOE_LOW_US (int)(IDLE_THRESHOLD_LOW_US - (IDLE_THRESHOLD_LOW_US * SIGNAL_MOE_DECIMAL))
#define IDLE_MOE_HIGH_US (int)(IDLE_THRESHOLD_HIGH_US + (IDLE_THRESHOLD_HIGH_US * SIGNAL_MOE_DECIMAL))

/*COLLISION HIGH AND LOW THRESHOLDS IN MICROSECONDS*/
#define COLLISION_THRESHOLD_LOW_US 1040
#define COLLISION_THRESHOLD_HIGH_US 1140

/*COLLISION HIGH AND LOW MOE IN MICROSECONDS*/
#define COLLISION_MOE_LOW_US (int)(COLLISION_THRESHOLD_LOW_US - (COLLISION_THRESHOLD_LOW_US * SIGNAL_MOE_DECIMAL))
#define COLLISION_MOE_HIGH_US (int)(COLLISION_THRESHOLD_HIGH_US + (COLLISION_THRESHOLD_HIGH_US * SIGNAL_MOE_DECIMAL))

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
