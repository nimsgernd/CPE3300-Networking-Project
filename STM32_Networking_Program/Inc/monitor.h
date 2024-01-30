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
// Error percentage = 1.32%
#define SIGNAL_MOE_PERCENTAGE 1.32f

// Error percentage expressed as a decimal i.e. 1.32/100 = 0.0132
#define SIGNAL_MOE_DECIMAL (float)(SIGNAL_MOE_PERCENTAGE / 100)

// Standard signal period... i.e. without 1.32% tolerance.
// 1 ms
#define NOMINAL_SIGNAL_PERIOD_US 1000

// 130 uS tolerance to period = 1.13 MAX TOLERANCE
#define SIGNAL_PERIOD_TOLERANCE_US 130

// 1000 uS + 130 uS = 1130 uS total period
#define ADJUSTED_SIGNAL_PERIOD_US NOMINAL_SIGNAL_PERIOD_US + SIGNAL_PERIOD_TOLERANCE_US

// Frequency representing the adjusted period. f = 1/T = 884 Hz
#define SIGNAL_TOLERANCE_FREQUENCY_HZ (int)(1/(long long)(ADJUSTED_SIGNAL_PERIOD_US * 1e-6))

// Number of ticks = Timer frequency (Hz) * Desired period (s)
#define THRESHOLD_TICKS (int)(F_CPU * (long long)ADJUSTED_SIGNAL_PERIOD_US*1e-6)-1
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
