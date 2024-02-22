/**
 ******************************************************************************
 * @file	: monitor.h
 * @authors	: File created by	Zach Kohlman	<kohlmanz@msoe.edu>
 * 			: Formatting by		Daniel Nimsgern	<nimsgernd@msoe.edu>
 * 			:
 * @brief 	: State enumerations and function prototypes for monitor.c
 ******************************************************************************
 */
#include <stdint.h>

#ifndef NETWORK_H_
#define NETWORK_H_
/*
 ******************************************************************************
 * Values
 ******************************************************************************
 */

#define RT_MAX_VALUE 			0x7F7F7F
#define TX_DELAY_SCALAR			((double)16000/(double)65535)
#define RXDATA_INITSIZE_BYTES 	261
#define RXDATA_INITSIZE_BITS 	(BYTE * RXDATA_INITSIZE_BYTES) // 40 Bytes
#define BYTE_LEN 				8 	// 1 Byte = 8 bits
#define MAX_MSG_LEN_BYTES 		255 // Msg section in data link layer supports up to 255 bytes
#define MAX_MSG_LEN_BITS 		MAX_MSG_LEN_BYTES * BYTE_LEN // 255 bytes in bits
#define PACKET_LEN				7
#define MIN_PACKET_LEN_BYTES	((BYTE)*(PACKET_LEN-1))
#define DECIMAL					10
#define BYTE 8


/*
 ******************************************************************************
 * Thresholds & Timeouts
 ******************************************************************************
 */

/*SIGNAL THRESHOLDS*/
// Standard signal period... i.e. without 1.32% tolerance.
// 1 ms
#define NOMINAL_SIGNAL_PERIOD_US 1000

// 130 uS tolerance to period = 1.13 MAX TOLERANCE.
#define SIGNAL_PERIOD_TOLERANCE_US 130

// 1000 uS + 130 uS = 1130 uS total period
// THIS IS INSIDE IDLE THRESHOLD (1.113 ms - 1.188 ms) and
// COLLISION TRESHOLD (1.04 ms - 1.14 ms)
#define ADJUSTED_SIGNAL_PERIOD_US (NOMINAL_SIGNAL_PERIOD_US + SIGNAL_PERIOD_TOLERANCE_US)

// Number of ticks = Timer frequency (Hz) * Desired period (s)
#define THRESHOLD_TICKS (int)(F_CPU * (long long)ADJUSTED_SIGNAL_PERIOD_US*1e-6)

/*
 ******************************************************************************
 * Enumerations
 ******************************************************************************
 */

typedef enum
{
    BUSY,
    IDLE,
    COLLISION
}State;

typedef enum
{
	YES,
	NO
}Delay;

typedef struct
{
	uint8_t PREAMBLE;
	uint8_t SRC;
	uint8_t DEST;
	uint8_t LEN;
	uint8_t CRC;
	char* MSG;
	uint8_t TRAILER;

}packet;
/*
 ******************************************************************************
 * Function Prototypes
 ******************************************************************************
 */

extern void monitor_init(void);
extern void clear(void);
extern int new_message_flag(void);
void set_sender(int addr);
void set_reciever(int addr);
int get_sender(void);
int get_reciever(void);
extern int* get_raw_data(void);
extern char* get_ascii_data(void);
extern int get_dataSize(void);
extern void encode(char* msg);
extern void post_collision_delay(void);
extern void TIM8_UP_TIM13_IRQHandler(void);
extern void TIM2_IRQHandler(void);
extern void decode(void);
extern void reset_rx_data(void);
#endif
