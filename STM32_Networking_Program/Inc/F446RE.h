/**
 ******************************************************************************
 * @file	: gpio.h
 * @authors	: File created by	  Zachary Kohlman	<kohlmanz@msoe.edu>
 * 			: Updated Comments by Daniel Nimsgern <nimsgernd@msoe.edu>
 * @version	: V1.1
 * @brief	: This header file defines various structs used in controlling GPIO
 * 			: and other component registers.
 ******************************************************************************
 */

#ifndef F446RE_H_
#define F446RE_H_

/*
 ******************************************************************************
 * Includes
 ******************************************************************************
 */

// Library
#include <stdint.h>

// Project


/*
 ******************************************************************************
 * Debug defines
 ******************************************************************************
 */

//#define DE_CONSOLE
//#define DE_NET_MON
#define DE_NET_RX
//#define DE_NET_TX

/*
 ******************************************************************************
 * Processor frequency
 ******************************************************************************
 */

// CPU Frequency in Hz
#define F_CPU 16000000UL

/*
 ******************************************************************************
 * REG ADDRESSES
 ******************************************************************************
 */

#define RCC_BASE			0x40023800
#define GPIOB_BASE			0x40020400
#define GPIOA_BASE			0x40020000
#define TIM2_BASE 			0x40000000
#define TIM8_BASE 			0x40010400
#define TIM14_BASE			0x40002000
#define RTC_BASE			0x40002800
#define NVIC_BASE			0xE000E100
#define NVIC_ISER1			0xE000E104
#define SYSCFG_EXTICR2_BASE 0x4001380C

/*
 ******************************************************************************
 * TIM BITS
 ******************************************************************************
 */

// Clock enable bit in TIM control register (CR)
#define CEN	1<<0	// [0 = disabled, 1 = enabled]

#define UIE	1 << 0		// Timer interrupt enable [0 = disabled, 1 = enabled]
#define UIF	1 <<0		// Update Interrupt flag

// RCC TIM Enable bit pos for 16-bit timers (TIM3, TIM4)
#define TIM2EN 	1 << 0	// [0 = disabled, 1 = enabled]
#define TIM8EN 	1 << 1
#define TIM14EN	1 << 8	// [0 = disabled, 1 = enabled]

// NVIC position of 16-bit timers (TIM3 and TIM4)
#define TIM2_POS 			1 << 28	// [0 = disabled, 1 = enabled]
#define TIM8_UP_TIM13_POS 	44

// Capture/Compare 1 interrupt enable for channel 1
#define CC1IE	1 << 1	// [0 = disabled, 1 = enabled]
#define CC2IE	1 << 2

// CCMR to configure TIM3 to Timer Input Capture TIC
#define CC1S	1 << 0	// [0 = CC1 channel is configured as output, 1 = CC1 channel is configured as input, IC1 is mapped on TI1]
#define CC2S	0b01 << 8

// Capture Compare 1 Enable bit in CCER
#define CC1E	1 << 0	// [0  = disabled, 1 = enabled]
#define CC2E	1 << 4

// Capture Compare 1 Polarity for Capture/Compare on high
#define CC1P	1 << 0
#define CC2P	1 << 5

// Capture Compare 1 Complimentary output enable
#define CC1NP	1 << 0
#define CC2NP	1 << 7

// Capture Compare 1 Interrupt Flag
// This bit is set by hardware on a capture. It is cleared by software or by
// reading the TIMx_CCR1 register
#define CC1IF	1 << 1 // [0 = No input capture occurred, 1 = Input capture occurred in TIMx_CCR1 register
#define CC2IF	1 << 2

// Event Generation Register
#define UG 1 << 0 // [0 = No action, 1 = Reinitialize the counter and generates an update of the registers]

/*
 ******************************************************************************
 * GPIO BITS
 ******************************************************************************
 */

// RCC AHB1 peripheral clock enable register GPIO Enable bits
#define GPIOAEN	1 << 0	// [0  = disabled, 1 = enabled]
#define GPIOBEN 1 << 1	// [0  = disabled, 1 = enabled]

/* MODER Settings */
// Modes
#define MODER_IN	0b00
#define MODER_OUT	0b01
#define MODER_AF	0b10
#define MODER_ALOG	0b11

// Output
#define GPIO_MODER_Px1_OUT	MODER_OUT << 2

// Alternate Function
#define GPIO_MODER_Px3_AF	MODER_AF << 6
#define GPIO_MODER_Px15_AF	MODER_AF << 30

/* PUPDR Settings */
// Modes
#define PUPDR_NONE	0b00
#define PUPDR_UP	0b01
#define PUPDR_DOWN	0b10

// Pull Up
#define GPIO_PUPDR_Px3_UP	PUPDR_UP << 6

/* IDR Reg */
#define GPIO_IDR_Px0	1 << 0
#define GPIO_IDR_Px1	1 << 1
#define GPIO_IDR_Px2	1 << 2
#define GPIO_IDR_Px3	1 << 3
#define GPIO_IDR_Px4	1 << 4
#define GPIO_IDR_Px5	1 << 5
#define GPIO_IDR_Px6	1 << 6
#define GPIO_IDR_Px7	1 << 7
#define GPIO_IDR_Px8	1 << 8
#define GPIO_IDR_Px9	1 << 9
#define GPIO_IDR_Px10	1 << 10
#define GPIO_IDR_Px11	1 << 11
#define GPIO_IDR_Px12	1 << 12
#define GPIO_IDR_Px13	1 << 13
#define GPIO_IDR_Px14	1 << 14
#define GPIO_IDR_Px15	1 << 15 // GPIO_IDR PA15 bit

/* ODR Reg */
#define GPIO_ODR_Px0	~(1 << 0)
#define GPIO_ODR_Px1	~(1 << 1)
#define GPIO_ODR_Px2	~(1 << 2)
#define GPIO_ODR_Px3	~(1 << 3)
#define GPIO_ODR_Px4	~(1 << 4)
#define GPIO_ODR_Px5	~(1 << 5)
#define GPIO_ODR_Px6	~(1 << 6)
#define GPIO_ODR_Px7	~(1 << 7)
#define GPIO_ODR_Px8	~(1 << 8)
#define GPIO_ODR_Px9	~(1 << 9)
#define GPIO_ODR_Px10	~(1 << 10)
#define GPIO_ODR_Px11	~(1 << 11)
#define GPIO_ODR_Px12	~(1 << 12)
#define GPIO_ODR_Px13	~(1 << 13)
#define GPIO_ODR_Px14	~(1 << 14)
#define GPIO_ODR_Px15	~(1 << 15)

/* ALternate Function Settings */
// Modes
#define AF_0	0b0000
#define AF_1	0b0001
#define AF_2	0b0010
#define AF_3	0b0011
#define AF_4	0b0100
#define AF_5	0b0101
#define AF_6	0b0110
#define AF_7	0b0111
#define AF_8	0b1000
#define AF_9	0b1001
#define AF_10	0b1010
#define AF_11	0b1011
#define AF_12	0b1100
#define AF_13	0b1101
#define AF_14	0b1110
#define AF_15	0b1111

// Alternate Function 0

// Alternate Function 1
#define GPIO_AFRL_Px3_AF1	AF_1 << 12
#define GPIO_AFRH_Px15_AF1	AF_1 << 28

// Alternate Function 2

// Alternate Function 3

// Alternate Function 4

// Alternate Function 5

// Alternate Function 6

// Alternate Function 7

// Alternate Function 8

// Alternate Function 9

// Alternate Function 10

// Alternate Function 11

// Alternate Function 12

// Alternate Function 13

// Alternate Function 14

// Alternate Function 15

/*
 ******************************************************************************
 * LSI Bits
 ******************************************************************************
 */

#define LSION 0

/*
 ******************************************************************************
 * RTC Bits
 ******************************************************************************
 */

#define RTCEN 1 << 15
#define RTCSEL_LSE 0b01 << 8
#define RTCSEL_LSI 0b10 << 8
#define RTCSEL_HSE 0b11 << 8

/*
 ******************************************************************************
 * CONTROLLER STRUCTS
 ******************************************************************************
 */

typedef struct
{
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    volatile uint32_t RESERVED0;
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t RESERVED1[2];
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t AHB3ENR;
    volatile uint32_t RESERVED2;
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t RESERVED3[2];
    volatile uint32_t AHB1LPENR;
    volatile uint32_t AHB2LPENR;
    volatile uint32_t AHB3LPENR;
    volatile uint32_t RESERVED4;
    volatile uint32_t APB1LPENR;
    volatile uint32_t APB2LPENR;
    volatile uint32_t RESERVED5[2];
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    volatile uint32_t RESERVED6[2];
    volatile uint32_t SSCGR;
    volatile uint32_t PLLI2SCFGR;
    volatile uint32_t PLLSAICFGR;
    volatile uint32_t DCKCFGR;
    volatile uint32_t CKGATENR;
    volatile uint32_t DCKCFGR2;
}RCC;

typedef struct {
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDER;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
}GPIO;

typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMCR;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	volatile uint32_t CCMR1;
	volatile uint32_t CCMR2;
	volatile uint32_t CCER;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
	volatile uint32_t RCR;
	volatile uint32_t CCR1;
	volatile uint32_t CCR2;
	volatile uint32_t CCR3;
	volatile uint32_t CCR4;
	volatile uint32_t BDTR;
	volatile uint32_t DCR;
	volatile uint32_t DMAR;
}ACTIM16B;

typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMCR;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	volatile uint32_t CCMR1;
	volatile uint32_t CCMR2;
	volatile uint32_t CCER;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
	const uint32_t RESERVRED;
	volatile uint32_t CCR1;
	volatile uint32_t CCR2;
	volatile uint32_t CCR3;
	volatile uint32_t CCR4;
	const uint32_t RESERVED1;
	volatile uint32_t DCR;
	volatile uint32_t DMAR;
	volatile uint32_t TIM2_OR;
	volatile uint32_t TIM5_OR;
}GPTIM16B32B;

typedef struct{
	volatile uint32_t CR1;
	const uint32_t RESERVED;
	volatile uint32_t SMCR;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	volatile uint32_t CCMR1;
	const uint32_t RESERVED1;
	volatile uint32_t CCER;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
	const uint32_t RESERVED2;
	volatile uint32_t CCR1;
	const uint32_t RESERVED3;
	const uint32_t RESERVED4;
	const uint32_t RESERVED5;
	const uint32_t RESERVED6;
	const uint32_t RESERVED7;
	const uint32_t RESERVED8;
	volatile uint32_t OR;
}GPTIM16B;

typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	const uint32_t RESERVED;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	const uint32_t RESERVED1;
	const uint32_t RESERVED2;
	const uint32_t RESERVED3;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
}BTIM16B;

typedef struct{
	volatile uint32_t TR;
	volatile uint32_t DR;
	volatile uint32_t CR;
	volatile uint32_t ISR;
	volatile uint32_t PRER;
	volatile uint32_t WUTR;
	volatile uint32_t CALIBR;
	volatile uint32_t ALRMAR;
	volatile uint32_t ALRMBR;
	volatile uint32_t WPR;
	const uint32_t SSR;
	volatile uint32_t SHIFTR;
	const uint32_t TSTR;
	const uint32_t TSDR;
	const uint32_t TSSSR;
	volatile uint32_t CALR;
	volatile uint32_t TAFCR;
	volatile uint32_t ALRMASSR;
	volatile uint32_t ALRMBSSR;
}RTC;

typedef struct{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR1;
	volatile uint32_t EXTICR2;
	volatile uint32_t EXTICR3;
	volatile uint32_t EXTICR4;
	volatile uint32_t CMPCR;
	volatile uint32_t CFGR;
}SYSCFG;

typedef struct{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI;


#endif /* F446RE_H_ */
