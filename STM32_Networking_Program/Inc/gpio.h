/**
  ******************************************************************************
  * @file    gpio.h
  * @author  Zachary Kohlman
  * @email	 kohlmanz@msoe.edu
  * @version V1.0
  * @brief   Default main function.
  * Description: This header file defines various structs used in tone.c along
  * with the methods used.
  ******************************************************************************
*/

#ifndef GPIO_H_
#define GPIO_H_

#include <stdint.h>

/**********************************************************************************************
 *                                      REG ADDRESSES                                         *
 **********************************************************************************************/
#define RCC_BASE 0x40023800
#define GPIOB_BASE 0x40020400
#define GPIOA_BASE 0x40020000
#define TIM3_BASE 0x40000400
#define TIM4_BASE 0x40000800
#define NVIC_BASE 0xE000E100
#define SYSCFG_EXTICR2_BASE 0x4001380C

/**********************************************************************************************
 *                                          TIM BITS                                          *
 **********************************************************************************************/
// Clock enable bit in TIM control register (CR) [0 = disabled, 1 = enabled]
#define CEN 1<<0

// RCC TIM Enable bit pos for 16-bit timers (TIM3, TIM4) [0 = disabled, 1 = enabled]
#define TIM3EN 1 << 1
#define TIM4EN 1 << 2

// NVIC position of 16-bit timers (TIM3 and TIM4) [0 = disabled, 1 = enabled]
#define TIM3_POS 1 << 29
#define TIM4_POS 1 << 30

// Capture/Compare 1 interrupt enable for channel 1 [0 = disabled, 1 = enabled]
#define CC1IE 1<<1

// Status Register Interrupt Pending Flag for channel 1
// This bit is set by hardware on a capture. It is cleared by software or by
// reading the TIMx_CCR1 register
#define TIMx_SR_CC1F 1<<1

// CCMR to configure TIM3 to Timer Input Capture TIC
#define CC1S 0b10

// Capture Compare 1 Enable bit in CCER [0  = disabled, 1 = enabled]
#define CC1E 1<<0

// Capture Compare 1 Polarity for Capture/Compare on high
#define CC1P 1 << 1

// Capture Compare 1 Complimentary output enable
#define CC1NP 1 << 3



/**********************************************************************************************
 *                                          GPIO BITS                                         *
 **********************************************************************************************/
// RCC AHB1 peripheral clock enable register GPIO Enable bits [0  = disabled, 1 = enaabled]
#define GPIOAEN 1<<0
#define GPIOBEN 1<<1

// Alternate Function bit enable position for TIM3..5. Set in GPIO_AFRH/AFRL for TIC/TOC. 
// [0 = disabled, 1 = enabled]
#define AF2 0b0010
#define AFRL_PA6_AF2 AF2 << 6
#define GPIOA_PA6_MODER_AF AF2 << 12

/**********************************************************************************************
 *                                          EXTI BITS                                         *
 **********************************************************************************************/
// EXTICR2 used for lines 4-7 corresponding with PA4-PA7
#define SYSCFG_PA6_EXTICR2 0b0000



/**********************************************************************************************
 *                                   CONTROLLER STRUCTS                                       *
 **********************************************************************************************/
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
	const uint32_t RESERVRED;
	volatile uint32_t CCR1;
	volatile uint32_t CCR2;
	volatile uint32_t CCR3;
	volatile uint32_t CCR4;
	const uint32_t RESERVED2;
	volatile uint32_t DCR;
	volatile uint32_t DMAR;
	volatile uint32_t TIM2_OR;
	volatile uint32_t TIM5_OR;
}TIMER;

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
#endif /* GPIO_H_ */
