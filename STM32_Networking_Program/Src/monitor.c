#include <stdlib.h>
#include <stdint.h>
#include "monitor.h"
#include "gpio.h"


/**
 * IMPORTANT: INPUT SIGNAL ASSUMED TO BE ON ARDUINO HEADER/PIN PA_6!!!!
*/

/* Addresses*/
static volatile TIMER* const tim3 = (TIMER*)TIM3_BASE;
static volatile RCC* const rcc = (RCC*)RCC_BASE;
static volatile uint32_t* const nvic_iser0 = (uint32_t*)NVIC_BASE;
static volatile GPIO* const gpioa = (GPIO*)GPIOA_BASE;

/* File variables and constants*/
static enum State state = IDLE; // Current state

void monitor_init(void)
{
    // Enable clock for GPIOA
    rcc->AHB1ENR |= GPIOAEN;

    // Enable clock for tim3
    rcc->APB1ENR |= TIM3EN;

    // For additional timer, un-comment the following line for TIM4
    // rcc->APB1ENR |= TIM4EN; 

    // Set PA6 to alternate function for TIC/TOC
    gpioa->MODER |= GPIOA_PA6_MODER_AF;

    // Set PA6 to alternate function in Alternate Function Register Low (for GPIO pins 0 - 7)
    gpioa->AFRL = AF2 << AFRL_PA6_AF2;

    // Open interrupt for TIM3
    *nvic_iser0 = TIM3_POS;

    // Set to TIC in compare compare mode register 1 in CC
    tim3->CCMR1 = CC1S;

    // Enable the capture compare for the channel
    tim3->CCER |= CC1E;

    // Set the direction of the input capture (rising edge, falling edge, or both)
    tim3->CCER |= CC1P | CC1NP; // Rising edge

    // Load the capture compare register with a value of 1.13ms (for TOC)
    //tim3->CCR1 = 0x2C240;
	
    // Enable the interrupt on capture compare
    tim3->DIER |= CC1IE;

    // Enable the counter
    tim3->CR1 |= CEN;
}


// Timer 3 interrupt fires when the timer is active for over 1.13ms
void TIM3_IRQHandler(void)
{
	// Clear interrupt flag
	tim3->SR = 0;
	int channel = (gpioa->IDR)&0x40; // Mask for bit 6 (PA6)
	channel = channel >> 6;          // Right shift to position 0
	if(channel == 1){
		// channel has been high for over 1.13ms, revert to idle
		state = IDLE;
	} else {
		// channel has been low for over 1.13ms, collision detected
		state = COLLISION;
	}
}
