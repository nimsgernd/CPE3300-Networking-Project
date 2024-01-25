#ifndef LED_H_
#define LED_H_

#define LED_MASK 0b1111011111100000    
#define LED_MASK_U4 0b1111000000        // Upper 4 bits
#define LED_MASK_L6 0b111111            // Lower 6 bits
#define MODE_MASK 0xFF3FFC00;           // Mode mask for setting GPIOB

// GPIOB Modes: Pins 8-15 as Alternate Function (10), Pins 4-7 as Output (01)
#define GPIOB_PINS_8_TO_15_AF 0x55000000
#define GPIOB_PINS_4_TO_7_OUT 0x00155400
#define GPIOB_LED_MODES (GPIOB_PINS_8_TO_15_AF | GPIOB_PINS_4_TO_7_OUT)

void led_init(void);
void led_enable(int number);


#endif