/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <asf.h>

#define U1_IN_LED	AVR32_PIN_PA29		// uart1 led input indicator, init as gpio output, value 0 (OFF)
#define U0_IN_LED	AVR32_PIN_PA30		// uart0 led input indicator, init as gpio output, value 0 (OFF)

void greenOn() { gpio_set_gpio_pin(U0_IN_LED); }
void greenOff() { gpio_clr_gpio_pin(U0_IN_LED); }
void redOn() { gpio_set_gpio_pin(U1_IN_LED); }
void redOff() { gpio_clr_gpio_pin(U1_IN_LED); }
	
void init() {
	greenOff();
	redOff();
}

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */

	board_init();
	init();
	while(1) {
		 // uart1 input LED on
		greenOn(); // uart1 input LED off
		delay_ms(10);
		greenOff();
		delay_ms(190);
		greenOn();
		delay_ms(10);
		greenOff();
		delay_ms(790);
	}
	/* Insert application code here, after the board has been initialized. */
}
