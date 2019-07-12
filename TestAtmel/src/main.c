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

#define USART_SERIAL                     &USARTD0
#define USART_SERIAL_BAUDRATE            115200
#define USART_SERIAL_CHAR_LENGTH         US_MR_CHRL_8_BIT
#define USART_SERIAL_PARITY              US_MR_PAR_NO
#define USART_SERIAL_STOP_BIT            false

static usart_serial_options_t usart_options = {
	.baudrate = USART_SERIAL_BAUDRATE,
	.charlength = USART_SERIAL_CHAR_LENGTH,
	.paritytype = USART_SERIAL_PARITY,
	.stopbits = USART_SERIAL_STOP_BIT
};

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */

	board_init();
	init();
	usart_serial_init(USART_SERIAL, &usart_options);

	const char * banner = "Good Mornin!";
	while (*banner) usart_serial_putchar(USART_SERIAL, *banner++);
	
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
