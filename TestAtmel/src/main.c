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
/*	static const gpio_map_t USART_GPIO_MAP =
	{
		{EXAMPLE_USART_RX_PIN, EXAMPLE_USART_RX_FUNCTION},
		{EXAMPLE_USART_TX_PIN, EXAMPLE_USART_TX_FUNCTION}
	};

	// USART options.
	static const usart_options_t USART_OPTIONS =
	{
		.baudrate     = 57600,
		.charlength   = 8,
		.paritytype   = USART_NO_PARITY,
		.stopbits     = USART_1_STOPBIT,
		.channelmode  = USART_NORMAL_CHMODE
	};

	// Assign GPIO to USART.
	gpio_enable_module(USART_GPIO_MAP,
	sizeof(USART_GPIO_MAP) / sizeof(USART_GPIO_MAP[0]));

	// Initialize USART in RS232 mode.
	usart_init_rs232(EXAMPLE_USART, &USART_OPTIONS, EXAMPLE_TARGET_PBACLK_FREQ_HZ);

	// Hello world!
	usart_write_line(EXAMPLE_USART, "Hello, this is the AVR UC3 MCU saying hello! (press enter)\r\n");

	// Press enter to continue.
	while (usart_get_echo_line(EXAMPLE_USART) == USART_FAILURE);  // Get and echo characters until end of line.

	usart_write_line(EXAMPLE_USART, "Goodbye.\r\n"); */

#define USART_SERIAL                     &AVR32_USART1

static usart_serial_options_t usart_options = {
	.baudrate = 115200,
	.charlength = 8,
	.paritytype = USART_NO_PARITY,
	.stopbits = USART_1_STOPBIT
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
