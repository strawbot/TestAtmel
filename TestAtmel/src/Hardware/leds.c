/*
 * leds.c
 *
 * Created: 2020-03-30 5:22:50 PM
 *  Author: Robert Chapman
 */ 

#include <asf.h>
#include "user_board.h"

void greenOn()  { gpio_set_gpio_pin(U0_IN_LED); }
void greenOff() { gpio_clr_gpio_pin(U0_IN_LED); }
void redOn()    { gpio_set_gpio_pin(U1_IN_LED); }
void redOff()   { gpio_clr_gpio_pin(U1_IN_LED); }

void im_alive() {
	greenOn();
	delay_ms(10);
	greenOff();
	delay_ms(190);
	greenOn();
	delay_ms(10);
	greenOff();
	delay_ms(790);
}
