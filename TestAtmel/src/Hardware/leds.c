/*
 * leds.c
 *
 * Created: 2020-03-30 5:22:50 PM
 *  Author: Robert Chapman
 */ 

#include "pm.h"
#include <asf.h> // must have pm.h included before for some dang reason
#include <interrupt.h>
#include "intc.h"
#include "tc.h"
#include "tea.h"

void greenOn()  { gpio_set_gpio_pin(U0_IN_LED); }
void greenOff() { gpio_clr_gpio_pin(U0_IN_LED); }
bool redOn()    { gpio_set_gpio_pin(U1_IN_LED); return true; }
bool redOff()   { gpio_clr_gpio_pin(U1_IN_LED); return false; }

#define sleeping() false

// board led
static void blink_led() { greenOn();  in(msec(5), greenOff); }

void im_alive() {
	blink_led();
	in(msec(200), blink_led);

	Long period = sleeping()? 5 : 1;
	in(secs(period), im_alive);
}

void init_leds() {
	redOff();
	greenOff();
	later(im_alive);
}