#include "tea.h"
#include "gpio.h"
#include "printers.h"
// #include "statCtrs.h"
#include "cli.h"
#include "rtc.h"
#include "tc.h"
#include "sysclk.h"
#include "pm.h"

void over_due() { /* incCtr(overDueTea); */ }

Event alarmEvent;

Long raw_time() { return rtc_get_value(&AVR32_RTC); }

void set_alarm(Long t) {
    Wr_bitfield(AVR32_TC.channel[0].rc, AVR32_TC_RC_MASK, t);
	AVR32_TC.channel[0].ccr = AVR32_TC_SWTRG_MASK | AVR32_TC_CLKEN_MASK;
}

// Clocks
void LETIMER_IRQ() { 
	now(*alarmEvent);
}

void init_clocks() {
	never(alarmEvent);
	AVR32_PM.oscctrl32 =
		AVR32_PM_OSCCTRL32_MODE_EXT_CLOCK |
		AVR32_PM_OSCCTRL32_OSC32EN_MASK;
//	rtc_init(&AVR32_RTC, RTC_OSC_32KHZ, RTC_PSEL_32KHZ_1KHZ);
	AVR32_TC.channel[0].cmr =
		AVR32_TC_WAVE_MASK |
		AVR32_TC_WAVSEL_UP_AUTO << AVR32_TC_WAVSEL_OFFSET |
		AVR32_TC_CPCSTOP_MASK;
	AVR32_TC.channel[0].ier = AVR32_TC_IER0_CPCS_MASK;
}

void show_timer() {
	// print("CNT:");
	// dotnb(4, 4, raw_time(), 16);
	// print("  LETIMER:");
	// dotnb(4, 4, LETIMER0->CNT, 16);
	// print("  ticks/S:");
	// printDec(ONE_SECOND);
}

void get_clock_frequency() {
	lit(sysclk_get_main_hz());
}

/*
 RTC is used as a free running ms counter. Its clock is the 32KHz/32 so it gets 1024 ticks/second
  RTC is the reference. TIMER0 is the interrupt.
*/
