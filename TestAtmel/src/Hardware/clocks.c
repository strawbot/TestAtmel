// Clocks
/*
 RTC is used as a free running ms counter. Its clock is the 32KHz/32 so it gets 1024 ticks/second
  RTC is the reference. TIMER0 is the time event interrupt which also runs from 32KHz. It is set
  byte the next time event.
*/
/* Kinda impordint
 The recommended way of clearing an interrupt request is a store operation
 to the controlling peripheral register, followed by a dummy load operation 
 from the same register. This causes a pipeline stall, which prevents the 
 interrupt from accidentally re-triggering in case the handler is exited 
 and the interrupt mask is cleared before the interrupt request is cleared.
*/
/* Oh, and BTW:
 Always clear or disable peripheral interrupts with the following sequence:
	1: Mask the interrupt in the CPU by setting GM (or IxM) in SR.
	2: Perform the bus access to the peripheral register that clears or
	disables the interrupt.
	3: Wait until the interrupt has actually been cleared or disabled by the
	peripheral. This is usually performed by reading from a register in the
	same peripheral (it DOES NOT have to be the same register that was
	accessed in step 2, but it MUST be in the same peripheral), what takes
	bus system latencies into account, but peripheral internal latencies
	(generally 0 cycle) also have to be considered.
	4: Unmask the interrupt in the CPU by clearing GM (or IxM) in SR.
	
	Note that steps 1 and 4 are useless inside interrupt handlers as the
	corresponding interrupt level is automatically masked by IxM (unless IxM
	is explicitly cleared by the software).
*/

#include "tea.h"
#include "gpio.h"
#include "printers.h"
// #include "statCtrs.h"
#include "cli.h"
#include "rtc.h"
#include "tc.h"
#include "sysclk.h"
#include "pm.h"
#include "intc.h"

// RTC is run from 32Khz/32 clock. TC is run by 32Khz clock. 
#define RTC_SCALE 32

void over_due() { /* incCtr(overDueTea); */ }

Event alarmEvent;

Long raw_time() { return rtc_get_value(&AVR32_RTC); }

void set_alarm(Long t) {
    Wr_bitfield(AVR32_TC.channel[0].rc, AVR32_TC_RC_MASK, t * RTC_SCALE);
	AVR32_TC.channel[0].ccr = AVR32_TC_SWTRG_MASK;
	(void) AVR32_TC.channel[0].ccr;
}

static int counts = 0;
ISR(TIMER_IRQ, 14, 0) {
	AVR32_TC.channel[0].sr;
	now(*alarmEvent);
	counts++;
}

static void init_32khz() {
	AVR32_PM.oscctrl32 =
		AVR32_PM_OSCCTRL32_MODE_EXT_CLOCK |
		AVR32_PM_OSCCTRL32_OSC32EN_MASK;
}

static void rtc_init_ext32() {
	volatile avr32_rtc_t *rtc = &AVR32_RTC;

	rtc->ctrl =
	 	RTC_OSC_32KHZ << AVR32_RTC_CTRL_CLK32_OFFSET |
		(5-1) << AVR32_RTC_CTRL_PSEL_OFFSET |
		AVR32_RTC_CTRL_CLKEN_MASK;

	while (rtc_is_busy(rtc));

	rtc_set_value(rtc, 0x00000000);
	rtc_set_top_value(rtc, 0xFFFFFFFF);
	rtc_enable(rtc);
}

static void init_tc() {
	AVR32_TC.channel[0].cmr =
		AVR32_TC_WAVE_MASK |
		AVR32_TC_WAVSEL_UP_AUTO << AVR32_TC_WAVSEL_OFFSET |
		AVR32_TC_CPCSTOP_MASK |
		AVR32_TC_TIMER_CLOCK1;
	AVR32_TC.channel[0].ier = AVR32_TC_IER0_CPCS_MASK;
	AVR32_TC.channel[0].ccr = AVR32_TC_CLKEN_MASK;
}

void show_timer() {
	print("CNT:");
	dotnb(4, 4, raw_time(), 16);
	print("  Timer:");
	dotnb(4, 4, AVR32_TC.channel[1].cv, 16);
	print("  ticks/S:");
	printDec(ONE_SECOND);
}

void get_clock_frequency() {
	lit(sysclk_get_main_hz());
}

void init_clocks() {
	never(alarmEvent);
	init_32khz();
	rtc_init_ext32();
	init_tc();

	INTC_register_interrupt(&TIMER_IRQ, AVR32_TC_IRQ0, AVR32_INTC_INT0);
}
