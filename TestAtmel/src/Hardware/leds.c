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

static void set_timer1(Long t) {
    Wr_bitfield(AVR32_TC.channel[1].rc, AVR32_TC_RC_MASK, t);
	AVR32_TC.channel[1].ccr = AVR32_TC_SWTRG_MASK | AVR32_TC_CLKEN_MASK;
	(void) AVR32_TC.channel[1].ccr;
}

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

static void toggle_red() {
	static bool on = false;
	on = on ? redOff() : redOn();
}

Event TimeoutEvent;

 ISR (TIMER1_OVF_vect, 14, 0)    // Timer1 ISR
 {
	now(*TimeoutEvent);
	AVR32_TC.channel[1].sr;
	set_timer1(32767);  // 1s  32KHz clock
//	set_timer1(62500);  // 1s  PBA@8MHz/128 is clock
 }

#define AVR32_TC_WAVSEL_UP_AUTO_MASK (AVR32_TC_WAVSEL_UP_AUTO << AVR32_TC_WAVSEL_OFFSET)

 void init_leds() {
	 when(TimeoutEvent, toggle_red);
	irq_initialize_vectors();
	AVR32_TC.channel[1].cmr = AVR32_TC_WAVE_MASK
						    | AVR32_TC_WAVSEL_UP_AUTO_MASK
							| AVR32_TC_CPCSTOP_MASK
							| AVR32_TC_TIMER_CLOCK1;
	AVR32_TC.channel[1].ier = AVR32_TC_IER1_CPCS_MASK;
	// TIMER_CLOCK1 is 32KHz crystal which is not running yet? try PBA/32
	set_timer1(100);

	INTC_init_interrupts();

	INTC_register_interrupt(&TIMER1_OVF_vect, AVR32_TC_IRQ1, AVR32_INTC_INT0);
 	Enable_global_interrupt();
 }