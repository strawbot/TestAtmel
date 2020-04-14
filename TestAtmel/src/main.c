#include "init_hw.h"
#include "tea.h"
#include "intc.h"

void init_leds();
void init_console();

void init_apps() {
	init_leds();
	init_console();
	Enable_global_interrupt();
}

int main (void) {
	init_hw();
	init_tea();
	init_apps();
	while(true) {
		run();
	}
}
