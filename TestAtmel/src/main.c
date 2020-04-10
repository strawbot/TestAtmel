#include "init_hw.h"
#include "tea.h"

int main (void) {
	init_hw();
	init_tea();
	while(1) {
		im_alive();
		console();
	}
}
