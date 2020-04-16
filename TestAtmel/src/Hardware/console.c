#include "pm.h"
#include <asf.h>

#include "gpio.h"
#include "tea.h"
#include "link.h"
#include "sfp.h"
#include "printers.h"
#include "cli.h"
#include "byteq.h"
#include "revision.h"
#include "talkHandler.h"
#include "node.h"
// #include "low_power.h"
#include "project_defs.h"
#include "user_board.h"

static BYTEQ(100, uartRxq);
static BYTEQ(500, uartTxq);
//static Byte * tx_data;
static Long tx_length;

Event ConsoleTxDone, ConsoleRxChar;

static Long last_activity = 0;

static void note_activity() { last_activity = getTime(); }

/*
TXRDY TXEMP txqQu State  action
1		1	1	transfer 1 byte
1		0	1	transfer 1 byte
0		0	1	wait for txrdy
1		0	0	wait for tx empty
0		0	0	wait for tx empty
1		1	0	disable interrupts; now(*event);

0		1	1	cant happen
0		1	0	cant happen

*/
ISR(CONSOLE_IRQHandler, 8, 1) {
	Long status = CONSOLE->csr;
	Long interrupts = status & CONSOLE->imr;

	if (status & AVR32_USART_CSR_RXRDY_MASK) { // character ready to read
		pushbq(CONSOLE->RHR.rxchr, uartRxq);
		CONSOLE->ier = AVR32_USART_CSR_TXRDY_MASK;
		now(*ConsoleRxChar);
//		record_event(FIRST_EVENT);
	}
	// The XXXX bit is cleared by writing the Control Register (CR) with the RSTSTA (Reset Status) bit at 1
	if (status & AVR32_USART_CSR_RXBRK_MASK)
	record_event("RXBRK"), CONSOLE->cr = AVR32_USART_CR_RSTSTA;
	if (status & AVR32_USART_CSR_PARE_MASK)
	record_event("PARE"), CONSOLE->cr = AVR32_USART_CR_RSTSTA;
	if (status & AVR32_USART_CSR_OVRE_MASK)
	record_event("OVRE"), CONSOLE->cr = AVR32_USART_CR_RSTSTA;
	if (status & AVR32_USART_CSR_FRAME_MASK)
	record_event("FRAME"), CONSOLE->cr = AVR32_USART_CR_RSTSTA;

	if (qbq(uartTxq)) {
		if (status & AVR32_USART_CSR_TXRDY_MASK)
			CONSOLE->THR.txchr = pullbq(uartTxq);
		// else if (0 == (interrupts & (AVR32_USART_CSR_TXRDY_MASK | AVR32_USART_CSR_TXEMPTY_MASK)))
		// 	CONSOLE->ier = AVR32_USART_CSR_TXRDY_MASK;
	} else {
		if (interrupts & AVR32_USART_CSR_TXRDY_MASK) {
			CONSOLE->idr = AVR32_USART_CSR_TXRDY_MASK;
			CONSOLE->ier = AVR32_USART_CSR_TXEMPTY_MASK;
		}

		if (interrupts & AVR32_USART_CSR_TXEMPTY_MASK) { // all sent and done
			CONSOLE->idr = AVR32_USART_CSR_TXEMPTY_MASK;
			now(*ConsoleTxDone);
		}
	}
	note_activity();
}



// 	if (USART_StatusGet(CONSOLE_PORT) & USART_STATUS_RXDATAV) {
// 		pushbq(USART_RxDataGet(CONSOLE_PORT), uartRxq);
// 		now(*ConsoleRxChar);
// 		note_activity();
// 	}


// ISR(CONSOLE_TX_IRQHandler, 8, 0) {
//     // Tx related code
//     if ((CONSOLE_PORT->IF & USART_IF_TXBL) && (CONSOLE_PORT->IEN & USART_IEN_TXBL)) {
//     	if (qbq(uartTxq)) { // check queue first
//     		CONSOLE_PORT->TXDATA = pullbq(uartTxq);
// 		} else if (tx_length) { // buffer second
// 			CONSOLE_PORT->TXDATA = *tx_data++;
// 			tx_length--;
// 		} else { // wait for transmission complete
// 			CONSOLE_PORT->IEN &= ~(USART_IEN_TXBL);
// 			CONSOLE_PORT->IFC = USART_IFC_TXC;
// 			CONSOLE_PORT->IEN |= USART_IEN_TXC;
// 		}
// }

//     if ((CONSOLE_PORT->IF & USART_IF_TXC) && (CONSOLE_PORT->IEN & USART_IEN_TXC)) {
//     	CONSOLE_PORT->IEN &= ~(USART_IEN_TXC);
//     	CONSOLE_PORT->IFC = USART_IFC_TXC;
// 		now(*ConsoleTxDone);
//     }
// 	note_activity();
// }

// pin edge detection; used for console wakeup (C6)
// #define RX_PIN_MASK (1<<PIN(CONSOLE_RX))

// static void enable_even_irq() {
// 	GPIO_IntClear(RX_PIN_MASK);
// 	set_clock_needs(CONSOLE_LP, HF_OFF);
// 	GPIO->IEN = RX_PIN_MASK;
// }

static bool pending_finish = false;

static void checking_console() {
	if (2 * ONE_SECOND < getTime() - last_activity) {
		pending_finish = false;
		// enable_even_irq();
	} else
		after(secs(1), checking_console);
}

void finished_console() {
	if (pending_finish)
		return;
	pending_finish = true;
	later(checking_console);
}

// void GPIO_EVEN_IRQHandler(void) {
// 	GPIO->IEN = 0;
// 	set_clock_needs(CONSOLE_LP, HF_RC);
// 	finished_console();
// 	note_activity();
// }

void console_enable_tx() {
	CONSOLE->ier = AVR32_USART_CSR_TXRDY_MASK;
// 	set_clock_needs(CONSOLE_LP, HF_RC);
	finished_console();
}

// tie in to CLI
static Long keys_in() { return qbq(keyq); }
static bool cli_pending = false;
static void cliMachine();

static void run_cli() {
	cli_pending = false;
	cli();
	later(cliMachine);
}

static void cliMachine() {
	if (!cli_pending && keys_in()) {
		later(run_cli);
	}
}

void console_sfp(sfpLink_t * link) {
    link->rxq = uartRxq;
    link->txq = uartTxq;
}

static bool console_done() {
	bool done = true;
	if (qbq(emitq))  { sendeqSfp(); done = false; }
	for (Byte i=0; i < NUM_LINKS; i++) {
		sfpLink_t *link = nodeLink(i);
		if (link && queryq(link->npsq)) {
			link->serviceTx(link);
			done = false;
		}
	}
	if (qbq(uartTxq)) { console_enable_tx(); done = false; }

	return done;
}

static void console_run() {
	if (console_done())
		after(secs(1), console_run);
	else
		later(console_run);
}

bool uart_idle() {
	return true; //CONSOLE_PORT->STATUS & USART_STATUS_TXIDLE; // note: not idle at startup
}

static void init_cli() {
	setPrompt("al200: ");
	resetCli();
	print("AL200 Encoder Alfa  ("), print(REVISION_BRANCH), print(" "), print(REVISION_NUMBER), print(")");
	dotPrompt();
}

// void systemReset() {
// 	NVIC_SystemReset();
// }

void init_map_uart3(void) {
    // define UART3 pin mapping, RX and TX pins as function
    static const gpio_map_t USART_GPIO_MAP3 = {
        { AVR32_USART3_RXD_0_0_PIN, AVR32_USART3_RXD_0_0_FUNCTION },
        { AVR32_USART3_TXD_0_0_PIN, AVR32_USART3_TXD_0_0_FUNCTION },
        { AVR32_USART3_CLK_0_PIN, AVR32_USART3_CLK_0_FUNCTION }
    };

    gpio_enable_module(USART_GPIO_MAP3, sizeof(USART_GPIO_MAP3) / sizeof(USART_GPIO_MAP3[0]));
}

static usart_serial_options_t usart_options = {
	.baudrate = 57600, // 115200,
	.charlength = 8,
	.paritytype = USART_NO_PARITY,
	.stopbits = USART_1_STOPBIT
};

void tx_enable() {
	CONSOLE->ier = AVR32_USART_CSR_TXRDY_MASK;
}

void init_console() {
	zerobq(uartRxq);
	zerobq(uartTxq);
	tx_length = 0;

	never(ConsoleRxChar);
	never(ConsoleTxDone);
	
	usart_reset(CONSOLE);
    init_map_uart3();
	usart_serial_init(CONSOLE, &usart_options);

	initSfp();
	when(RxLine, cliMachine);
	later(console_run);
	namedAction(run_cli);
	namedAction(console_run);
	namedAction(cliMachine);
	namedAction(finished_console);
	init_cli();
	CONSOLE->ier = AVR32_USART_CSR_RXRDY_MASK;

	INTC_register_interrupt(&CONSOLE_IRQHandler, AVR32_USART3_IRQ, AVR32_INTC_INT1);
}

