#include "gpio.h"
#include "em_usart.h"
#include "InitDevice.h"
#include "tea.h"
#include "link.h"
#include "sfp.h"
#include "printers.h"
#include "cli.h"
#include "byteq.h"
#include "revision.h"
#include "talkHandler.h"
#include "node.h"
#include "low_power.h"
#include "project_defs.h"

static BYTEQ(100, uartRxq);
static BYTEQ(500, uartTxq);
static Byte * tx_data;
static Long tx_length;

Event ConsoleTxDone, ConsoleRxChar;

static Long last_activity = 0;

static void note_activity() { last_activity = raw_time(); }

void USART0_RX_IRQHandler() {
	if (USART_StatusGet(CONSOLE_PORT) & USART_STATUS_RXDATAV) {
		pushbq(USART_RxDataGet(CONSOLE_PORT), uartRxq);
		now(*ConsoleRxChar);
		note_activity();
	}
}

void USART0_TX_IRQHandler() {
    // Tx related code
    if ((CONSOLE_PORT->IF & USART_IF_TXBL) && (CONSOLE_PORT->IEN & USART_IEN_TXBL)) {
    	if (qbq(uartTxq)) { // check queue first
    		CONSOLE_PORT->TXDATA = pullbq(uartTxq);
		} else if (tx_length) { // buffer second
			CONSOLE_PORT->TXDATA = *tx_data++;
			tx_length--;
		} else { // wait for transmission complete
			CONSOLE_PORT->IEN &= ~(USART_IEN_TXBL);
			CONSOLE_PORT->IFC = USART_IFC_TXC;
			CONSOLE_PORT->IEN |= USART_IEN_TXC;
		}
    }

    if ((CONSOLE_PORT->IF & USART_IF_TXC) && (CONSOLE_PORT->IEN & USART_IEN_TXC)) {
    	CONSOLE_PORT->IEN &= ~(USART_IEN_TXC);
    	CONSOLE_PORT->IFC = USART_IFC_TXC;
		now(*ConsoleTxDone);
    }
	note_activity();
}

// pin edge detection; used for console wakeup (C6)
#define RX_PIN_MASK (1<<PIN(CONSOLE_RX))

static void enable_even_irq() {
	GPIO_IntClear(RX_PIN_MASK);
	set_clock_needs(CONSOLE_LP, HF_OFF);
	GPIO->IEN = RX_PIN_MASK;
}

static bool pending_finish = false;

static void checking_console() {
	if (2 * ONE_SECOND < raw_time() - last_activity) {
		pending_finish = false;
		enable_even_irq();
	} else
		after(secs(1), checking_console);
}

void finished_console() {
	if (pending_finish)
		return;
	pending_finish = true;
	later(checking_console);
}

void GPIO_EVEN_IRQHandler(void) {
	GPIO->IEN = 0;
	set_clock_needs(CONSOLE_LP, HF_RC);
	finished_console();
	note_activity();
}

void console_enable_tx() {
    CONSOLE_PORT->IEN |= USART_IEN_TXBL;
	set_clock_needs(CONSOLE_LP, HF_RC);
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
	return CONSOLE_PORT->STATUS & USART_STATUS_TXIDLE; // note: not idle at startup
}

static void init_cli() {
	setPrompt("sr50: ");
	resetCli();
	print("SR50B Sensor V1  ("), print(REVISION_BRANCH), print(" "), print(REVISION_NUMBER), print(")");
	dotPrompt();
}

void init_console() {
	zerobq(uartRxq);
	zerobq(uartTxq);
	tx_length = 0;

	never(ConsoleRxChar);
	never(ConsoleTxDone);

	CONSOLE_PORT->IEN = USART_IEN_RXDATAV;
	NVIC_EnableIRQ(USART0_RX_IRQn);
	NVIC_EnableIRQ(USART0_TX_IRQn);
	GPIO_IntConfig(gpioPortC, CONSOLE_RX_PIN, false, true, true);
	initSfp();
	when(RxLine, cliMachine);
	later(console_run);
	namedAction(run_cli);
	namedAction(console_run);
	namedAction(cliMachine);
	namedAction(finished_console);
	init_cli();
	GPIO->IFS = RX_PIN_MASK;
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

void systemReset() {
	NVIC_SystemReset();
}
