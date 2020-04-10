#pragma GCC optimize ("O2")

// Simple network: two nodes sharing a link  Robert Chapman III  Apr 7, 2015

#include "framePool.h"
#include "node.h"
#include "stats.h"
#include "sfpTxSm.h"
#include "sfpRxSm.h"
#include "services.h"
#include "byteq.h"
#include "talkhandler.h"
#include "tea.h"
#include "console.h"
#include "cli.h"
#include "project_defs.h"
#include "pm.h"
#include "board.h"
#include <asf.h>

void initTalkHandler(void);
extern Event ConsoleRxChar;

static sfpNode_t myNode;
static sfpLink_t uartLink;

static QUEUE(MAX_FRAMES, frameq);
static QUEUE(MAX_FRAMES, npsq);

// queue drivers
static bool rxAvailable(sfpLink_t * link)
{
    return (qbq(link->rxq) != 0);
}

static Byte rxGet(sfpLink_t * link)
{
    BytesIn(link);
    return pullbq(link->rxq);
}

void sfpStates() {
	for (Byte i=0; i < NUM_LINKS; i++) {
		sfpLink_t *link = nodeLink(i);
		if (link) {
            link->serviceTx(link);
            sfpTxSm(link);
			sfpRxSm(link);
		}
	}
}

void output() { // called from safeEmit when emitq is full
	sendeqSfp();
	for (Byte i=0; i < NUM_LINKS; i++) {
		sfpLink_t *link = nodeLink(i);
		if (link) {
            link->serviceTx(link);
            sfpTxSm(link);
		}
	}
}

// interrupt driven sfp
void frameOut(sfpFrame * frame) {
    (void)frame;
    // USART_IntEnable(CONSOLE_PORT, USART_IEN_TXBL);
}

static bool sfpTXStates() {
	bool worktodo = false;

	for (Byte i=0; i < NUM_LINKS; i++) {
		sfpLink_t *link = nodeLink(i);
		if (link) {
            link->serviceTx(link);

			if (checkBit(TX_WORK, link->txFlags) || queryq(link->npsq)) {
            	worktodo = true;
				sfpTxSm(link);
			}
		}
	}
	return worktodo;
}

static bool rxSmPending = false;

static void consoleRxSM() {
	sfpLink_t * link = &uartLink;

	while (link->sfpRx(link)) 
		sfpRxSm(link);

	rxSmPending = false;
}

static void scheduleRxSM() {
	if (!rxSmPending) {
        later(consoleRxSM);
        rxSmPending = true;
    }
}

static bool sfpBusy = false;

static void sfpStateAction() {
	if (sfpTXStates())
		later(sfpStateAction);
	else
		sfpBusy = false;
}

static void sfpStateMachine() {
	if (!sfpBusy) {
		sfpBusy = true;
		sfpStateAction();
	}
}

static void service_tx(sfpLink_t * link) {
	if (qbq(link->txq) || link->sfpBytesToTx) { 
		// if (uart_idle()) console_enable_tx();
	}
}

static bool emitting = false;

static void emitAction() {
	if (sendeqSfp())
		later(emitAction);
	else
		emitting = false;
}

static void emitMachine() {
	if (!emitting) {
		emitting = true;
		emitAction();
	}
}

void initSfp(void)
{
	namedAction(emitMachine);
	namedAction(emitAction);
	namedAction(sfpStateMachine);
	namedAction(processFrames);
	namedAction(scheduleRxSM);
	namedAction(consoleRxSM);
	namedAction(sfpStateAction);

	sfpLink_t *link;

	// initialize pool of frame buffers
    initFramePool();

	// initialize the node
    initNode(&myNode);
    setNode(&myNode);
	addLink(&uartLink);

    setRouteTo(DIRECT, &uartLink);
    setRouteTo(MAIN_HOST, &uartLink);

	// initialize UART link
	link = &uartLink;
	initLink(link, "UART Link");
	link->disableSps = true;
	// console_sfp(link);

	// initialize state machines
	initSfpRxSM(link, frameq);
	initSfpTxSM(link, npsq, NULL);
	link->serviceTx = service_tx;
	link->sfpRx = rxAvailable;
	link->sfpGet = rxGet;

	// initialize services and stats
    initSfpStats();
	initServices();
	initTalkHandler();

	when(RxFrame, processFrames);
	// when(ConsoleRxChar, scheduleRxSM);
	when(EmitEvent, emitMachine);
	when(NpsEvent, sfpStateMachine);
}
