/**
 * \file
 *
 * \brief User board definition template
 *
 * ALERT2 AVR32 Version3 Board Definition.
 *
 * Copyright 2008, 2009, 2011 Blue Water Design LLC.
 * rcroark@bluewaterdesign.us
 * 
 * May 2011 - fixed the mapping for:
 * 			#define RTCMEM_SPI_MISO	 AVR32_PIN_PA17
 * 			#define RTCMEM_SPI_SCK	AVR32_PIN_PA15
 * 			- removed the SW1 - SW4 pins, boards do not have SW1-SW4
 * Sept 2009 - Created for Version3 Hardware
 * 
 * THIS SOFTWARE IS PROVIDED BY BLUE WATER DESIGN LLC ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT ARE 
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED.  IN NO EVENT SHALL BLUE WATER 
 * DESIGN LLC BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 * The "ALERT2 AVR32 Version3 Board Definition" is distributed in the hope
 * it will be useful, but WITHOUT ANY WARRANTY. 
 */

#ifndef USER_BOARD_H
#define USER_BOARD_H

#define CSI			//include CSI specific code
#ifdef CSI
#include "CSI_A200.h"
#endif

/* This file is intended to contain definitions and configuration details for
 * features and devices that are available on the board, e.g., frequency and
 * startup time for an external crystal, external memory devices, LED and USART
 * pins.
 */

// I/O pin definitions
#define U0_RX		AVR32_PIN_PA00	// init as module pin
#define U0_TX		AVR32_PIN_PA01	// init as module pin
#define U0_EXTCLK	AVR32_PIN_PA02	// external clock for the UART, init input, no pull-up
#define U0_RTS		AVR32_PIN_PA03	// optional, init as gpio output, value 0
#define U0_CTS		AVR32_PIN_PA04	// optional, init as input

#define U1_RX		AVR32_PIN_PA05	// init as gpio input  
#define U1_TX		AVR32_PIN_PA06	// init as gpio output, value 0
#define U1_EXTCLK	AVR32_PIN_PA07	// external clock for the UART, init as input, no-pull up
#define U1_RTS		AVR32_PIN_PA08	// optional, init as gpio output, value 0
#define U1_CTS		AVR32_PIN_PA09	// optional, init as input

#define DAC_SPI						(&AVR32_SPI0)					// defined as below....
//#define DAC_SPI_IRQ				AVR32_SPI0_IRQ                    // 288
//#define DAC_SPI_SPI_CS_MSB		AVR32_SPI0_SPI_CS_MSB             // 3
#define DAC_SPI_CS0					AVR32_PIN_PA10	// init as module function for DAC
#define DAC_SPI_NPCS_PIN			AVR32_SPI0_NPCS_0_0_PIN             // 10
#define DAC_SPI_NPCS_FUNCTION		AVR32_SPI0_NPCS_0_0_FUNCTION        // 0
#define DAC_SPI_MISO				AVR32_PIN_PA11	// init as SPI0_MISO module funtion for DAC (USB_IDA is DNP on board)
#define DAC_SPI_MISO_PIN			AVR32_SPI0_MISO_0_0_PIN             // 11
#define DAC_SPI_MISO_FUNCTION		AVR32_SPI0_MISO_0_0_FUNCTION        // 0
#define DAC_SPI_MOSI				AVR32_PIN_PA12	// init as SPIO_MOSI function for DAC
#define DAC_SPI_MOSI_PIN			AVR32_SPI0_MOSI_0_0_PIN             // 12
#define DAC_SPI_MOSI_FUNCTION		AVR32_SPI0_MOSI_0_0_FUNCTION        // 0
#define DAC_SPI_SCK					AVR32_PIN_PA13	// init as SPIO_SCK function for DAC
#define DAC_SPI_SCK_PIN				AVR32_SPI0_SCK_0_0_PIN              // 13
#define DAC_SPI_SCK_FUNCTION		AVR32_SPI0_SCK_0_0_FUNCTION         // 0

#define RTCMEM_SPI					(&AVR32_SPI1)	// used for RTC chip and Quickfilter
#define MEM_SPI_CS0					AVR32_PIN_PA14
#define MEM_SPI_NPCS_PIN         	AVR32_SPI1_NPCS_0_0_PIN
#define MEM_SPI_NPCS_FUNCTION     	AVR32_SPI1_NPCS_0_0_FUNCTION
#define RTCMEM_SPI_MISO				AVR32_PIN_PA17
#define RTCMEM_SPI_MISO_PIN        	AVR32_SPI1_MISO_0_0_PIN
#define RTCMEM_SPI_MISO_FUNCTION    AVR32_SPI1_MISO_0_0_FUNCTION
#define RTCMEM_SPI_MOSI				AVR32_PIN_PA16
#define RTCMEM_SPI_MOSI_PIN        	AVR32_SPI1_MOSI_0_0_PIN
#define RTCMEM_SPI_MOSI_FUNCTION   	AVR32_SPI1_MOSI_0_0_FUNCTION 
#define RTCMEM_SPI_SCK				AVR32_PIN_PA15
#define RTCMEM_SPI_SCK_PIN         	AVR32_SPI1_SCK_0_0_PIN 
#define RTCMEM_SPI_SCK_FUNCTION    	AVR32_SPI1_SCK_0_0_FUNCTION
#define RTC_SPI_CS1					AVR32_PIN_PA18
#define RTC_SPI_NPCS_PIN            AVR32_SPI1_NPCS_1_0_PIN
#define RTC_SPI_NPCS_FUNCTION       AVR32_SPI1_NPCS_1_0_FUNCTION

#define PA19_1		AVR32_PIN_PA19		// spare I/O, init as output, value 0

//#define SWL0		AVR32_PIN_PA20  	// dip switch input line, intit as gpio input with internal pull up
//#define SWL1		AVR32_PIN_PA21		// dip switch input line, intit as gpio input with internal pull up
//#define SWL2		AVR32_PIN_PA22  	// dip switch input line, intit as gpio input with internal pull up
//#define SWL3		AVR32_PIN_PA23		// dip switch input line, intit as gpio input with internal pull up
//#define SWL4		AVR32_PIN_PA24  	// dip switch input line, intit as gpio input with internal pull up

#define BOOT_DFU_ISU	AVR32_PIN_PA25	// force boot pin, (possibly with external 100K pull up) init as gpio input with internal pull up
#define U0_SDN_N	AVR32_PIN_PA26		// init as gpio output, value 0 = RS232 driver shutdown (board has 100K pull down)
#define U1_SDN_N	AVR32_PIN_PA27		// init as gpio output, value 0 = RS232 driver shutdown (board has 100K pull down)
#define U3_SDN_N	AVR32_PIN_PA28		// init as gpio output, value 0 = RS232 driver shutdown (board has 100K pull down)

#define U1_IN_LED	AVR32_PIN_PA29		// uart1 led input indicator, init as gpio output, value 0 (OFF)
#define U0_IN_LED	AVR32_PIN_PA30		// uart0 led input indicator, init as gpio output, value 0 (OFF)

#define X32K_XIN32	AVR32_PIN_PC00		// init as module function 32KHz oscillator, external 32768 osc input...
#define X32K_XOUT32	AVR32_PIN_PC01  	// init as module function 32KHz oscillator, no connection
#define X12M_XIN0	AVR32_PIN_PC02  	// init as module funciton OSC0
#define X12M_XOUT0	AVR32_PIN_PC03  	// init as module function OSC0

// Pin PC04 is not connected, init it as gpio output, value 0

#define GPS_LK_LED	AVR32_PIN_PC05  	// GPS module lock indicator, init as gpio output, value 0 (OFF)

#define PPS_IN      AVR32_PIN_PB00  	// init as gpio input; there is a 100K pull down on the board
#define GPS_ON		AVR32_PIN_PB01  	// SkyTraq Venus 6 power regulator, active high (100K pull down on board), init gpio output, value 0 (OFF)
										// W2SG0084i GPS "GPS Enable" line; pulse turns it on or turns it into hibernation

//Pins PB02 and PB03 are not connected, set them as gpio outputs, value 0

#define U3_EXTCLK	AVR32_PIN_PB04		// external clock for the UART, init input, no pull-up

// SW1 - SW5 are not used, in the init.c routine the pins PB05 - PB08 are set to inputs with pul ups
//#define SW1			AVR32_PIN_PB05		// change to output with value 0 to drive SWx and read SWl0-SWL4
//#define SW2			AVR32_PIN_PB06		// change to output with value 0 to drive SWx and read SWl0-SWL4
//#define SW3			AVR32_PIN_PB07		// change to output with value 0 to drive SWx and read SWl0-SWL4
//#define SW4			AVR32_PIN_PB08		// change to output with value 0 to drive SWx and read SWl0-SWL4
//#define SW5			AVR32_PIN_PB09		// change to output with value 0 to drive SWx and read SWl0-SWL4

#define U3_RX		AVR32_PIN_PB10		// init input, then change to as module pin, Console UART
#define U3_TX		AVR32_PIN_PB11		// init as output, value 0, then change to module pin, Console UART

#define XTIMER_IN	AVR32_PIN_PB12	// RTC option, ALARM output interrupt line, set as gpio input 

// Pins PB13 and PB14 are unconnected, set them as gpio outputs, value 0

#define U1_INVLD_N	AVR32_PIN_PB15	// U1 RS232 driver output, init as gpio input, no pull up
#define USB_IDB		AVR32_PIN_PB16	// optional, init as gpio input, with pull up resistor
#define U0_INVLD_N	AVR32_PIN_PB17	// U0 RS232 driver output, init as gpio input, no pull up

#define USB_VBOF	AVR32_PIN_PB18	// test point only, init as output value 0

// output the CPU clock for diagnositics using a GCLK module function
#define CPU_LED		AVR32_PIN_PB19  // used as PM GCLOCK module function, init as gpio output, value 0 (OFF)
#define GCLK_ID		0
#define GCLK_PIN	AVR32_PM_GCLK_0_1_PIN			// AVR32 "pin 51", PB19, see uc3a1512.h header file
#define GCLK_FUNCTION	AVR32_PM_GCLK_0_1_FUNCTION

// Pin PB20 is unconnected, set as gpio output, value 0

// External UART cclock for inputs to UARTs 0, 1, 2, and 3
#define EXT_UCLK	AVR32_PIN_PB21  // used as PM GCLOCK module function, init as gpio output, value 1
#define EUCLK_ID	2
#define EUCLK_PIN	AVR32_PM_GCLK_2_0_PIN			// AVR32 "pin 53", PB21, see /uc3a1512.h header file
#define EUCLK_FUNCTION	AVR32_PM_GCLK_2_0_FUNCTION

#define U3_AON_N	AVR32_PIN_PB22	// Force On for UART3 driver chip (console), 100K pull down on board, init as output, value 1 (SJ1 open)?
#define GPS_AWAKE	AVR32_PIN_PB23	// PB23 is the signal from W2SG0084i that the GPS chip is active in BOARD_REV2
#define U3_INVLD_N	AVR32_PIN_PB23	// or U3 RS232 driver output, init as gpio input, no pull up in the original board layout

// Pin PB24 is unconnected, set as gpio output, value 0
 
#define R_CH_SEL	AVR32_PIN_PB25  // Radio Channel select output to Ritron, init as gpio output, value 0
#define FLTR_SDN	AVR32_PIN_PB26	// 0 is shutdown, init as gpio output, value 0
#define XMTRON_PIN  AVR32_PIN_PB27	// 1 is on, init as gpio output, value 0
#define PTTON_PIN   AVR32_PIN_PB28  // 1 is on, init as gpio output, value 0

#define U2_RX		AVR32_PIN_PB29	// GPS option, init as gpio input with pull up?, later as module pin
#define U2_TX		AVR32_PIN_PB30	// GPS option, init as gpio output, value 0, later as module pin
#define U2_EXTCLK	AVR32_PIN_PB31	// GPS option, init as gpio input, later as module pin (ext uart clock)

#define pm  &AVR32_PM
extern volatile avr32_tc_t * tc;
extern volatile avr32_rtc_t * rtc;
#define USART0 AVR32_USART0
#define USART1 AVR32_USART1
#define USART2 AVR32_USART2
#define USART3 AVR32_USART3

extern void init_start_gclk_uart3(void);
extern void init_start_ext_uart_clk(void);

#endif // USER_BOARD_H
