/**
 * \file
 *
 * \brief User board definition template
 *
 */

 /* This file is intended to contain definitions and configuration details for
 * features and devices that are available on the board, e.g., frequency and
 * startup time for an external crystal, external memory devices, LED and USART
 * pins.
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#ifndef USER_BOARD_H
#define USER_BOARD_H

//Radio TX level programmable pot
#define CSI_RF_CS_N		AVR32_PIN_PC04  // programmable pot chip select, init as gpio output, value 0 (OFF); pot power normally OFF
#define CSI_RF_GAIN		AVR32_PIN_PB18  // programmable pot gain up, init as gpio output, value 0 (OFF); pot power normally OFF; USB_VBOF on G2010

//CSI ADC Mux Selects
#define CSI_SEL_CHA		AVR32_PIN_PB08
#define CSI_SEL_CHB		AVR32_PIN_PB09
#define CSI_SEL_CHC		AVR32_PIN_PB13
#define MUX_VIN	0			//VIN on CH0
#define MUX_4_20 1		// 4 to 20 mA on CH1
#define MUX_REF 2			// VREF on CH2
#define MUX_GND 3			//GND on CH3
#define MUX_AUDIO 4		//Audio level on CH4
#define MUX_VBAT 5		//battery voltage on CH5
#define MUX_OFF 6			//mux selects LOW
#define CSI_4_20MA_EN		AVR32_PIN_PB03	// 4 to 20 mA resistor; HI to switch in
#define CSI_SDI12_EN_TX	AVR32_PIN_PB02	//SDI12 enable TX driver
#define mV_batt_cal 307.993e-3F		//conversion factor from full scale of 5.123V  to mV (5123/65536), and battery voltage through divider (3.94)
#define mV_cal 78.171e-3F		//conversion factor from full scale of 5.123V  to mV (5123/65536)
#define mA_cal 78.171e-5F		//conversion factor from full scale of 20.574 mA  to mA ((5123/65536)/100); 100 ohm current sense resistor
#define NAN (__builtin_nanf(""))

//CSI Wakeups (external interrupts)
#define CSI_INT_XTIMER	AVR32_PIN_PA20	//RTC wakeup
#define CSI_INT_U1_RX		AVR32_PIN_PA22	//UART 1 RX wakeup
#define CSI_INT_U1_IVLD	AVR32_PIN_PA23	//UART 1 INVALID wakeup

//CSI 9pin I/O
#define CSI_DL_TXD_EN		AVR32_PIN_PB07	//CSI/O DL TXD enable (active HI)
#define CSI_DL_SDE_N		AVR32_PIN_PA21	//CSI/O SDE input (active LO)
#define CSI_DL_SDC_RST	AVR32_PIN_PA11	//CSI/O SDC reset input (active HI)
#define CSI_DL_RING_EN	AVR32_PIN_PC01	//CSI/O RING output (active HI)
#define DRIVE_RING      0x10
#define SDC_OFF             0x00
#define SDC_ON              0x01


//CSIO ISR State definitions
#define CSIO_rst	0											//CSIO port reset (CLK & SDE LO @ 9p)
#define CSIO_sde	1											//SDE rising edge (@9p) detected
#define CSIO_adr	2											//address received & matches AL200 ADC address;
//first byte of SDC interchange sent; free buffer space in terms of 16 byte blocks; limit of 0xfe blocks
#define CSIO_byte2_txd	3								//second byte of SDC interchange; RxBodyLen set to # bytes in buffer (0xff max)
#define CSIO_xtra1_txd	4								//extra byte sent out here for CR5000
#define CSIO_block_txd	5								//RxBodyLen bytes are read from RS232 buffer, till RxBodyLen=0
#define CSIO_xtra2_txd	6								//second extra byte sent out here for CR5000
#define CSIO_turnaround_txd	7						//last byte of valid transmit data has been SHIFTED OUT of SPI TX reg
#define CSIO_block_rxd	8								//stuffs incoming data into buffer


#define CSI_SWI_IN			AVR32_PIN_PB14	//switch closure input
#define CSI_BUTTON_IN		AVR32_PIN_PA24	//button input

//CSI power control
#define CSI_SW_12V_EN		AVR32_PIN_PB20			//CSI SDI 12V enable
#define CSI_ANA_5V_EN		AVR32_PIN_PB24			//CSI analog 5.2V supply enable
#define CSI_GPS_WU			AVR32_PIN_PB05			//GPS WU signal (HI when GPS powered)


#define ADC_SLP 0x9000			//EN1, EN2, SPD, SLP in top nibble (sleep has VRef off)
#define ADC_NAP 0x8000			//EN1, EN2, SPD, SLP
#define SLP 0								//for ADC control
#define NAP 1
#define ADC_CS_PIN		0x00000000		// add to transmit data to select LTC ADC
#define LAST_TFER_FLAG	0x01000000		// add to last transmit data to deselect ADC
// when using CSSAAT=1 flag (allowing multiple reads)

//External switch closure (Rain Gage) counter
#define SWITCH_TC_CHANNEL		2

//CSIO buffers
#define CSIO_TX_COM_CBUF_SIZE	0x200			//data going out (TXed) on CSIO
#define CSIO_RX_COM_CBUF_SIZE	0x400			//data being received (RXed) on CSIO
#define CSIO_TX_CBUF_MASK	(CSIO_TX_COM_CBUF_SIZE - 1)
#define CSIO_RX_CBUF_MASK	(CSIO_RX_COM_CBUF_SIZE - 1)

//Sensor Input Defines
#define SensorInput 1
#define Disabled 0								//control port or SE1 disabled
#define Status 1									//control port status setting
#define SDI12 2										//SDI-12 port setting
#define mV 1											//SE1 mV mode
#define mA 2											//SE1 mA mode
#define TBR_Acc_Start 0x8003F000  // starting address in flash of TBR Accumulator page
#define TBR_Acc_End  0x8003F1FF  // ending address in flash of TBR Accumulator page

//Sensor Measurement State Machine Defines
#define SDI_time 0
#define SW12_time 1
#define SDI_meas 2
#define SE1_time 3

#	ifndef __ALERT1__
#define CSI_VERSION_NUMBER "AL200.ALERT2.033"	//ALERT2
# else
#define CSI_VERSION_NUMBER "AL200.ALERT.015"	//ALERT1
# endif
//Use AL200.Std.01.00 for released code; only put 17 byte stuff for the 17 byte version

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

#define BOOT_DFU_ISU	AVR32_PIN_PA25	// force boot pin, (possibly with external 100K pull up) init as gpio input with internal pull up
#define U0_SDN_N	AVR32_PIN_PA26		// init as gpio output, value 0 = RS232 driver shutdown (board has 100K pull down)
#define U1_SDN_N	AVR32_PIN_PA27		// init as gpio output, value 0 = RS232 driver shutdown (board has 100K pull down)
#define U3_SDN_N	AVR32_PIN_PA28		// init as gpio output, value 0 = RS232 driver shutdown (board has 100K pull down)

#define U1_IN_LED	AVR32_PIN_PA29		// uart1 led input indicator, init as gpio output, value 0 (OFF)
#define U0_IN_LED	AVR32_PIN_PA30		// uart0 led input indicator, init as gpio output, value 0 (OFF)

#define X32K_XIN32	AVR32_PIN_PC00		// init as module function 32KHz oscillator, external 32768 osc input...
#define X32K_XOUT32	AVR32_PIN_PC01  	// init as module function 32KHz oscillator, no connection
#define X12M_XIN0	AVR32_PIN_PC02  	// init as module function OSC0
#define X12M_XOUT0	AVR32_PIN_PC03  	// init as module function OSC0

// Pin PC04 is not connected, init it as gpio output, value 0

#define GPS_LK_LED	AVR32_PIN_PC05  	// GPS module lock indicator, init as gpio output, value 0 (OFF)

#define PPS_IN      AVR32_PIN_PB00  	// init as gpio input; there is a 100K pull down on the board
#define GPS_ON		AVR32_PIN_PB01  	// SkyTraq Venus 6 power regulator, active high (100K pull down on board), init gpio output, value 0 (OFF)
// W2SG0084i GPS "GPS Enable" line; pulse turns it on or turns it into hibernation

//Pins PB02 and PB03 are not connected, set them as gpio outputs, value 0

#define U3_EXTCLK	AVR32_PIN_PB04		// external clock for the UART, init input, no pull-up

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

#define GCLK_UART_CD_57_6	1
#define GCLK_UART_FP_57_6	6

#define CONSOLE  (&AVR32_USART3)

// External oscillator settings.
// Uncomment and set correct values if external oscillator is used.

// External oscillator frequency
//#define BOARD_XOSC_HZ          8000000

// External oscillator type.
//!< External clock signal
//#define BOARD_XOSC_TYPE        XOSC_TYPE_EXTERNAL
//!< 32.768 kHz resonator on TOSC
//#define BOARD_XOSC_TYPE        XOSC_TYPE_32KHZ
//!< 0.4 to 16 MHz resonator on XTALS
//#define BOARD_XOSC_TYPE        XOSC_TYPE_XTAL

// External oscillator startup time
//#define BOARD_XOSC_STARTUP_US  500000

#endif // USER_BOARD_H
