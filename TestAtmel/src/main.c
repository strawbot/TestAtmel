/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include "pm.h"
#include "board.h"
#include "conf_clock.h"
#include <asf.h>

#define U1_IN_LED	AVR32_PIN_PA29		// uart1 led input indicator, init as gpio output, value 0 (OFF)
#define U0_IN_LED	AVR32_PIN_PA30		// uart0 led input indicator, init as gpio output, value 0 (OFF)

void greenOn() { gpio_set_gpio_pin(U0_IN_LED); }
void greenOff() { gpio_clr_gpio_pin(U0_IN_LED); }
void redOn() { gpio_set_gpio_pin(U1_IN_LED); }
void redOff() { gpio_clr_gpio_pin(U1_IN_LED); }
	
/*	static const gpio_map_t USART_GPIO_MAP =
	{
		{EXAMPLE_USART_RX_PIN, EXAMPLE_USART_RX_FUNCTION},
		{EXAMPLE_USART_TX_PIN, EXAMPLE_USART_TX_FUNCTION}
	};

	// USART options.
	static const usart_options_t USART_OPTIONS =
	{
		.baudrate     = 57600,
		.charlength   = 8,
		.paritytype   = USART_NO_PARITY,
		.stopbits     = USART_1_STOPBIT,
		.channelmode  = USART_NORMAL_CHMODE
	};

	// Assign GPIO to USART.
	gpio_enable_module(USART_GPIO_MAP,
	sizeof(USART_GPIO_MAP) / sizeof(USART_GPIO_MAP[0]));

	// Initialize USART in RS232 mode.
	usart_init_rs232(EXAMPLE_USART, &USART_OPTIONS, EXAMPLE_TARGET_PBACLK_FREQ_HZ);

	// Hello world!
	usart_write_line(EXAMPLE_USART, "Hello, this is the AVR UC3 MCU saying hello! (press enter)\r\n");

	// Press enter to continue.
	while (usart_get_echo_line(EXAMPLE_USART) == USART_FAILURE);  // Get and echo characters until end of line.

	usart_write_line(EXAMPLE_USART, "Goodbye.\r\n"); */

#define USART_SERIAL                     &AVR32_USART3

static usart_serial_options_t usart_options = {
	.baudrate = 115200,
	.charlength = 8,
	.paritytype = USART_NO_PARITY,
	.stopbits = USART_1_STOPBIT
};

void init_map_uart0(void) {
    // define UART0 pin mapping, RX and TX pins as function
    static const gpio_map_t USART_GPIO_MAP0 = {
        { AVR32_USART0_RXD_0_0_PIN, AVR32_USART0_RXD_0_0_FUNCTION },
        { AVR32_USART0_TXD_0_0_PIN, AVR32_USART0_TXD_0_0_FUNCTION },
        { AVR32_USART0_CLK_0_PIN, AVR32_USART0_CLK_0_FUNCTION }
    };

    gpio_enable_module(USART_GPIO_MAP0, sizeof(USART_GPIO_MAP0) / sizeof(USART_GPIO_MAP0[0]));
}
/*************************************************************************************
 */
void init_map_uart1(void) {
    // define UART1 pin mapping, RX and TX pins as function
    static const gpio_map_t USART_GPIO_MAP1 = {
        { AVR32_USART1_RXD_0_0_PIN, AVR32_USART1_RXD_0_0_FUNCTION },
        { AVR32_USART1_TXD_0_0_PIN, AVR32_USART1_TXD_0_0_FUNCTION },
        { AVR32_USART1_CLK_0_PIN, AVR32_USART1_CLK_0_FUNCTION }
    };

    gpio_enable_module(USART_GPIO_MAP1, sizeof(USART_GPIO_MAP1) / sizeof(USART_GPIO_MAP1[0]));
}
/*************************************************************************************
 * GPS UART is UART2
 */
void init_map_uart2(void) {
    // define UART2(GPS) pin mapping, RX and TX pins as function
    static const gpio_map_t USART_GPIO_MAP2 = {
        { AVR32_USART2_RXD_0_0_PIN, AVR32_USART2_RXD_0_0_FUNCTION },
        { AVR32_USART2_TXD_0_0_PIN, AVR32_USART2_TXD_0_0_FUNCTION },
        { AVR32_USART2_CLK_0_PIN, AVR32_USART2_CLK_0_FUNCTION }
    };

    gpio_enable_module(USART_GPIO_MAP2, sizeof(USART_GPIO_MAP2) / sizeof(USART_GPIO_MAP2[0]));
}
/*************************************************************************************
 */
void init_map_uart3(void) {
    // define UART3 pin mapping, RX and TX pins as function
    static const gpio_map_t USART_GPIO_MAP3 = {
        { AVR32_USART3_RXD_0_0_PIN, AVR32_USART3_RXD_0_0_FUNCTION },
        { AVR32_USART3_TXD_0_0_PIN, AVR32_USART3_TXD_0_0_FUNCTION },
        { AVR32_USART3_CLK_0_PIN, AVR32_USART3_CLK_0_FUNCTION }
    };

    gpio_enable_module(USART_GPIO_MAP3, sizeof(USART_GPIO_MAP3) / sizeof(USART_GPIO_MAP3[0]));
}

static void al200Board_init() {
	pm_switch_to_osc0(pm, FOSC0, OSC0_STARTUP);
	pm_pll_disable(pm, 0);
	pm_cksel(pm,
	0, // disable the PBA (slow speed peripherals) divider
	0, // no divide
	0, // disable the PBB (USB, Ethernet, external RAM) divider
	0, // no divide
	0, // don't enable the CPU clock HSB divider
	0); // no division

	// Set the wait-state (WS) for flash controller. 0 WS access is up to 30MHz for HSB/CPU clock.
	// We have <30 MHz on HSB/CPU clock, we need to set 0 WS on flash controller.
	flashc_set_wait_state(0);

	gpio_clr_gpio_pin(XMTRON_PIN); // init transceiver off
    gpio_clr_gpio_pin(PTTON_PIN); // init PTT off
    gpio_clr_gpio_pin(FLTR_SDN); // init Opamps in filter off

    gpio_clr_gpio_pin(CSI_SW_12V_EN); //CSI SDI 12V enable off
    gpio_clr_gpio_pin(CSI_ANA_5V_EN); //CSI analog 5.2V supply enable off

    //CPU_LED pin on G2010 is ~GAIN_CS pin on A200; must change; changed on Rev 1 PCB
    gpio_set_gpio_pin(CPU_LED); // init CPU_LED as gpio output with led off;
        // later becomes GCLOCK "CPU Clock" function
    //Need to change as GPS pwr function is a toggle, not level
    gpio_clr_gpio_pin(GPS_ON); // GPS power and LED off
    gpio_clr_gpio_pin(GPS_LK_LED); // GPS Lock LED off
    gpio_enable_gpio_pin(CSI_GPS_WU); //GPS wakeup feedback
    gpio_clr_gpio_pin(U0_IN_LED); // uart0 input LED off
    gpio_clr_gpio_pin(U1_IN_LED); // uart1 input LED off

    //CSI Radio RF level pot
    gpio_clr_gpio_pin(CSI_RF_CS_N); //RF level pot initially powered down
    gpio_clr_gpio_pin(CSI_RF_GAIN);

    //CSI ADC Mux Selects
    gpio_clr_gpio_pin(CSI_SEL_CHA); //Analog supply powered down; init as low
    gpio_clr_gpio_pin(CSI_SEL_CHB);
    gpio_clr_gpio_pin(CSI_SEL_CHC);

    gpio_clr_gpio_pin(CSI_4_20MA_EN); // 4 to 20 mA resistor; HI to switch in
    gpio_clr_gpio_pin(CSI_SDI12_EN_TX); //SDI12 enable TX driver
    gpio_enable_gpio_pin(CSI_SWI_IN); //switch closure input
    gpio_enable_gpio_pin(CSI_BUTTON_IN); //button input

    //CSI Wakeups (external interrupts)
    // gpio_enable_gpio_pin(CSI_INT_XTIMER); //RTC wakeup
    // gpio_enable_gpio_pin(CSI_INT_U1_RX); //UART 1 RX wakeup
    // gpio_enable_gpio_pin(CSI_INT_U1_IVLD); //UART 1 INVALID wakeup

    //CSI 9pin I/O
    gpio_clr_gpio_pin(CSI_DL_TXD_EN); //CSI/O DL TXD enable (active HI)
    gpio_enable_gpio_pin(CSI_DL_SDE_N); //CSI/O SDE input (active LO)
    gpio_enable_gpio_pin(CSI_DL_SDC_RST); //CSI/O SDC reset input (active HI)
    gpio_clr_gpio_pin(CSI_DL_RING_EN); ////CSI/O RING output (active HI)

    // UART1 lines
    gpio_enable_gpio_pin(U1_RX); // RX as input, later a module pin
    gpio_clr_gpio_pin(U1_TX); // TX as output, value 0, later a module pin
    gpio_enable_pin_pull_up(U1_TX); // *** erata claims this is needed if UART module is turned off***

    gpio_clr_gpio_pin(U1_SDN_N); // RS232 driver shutdown as output, value 0 (shutdown)
    gpio_clr_gpio_pin(U1_RTS); // RTS as output, value 0
    gpio_enable_gpio_pin(U1_CTS); // CTS as input
    gpio_enable_gpio_pin(U1_INVLD_N); // driver invalid output as input
    gpio_enable_gpio_pin(U1_EXTCLK);

    // UART0 lines
    gpio_enable_gpio_pin(U0_RX); // RX as input
    gpio_clr_gpio_pin(U0_TX); // TX as output, value 0
    gpio_enable_pin_pull_up(U0_TX); // *** erata claims this is needed if UART module is turned off***
    //U0_SDN_N pin is NC on A200
    //	gpio_clr_gpio_pin(U0_SDN_N);	// RS232 driver shutdown as output, value 0 (shutdown)
    //U0_RTS pin is NC on A200
    //	gpio_clr_gpio_pin(U0_RTS);		// RTS as output, value 0
    //U0_CTS pin is NC on A200
    //	gpio_enable_gpio_pin(U0_CTS);	// CTS as input
    //U0_INVLD_N pin is NC on A200; should be USB_VBOF anyways
    //	gpio_enable_gpio_pin(U0_INVLD_N);	// RS232 driver invalid output as input
    //~BUF_DL_CLK on A200 (still an input)
    gpio_enable_gpio_pin(U0_EXTCLK);

    // UART3 lines
    gpio_enable_gpio_pin(U3_RX); // RX as input, later a module pin
    gpio_clr_gpio_pin(U3_TX); // TX as output, value 0, later a module pin
    gpio_enable_pin_pull_up(U3_TX); // *** erata claims this is needed if UART module is turned off***
    gpio_enable_gpio_pin(U3_EXTCLK); // fixed a copy paste error from original code

    // GPS & GPS UART2
    gpio_enable_gpio_pin(PPS_IN); // PPS_IN as input (100K pull down on board)
    // UART2 lines  (GPS)
    gpio_enable_gpio_pin(U2_RX); // RX as input, later a module input
    gpio_clr_gpio_pin(U2_TX); // TX an output value 0, later a module output
    gpio_enable_pin_pull_up(U2_TX); // *** erata claims this is needed if UART module is turned off***
    gpio_enable_gpio_pin(U2_EXTCLK);

    // setup the GCLK line that will become the external UART Clock
    gpio_clr_gpio_pin(EXT_UCLK);

    // USB_IDB optional
    gpio_enable_gpio_pin(USB_IDB); // USB ID output as input, with pull up
    gpio_enable_pin_pull_up(USB_IDB);

    //init SPI0 I/O lines used for the DAC
    gpio_set_gpio_pin(DAC_SPI_CS0); // CS as gpio output, value 1
    gpio_clr_gpio_pin(DAC_SPI_MOSI); // MasterOut as gpio output, value 0
    gpio_clr_gpio_pin(DAC_SPI_SCK); // CLK as gpio output, value 0

    //init SPI1 I/O, optionally used for Quickfilter chip/module
    //SPI_CS0 used for ADC on AL200; want inited to 1, as ADC power is always ON
    gpio_set_gpio_pin(MEM_SPI_CS0); // gpio output at 1 for CS to ADC
    //this pin is actually SPI_SCK
    gpio_enable_gpio_pin(RTCMEM_SPI_MISO); // gpio input 100K pull down on board
    gpio_clr_gpio_pin(RTCMEM_SPI_MOSI); // gpio output, value 0
    //this pin is actually SPI_MISO
    gpio_clr_gpio_pin(RTCMEM_SPI_SCK); // gpio output, value 0
    gpio_set_gpio_pin(RTC_SPI_CS1); // gpio output, value 1

    // Dallas RTC chip I/O
    gpio_enable_gpio_pin(X32K_XIN32); // RTC out goes to external 32768 kHz input
    gpio_enable_gpio_pin(XTIMER_IN); // XTIMER is alarm line from RTC

    gpio_clr_gpio_pin(PA19_1); // unused pin with test point, set as output value 0
    gpio_enable_gpio_pin(BOOT_DFU_ISU); // boot setup DIP switch, set as input (100K pull down on board)
    //GAIN_UP on A200
    gpio_clr_gpio_pin(USB_VBOF); // unused pin with test point, set as output value 0
    gpio_clr_gpio_pin(R_CH_SEL); // channel select driver, output value 0 to turn transistor off,
        //     floating the Radio Channel Select output.

    // Unconnected I/O
    // Recent spec sheet stated lowest power is
    // to leave unused pins as inputs(as defined by reset)
    // but enable the pull ups.
    // PA all pins used, no unconnected pins
    /*
	// PB... 6 pins
//SDI12_EN_TX on A200
	gpio_enable_pin_pull_up(AVR32_PIN_PB02);
//EN_4_20MA on A200
	gpio_enable_pin_pull_up(AVR32_PIN_PB03);
//SEL_CHC on A200
	gpio_enable_pin_pull_up(AVR32_PIN_PB13);
//TC_CLK2 on A200
	gpio_enable_pin_pull_up(AVR32_PIN_PB14);
//SW_12V_EN on A200
	gpio_enable_pin_pull_up(AVR32_PIN_PB20);		// UART Clock
//EN_5.2V on A200
	gpio_enable_pin_pull_up(AVR32_PIN_PB24);
*/

    //CSI unconnected I/O
    //	gpio_enable_pin_pull_up(AVR32_PIN_PA19);	//test point on G2010; driven LO; NC on A200
    gpio_clr_gpio_pin(U0_SDN_N); // RS232 driver shutdown as output, value 0 (shutdown); not used on CSI (PA26)
    gpio_clr_gpio_pin(U3_SDN_N); // RS232 driver shutdown as output, value 0 (shutdown); not used on CSI (PA28)
    gpio_clr_gpio_pin(U0_RTS); // RTS as output, value 0; not used on CSI (PA03)
    gpio_clr_gpio_pin(U3_AON_N); // RS232 driver force on not, value 1 => shutdown line controls on/off; not used on CSI (PB22)

    gpio_enable_pin_pull_up(AVR32_PIN_PB17); //U0_INVLD_N on G2010
    gpio_enable_pin_pull_up(AVR32_PIN_PB23); //U3_INVLD_N on G2010
    gpio_enable_pin_pull_up(AVR32_PIN_PB06); //was DL_SDC_RST

    //	gpio_enable_pin_pull_up(AVR32_PIN_PA04);
    gpio_enable_gpio_pin(AVR32_PIN_PA04); //tied to GND

    // PC... 2 pins
    //	gpio_enable_pin_pull_up(AVR32_PIN_PC01);	// used for CSI_DL_RING_EN on Rev1
    //	gpio_enable_pin_pull_up(AVR32_PIN_PC04);	//used for CSI_RF_CS_N on Rev1

    // reset and then MAP the module pins (UARTS)
    usart_reset(&USART0);
    init_map_uart0(); // uart0
    usart_reset(&USART1);
    init_map_uart1(); // uart1
    usart_reset(&USART2);
    init_map_uart2(); // gps uart (uart2)
    usart_reset(&USART3);
    init_map_uart3(); // uart3 (console)

    //reset, disable and MAP the module pins (SPI)
    RTCMEM_SPI->cr = AVR32_SPI_CR_SWRST_MASK;
    ; // software reset on SPI
    RTCMEM_SPI->cr = AVR32_SPI_CR_SPIDIS_MASK;
    //init_map_RTCMEM_spi(); // RTC (and ADC (uses G2010 MEM CS0))

    DAC_SPI->cr = AVR32_SPI_CR_SWRST_MASK; // software reset on SPI
    DAC_SPI->cr = AVR32_SPI_CR_SPIDIS_MASK;
    //init_map_DAC_spi(); // DAC for transmitter output
    /**********************************************************************
* Function: init_map_DAC_spi()
* Description: map the module pins for the RCT/EEPROM SPI 
* **********************************************************************/
    // setup SPI for DAC
    static const gpio_map_t DAC_SPI_GPIO_MAP = {
        { DAC_SPI_SCK_PIN, DAC_SPI_SCK_FUNCTION }, // SPI clock
        //		{DAC_SPI_MISO_PIN,	DAC_SPI_MISO_FUNCTION},	// SPI MI SO; Rev 01 board uses this pin for DL_SDC_RST
        { DAC_SPI_NPCS_PIN, DAC_SPI_NPCS_FUNCTION }, // SPI chip select
        { DAC_SPI_MOSI_PIN, DAC_SPI_MOSI_FUNCTION } // SPI MO SI
    };
    // assign I/Os to DAC_SPI
    gpio_enable_module(DAC_SPI_GPIO_MAP, sizeof(DAC_SPI_GPIO_MAP) / sizeof(DAC_SPI_GPIO_MAP[0]));

    if (gpio_get_pin_value(CSI_GPS_WU)) { //GPS wakeup output; HI when powered; LO when sleeping
        gpio_set_gpio_pin(GPS_ON);
        delay_ms(100);
        gpio_clr_gpio_pin(GPS_ON); // GPS power ON
        delay_ms(500); //wait for GPS to be ready to accept input
    }

    //init CSI Specific variables
    //CSIO_state = CSIO_rst;

}

/*************************************************************************************
 * setup UART3 for gclk input
 * leave at defaults:
 * SYNC/CPHA = 0 - asynchronous mode
 * MODE9 = 0 - not in 9 bit mode
 * CLKO = 0 - don't put clock on SCK pin
 * OVER = 1 for 8x oversampling for 800 kHz clock
 * INACK, DSNACK, VAR_SYNC, MAX_ITERATION, 
 * FILTER, MAN, MODSYNC, ONEBIT = default (0)
 */
#define GCLK_UART_CD_57_6	1
#define GCLK_UART_FP_57_6	6

void init_start_gclk_uart3(void) {
    // stop, reset all flags, disable ints
    //usart_reset(&USART3);
    // for 800 kHz external clock, set the oversample rate to 8 instead of 16
    (&USART3)->mr |= AVR32_USART_OVER_X8 << AVR32_USART_OVER_OFFSET;
// set the baud rate whole and fractional divisor for the input gclk
    (&USART3)->brgr = (U16)(GCLK_UART_CD_57_6) << AVR32_USART_BRGR_CD_OFFSET | (U16)(GCLK_UART_FP_57_6) << AVR32_USART_BRGR_FP_OFFSET;
    // set clock to external SCK pin
    (&USART3)->mr &= ~(AVR32_USART_USCLKS_SCK << AVR32_USART_USCLKS_OFFSET);
    // char len 8
    (&USART3)->mr |= (8 - 5) << AVR32_USART_MR_CHRL_OFFSET;
    // no parity
    (&USART3)->mr |= USART_NO_PARITY << AVR32_USART_MR_PAR_OFFSET;
    // set normal channel mode (not local echo, etc)
    (&USART3)->mr |= 0 << AVR32_USART_MR_CHMODE_OFFSET;
    // set 1 stop bit
    (&USART3)->mr |= USART_1_STOPBIT << AVR32_USART_MR_NBSTOP_OFFSET;
    // set normal usart mode (not RS485, IrDa, etc.)
    (&USART3)->mr = ((&USART3)->mr & ~AVR32_USART_MR_MODE_MASK) | AVR32_USART_MR_MODE_NORMAL << AVR32_USART_MR_MODE_OFFSET;
    // Setup complete; enable communication.
    // Enable input and output.
    (&USART3)->cr = AVR32_USART_CR_RXEN_MASK | AVR32_USART_CR_TXEN_MASK;
}

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	//sysclk_init();
	al200Board_init();
	//usart_init_rs232(USART_SERIAL, &usart_options, PBA_CLOCK);
//init_start_gclk_uart3();
usart_serial_init(USART_SERIAL, &usart_options);
	//init_start_gclk_uart3();
	while(1) {
redOn();
	const char * banner = "Good Mornin!";
	while (*banner) usart_serial_putchar(USART_SERIAL, *banner++);
redOff();
		 // uart1 input LED on
		greenOn(); // uart1 input LED off
		delay_ms(10);
		greenOff();
		delay_ms(190);
		greenOn();
		delay_ms(10);
		greenOff();
		delay_ms(790);
	}
	/* Insert application code here, after the board has been initialized. */
}
