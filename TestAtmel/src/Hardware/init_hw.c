#include "pm.h"
#include "board.h"
#include "conf_clock.h"
#include "init_hw.h"
#include <asf.h>
#include "project_defs.h"

void system_failure() { safe( redOn(); while (true); ) } // DEBUGGING

void init_leds();

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

    // reset and then MAP the module pins (UARTS)
    usart_reset(&AVR32_USART0);
    init_map_uart0(); // uart0
    usart_reset(&AVR32_USART1);
    init_map_uart1(); // uart1
    usart_reset(&AVR32_USART2);
    init_map_uart2(); // gps uart (uart2)

    //reset, disable and MAP the module pins (SPI)
    RTCMEM_SPI->cr = AVR32_SPI_CR_SWRST_MASK;
    // software reset on SPI
    RTCMEM_SPI->cr = AVR32_SPI_CR_SPIDIS_MASK;
    //init_map_RTCMEM_spi(); // RTC (and ADC (uses G2010 MEM CS0))

    DAC_SPI->cr = AVR32_SPI_CR_SWRST_MASK; // software reset on SPI
    DAC_SPI->cr = AVR32_SPI_CR_SPIDIS_MASK;
    // setup SPI for DAC
    static const gpio_map_t DAC_SPI_GPIO_MAP = {
        { DAC_SPI_SCK_PIN, DAC_SPI_SCK_FUNCTION }, // SPI clock
        //		{DAC_SPI_MISO_PIN,	DAC_SPI_MISO_FUNCTION},	// SPI MI SO; Rev 01 board uses this pin for DL_SDC_RST
        { DAC_SPI_NPCS_PIN, DAC_SPI_NPCS_FUNCTION }, // SPI chip select
        { DAC_SPI_MOSI_PIN, DAC_SPI_MOSI_FUNCTION } // SPI MO SI
    };
    // assign I/Os to DAC_SPI
    gpio_enable_module(DAC_SPI_GPIO_MAP, sizeof(DAC_SPI_GPIO_MAP) / sizeof(DAC_SPI_GPIO_MAP[0]));

    // if (gpio_get_pin_value(CSI_GPS_WU)) { //GPS wakeup output; HI when powered; LO when sleeping
        gpio_set_gpio_pin(GPS_ON);
    //     delay_ms(100);
    //     gpio_clr_gpio_pin(GPS_ON); // GPS power ON
    //     delay_ms(500); //wait for GPS to be ready to accept input
    // }
}

void init_hw() {
    al200Board_init();
    irq_initialize_vectors();
    INTC_init_interrupts();
}
