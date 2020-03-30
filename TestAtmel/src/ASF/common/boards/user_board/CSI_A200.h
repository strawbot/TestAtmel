/* Header File
 * Designed for A2 V3 hardware
 *
 * - Compiler:       GNU GCC for AVR32
 * - Supported devices:  Atmel AVR32
 *
 * - Author			Rick Spielmaker, Campbell Scientific, Inc.
 */

#include <compiler.h>

#ifndef CSI_A200_H
#define CSI_A200_H


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


typedef struct
{
	U16 Seq;
	S16 ID;
	F32 Mult;
	F32 Offset;
	F32 TX_Change;
} SDI12_t;

typedef struct
{
	U16 Seq;
	S16 ID;
	F32 Mult;
	F32 Offset;
	U16 TX_Change;
} SDI12_A1_t;

typedef struct
{
	U16 Value;
	F32 Raw;
	F32 Scaled;
	F32 TX_2;
	F32 TX_1;
	F32 TX_0;
} SDI12_DC_t;


/* IMPORTANT NOTE:  
The CRC for parameter loading is done on a bytewise basis starting
with the address of member Mode (endianes is not a problem).  BUT the GNU 
compiler aligns structures on 32 bit (word) boundries!  Therefore, there must be 
a exact multiple of words (U32) in the struct, an even number of U16 members between 
U32 members, and an even number of U8 members between U16 members and/or a multiple of
4 U8 members between U32 members.
*/ 

//! Structure type containing initialization variables to store in NVRAM
#	ifndef __ALERT1__
typedef struct
{
	U8 Mode;
	U8 Clock_Status;
	U8 Clock_Status_Sensor_ID;
	U8 MSR_Enable;
	U8 Spare2;
	U8 Spare3;
	U8 P1_Enable;
	U8 SE1_Mode;
	U8 SE1_ID;
	U8 C1_ID;
	U8 SDI_Value;
	U8 Control_Port_Mode;
	char SDI_Cmd[8];
	U16 S2;
	U16 DevCon_Int;
	U16 Mod_Voltage;
	U16 Sensor_Scan;
	U32 Self_Report_Interval;
	F32 SE1_Mult;
	F32 SE1_Off;
	F32 SE1_TX_change;
	SDI12_t SDI12Data[9];
	U8 SDC;
	S8 SW12_Warmup;
	U16 crc;
} init_CSI_t;
# endif  /*  ifndef __ALERT1__  */

# ifdef __ALERT1__
typedef struct
{
	U8 Mode;
	U8 Clock_Status;
	U8 Clock_Status_Sensor_ID;
	U8 MarkSpace_Tone;
	U8 WaterLog;
	U8 Spare1;
	U16 Spare2;
	U8 P1_Enable;
	U8 SE1_Mode;
	U16 SE1_ID;
	U16 C1_ID;
	U16 P1_ID;
	U16 BATT_ID;
	U8 SDI_Value;
	U8 Control_Port_Mode;
	char SDI_Cmd[8];
	U16 S2;
	U16 DevCon_Int;
	U16 Mod_Voltage;
	U16 Sensor_Scan;
	U32 Self_Report_Interval;
	F32 SE1_Mult;
	F32 SE1_Off;
	U16 SE1_TX_change;
	U16 Spare3;
	SDI12_t SDI12Data[9];
	U8 SDC;
	S8 SW12_Warmup;
	U16 crc;
} init_CSI_t;

typedef struct
{
	U16 AL1_addr;
	U16 AL1_data;
} AL1_t;

# endif  /*  ifdef __ALERT1__  */

typedef struct
{
	U8 Control;
	U8 Type;
	U16 Length;
	char Data[255];
} PDU_t;

typedef struct
{
	U16 Length;
	char Data[70];
} GSR_t;

typedef struct
{
	U8 Length;
	U8 Data_Flags;					//MSB is ID 8; LSB is ID 1
	S16 Air;								//same for English & Metric	ID = 1
	U8 Humidity;						//same for English & Metric	ID = 2
	U16 Pressure;						//same for English & Metric	ID = 3
	U16 Wind_Spd;						//U8 for English, U16 for Metric 	ID = 4
	U16 Wind_Dir;						//same for English & Metric	ID = 5
	U16 Wind_Peak;					//U8 for English, U16 for Metric 	ID = 6
	S16 Stage_E;						//S16 for English					  ID =7
	S32 Stage_M;						//S24 for Metric (3 bytes) 	ID =7
	U8 Battery;							//same for English & Metric	ID = 8
} MSR_t;

typedef struct
{
	U16 Length;
	U16 Accumulator;
	U16 Stored_Accum;
	U8 NumTips;
	char TimeOffsets[350];
} Tip_t;



//global variables

//console UART3 variables
//extern volatile U8 uart3_rx_cb[CON_COM_CBUF_SIZE];
//extern volatile U16 uart3_rx_wr_idx;	// Console circular buffer
//extern volatile U16 uart3_rx_rd_idx;
//extern volatile U16 uart3_rx_err;		// error code from uart3 rx interrupt handler


//flags
extern volatile U16 sde_active;		// flag for SDE state
extern const U8 bit_rev[256];
extern volatile U16 CSIO_state;
extern volatile U8 addr_byte;
extern volatile U16 RxBodyLen;
extern volatile U8 extra_byte;		//byte used in state machine during SPI RX/TX turnaround
extern volatile U8 SDC_address;	//SDC address in top nibble

//CSIO Buffers
extern volatile U8 CSIO_rx_cb[CSIO_RX_COM_CBUF_SIZE];			//data being received (RXed) on CSIO
extern volatile U8 CSIO_tx_cb[CSIO_TX_COM_CBUF_SIZE];			//data going out (TXed) on CSIO

//CSIO Buffer Indexes
extern volatile U16 CSIOrx_wr_idx;	//incremented on write (by Application) to CSIO_rx_cb
extern volatile U16 CSIOrx_rd_idx;	//incremented on read (by SPI ISR) from CSIO_rx_cb
extern volatile U16 CSIOtx_wr_idx;	//incremented on write (by SPI ISR) to CSIO_tx_cb
extern volatile U16 CSIOtx_rd_idx;	//incremented on read (by Application) from CSIO_tx_cb

//CSI Stored parameters
extern init_CSI_t gCSIRamCpy;
extern const init_CSI_t CSI_flash_data;		// flash copy

//RF out digital pot setting
extern int digital_pot_cnt;

//GPS fix time string
extern char GPS_last_fix_str[32];
extern char GPS_date_str[12];
//timers for sensor scanning
extern U32 SelfReportTime;						// # TDMA frames till self report
extern U32 cpySelfReportTime;					//copy of above to be decremented
extern U32 TXFrame_Interval;
extern U16 cpyDevCon_Interval;
extern S16 SDI12_scan_time;
extern U8 sensor_scan_time;
extern U32 NextScanTime;							// time in half seconds till the next sensor scan
extern volatile bool NextScanStart_flag;
extern volatile bool SelfReportStart_flag;
extern volatile bool SensorScanStart_flag;
extern volatile bool DevConStart_flag;
extern bool			gGpsNoTime_flag;					// NEVER completed a full GPS - UTC time set
extern bool			gGpsHighDrift_flag;				// exceeded the needed time between GPS updates

extern U16 CSI_gpsState;

extern U32 TotalTips;

//DevConfig variables
F32 SE1_rdgs [3];
F32 C1_rdgs[3];
U16 P1_rdgs[3];
F32 DevCon_Batt, DevCon_SE1, DevCon_SDI;
U16 DevCon_P1;
U8 DevCon_C1;
extern U8 DevCon_Active;

extern U16 get_uart3_rx_buf(U8 * char_out);
extern void CSI_board_init (void);
extern void ls_10usec_delay(S32 n);
extern void set_mux(U8 CH_num);
extern U16 read_write_ADC_byte(U8 ADC_state);
extern void init_RTC_spi_cs0_clk(void);
extern U16 read_ADC_Cal(U8 ADC_CH);
extern void init_map_TC2(void);
extern void init_switch_tc_ints(volatile avr32_tc_t *tc);
extern __attribute__((__interrupt__)) void gpio_sde_int_handler(void);
extern __attribute__((__interrupt__)) void gpio_CSIO_rst_int_handler(void);
extern __attribute__((__interrupt__)) void USART0_SPI_int_handler(void);
extern void put_CSIOrx_buf(U8 data);	//writes passed data into CSIOrx buffer
extern void put_CSIOtx_buf(U8 data);	//writes passed data into CSIOtx buffer
extern U16 get_CSIOrx_buf(volatile U8 * char_out);	//gets data from CSIOrx buffer
extern void get_CSIOtx_buf(volatile U8 * char_out);	//gets data from CSIOtx buffer
extern void CSI_config_brd (volatile avr32_usart_t * uart);
extern void empty_CSIOrx_buffer (void);
extern void empty_CSIOtx_buffer (void);
extern void init_USART0_CSIO (void)	;
extern void init_uart3_devcon(void);			//uart3 rx used for device configuration input
extern void check_devcon(void);
extern void check_sensors(void);
extern void set_CSI_defaults (void);
extern void configure_CSI_ports (void);
extern void configure_Rain_TC (void);
extern void adjust_digital_pot (void);
extern void blockmoveSWAP4(void *dst,void *src);
extern F32 SE1_read (void);
extern void init_USART0_SDI12 (void);
extern void SDI12_send_break(void);
extern void SDI12_ON(void);
extern void SDI12_OFF(void);
extern void SDI12_send_data (volatile char *string);
extern unsigned char * sdi12_cmd(volatile char *outbuf, unsigned char *inbuf, S32 timeout, int crcflag);
extern void TX_Dev_Test(volatile avr32_usart_t *uart);
extern U32 getTXTimeOffset(void);
extern void SDI12_test (volatile avr32_usart_t *uart);
extern void TX_Test(volatile avr32_usart_t *uart);
extern bool save_gCSIRamCpy_config_params_in_flash(void);




#endif /* CSI_A200_H */

