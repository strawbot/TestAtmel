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

