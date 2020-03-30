/*****************************************************************************
 *
 * \file
 *
 * \brief AT32UC3A EVK1100 board header file.
 *
 * This file contains definitions and services related to the features of the
 * EVK1100 board rev. B and C.
 *
 * To use this board, define BOARD=EVK1100.
 *
 * Copyright (c) 2009-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 ******************************************************************************/
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */


#ifndef _EVK1100_H_
#define _EVK1100_H_

#include "compiler.h"

#ifdef __AVR32_ABI_COMPILER__ // Automatically defined when compiling for AVR32, not when assembling.
#  include "led.h"
#endif  // __AVR32_ABI_COMPILER__


/*! \name Oscillator Definitions
 */
//! @{

#define FOSC32          32768                                 //!< Osc32 frequency: Hz.
#define OSC32_STARTUP   AVR32_PM_OSCCTRL32_STARTUP_8192_RCOSC //!< Osc32 startup time: RCOsc periods.

#define FOSC0           8000000                              //!< Osc0 frequency: Hz.
#define OSC0_STARTUP    AVR32_PM_OSCCTRL0_STARTUP_2048_RCOSC  //!< Osc0 startup time: RCOsc periods.

// Osc1 crystal is not mounted by default. Set the following definitions to the
// appropriate values if a custom Osc1 crystal is mounted on your board.
//#define FOSC1           12000000                              //!< Osc1 frequency: Hz.
//#define OSC1_STARTUP    AVR32_PM_OSCCTRL1_STARTUP_2048_RCOSC  //!< Osc1 startup time: RCOsc periods.

//! @}

/* These are documented in services/basic/clock/uc3a0_a1/osc.h */
#define BOARD_OSC0_HZ           8000000
#define BOARD_OSC0_STARTUP_US   17000
#define BOARD_OSC0_IS_XTAL      true
#define BOARD_OSC32_HZ          32768
#define BOARD_OSC32_STARTUP_US  71000
#define BOARD_OSC32_IS_XTAL     false

/*! \name SDRAM Definitions
 */
//! @{

//! Part header file of used SDRAM(s).
//#define SDRAM_PART_HDR  "mt48lc16m16a2tg7e/mt48lc16m16a2tg7e.h"omm

//! Data bus width to use the SDRAM(s) with (16 or 32 bits; always 16 bits on
//! UC3).
//#define SDRAM_DBW       16

//! @}


/*! \name USB Definitions
 */
//! @{

//! Multiplexed pin used for USB_ID: AVR32_USBB_USB_ID_x_x.
//! To be selected according to the AVR32_USBB_USB_ID_x_x_PIN and
//! AVR32_USBB_USB_ID_x_x_FUNCTION definitions from <avr32/uc3axxxx.h>.
#define USB_ID                      AVR32_USBB_USB_ID_0_0

//! Multiplexed pin used for USB_VBOF: AVR32_USBB_USB_VBOF_x_x.
//! To be selected according to the AVR32_USBB_USB_VBOF_x_x_PIN and
//! AVR32_USBB_USB_VBOF_x_x_FUNCTION definitions from <avr32/uc3axxxx.h>.
#define USB_VBOF                    AVR32_USBB_USB_VBOF_0_1

//! Active level of the USB_VBOF output pin.
#define USB_VBOF_ACTIVE_LEVEL       LOW

//! USB overcurrent detection pin.
#define USB_OVERCURRENT_DETECT_PIN  AVR32_PIN_PX33

//! @}


//! Number of LEDs.
#define LED_COUNT   4

/*! \name GPIO Connections of LEDs
 */
//! @{
#define LED0_GPIO   AVR32_PIN_PB05
#define LED1_GPIO   AVR32_PIN_PC05
#define LED2_GPIO   AVR32_PIN_PA29
#define LED3_GPIO   AVR32_PIN_PA30
//! @}

/*! \name PWM Channels of LEDs
 */
//! @{
#define LED0_PWM    (-1)
#define LED1_PWM    (-1)
#define LED2_PWM    (-1)
#define LED3_PWM    (-1)
//! @}

/*! \name PWM Functions of LEDs
 */
//! @{
#define LED0_PWM_FUNCTION   (-1)
#define LED1_PWM_FUNCTION   (-1)
#define LED2_PWM_FUNCTION   (-1)
#define LED3_PWM_FUNCTION   (-1)
//! @}

/*! \name Color Identifiers of LEDs to Use with LED Functions
 */
//! @{
#define LED_MONO0_GREEN   LED0
#define LED_MONO1_GREEN   LED1
#define LED_MONO2_GREEN   LED2
#define LED_MONO3_GREEN   LED3
//! @}


/*! \name GPIO Connections of Push Buttons
 */
//! @{
#define GPIO_PUSH_BUTTON_0            AVR32_PIN_PA24
#define GPIO_PUSH_BUTTON_0_PRESSED    0
//! @}


/*! \name GPIO Connections of the Joystick
 */
//! @{
//! @}


/*! \name ADC Connection of the Potentiometer
 */
//! @{
//! @}


/*! \name ADC Connection of the Temperature Sensor
 */
//! @{
//! @}


/*! \name ADC Connection of the Light Sensor
 */
//! @{
//! @}


/*! \name SPI Connections of the DIP204 LCD
 */
//! @{
//! @}

/*! \name GPIO and PWM Connections of the DIP204 LCD Backlight
 */
//! @{
//! @}


/*! \name SPI Connections of the AT45DBX Data Flash Memory
 */
//! @{
//! @}


/*! \name GPIO and SPI Connections of the SD/MMC Connector
 */
//! @{
// Keep it for SD MMC stack ASF V1.7
//! @}

/*! \name USART connections to GPIO
 */
//! @{
#define USART                       (&AVR32_USART3)
#define USART_RXD_PIN               AVR32_USART3_RXD_0_0_PIN
#define USART_RXD_FUNCTION          AVR32_USART3_RXD_0_0_FUNCTION
#define USART_TXD_PIN               AVR32_USART3_TXD_0_0_PIN
#define USART_TXD_FUNCTION          AVR32_USART3_TXD_0_0_FUNCTION
#define USART_IRQ                   AVR32_USART3_IRQ
#define USART_IRQ_GROUP             AVR32_USART3_IRQ_GROUP
#define USART_SYSCLK                SYSCLK_USART3
//! @}

/*! \name TWI Connections of the Spare TWI Connector
 */
//! @{
//! @}


/*! \name SPI Connections of the Spare SPI Connector
 */
//! @{
//! @}

/*! \name MACB connections to the DP83848 external phy controller
 */
//! @{

//! GPIO connection of the MAC PHY PWR_DOWN/INT signal for the external phy controller

//! @}

#endif  // !EVK1100_REVA

