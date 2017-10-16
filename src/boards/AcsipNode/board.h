/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Target board general functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Andreas Pella (IMST GmbH), Miguel Luis and Gregory Cristian
*/
#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#if defined(STM32F072xB)
#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#elif defined(STM32F401xC)
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#elif defined(STM32L073xx)
#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"
#endif
#include "utilities.h"
#include "timer.h"
#include "delay.h"
#include "gpio.h"
#include "adc.h"
#include "spi.h"
#include "i2c.h"
#include "uart.h"
#include "fifo.h"
#include "radio.h"
#include "sx1276/sx1276.h"
#include "adc-board.h"
#include "rtc-board.h"
#include "hw-timer-board.h"
#include "uart-board.h"
#include "sx1276-board.h"
#include "eeprom-board.h"

/*!
 * Generic definition
 */
#ifndef SUCCESS
#define SUCCESS                                     1
#endif

#ifndef FAIL
#define FAIL                                        0
#endif

#ifndef debug_printf
#define debug_printf UartPrintLF
#endif
/*!
 * Enables the choice between Led1 and Potentiometer.
 * LED1 and Potentiometer are exclusive.
 * \remark When using Potentimeter don't forget  that the connection between
 *         ADC input pin of iM880A and the Demoboard Poti requires a connection
 *         between X5:11 - X5:18.
 *         Remove the original jumpers for that. 
 *         On SK-iM880A X5 is the 20 pin header close to the DIP SW and Buttons
 */
#define USE_POTENTIOMETER                           1


/*!
 * AcSiP Default Disable
 */
#define ACSIP_DEFAULT_DISABLE 0

/*!
 * AcSiP Change
 */
#define ACSIP_CHANGE 1

/*!
 * LED Control Enable
 */
#define LED_ENABLE   0

/*!
 * Use Battery or USB
 */
#define USING_BATTERY   0

/*!
 * UART Control Enable
 */
#define UART_ENABLE   1

/*!
 * Enable UART2
 */
#define UART2_ENABLE  0

/*!
 * UART Baud Rate Setting
 */
#define UART_BAUDRATE 115200

/*!
 * UART Tx Rx Buffer
 */
#define UART_LENGTH 384

/*!
 * Enable Power Saving Demo or Not
 */
#define POWER_SAVING_DEMO      0  // 1:Enable, 0:Disable

/*!
 * Set Power Saving Interval Time
 */
#define POWER_SAVING_INTERVAL  6  // second

/*!
 * Enable RTC external wake-up pin
 */
#define WAKE_UP_BY_PIN         1  // 1:Enable, 0:Disable

/*!
 * Decide Log Level: DEBUG or INFO
 */
#define LOG_LEVEL              INFO

/*!
 * Enable RTC LED debug pin (optional)
 */
#define LED_DEBUG_PIN          0  // 1:Enable, 0:Disable

/*!
 * Enable I2C
 */
#define ENABLE_I2C             1

/*!
 * Board MCU pins definitions
 */

#if defined(STM32F072xB) || defined(STM32F401xC)
//Change Radio Related Pins
#define RADIO_RESET                                 PA_8
#define RADIO_MOSI                                  PB_15
#define RADIO_MISO                                  PB_14
#define RADIO_SCLK                                  PB_13
#define RADIO_NSS                                   PB_12
//#define RADIO_RESET                                 PB_0 // For SPI1 debug port
//#define RADIO_MOSI                                  PA_7 // For SPI1 debug port
//#define RADIO_MISO                                  PA_6 // For SPI1 debug port
//#define RADIO_SCLK                                  PA_5 // For SPI1 debug port
//#define RADIO_NSS                                   PA_4 // For SPI1 debug port
#define RADIO_DIO_0                                 PB_5
#define RADIO_DIO_1                                 PA_10
#define RADIO_DIO_2                                 PA_9
#define RADIO_DIO_3                                 PB_10
#define RADIO_DIO_4                                 PB_1
#define RADIO_DIO_5                                 PC_13
#elif defined (STM32L073xx)
#define RADIO_RESET                                 PB_10
#define RADIO_MOSI                                  PB_15
#define RADIO_MISO                                  PB_14
#define RADIO_SCLK                                  PB_13
#define RADIO_NSS                                   PB_12
//AcSiP(m), SPI debug 0819
//#define RADIO_RESET                                 PC_4 // For SPI1 debug port
//#define RADIO_MOSI                                  PA_7 // For SPI1 debug port
//#define RADIO_MISO                                  PA_6 // For SPI1 debug port
//#define RADIO_SCLK                                  PA_5 // For SPI1 debug port
//#define RADIO_NSS                                   PA_4 // For SPI1 debug port
#define RADIO_DIO_0                                 PB_11
#define RADIO_DIO_1                                 PC_13
#define RADIO_DIO_2                                 PB_9
#define RADIO_DIO_3                                 PB_4
#define RADIO_DIO_4                                 PB_3
#define RADIO_DIO_5                                 PA_15
#endif

//Disable Ant Switch GPIOs
#if defined (STM32L073xx)
#define RADIO_ANT_SWITCH_RXTX                       PA_1 //1:Rx, 0:Tx
#endif


#if defined(STM32F072xB) || defined(STM32F401xC)
#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15
#define OSC_HSE_IN                                  PF_0
#define OSC_HSE_OUT                                 PF_1
#elif defined (STM32L073xx)
#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15
#define OSC_HSE_IN                                  PH_0
#define OSC_HSE_OUT                                 PH_1
#endif

//Disable USB D+ D-
#if ACSIP_DEFAULT_DISABLE
#define USB_DM                                      PA_11
#define USB_DP                                      PA_12
#endif

#if defined(STM32F072xB) || defined(STM32F401xC)
#define JTAG_TMS                                    PA_13
#define JTAG_TCK                                    PA_14
#define JTAG_TDI                                    PA_15
#define JTAG_TDO                                    PB_3
#define JTAG_NRST                                   PB_4
#endif

#if defined(STM32F072xB) || defined(STM32F401xC)
#define I2C_SCL                                     PB_8
#define I2C_SDA                                     PB_9
#elif defined(STM32L073xx)
#define I2C_SCL                                     PB_6
#define I2C_SDA                                     PB_7
#endif

#if defined(STM32F072xB) || defined(STM32F401xC)
#define UART_TX                                     PA_2  // Watch-out!! Same as Radio SPI
#define UART_RX                                     PA_3  // Watch-out!! Same as Radio SPI
#elif defined(STM32L073xx)
#define UART_TX                                     PA_9
#define UART_RX                                     PA_10
#endif

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL                       DMA1_Channel4
#define USARTx_RX_DMA_CHANNEL                       DMA1_Channel5

#if ACSIP_DEFAULT_DISABLE
#if ( USE_POTENTIOMETER == 1 )
#define POTI                                        PB_0
#else
#define LED_1                                       PB_0
#endif
#define LED_2                                       PA_0
#define LED_3                                       PA_1
#define LED_4                                       PA_8
#endif

#if LED_ENABLE
#define LED_2                                       PB_2
#define LED_4                                       PA_7
#endif



/*!
 * LED GPIO pins objects
 */
#if ( USE_POTENTIOMETER == 0 )
extern Gpio_t Led1;
#endif
extern Gpio_t Led2;
extern Gpio_t Led3;
extern Gpio_t Led4;

/*!
 * MCU objects
 */
extern Adc_t Adc;
extern I2c_t I2c;
extern Uart_t Uart;
enum BoardPowerSource
{
    USB_POWER = 0,
    BATTERY_POWER
};

/*!
 * UART log level
 */
enum UartLogLevel
{
    DEBUG = 0,
    INFO
};

/*!
 * UART Tx Rx Buffer objects
 */
extern uint8_t UartTxBuff[UART_LENGTH];
extern uint8_t UartRxBuff[UART_LENGTH];

/*!
 * Initialize global variable from EEPROM.
 */
void variable_init(void);

/*!
 * \brief Uart Reset by calling UartInit & UartConfig again
 * \param [IN] assgin a new uart baudrate
 */
void BoardUartReset( uint32_t baudrate );

/*!
 * \brief Initializes the target board peripherals.
 */
void BoardInitMcu( void );

/*!
 * \brief Initializes the boards peripherals.
 */
void BoardInitPeriph( void );

/*!
 * \brief De-initializes the target board peripherals to decrease power
 *        consumption.
 */
void BoardDeInitMcu( void );

/*!
 * \brief Measure the Potentiometer level
 *
 * \retval value  Potentiometer level ( value in percent )
 */
uint8_t BoardMeasurePotiLevel( void );

/*!
 * \brief Measure the VDD voltage
 *
 * \retval value  VDD voltage in milivolts
 */
uint16_t BoardMeasureVdd( void );

/*!
 * \brief Get the current battery level
 *
 * \retval value  battery level ( 0: very low, 254: fully charged )
 */
uint8_t BoardGetBatteryLevel( void );

/*!
 * Returns a pseudo random seed generated using the MCU Unique ID
 *
 * \retval seed Generated pseudo random seed
 */
uint32_t BoardGetRandomSeed( void );

/*!
 * \brief Gets the board 64 bits unique ID 
 *
 * \param [IN] id Pointer to an array that will contain the Unique ID
 */
void BoardGetUniqueId( uint8_t *id );

/*!
 * \brief Gets the board 32 bits unique ID
 *
 * \retval value  32 bits unique ID
 */
uint32_t BoardGetUniqueIdLower32bits( void );

/*!
 * \brief Get the board power source
 *
 * \retval value  power source ( 0: USB_POWER,  1: BATTERY_POWER )
 */
uint8_t GetBoardPowerSource( void );

/*!
 * \brief Before entering power saving, it needs to be called
 */
void RCC_Power_Saving_Config(void);

/*!
 * \brief By using external pin to wake up RTC power saving mode
 *
 * \param [IN] GPIO Type (e.g. GPIOA, GPIOB, ...)
 */
void Wake_Up_Pin_Init(GPIO_TypeDef  *GPIOx);

/*!
 * \brief Before entering power saving, it needs to be called
 *
 * \param [IN] WakeUpTime: The total power-saving time of what we want. Unit: s
 * \param [IN] GPIO Type (e.g. GPIOA, GPIOB, ...)
 */
void Stop_Entry(RTC_HandleTypeDef *hrtc, unsigned int WakeUpTime, GPIO_TypeDef *WakeUpGPIO);

/*!
 * \brief Before entering power saving, it needs to be called by main.o
 *
 * \param [IN] WakeUpTime: The total power-saving time of what we want. Unit: s
 * \param [IN] GPIO Type (e.g. GPIOA, GPIOB, ...)
 */
void Demo_Enter_Stop_Mode(unsigned int WakeUpTime, GPIO_TypeDef *WakeUpGPIO);
#endif // __BOARD_H__

/*!
 * \brief I2C communication
 */
#if ENABLE_I2C
void I2C_CpltPolling(unsigned char addr);
#endif
