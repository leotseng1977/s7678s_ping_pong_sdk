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
#include "board.h"

/*!
 * Potentiometer max and min levels definition
 */
#define POTI_MAX_LEVEL 900
#define POTI_MIN_LEVEL 10

/*!
 * Vref values definition
 */
#define PDDADC_VREF_BANDGAP                             1224 // mV
#define PDDADC_MAX_VALUE                                4096

/*!
 * Battery level ratio (battery dependent)
 */
#define BATTERY_STEP_LEVEL                          0.23

/*!
 * Unique Devices IDs register set
 */
#if defined(STM32F072xB)
//AcSiP(m), For getting UUID on STM32F0xx
#define         ID1                                 ( 0x1FFFF7AC )
#define         ID2                                 ( 0x1FFFF7B0 )
#define         ID3                                 ( 0x1FFFF7B4 )
#elif defined(STM32F401xC)
#define         ID1                                 ( 0x1FFF7A10 )
#define         ID2                                 ( 0x1FFF7A14 )
#define         ID3                                 ( 0x1FFF7A18 )
#elif defined(STM32L073xx)
#define         ID1                                 ( 0x1FF80050 )
#define         ID2                                 ( 0x1FF80054 )
#define         ID3                                 ( 0x1FF80064 )
#endif

#if ( USE_POTENTIOMETER == 0 )
Gpio_t Led1;
#endif
Gpio_t Led2;
Gpio_t Led3;
Gpio_t Led4;

/*
 * MCU objects
 */
Adc_t Adc;
I2c_t I2c;
Uart_t Uart;

/*
 * Uart Tx Rx Buffer
 */
uint8_t UartTxBuff[UART_LENGTH];
uint8_t UartRxBuff[UART_LENGTH];

/*!
 * Initializes the unused GPIO to a know status
 */
static void BoardUnusedIoInit( void );

/*!
 * System Clock Configuration
 */
static void SystemClockConfig( void );

/*!
 * Used to measure and calibrate the system wake-up time from STOP mode
 */
static void CalibrateSystemWakeupTime( void );

/*!
 * System Clock Re-Configuration when waking up from STOP mode
 */
static void SystemClockReConfig( void );

/*!
 * Timer used at first boot to calibrate the SystemWakeupTime
 */
static TimerEvent_t CalibrateSystemWakeupTimeTimer;

/*!
 * Flag to indicate if the MCU is Initialized
 */
static bool McuInitialized = false;

/*!
 * Flag to indicate if the SystemWakeupTime is Calibrated
 */
static bool SystemWakeupTimeCalibrated = false;

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler */
}

/*!
 * Callback indicating the end of the system wake-up time calibration
 */
static void OnCalibrateSystemWakeupTimeTimerEvent( void )
{
    SystemWakeupTimeCalibrated = true;
}

void BoardInitPeriph( void )
{
#if ACSIP_DEFAULT_DISABLE
    /* Init the GPIO extender pins */
#if ( USE_POTENTIOMETER == 0 )
    GpioInit( &Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
#endif
    GpioInit( &Led2, LED_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    GpioInit( &Led3, LED_3, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    GpioInit( &Led4, LED_4, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );

    // Switch LED 1, 2, 3, 4 OFF
#if ( USE_POTENTIOMETER == 0 )
    GpioWrite( &Led1, 0 );
#endif
    GpioWrite( &Led2, 0 );
    GpioWrite( &Led3, 0 );
    GpioWrite( &Led4, 0 );
#endif

#if LED_ENABLE
    GpioInit( &Led2, LED_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    GpioInit( &Led4, LED_4, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );

    GpioWrite( &Led2, 1 );
    GpioWrite( &Led4, 1 );
#endif

}


#if ENABLE_I2C
I2C_HandleTypeDef         I2cHandle;
static void Init_I2C3()
{
    I2cHandle.Instance             = I2C3;
    I2cHandle.Init.Timing          = 0x10A13E56;
    I2cHandle.Init.OwnAddress1     = 0x02;
    I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2cHandle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    I2cHandle.Init.OwnAddress2     = 0xFF;
    I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    HAL_I2C_Init(&I2cHandle);
    HAL_I2CEx_ConfigAnalogFilter(&I2cHandle,I2C_ANALOGFILTER_ENABLE);
}
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;
    GPIO_InitTypeDef          GPIO_InitStruct;

    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C3;
    RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2C3CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
    __HAL_RCC_I2C3_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_I2C3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_NVIC_SetPriority(I2C3_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(I2C3_IRQn);
}
void I2C3_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&I2cHandle);
    HAL_I2C_ER_IRQHandler(&I2cHandle);
}
#endif

void BoardUartReset ( uint32_t baudrate )
{
    Uart.IsInitialized = false;
    UartInit(&Uart, UART_1, UART_TX, UART_RX);
    UartConfig(&Uart,
               RX_TX,
               baudrate,
               UART_8_BIT,
               UART_1_STOP_BIT,
               NO_PARITY,
               NO_FLOW_CTRL);
}

void BoardInitMcu( void )
{
    if( McuInitialized == false )
    {
#if defined( USE_BOOTLOADER )
        // Set the Vector Table base location at 0x3000
        SCB->VTOR = FLASH_BASE | 0x3000;
#endif
        HAL_Init( );

        SystemClockConfig( );

        //RtcInit( );

        TimerHwInit();
#if UART_ENABLE
        BoardUartReset(UART_BAUDRATE);

        // Using FIFOs for UART Rx Tx
        FifoInit(&Uart.FifoTx, UartTxBuff, UART_LENGTH);
        FifoInit(&Uart.FifoRx, UartRxBuff, UART_LENGTH);
#endif
        BoardUnusedIoInit( );
    }
    else
    {
        // Init Periphral Pins Again
        // Now only UART needs to be init GPIO again
        BoardUartReset(UART_BAUDRATE);

        SystemClockReConfig( );
    }

#if ACSIP_DEFAULT_DISABLE
    AdcInit( &Adc, POTI );
#endif

    SpiInit( &SX1276.Spi, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, NC );
    SX1276IoInit( );

#if defined(STM32F072xB)
    //Enable GPIO_F Clock
    __HAL_RCC_GPIOF_CLK_ENABLE();
#elif defined(STM32F401xC) || defined(STM32L073xx)
    //Enable GPIO_H Clock
    __HAL_RCC_GPIOH_CLK_ENABLE();
#endif

    if( McuInitialized == false )
    {
        McuInitialized = true;
        if( GetBoardPowerSource( ) == BATTERY_POWER )
        {
            CalibrateSystemWakeupTime( );
        }
    }

#if ENABLE_I2C
    Init_I2C3();
#endif
}

#if ENABLE_I2C
void I2C_CpltPolling(unsigned char addr)
{	//example for the temperature sensor HTU21D
    static uint8_t 	HTU21D_buffer[8];
    if(HAL_I2C_IsDeviceReady(&I2cHandle, addr, 3, 20)==HAL_OK)
    {
        //Transmit 1 byte to I2C3. request temperature data
        HTU21D_buffer[0] = 0xE3;
        while(HAL_I2C_Master_Transmit_IT(&I2cHandle, addr, HTU21D_buffer, 1)!= HAL_OK);
        while (HAL_I2C_GetState(&I2cHandle)!= HAL_I2C_STATE_READY);

        //Receive 3 byte to I2C3. receive 2 byte data+ 1 byte CRC
        while(HAL_I2C_Master_Receive_IT(&I2cHandle, addr, HTU21D_buffer + 1, 7)!= HAL_OK);
        while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY);
        UartPrint("I2C:%2X,%2X,%2X\n", HTU21D_buffer[1], HTU21D_buffer[2], HTU21D_buffer[3]);
    }
    return;
}
#endif

void BoardDeInitMcu( void )
{
    SpiDeInit( &SX1276.Spi );
    SX1276IoDeInit( );
    UartDeInit( &Uart );

#if ACSIP_DEFAULT_DISABLE
    Gpio_t ioPin;

    GpioInit( &ioPin, OSC_HSE_IN, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    GpioInit( &ioPin, OSC_HSE_OUT, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 1 );

    GpioInit( &ioPin, OSC_LSE_IN, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 1 );
    GpioInit( &ioPin, OSC_LSE_OUT, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 1 );
#endif
}

uint32_t BoardGetRandomSeed( void )
{
    return ( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
}

void BoardGetUniqueId( uint8_t *id )
{
    id[7] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 24;
    id[6] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 16;
    id[5] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 8;
    id[4] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) );
    id[3] = ( ( *( uint32_t* )ID2 ) ) >> 24;
    id[2] = ( ( *( uint32_t* )ID2 ) ) >> 16;
    id[1] = ( ( *( uint32_t* )ID2 ) ) >> 8;
    id[0] = ( ( *( uint32_t* )ID2 ) );
}

uint32_t BoardGetUniqueIdLower32bits( void )
{
    // Random seed initialization
    srand1( BoardGetRandomSeed( ) );
    // Return a random uid
    return randr( 0, 0xFFFFFFFF );
}

uint8_t BoardMeasurePotiLevel( void )
{
    uint8_t potiLevel = 0;
    uint16_t MeasuredLevel = 0;

#if ACSIP_DEFAULT_DISABLE
    // read the current potentiometer setting
    MeasuredLevel = AdcMcuRead( &Adc , ADC_CHANNEL_3 );
#endif

    // check the limits
    if( MeasuredLevel >= POTI_MAX_LEVEL )
    {
        potiLevel = 100;
    }
    else if( MeasuredLevel <= POTI_MIN_LEVEL )
    {
        potiLevel = 0;
    }
    else
    {
        // if the value is in the area, calculate the percentage value
        potiLevel = ( ( MeasuredLevel - POTI_MIN_LEVEL ) * 100 ) / POTI_MAX_LEVEL;
    }
    return potiLevel;
}

uint16_t BoardMeasureVdd( void )
{
    uint32_t milliVolt = 0;

#if ACSIP_DEFAULT_DISABLE
    uint16_t MeasuredLevel = 0;
    // Read the current Voltage
    MeasuredLevel = AdcMcuRead( &Adc , ADC_CHANNEL_17 );

    // We don't use the VREF from calibValues here.
    // calculate the Voltage in miliVolt
    milliVolt = ( uint32_t )PDDADC_VREF_BANDGAP * ( uint32_t )PDDADC_MAX_VALUE;
    milliVolt = milliVolt / ( uint32_t ) MeasuredLevel;
#else
    // Dummy value for 071H
    milliVolt = 3000;
#endif
    
    return ( uint16_t ) milliVolt;
}

uint8_t BoardGetBatteryLevel( void )
{
    uint8_t batteryLevel = 0;
    uint16_t measuredLevel = 0;

    measuredLevel = BoardMeasureVdd( );

    if( measuredLevel >= 3000 )
    {
        batteryLevel = 254;
    }
    else if( measuredLevel <= 2400 )
    {
        batteryLevel = 1;
    }
    else
    {
        batteryLevel = ( measuredLevel - 2400 ) * BATTERY_STEP_LEVEL;
    }
    return batteryLevel;
}

static void BoardUnusedIoInit( void )
{
#if ACSIP_DEFAULT_DISABLE
    Gpio_t ioPin;
    
    if( GetBoardPowerSource( ) == BATTERY_POWER )
    {
        GpioInit( &ioPin, USB_DM, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        GpioInit( &ioPin, USB_DP, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    }
#endif

#if defined( USE_DEBUGGER )
    HAL_DBGMCU_EnableDBGStopMode( );
    //HAL_DBGMCU_EnableDBGSleepMode( );
    HAL_DBGMCU_EnableDBGStandbyMode( );
#else
    //HAL_DBGMCU_DisableDBGSleepMode( );
    HAL_DBGMCU_DisableDBGStopMode( );
    HAL_DBGMCU_DisableDBGStandbyMode( );

#if defined(STM32F072xB) || defined(STM32F401xC)
    GpioInit( &ioPin, JTAG_TMS, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, JTAG_TCK, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, JTAG_TDI, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, JTAG_TDO, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, JTAG_NRST, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif
#endif
}

#if POWER_SAVING_DEMO

#if WAKE_UP_BY_PIN
void Wake_Up_Pin_Init(GPIO_TypeDef  *GPIOx)
{   //set wekeup pin = UART1 & UART2
    GPIO_InitTypeDef  GPIO_Struct;
    GPIO_Struct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_9|GPIO_PIN_10;
    GPIO_Struct.Mode = GPIO_MODE_IT_RISING;
    GPIO_Struct.Pull = GPIO_PULLUP;
    GPIO_Struct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOx, &GPIO_Struct);

    return;
}
#endif //WAKE_UP_BY_PIN

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
    /* Clear Wake Up Flag */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
}

void RCC_Power_Saving_Config(void)
{   //set RCC Clock
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
    __HAL_RCC_RTC_ENABLE();
    HAL_NVIC_SetPriority(RTC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(RTC_IRQn);

    return;
}

void RTC_WakeUp_Setting(RTC_HandleTypeDef *hrtc, unsigned int seconds)
{   //set wake up time, need RTC_Config's return
    HAL_RTCEx_DeactivateWakeUpTimer(hrtc);
    __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);
    HAL_RTCEx_SetWakeUpTimer_IT(hrtc, seconds-1, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
    return;
}

void Stop_Entry(RTC_HandleTypeDef *hrtc, unsigned int WakeUpTime, GPIO_TypeDef *WakeUpGPIO)
{
    GPIO_InitTypeDef  GPIO_Struct;

    //sx1276 sleep commands
    SX1276SetSleep();
    SX1276Write(REG_OPMODE, SX1276Read(REG_OPMODE)&0xF8);
    //set wake up by RTC
    RTC_WakeUp_Setting(hrtc, WakeUpTime);

    GPIO_Struct.Pin = GPIO_PIN_All;
    GPIO_Struct.Mode = GPIO_MODE_ANALOG;
    GPIO_Struct.Pull = GPIO_NOPULL;
    GPIO_Struct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_Struct);
    HAL_GPIO_Init(GPIOB, &GPIO_Struct);
    HAL_GPIO_Init(GPIOC, &GPIO_Struct);
    HAL_GPIO_Init(GPIOD, &GPIO_Struct);
    HAL_GPIO_Init(GPIOE, &GPIO_Struct);
    HAL_GPIO_Init(GPIOH, &GPIO_Struct);

    //GPIO interrupt PIN can be not analog PIN
#if WAKE_UP_BY_PIN
    if(WakeUpGPIO == GPIOA)
    {
        Wake_Up_Pin_Init(WakeUpGPIO);
    }
#endif

    //sx1276 SPI PIN can be not analog PIN
    GPIO_Struct.Pin =  GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_Struct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Struct.Pull = GPIO_PULLUP;
    GPIO_Struct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_Struct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_PWREx_EnableUltraLowPower();
    HAL_PWR_EnterSTOPMode( PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI );

    //RTC & GPIO interrupt do nothing, S76 will wake up here.
    BoardInitMcu();
    SX1276SetStby();
    SX1276Write(REG_OPMODE, SX1276Read(REG_OPMODE)|0x01);

#if UART2_ENABLE
    Uart2Init();
#endif
#ifdef LED_DEBUG_PIN
    GPIO_Struct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_Struct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Struct.Pull = GPIO_PULLUP;
    GPIO_Struct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_Struct);
#endif

    return;
}

void Demo_Enter_Stop_Mode(unsigned int WakeUpTime, GPIO_TypeDef *WakeUpGPIO)
{
    RTC_HandleTypeDef *hrtc;

    __HAL_RCC_TIM3_CLK_DISABLE();
    RCC_Power_Saving_Config();
    hrtc = RTC_Config();

#if LED_DEBUG_PIN
    GPIO_InitTypeDef  GPIO_Struct;
    __GPIOC_CLK_ENABLE();

    GPIO_Struct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_Struct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Struct.Pull = GPIO_PULLUP;
    GPIO_Struct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_Struct);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
#endif

    //Enter Stop Mode First (e.g. 10s)
    Stop_Entry(hrtc, WakeUpTime, WakeUpGPIO);
}
#endif //POWER_SAVING_DEMO

void SystemClockConfig( void )
{
#if defined(STM32F072xB)
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV13;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
#elif defined(STM32F401xC)
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 26;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
#elif defined(STM32L073xx)
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 1);
#endif
}

void CalibrateSystemWakeupTime( void )
{
    if( SystemWakeupTimeCalibrated == false )
    {
        TimerInit( &CalibrateSystemWakeupTimeTimer, OnCalibrateSystemWakeupTimeTimerEvent );
        TimerSetValue( &CalibrateSystemWakeupTimeTimer, 1000 );
        TimerStart( &CalibrateSystemWakeupTimeTimer );
        while( SystemWakeupTimeCalibrated == false )
        {
            TimerLowPowerHandler( );
        }
    }
}

void SystemClockReConfig( void )
{
    __HAL_RCC_PWR_CLK_ENABLE( );
#if defined(STM32L073xx)
    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );
#endif
    /* Enable HSE */
    __HAL_RCC_HSE_CONFIG( RCC_HSE_ON );

    /* Wait till HSE is ready */
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET)
    {
    }

    /* Enable PLL */
    __HAL_RCC_PLL_ENABLE( );

    /* Wait till PLL is ready */
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_PLLRDY ) == RESET )
    {
    }

    /* Select PLL as system clock source */
    __HAL_RCC_SYSCLK_CONFIG ( RCC_SYSCLKSOURCE_PLLCLK );

    /* Wait till PLL is used as system clock source */
    while( __HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_PLLCLK )
    {
    }
}

void SysTick_Handler( void )
{
    HAL_IncTick( );
    HAL_SYSTICK_IRQHandler( );
}

uint8_t GetBoardPowerSource( void )
{
#if USING_BATTERY
    return BATTERY_POWER;
#else
    return USB_POWER;
#endif
}

#ifdef USE_FULL_ASSERT
/*
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 */
void assert_failed( uint8_t* file, uint32_t line )
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while( 1 )
    {
    }
}
#endif
