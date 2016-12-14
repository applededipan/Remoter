#ifndef _HAL_
#define _HAL_


/******************************************************************************/ 
//! Keys_apple

#define KEYS_GPIO_REG_BUTTON_POWER    GPIOC->IDR  //! PC.01
#define KEYS_GPIO_PIN_BUTTON_POWER    GPIO_Pin_1

#define KEYS_GPIO_REG_BUTTON_P        GPIOE->IDR
#define KEYS_GPIO_PIN_BUTTON_P        GPIO_Pin_6  //! PE.06
  
#define KEYS_GPIO_REG_BUTTON_A        GPIOE->IDR
#define KEYS_GPIO_PIN_BUTTON_A        GPIO_Pin_5  //! PE.05
  
#define KEYS_GPIO_REG_BUTTON_X        GPIOE->IDR             //!MANUAL
#define KEYS_GPIO_PIN_BUTTON_X        GPIO_Pin_1  //! PE.01
  
#define KEYS_GPIO_REG_BUTTON_L        GPIOC->IDR             //!AUTO
#define KEYS_GPIO_PIN_BUTTON_L        GPIO_Pin_7  //! PC.07
  
#define KEYS_GPIO_REG_BUTTON_H        GPIOC->IDR             //!HOME
#define KEYS_GPIO_PIN_BUTTON_H        GPIO_Pin_6  //! PC.06
  
#define KEYS_GPIO_REG_BUTTON_TL1      GPIOC->IDR
#define KEYS_GPIO_PIN_BUTTON_TL1      GPIO_Pin_9  //! PC.09
  
#define KEYS_GPIO_REG_BUTTON_TL2      GPIOC->IDR
#define KEYS_GPIO_PIN_BUTTON_TL2      GPIO_Pin_8  //! PC.08
  
#define KEYS_GPIO_REG_BUTTON_TR1      GPIOE->IDR
#define KEYS_GPIO_PIN_BUTTON_TR1      GPIO_Pin_3  //! PE.03

#define KEYS_GPIO_REG_BUTTON_TR2      GPIOE->IDR
#define KEYS_GPIO_PIN_BUTTON_TR2      GPIO_Pin_4  //! PE.04
  
#define KEYS_RCC_AHB1Periph           (RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE)//打开IO时钟
#define KEYS_GPIOC_PINS               (KEYS_GPIO_PIN_BUTTON_POWER|KEYS_GPIO_PIN_BUTTON_L|KEYS_GPIO_PIN_BUTTON_H|KEYS_GPIO_PIN_BUTTON_TL1|KEYS_GPIO_PIN_BUTTON_TL2)
#define KEYS_GPIOE_PINS               (KEYS_GPIO_PIN_BUTTON_P|KEYS_GPIO_PIN_BUTTON_A|KEYS_GPIO_PIN_BUTTON_X|KEYS_GPIO_PIN_BUTTON_TR1|KEYS_GPIO_PIN_BUTTON_TR2) 

/****************************************************************************/






/****************************************************************************/
//! LED_apple

#define LED_GPIO_REG_H                  GPIOD
#define LED_GPIO_PIN_H                  GPIO_Pin_13

#define LED_GPIO_REG_L_A                GPIOA
#define LED_GPIO_PIN_L_A                GPIO_Pin_8

#define LED_GPIO_REG_X_M                GPIOC
#define LED_GPIO_PIN_X_M                GPIO_Pin_12

#define LED_GPIO_REG_A                  GPIOC
#define LED_GPIO_PIN_A                  GPIO_Pin_13

#define LED_GPIO_REG_P                  GPIOC
#define LED_GPIO_PIN_P                  GPIO_Pin_2

#define LED_GPIO_REG_POWER              GPIOC
#define LED_GPIO_PIN_POWER              GPIO_Pin_3

#define LED_RCC_AHB1Periph             (RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD) 
#define LED_GPIOA_PINS                 (LED_GPIO_PIN_L_A)
#define LED_GPIOC_PINS                 (LED_GPIO_PIN_A | LED_GPIO_PIN_X_M | LED_GPIO_PIN_P | LED_GPIO_PIN_POWER)
#define LED_GPIOD_PINS                 (LED_GPIO_PIN_H)

/****************************************************************************/







/****************************************************************************/
//! Power_apple

#define POWER_GPIO_REG_ENABLE          GPIOC
#define POWER_GPIO_PIN_ENABLE          GPIO_Pin_0

#define POWER_RCC_AHB1Periph          (RCC_AHB1Periph_GPIOC)

/****************************************************************************/








/****************************************************************************/
//! BEEP_apple
#define BEEP_GPIO_REG                  GPIOD
#define BEEP_GPIO_PIN                  GPIO_Pin_12

#define BEEP_RCC_AHB1Periph            (RCC_AHB1Periph_GPIOD)
#define BEEP_GPIOD_PINS                (BEEP_GPIO_PIN)

/****************************************************************************/








/****************************************************************************/
//! MOTOR_apple

#define MOTOR_GPIO_REG                  GPIOB
#define MOTOR_GPIO_PIN                  GPIO_Pin_3

#define MOTOR_RCC_AHB1Periph           (RCC_AHB1Periph_GPIOB)
#define MOTOR_GPIOB_PINS               (MOTOR_GPIO_PIN)

/****************************************************************************/








/****************************************************************************/
//! LCD_apple

#define BACKLIGHT_RCC_AHB1Periph       RCC_AHB1Periph_GPIOB
#define BACKLIGHT_RCC_APB1Periph       RCC_APB1Periph_TIM4
#define BACKLIGHT_TIMER                TIM4
#define BACKLIGHT_GPIO                 GPIOB
#define BACKLIGHT_GPIO_PIN             GPIO_Pin_9  //! PB.9
#define BACKLIGHT_GPIO_PinSource       GPIO_PinSource9
#define BACKLIGHT_GPIO_AF              GPIO_AF_TIM4

#define LCD_GPIO_REG_FSMC_D0           GPIOD 
#define LCD_GPIO_PIN_FSMC_D0           GPIO_Pin_14 //! PD.14FSMC
#define LCD_GPIO_PinSource_D0          GPIO_PinSource14

#define LCD_GPIO_REG_FSMC_D1           GPIOD                   
#define LCD_GPIO_PIN_FSMC_D1           GPIO_Pin_15 //! PD.15FSMC
#define LCD_GPIO_PinSource_D1          GPIO_PinSource15

#define LCD_GPIO_REG_FSMC_D2           GPIOD 
#define LCD_GPIO_PIN_FSMC_D2           GPIO_Pin_0  //! PD.0FSMC
#define LCD_GPIO_PinSource_D2          GPIO_PinSource0

#define LCD_GPIO_REG_FSMC_D3           GPIOD 
#define LCD_GPIO_PIN_FSMC_D3           GPIO_Pin_1  //! PD.1FSMC
#define LCD_GPIO_PinSource_D3          GPIO_PinSource1

#define LCD_GPIO_REG_FSMC_D4           GPIOE                   
#define LCD_GPIO_PIN_FSMC_D4           GPIO_Pin_7  //! PE.7FSMC
#define LCD_GPIO_PinSource_D4          GPIO_PinSource7

#define LCD_GPIO_REG_FSMC_D5           GPIOE 
#define LCD_GPIO_PIN_FSMC_D5           GPIO_Pin_8  //! PE.8FSMC
#define LCD_GPIO_PinSource_D5          GPIO_PinSource8

#define LCD_GPIO_REG_FSMC_D6           GPIOE                   
#define LCD_GPIO_PIN_FSMC_D6           GPIO_Pin_9  //! PE.9FSMC
#define LCD_GPIO_PinSource_D6          GPIO_PinSource9

#define LCD_GPIO_REG_FSMC_D7           GPIOE 
#define LCD_GPIO_PIN_FSMC_D7           GPIO_Pin_10 //! PE.10FSMC 
#define LCD_GPIO_PinSource_D7          GPIO_PinSource10

#define LCD_GPIO_REG_FSMC_D8           GPIOE                   
#define LCD_GPIO_PIN_FSMC_D8           GPIO_Pin_11 //! PE.11FSMC
#define LCD_GPIO_PinSource_D8          GPIO_PinSource11

#define LCD_GPIO_REG_FSMC_D9           GPIOE                   
#define LCD_GPIO_PIN_FSMC_D9           GPIO_Pin_12 //! PE.12FSMC
#define LCD_GPIO_PinSource_D9          GPIO_PinSource12

#define LCD_GPIO_REG_FSMC_D10          GPIOE                   
#define LCD_GPIO_PIN_FSMC_D10          GPIO_Pin_13 //! PE.13FSMC
#define LCD_GPIO_PinSource_D10          GPIO_PinSource13

#define LCD_GPIO_REG_FSMC_D11          GPIOE                   
#define LCD_GPIO_PIN_FSMC_D11          GPIO_Pin_14 //! PE.14FSMC
#define LCD_GPIO_PinSource_D11         GPIO_PinSource14

#define LCD_GPIO_REG_FSMC_D12          GPIOE                   
#define LCD_GPIO_PIN_FSMC_D12          GPIO_Pin_15 //! PE.15FSMC
#define LCD_GPIO_PinSource_D12         GPIO_PinSource15

#define LCD_GPIO_REG_FSMC_D13          GPIOD                   
#define LCD_GPIO_PIN_FSMC_D13          GPIO_Pin_8  //! PD.8FSMC
#define LCD_GPIO_PinSource_D13          GPIO_PinSource8

#define LCD_GPIO_REG_FSMC_D14          GPIOD                   
#define LCD_GPIO_PIN_FSMC_D14          GPIO_Pin_9  //! PD.9FSMC
#define LCD_GPIO_PinSource_D14          GPIO_PinSource9

#define LCD_GPIO_REG_FSMC_D15          GPIOD                   
#define LCD_GPIO_PIN_FSMC_D15          GPIO_Pin_10 //! PD.10FSMC
#define LCD_GPIO_PinSource_D15          GPIO_PinSource10

#define LCD_GPIO_REG_FSMC_RD           GPIOD                   
#define LCD_GPIO_PIN_FSMC_RD           GPIO_Pin_4  //! PD.4  读信号
#define LCD_GPIO_PinSource_RD          GPIO_PinSource4

#define LCD_GPIO_REG_FSMC_WR           GPIOD                   
#define LCD_GPIO_PIN_FSMC_WR           GPIO_Pin_5  //! PD.5  写信号
#define LCD_GPIO_PinSource_WR          GPIO_PinSource5

#define LCD_GPIO_REG_FSMC_CS           GPIOD                   
#define LCD_GPIO_PIN_FSMC_CS           GPIO_Pin_7  //! PD.7  片选信号
#define LCD_GPIO_PinSource_CS          GPIO_PinSource7

#define LCD_GPIO_REG_FSMC_RS           GPIOD                   
#define LCD_GPIO_PIN_FSMC_RS           GPIO_Pin_11 //! PD.11 寄存器/数据选择信号
#define LCD_GPIO_PinSource_RS          GPIO_PinSource11

#define LCD_GPIO_REG_RST               GPIOB
#define LCD_GPIO_PIN_RST               GPIO_Pin_4  //! PB.4  复位信号

#define LCD_RCC_AHB1Periph            (RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE)
#define LCD_FSMC_RCC_AHB3Periph        RCC_AHB3Periph_FSMC
#define LCD_GPIOD_PINS                (LCD_GPIO_PIN_FSMC_D0 | LCD_GPIO_PIN_FSMC_D1 | LCD_GPIO_PIN_FSMC_D2 | LCD_GPIO_PIN_FSMC_D3 | LCD_GPIO_PIN_FSMC_D13 | LCD_GPIO_PIN_FSMC_D14 | LCD_GPIO_PIN_FSMC_D15 | LCD_GPIO_PIN_FSMC_RD | LCD_GPIO_PIN_FSMC_WR | LCD_GPIO_PIN_FSMC_CS | LCD_GPIO_PIN_FSMC_RS)
#define LCD_GPIOE_PINS                (LCD_GPIO_PIN_FSMC_D4 | LCD_GPIO_PIN_FSMC_D5 | LCD_GPIO_PIN_FSMC_D6 | LCD_GPIO_PIN_FSMC_D7 | LCD_GPIO_PIN_FSMC_D8 | LCD_GPIO_PIN_FSMC_D9 |LCD_GPIO_PIN_FSMC_D10 | LCD_GPIO_PIN_FSMC_D11 | LCD_GPIO_PIN_FSMC_D12 )
#define LCD_GPIO_FSMC_AF               GPIO_AF_FSMC

/****************************************************************************/








/****************************************************************************/
//! ADC_apple

#define ADC_RCC_AHB1Periph             (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA2)
#define ADC_RCC_APB2Periph              RCC_APB2Periph_ADC1
  
#define ADC_GPIO_PIN_STICK_RV           GPIO_Pin_0  //!  PA.00
#define ADC_GPIO_PIN_STICK_RH           GPIO_Pin_1  //!  PA.01
#define ADC_GPIO_PIN_STICK_LH           GPIO_Pin_5  //!  PA.05
#define ADC_GPIO_PIN_STICK_LV           GPIO_Pin_4  //!  PC.04
#define ADC_GPIO_PIN_POTENMETER_1       GPIO_Pin_5  //!  PC.05
#define ADC_GPIO_PIN_POTENMETER_2       GPIO_Pin_4  //!  PA.04
#define ADC_GPIO_PIN_BAT                GPIO_Pin_6  //!  PA.06
#define ADC_GPIO_PIN_VIN                GPIO_Pin_7  //!  PA.07
 
#define ADC_CHANNEL_STICK_RV            ADC_Channel_0  //!  ADC1_IN0
#define ADC_CHANNEL_STICK_RH            ADC_Channel_1  //!  ADC1_IN1
#define ADC_CHANNEL_STICK_LH            ADC_Channel_5  //!  ADC1_IN5
#define ADC_CHANNEL_STICK_LV            ADC_Channel_14 //!  ADC1_IN14
#define ADC_CHANNEL_POTENMETER_1        ADC_Channel_15 //!  ADC1_IN15
#define ADC_CHANNEL_POTENMETER_2        ADC_Channel_4  //!  ADC1_IN4
#define ADC_CHANNEL_BAT                 ADC_Channel_6  //!  ADC1_IN6
#define ADC_CHANNEL_VIN                 ADC_Channel_7  //!  ADC1_IN7
  
/****************************************************************************/



 
 
 
 

/****************************************************************************/
//! Usart1 / USB Port
#define USART1_RCC_AHB1Periph          (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOD)
#define USART1_RCC_APB2Periph           RCC_APB2Periph_USART1
#define USART1_GPIO                     GPIOA
#define USART1_GPIO_PIN_TX              GPIO_Pin_9  //!  PA.9
#define USART1_GPIO_PIN_RX              GPIO_Pin_10 //!  PB.10
#define USART1_GPIO_PinSource_TX        GPIO_PinSource9
#define USART1_GPIO_PinSource_RX        GPIO_PinSource10
#define USART1_GPIO_AF                  GPIO_AF_USART1
#define USART1_USART                    USART1
#define USART1_USART_IRQHandler         USART1_IRQHandler
#define USART1_USART_IRQn               USART1_IRQn

#define USART1_USB_GPIO_REG             GPIOD->IDR  //! PD.03
#define USART1_USB_GPIO_PIN             GPIO_Pin_3



//! Usart3 / bluetooth Port
#define USART3_RCC_AHB1Periph           RCC_AHB1Periph_GPIOB
#define USART3_RCC_APB1Periph           RCC_APB1Periph_USART3
#define USART3_GPIO                     GPIOB
#define USART3_GPIO_PIN_TX              GPIO_Pin_10 //!  PB.10
#define USART3_GPIO_PIN_RX              GPIO_Pin_11 //!  PB.11
#define USART3_GPIO_PinSource_TX        GPIO_PinSource10
#define USART3_GPIO_PinSource_RX        GPIO_PinSource11
#define USART3_GPIO_AF                  GPIO_AF_USART3
#define USART3_USART                    USART3
#define USART3_USART_IRQHandler         USART3_IRQHandler
#define USART3_USART_IRQn               USART3_IRQn


//! Uart4 / Raspberry Pi Port  
#define UART4_RCC_AHB1Periph            RCC_AHB1Periph_GPIOC
#define UART4_RCC_APB1Periph            RCC_APB1Periph_UART4
#define UART4_GPIO                      GPIOC
#define UART4_GPIO_PIN_TX               GPIO_Pin_10 //!  PC.10
#define UART4_GPIO_PIN_RX               GPIO_Pin_11 //!  PC.11
#define UART4_GPIO_PinSource_TX         GPIO_PinSource10
#define UART4_GPIO_PinSource_RX         GPIO_PinSource11
#define UART4_GPIO_AF                   GPIO_AF_UART4
#define UART4_USART                     UART4
#define UART4_USART_IRQHandler          UART4_IRQHandler
#define UART4_USART_IRQn                UART4_IRQn


//! USART2 / p900 Port
#define TELEMETRY_RCC_AHB1Periph        RCC_AHB1Periph_GPIOA
#define TELEMETRY_RCC_APB1Periph        RCC_APB1Periph_USART2
#define TELEMETRY_GPIO                  GPIOA
#define TELEMETRY_GPIO_PIN_TX           GPIO_Pin_2  //!  PA.2
#define TELEMETRY_GPIO_PIN_RX           GPIO_Pin_3  //!  PA.3
#define TELEMETRY_GPIO_PinSource_TX     GPIO_PinSource2
#define TELEMETRY_GPIO_PinSource_RX     GPIO_PinSource3
#define TELEMETRY_GPIO_AF               GPIO_AF_USART2
#define TELEMETRY_USART                 USART2
#define TELEMETRY_USART_IRQHandler      USART2_IRQHandler
#define TELEMETRY_USART_IRQn            USART2_IRQn

/****************************************************************************/








/****************************************************************************/
//! I2C Bus: EEPROM M24512
#define I2C_RCC_AHB1Periph              RCC_AHB1Periph_GPIOB
#define I2C_RCC_APB1Periph              RCC_APB1Periph_I2C1
#define I2C                             I2C1
#define I2C_GPIO                        GPIOB
#define I2C_GPIO_PIN_SCL                GPIO_Pin_6  //!  PB.06
#define I2C_GPIO_PIN_SDA                GPIO_Pin_7  //!  PB.07
#define I2C_GPIO_PIN_WP                 GPIO_Pin_5  //!  PB.05
#define I2C_GPIO_AF                     GPIO_AF_I2C1
#define I2C_GPIO_PinSource_SCL          GPIO_PinSource6
#define I2C_GPIO_PinSource_SDA          GPIO_PinSource7

#define I2C_SPEED                       400000

#define I2C_ADDRESS_EEPROM              0xA0 //! 1010 000 0
#define I2C_EEPROM_PAGESIZE             128
/****************************************************************************/








/****************************************************************************/
//! USB Driver
#define USB_RCC_AHB1Periph              RCC_AHB1Periph_GPIOA
#define USB_GPIO                        GPIOA
#define USB_GPIO_PIN_VBUS               GPIO_Pin_15 //!  PA.15 apple: modified by apple original:PA9
#define USB_GPIO_PIN_DM                 GPIO_Pin_11 //!  PA.11
#define USB_GPIO_PIN_DP                 GPIO_Pin_12 //!  PA.12
#define USB_GPIO_PinSource_DM           GPIO_PinSource11
#define USB_GPIO_PinSource_DP           GPIO_PinSource12
#define USB_GPIO_AF                     GPIO_AF_OTG1_FS
/****************************************************************************/








/****************************************************************************/
//! SPI2 Driver modified by apple
#define SPI_RCC_AHB1Periph              (RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1)
#define SPI_RCC_APB1Periph               RCC_APB1Periph_SPI2
#define SPI_GPIO                         GPIOB
#define SPI_GPIO_PIN_SCK                 GPIO_Pin_13 //!  PB.13
#define SPI_GPIO_PIN_MISO                GPIO_Pin_14 //!  PB.14
#define SPI_GPIO_PIN_MOSI                GPIO_Pin_15 //!  PB.15
#define SPI_GPIO_AF                      GPIO_AF_SPI2
#define SPI_GPIO_PinSource_SCK           GPIO_PinSource13
#define SPI_GPIO_PinSource_MISO          GPIO_PinSource14
#define SPI_GPIO_PinSource_MOSI          GPIO_PinSource15
#define SPI                              SPI2
#define SPI_SPI_BaudRatePrescaler        SPI_BaudRatePrescaler_4 //!  default:SPI_BaudRatePrescaler_4 10.5<20MHZ, make sure < 20MHZ


//! SPI_FLASH CS  added by apple:12/05/2016
#define FLASH_RCC_AHB1Periph             RCC_AHB1Periph_GPIOB
#define FLASH_GPIO                       GPIOB
#define FLASH_GPIO_PIN_CS                GPIO_Pin_1  //!  PB.01 FLASH  chip choice signal
#define FLASH_GPIO_PinSource_CS          GPIO_PinSource1

//! SPI_HAPTIC CS
#define HAPTIC_RCC_AHB1Periph            RCC_AHB1Periph_GPIOB
#define HAPTIC_GPIO                      GPIOB
#define HAPTIC_GPIO_PIN_CS               GPIO_Pin_12 //! PB.12 HAPTIC choice signal
#define HAPTIC_GPIO_PIN_IQR              GPIO_Pin_0  //! PB.00 HAPTIC interrupt signal
#define HAPTIC_GPIO_PinSource_CS         GPIO_PinSource12
#define HAPTIC_GPIO_PinSource_IQR        GPIO_PinSource0

//! SPI_SDCARD CS
#define SD_RCC_AHB1Periph                RCC_AHB1Periph_GPIOA
#define SD_GPIO                          GPIOA
#define SD_GPIO_PIN_CS                   GPIO_Pin_15 //! PA.15 sdcard chip choice signal
#define SD_GPIO_PinSource_CS             GPIO_PinSource15

/****************************************************************************/






/****************************************************************************/
#if !defined(BOOT)
  #define SD_USE_DMA                    // Enable the DMA for SD
  #define SD_DMA_Stream_SPI_RX          DMA1_Stream3
  #define SD_DMA_Stream_SPI_TX          DMA1_Stream4
  #define SD_DMA_FLAG_SPI_TC_RX         DMA_FLAG_TCIF3
  #define SD_DMA_FLAG_SPI_TC_TX         DMA_FLAG_TCIF4
  #define SD_DMA_Channel_SPI            DMA_Channel_0
#endif
/****************************************************************************/






/****************************************************************************/
//! Heartbeat
#define HEARTBEAT_RCC_AHB1Periph        RCC_AHB1Periph_GPIOC
#define HEARTBEAT_RCC_APB2Periph        RCC_APB2Periph_USART6
#define HEARTBEAT_GPIO_PIN              GPIO_Pin_7  //! PC.07
#define HEARTBEAT_GPIO_PinSource        GPIO_PinSource7
#define HEARTBEAT_GPIO_AF               GPIO_AF_USART6
#define HEARTBEAT_USART                 USART6
#define HEARTBEAT_USART_IRQHandler      USART6_IRQHandler
#define HEARTBEAT_USART_IRQn            USART6_IRQn

/****************************************************************************/






/****************************************************************************/
//! 5ms Interrupt
#define INTERRUPT_5MS_APB1Periph        RCC_APB1Periph_TIM14
#define INTERRUPT_5MS_TIMER             TIM14
#define INTERRUPT_5MS_IRQn              TIM8_TRG_COM_TIM14_IRQn
#define INTERRUPT_5MS_IRQHandler        TIM8_TRG_COM_TIM14_IRQHandler

/****************************************************************************/






/****************************************************************************/
//! 2MHz Timer
#define TIMER_2MHz_APB1Periph           RCC_APB1Periph_TIM7
#define TIMER_2MHz_TIMER                TIM7

#endif


//! Internal Module
#define INTMODULE_RCC_AHB1Periph        (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA2)
#define INTMODULE_RCC_APB2Periph        RCC_APB2Periph_TIM1

#if defined(REVPLUS)
  #define INTMODULE_GPIO_PWR            GPIOC
  #define INTMODULE_GPIO_PIN_PWR        GPIO_Pin_6
#else
  #define INTMODULE_GPIO_PWR            GPIOD
  #define INTMODULE_GPIO_PIN_PWR        GPIO_Pin_15
#endif

#define INTMODULE_GPIO_PIN              GPIO_Pin_10 //! PA.10
#define INTMODULE_GPIO                  GPIOA
#define INTMODULE_GPIO_PinSource        GPIO_PinSource10
#define INTMODULE_GPIO_AF               GPIO_AF_TIM1
#define INTMODULE_TIMER                 TIM1






//! Trainer Port
#define TRAINER_RCC_AHB1Periph          RCC_AHB1Periph_GPIOC
#define TRAINER_RCC_APB1Periph          RCC_APB1Periph_TIM3
#define TRAINER_GPIO                    GPIOC
#define TRAINER_GPIO_PIN_IN             GPIO_Pin_8  // PC.08
#define TRAINER_GPIO_PIN_OUT            GPIO_Pin_9  // PC.09
#define TRAINER_GPIO_DETECT             GPIOA
#define TRAINER_GPIO_PIN_DETECT         GPIO_Pin_8  // PA.08
#define TRAINER_TIMER                   TIM3
#define TRAINER_TIMER_IRQn              TIM3_IRQn
#define TRAINER_GPIO_PinSource_IN       GPIO_PinSource8
#define TRAINER_GPIO_AF                 GPIO_AF_TIM3


//! External Module
#define EXTMODULE_RCC_AHB1Periph        (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA2)
#define EXTMODULE_RCC_APB2Periph        RCC_APB2Periph_TIM8
#define EXTMODULE_GPIO_PWR              GPIOD
//#define EXTMODULE_GPIO_PIN_PWR          GPIO_Pin_8 //PD.8
//#define EXTMODULE_GPIO_PIN              GPIO_Pin_7  // PA.07
#define EXTMODULE_GPIO                  GPIOA
#define EXTMODULE_TIMER                 TIM8
#define EXTMODULE_GPIO_AF               GPIO_AF_TIM8
#define EXTMODULE_GPIO_PinSource        GPIO_PinSource7
#define EXTMODULE_TIMER_IRQn            TIM8_CC_IRQn





