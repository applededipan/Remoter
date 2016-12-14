/*
 * Authors (alphabetical order)
 * - Andre Bernet <bernet.andre@gmail.com>
 * - Andreas Weitl
 * - Bertrand Songis <bsongis@gmail.com>
 * - Bryan J. Rentoul (Gruvin) <gruvin@gmail.com>
 * - Cameron Weeks <th9xer@gmail.com>
 * - Erez Raviv
 * - Gabriel Birkus
 * - Jean-Pierre Parisy
 * - Karl Szmutny
 * - Michael Blandford
 * - Michal Hlavinka
 * - Pat Mackenzie
 * - Philip Moss
 * - Rob Thomson
 * - Romolo Manfredini <romolo.manfredini@gmail.com>
 * - Thomas Husterer
 *
 * opentx is based on code named
 * gruvin9x by Bryan J. Rentoul: http://code.google.com/p/gruvin9x/,
 * er9x by Erez Raviv: http://code.google.com/p/er9x/,
 * and the original (and ongoing) project by
 * Thomas Husterer, th9x: http://code.google.com/p/th9x/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "../../opentx.h"

#if defined(__cplusplus) && !defined(SIMU)
extern "C" {
#endif
#include "STM32_USB-Host-Device_Lib_V2.1.0/Libraries/STM32_USB_OTG_Driver/inc/usb_dcd_int.h"
#include "STM32_USB-Host-Device_Lib_V2.1.0/Libraries/STM32_USB_OTG_Driver/inc/usb_bsp.h"
#if defined(__cplusplus) && !defined(SIMU)
}
#endif

void watchdogInit(unsigned int duration)
{
  IWDG->KR = 0x5555 ;      //! Unlock registers
  IWDG->PR = 3 ;           //! Divide by 32 => 1kHz clock
  IWDG->KR = 0x5555 ;      //! Unlock registers
  IWDG->RLR = duration ;   //! 1.5 seconds nominal
  IWDG->KR = 0xAAAA ;      //! reload
  IWDG->KR = 0xCCCC ;      //! start
}

//! Starts TIMER at 2MHz
void init2MhzTimer(void)
{
  TIMER_2MHz_TIMER->PSC = (PERI1_FREQUENCY * TIMER_MULT_APB1) / 2000000 - 1 ;    //! 0.5 uS, 2 MHz
  TIMER_2MHz_TIMER->ARR = 65535;
  TIMER_2MHz_TIMER->CR2 = 0;
  TIMER_2MHz_TIMER->CR1 = TIM_CR1_CEN;
}

//! Starts TIMER at 200Hz (5ms)
void init5msTimer(void)
{
  INTERRUPT_5MS_TIMER->ARR = 4999 ; //! 5mS
  INTERRUPT_5MS_TIMER->PSC = (PERI1_FREQUENCY * TIMER_MULT_APB1) / 1000000 - 1 ; //! 1uS from 30MHz
  INTERRUPT_5MS_TIMER->CCER = 0 ;
  INTERRUPT_5MS_TIMER->CCMR1 = 0 ;
  INTERRUPT_5MS_TIMER->EGR = 0 ;
  INTERRUPT_5MS_TIMER->CR1 = 5 ;
  INTERRUPT_5MS_TIMER->DIER |= 1 ;
  NVIC_EnableIRQ(INTERRUPT_5MS_IRQn) ;
  NVIC_SetPriority(INTERRUPT_5MS_IRQn, 7);
}

void stop5msTimer(void)
{
  INTERRUPT_5MS_TIMER->CR1 = 0 ; //! stop timer
  NVIC_DisableIRQ(INTERRUPT_5MS_IRQn) ;
}


void interrupt5ms(void)
{
  static uint32_t pre_scale ; //! Used to get 10 Hz counter

  if(++pre_scale >= 2) 
  {
    pre_scale = 0 ;
    per10ms();
  }

}

#if !defined(SIMU)
extern "C" void INTERRUPT_5MS_IRQHandler()
{
  INTERRUPT_5MS_TIMER->SR &= ~TIM_SR_UIF ;
  interrupt5ms() ;
}



/******************************************
 *@ brief init the sdcard, if failed return 1 and reinit the spibus device; if succeed return 0
******************************************/
void sdcardInit(uint8_t *sdCardError)
{
	if(sdInit())
	{
	    spiInit();  
        hapticInit();
        spiFlashInit();
        *sdCardError = 1; //! sdcard error			
	}
	else
	{
	    *sdCardError = 0; //! sdcard ok
	}
}




void boardInit()
{
  RCC_AHB1PeriphClockCmd(BACKLIGHT_RCC_AHB1Periph | POWER_RCC_AHB1Periph     | KEYS_RCC_AHB1Periph      |  
					     USART3_RCC_AHB1Periph    | LCD_RCC_AHB1Periph       | UART4_RCC_AHB1Periph     | 
						 ADC_RCC_AHB1Periph       | I2C_RCC_AHB1Periph       | USART1_RCC_AHB1Periph    | 
			             HAPTIC_RCC_AHB1Periph    | TELEMETRY_RCC_AHB1Periph | BEEP_RCC_AHB1Periph      |
						 LED_RCC_AHB1Periph       | EXTMODULE_RCC_AHB1Periph | 
						 SPI_RCC_AHB1Periph       | FLASH_RCC_AHB1Periph     | MOTOR_RCC_AHB1Periph     | 
						 HAPTIC_RCC_AHB1Periph    | SD_RCC_AHB1Periph        | HEARTBEAT_RCC_AHB1Periph, ENABLE); 						 
						 
  RCC_APB1PeriphClockCmd(BACKLIGHT_RCC_APB1Periph | UART4_RCC_APB1Periph     | USART3_RCC_APB1Periph    |  
                         I2C_RCC_APB1Periph       | INTERRUPT_5MS_APB1Periph | TIMER_2MHz_APB1Periph    |  
						 SPI_RCC_APB1Periph       | TELEMETRY_RCC_APB1Periph, ENABLE);
						 
  RCC_APB2PeriphClockCmd(ADC_RCC_APB2Periph       | USART1_RCC_APB2Periph    | HEARTBEAT_RCC_APB2Periph, ENABLE);
 
  RCC_AHB3PeriphClockCmd(LCD_FSMC_RCC_AHB3Periph ,ENABLE); //! FSMC  added by apple		
 
  delaysInit(); 
  init2MhzTimer();
  init5msTimer();
  __enable_irq();
  rtcInit();
  powerInit();
  powerOn(); 
  keysInit();
  ledsInit();
  beepInit();
  motorInit();
  adcInit();
  i2cInit();
  spiInit();  
  hapticInit();
  spiFlashInit();  
  usart3BthInit(115200);     //! connect to bluetooth
  usart2UavInit(115200);     //! connect to p900/uav 
  usart1UsbInit(115200);     //! connect to usb
  usart4RspInit(115200);     //! connect to Raspberry Pi
  sdcardInit(&g_eeGeneral.sdCardError);	  
  lcdInit();                 //! added by apple its location must be after delaysInit()  
  lcd_ClearScreen(BLACK);              
  displayCell(); 
  backLightEnable(100, 500); //! display big cell
  powerStartup();            //! check the eeprom flag first!!!
  backLightEnable(0, 0);
  displayStartmenu();        //! display startup window
  backLightEnable(100, 100);
  backLightEnable(0,  10000);
  lcd_ClearScreen(BACKCOLOR);

  spiFlashDownloadFont();
  
#if defined(DEBUG)
  DBGMCU_APB1PeriphConfig(DBGMCU_IWDG_STOP|DBGMCU_TIM1_STOP|DBGMCU_TIM2_STOP|DBGMCU_TIM3_STOP|DBGMCU_TIM6_STOP|DBGMCU_TIM8_STOP|DBGMCU_TIM10_STOP|DBGMCU_TIM13_STOP|DBGMCU_TIM14_STOP, ENABLE);
#endif
}

void boardOff()
{
  backLightEnable(0, 0);
  lcdOff();
  SysTick->CTRL = 0; //! turn off systick
  powerOff();
}

#endif //#if !defined(SIMU)


#if defined(USB_JOYSTICK) && !defined(SIMU)
extern USB_OTG_CORE_HANDLE USB_OTG_dev;

#endif 



