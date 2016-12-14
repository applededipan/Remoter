/****************************************************************************
 *  Copyright (c) 2014 by Michael Blandford. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the author nor the names of its contributors may
 *     be used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 *  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 *  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 *  SUCH DAMAGE.
 *
 ****************************************************************************
 * Other Authors:
 * - Andre Bernet
 * - Bertrand Songis
 * - Bryan J. Rentoul (Gruvin)
 * - Cameron Weeks
 * - Erez Raviv
 * - Jean-Pierre Parisy
 * - Karl Szmutny
 * - Michal Hlavinka
 * - Pat Mackenzie
 * - Philip Moss
 * - Rob Thomson
 * - Romolo Manfredini
 * - Thomas Husterer
 *
 ****************************************************************************/

/****************************************************************
  * @brief  Headers
****************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "board_taranis.h"
#include "eeprom_rlc.h"
#include "gui/Taranis/lcd.h"
#include "keys.h"
#include "sdcard.h"
#include "FatFs/ff.h"
#include "FatFs/diskio.h"
#include "translations/en.h"
#include "stamp-opentx.h"

#define BLOCK_LEN 4096


/****************************************************************
  * @brief  Local variables	
****************************************************************/
volatile uint8_t Tenms;
FIL      FlashFile;
UINT     BlockCount;

uint32_t firmwareAddress = FIRMWARE_ADDRESS;
uint32_t Block_buffer[1024];

uint32_t unlocked = 0;


void interrupt10ms(void)
{
  Tenms |= 1;			//! 10 mS has passed
}


void init10msTimer()
{
  TIM14->ARR = 9999;	//! 10mS
  TIM14->PSC = (PERI1_FREQUENCY * TIMER_MULT_APB1) / 1000000 - 1;	// 1uS from 12MHz
  TIM14->CCER = 0;
  TIM14->CCMR1 = 0;
  TIM14->EGR = 0;
  TIM14->CR1 = 5;
  TIM14->DIER |= 1;
  NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
}


extern "C" void TIM8_TRG_COM_TIM14_IRQHandler()
{
  TIM14->SR &= ~TIM_SR_UIF;
  interrupt10ms();
}


void writeFlashBlock()
{
  uint32_t blockOffset = 0;
  while(BlockCount) 
  {
    writeFlash((uint32_t *)firmwareAddress, &Block_buffer[blockOffset]);
    blockOffset += FLASH_PAGESIZE/4; //! 32-bit words
    firmwareAddress += FLASH_PAGESIZE;
    if (BlockCount > FLASH_PAGESIZE) 
	{
      BlockCount -= FLASH_PAGESIZE;
    }
    else 
	{
      BlockCount = 0;
    }
  }
}



/****************************************************************
  * @brief  Jump to App added by apple 24/05/2016
****************************************************************/
void jumpToApp(void)
{
  RCC->CSR |= RCC_CSR_RMVF;   //! clear the reset flags in RCC clock control & status register
  asm("mov.w	r1, #134217728");//! 0x8000000

  asm("add.w	r1, #32768");    //! 0x8000   

  asm("movw	r0, #60680");        //! 0xED08    
  
  asm("movt	r0, #57344");        //! 0xE000
  
  asm("str	r1, [r0, #0]");      //! Set the VTOR 

  asm("ldr	r0, [r1, #0]");      //! Stack pointer value
  
  asm("msr msp, r0");            //! Set it
  
  asm("ldr	r0, [r1, #4]");      //! Reset address
  
  asm("mov.w	r1, #1");
  
  asm("orr		r0, r1");        //! Set lsbit
  
  asm("bx r0");                  //! Execute application 	
}



/****************************************************************
  * @brief  Flash app code from sdcard to stm32 flash  added by apple 24/05/2016
****************************************************************/
int flashUpdate(void)
{
    FATFS FatFs;                     //! Work area (file system object) for logical drive 
    FRESULT fr;                      //! FatFs return code 
	firmwareAddress = FIRMWARE_ADDRESS + BOOTLOADER_SIZE;//! the addr of app
    memset(Block_buffer, 0, sizeof(Block_buffer));	
	uint32_t fileSize = 0;
    uint16_t step = 0;	
	
	f_mount(&FatFs, "0:", 0);	     //! Register work area to the default drive 

	if(!unlocked)                    //! unlock the flash
	{
		unlocked = 1;
		unlockFlash();
	}
	                                                                       
	if((fr = f_open(&FlashFile, "0:SmartConsole/firmware/opentx.bin", FA_READ)) != FR_OK)
	{	
		return (int)fr; //! Open a bin file, if failed, then jump to app
	}
	else
	{
	    fileSize = FlashFile.fsize - BOOTLOADER_SIZE; //! get app file size	
	}  
	
	if((fr = f_lseek(&FlashFile, BOOTLOADER_SIZE)) != FR_OK)  
	{
		return (int)fr; //! just copy the app's code NOT all, if failed, then jump to app		
	}
		
	for(;;) 
	{
		fr = f_read(&FlashFile, (BYTE *)Block_buffer, sizeof(Block_buffer), &BlockCount); 
		if (fr || BlockCount == 0) break; 
		writeFlashBlock();
		lcd_DrawFillRectangleDiff(90+6*step, 120, 5, 40, GREEN);
		step++;
		delayms(50);		
	}
    lockFlash();
	f_close(&FlashFile);
	f_mount(NULL, "0:", 0);
	
	uint8_t flag = 0x11;
	eepromWriteBlock(&flag, E_ADDR_UPDATE, 1);
	return (int)fr;			
}


/****************************************************************
  * @brief  to see if should update firmware  added by apple 27/05/2016
  
  * @brief  if sdcard init failed, not update and jump to app
  * @brief  if key "opentx_home" pushed down then update, or jump to app 
  * @brief  if E_ADDR_UPDATE'S number == 0x55 update, or jump to app
  * @brief  flag = 0x11, to let app know that just updated firmware and do not check power pin again, just run app
****************************************************************/
void doUpdate(void)
{
    if(sdInit()) jumpToApp();       
	uint8_t flag;
	eepromReadBlock(&flag, E_ADDR_UPDATE, 1);	     
    if(!((readKeys()&OPENTX_HOME)||(flag == 0x55))) jumpToApp();	
}


/****************************************************************
  * @brief  Bootloader main funciton start here     added by apple 24/05/2016
****************************************************************/
int main()
{  
  wdt_feed();
  RCC_AHB1PeriphClockCmd(POWER_RCC_AHB1Periph | KEYS_RCC_AHB1Periph | I2C_RCC_AHB1Periph       | 
                         SPI_RCC_AHB1Periph   | LCD_RCC_AHB1Periph  | BACKLIGHT_RCC_AHB1Periph |
                         LED_RCC_AHB1Periph   | SD_RCC_AHB1Periph, ENABLE);
						 
  RCC_APB1PeriphClockCmd(INTERRUPT_5MS_APB1Periph | I2C_RCC_APB1Periph | BACKLIGHT_RCC_APB1Periph |
                         SPI_RCC_APB1Periph, ENABLE);						 

  RCC_AHB3PeriphClockCmd(LCD_FSMC_RCC_AHB3Periph ,ENABLE);//! FSMC  added by apple
  
  delaysInit(); //! needed for lcdInit()
  __enable_irq();
  init10msTimer();   
  ledsInit();
  keysInit();  
  powerInit();
  delayms(200); //! must do! to let the capacitor discharge complete and RC will not startup automaticly!
  powerOn();
  i2cInit();
  spiInit();
  lcdInit();  
  delayms(100);
  doUpdate(); //! important function!!!
  lcd_ClearScreen(BLACK); 
  backLightEnable(100, 100);  
  
  while(1) 
  {
    wdt_feed();
    if(Tenms) sdPoll10ms(); //! every 10ms goto here 	   
    flashUpdate();	
    jumpToApp();
  }
  return 0;  
}
	
 	  









