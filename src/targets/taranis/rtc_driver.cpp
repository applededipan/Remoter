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



void rtcSetTime(struct gtm * t)
{
  RTC_TimeTypeDef RTC_TimeStruct;
  RTC_DateTypeDef RTC_DateStruct;

  RTC_TimeStructInit(&RTC_TimeStruct);
  RTC_DateStructInit(&RTC_DateStruct);

  RTC_TimeStruct.RTC_Hours = t->tm_hour;
  RTC_TimeStruct.RTC_Minutes = t->tm_min;
  RTC_TimeStruct.RTC_Seconds = t->tm_sec;
  RTC_DateStruct.RTC_Year = t->tm_year - 100;
  RTC_DateStruct.RTC_Month = t->tm_mon + 1;
  RTC_DateStruct.RTC_Date = t->tm_mday;
  
  RTC_SetTime(RTC_Format_BIN, &RTC_TimeStruct);
  RTC_SetDate(RTC_Format_BIN, &RTC_DateStruct);
}

void rtcGetTime(struct gtm * t)
{
  RTC_TimeTypeDef RTC_TimeStruct;
  RTC_DateTypeDef RTC_DateStruct;

  RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
  RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);
  
  t->tm_hour = RTC_TimeStruct.RTC_Hours;
  t->tm_min  = RTC_TimeStruct.RTC_Minutes;
  t->tm_sec  = RTC_TimeStruct.RTC_Seconds;
  t->tm_year = RTC_DateStruct.RTC_Year + 100;
  t->tm_mon  = RTC_DateStruct.RTC_Month - 1;
  t->tm_mday = RTC_DateStruct.RTC_Date;
}

void rtcInit()
{
   if(RTC_ReadBackupRegister(RTC_BKP_DR0) != 0)	
   {
      RTC_InitTypeDef RTC_InitStruct;
      //启用PWR和BKP的时钟
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);                        
      PWR_BackupAccessCmd(ENABLE);//使能RTC和后备寄存器访问
      RCC_LSEConfig(RCC_LSE_ON);  //设置外部低速晶振(LSE),使用外部低速晶振
      while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);
      RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);//设置RTC时钟，选择LSE为RTC时钟
      RCC_RTCCLKCmd(ENABLE);//使能RTC时钟
      RTC_WaitForSynchro(); //等待最近一次对RTC寄存器的写操作完成

      /* RTC time base = LSE / ((AsynchPrediv+1) * (SynchPrediv+1)) = 1 Hz */
      RTC_InitStruct.RTC_HourFormat = RTC_HourFormat_24;
      RTC_InitStruct.RTC_AsynchPrediv = 127;
      RTC_InitStruct.RTC_SynchPrediv = 255;
      RTC_Init(&RTC_InitStruct);

	  struct gtm t;
	  t.tm_min  = 59;
	  t.tm_hour = 18;
	  t.tm_mday = 30;
	  t.tm_mon  = 3;
	  g_rtcTime = gmktime(&t);
	  rtcSetTime(&t);	   
      RTC_WriteBackupRegister(RTC_BKP_DR0, 0);	   
   }
   else
   {
      struct gtm utm;   
      rtcGetTime(&utm);
      g_rtcTime = gmktime(&utm);	   
   }	   	   
}








