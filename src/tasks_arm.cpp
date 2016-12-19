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

#include "opentx.h"
#include "global.h"
/****************************************************************************/
#define MENUS_STACK_SIZE       2000 //! define each task's stack size
#define MIXER_STACK_SIZE       1200 //! default 500
#define COMMN_STACK_SIZE       1200 //! default 500


#if defined(_MSC_VER)
  #define _ALIGNED(x) __declspec(align(x))
#elif defined(__GNUC__)
  #define _ALIGNED(x) __attribute__ ((aligned(x)))
#endif

//! define tasks
OS_TID menusTaskId;
TaskStack<MENUS_STACK_SIZE> _ALIGNED(8) menusStack; //! menus stack must be aligned to 8 bytes otherwise printf for %f does not work!

OS_TID mixerTaskId;
TaskStack<MENUS_STACK_SIZE> mixerStack;

OS_TID commnTaskId;
TaskStack<COMMN_STACK_SIZE> commnStack;

//! define mutex id
OS_MutexID mixerMutex;

template<int SIZE>
void TaskStack<SIZE>::paint()
{
   for(uint32_t i=0; i<SIZE; i++) 
   {
       stack[i] = 0x55555555;
   }
}


template<int SIZE>
uint16_t TaskStack<SIZE>::size()
{
   return SIZE*4;
}


uint16_t getStackAvailable(void * address, uint16_t size)
{
   uint32_t * array = (uint32_t *)address;
   uint16_t i = 0;
   while (i < size && array[i] == 0x55555555) 
   {
      i++;
   }
   return i*4;
}


template<int SIZE>
uint16_t TaskStack<SIZE>::available()
{
    return getStackAvailable(stack, SIZE);
}


void stackPaint()
{  
   menusStack.paint();
   mixerStack.paint();
   commnStack.paint();
}


#if defined(CPUSTM32) && !defined(SIMU)
uint16_t stackSize()
{
  return ((unsigned char *)&_estack - (unsigned char *)&_main_stack_start) / 4;
}

uint16_t stackAvailable()
{
  return getStackAvailable(&_main_stack_start, stackSize());
}
#endif




/********************* mixerTask ********************************************/ 
void mixerTask(void *pdata)
{
  while(1) 
  { 
     CoEnterMutexSection(mixerMutex);
     doMixerCalculations();
     CoLeaveMutexSection(mixerMutex); 		
 
     CoTickDelay(3);  //! 6ms for now
  } 
}

 

/********************* menusTask *******************************************/
#define MENU_TASK_PERIOD_TICKS      10   

void menusTask(void *pdata)
{  
   opentxInit(); 
   
   while(1)
   {			
	  U64 start = CoGetOSTime();
	  perMain();		  
       
      displayTest();
	  uint8_t runtime = CoGetOSTime() - start;
#if defined APPLE_DEBUG
	  lcd_ShowNum(35, 30, WHITE, 18, 2, runtime, 0); 	
#endif
	  
	  if(runtime < MENU_TASK_PERIOD_TICKS) 
	  {
	     CoTickDelay(MENU_TASK_PERIOD_TICKS - runtime);
	  }
   }
   
   backLightEnable(0, 100); //! backlight disable
   opentxClose();
   boardOff(); //! Only turn power off if necessary 
} 



/********************** commnTask ******************************************/
#define COMN_TASK_PERIOD_TICKS      3  

void commnTask(void *pdata)
{  
  while(1) 
  { 
	  U64 start = CoGetOSTime(); 
	  uint8_t tempdata;	
      
	  if(!g_eeGeneral.firmwareUpdate) //! normal operate
	  {
		 if(comData.comStep<0)        //! it is usb plugged in
		 {
		    while(telemetryrxFifo.pop(tempdata)) usart1UsbSendChar(tempdata); //! data from uav to usb
		 }
		 else if(comData.comStep>0)   //! it is Raspberry Pi plugged in
		 {
		    while(telemetryrxFifo.pop(tempdata)) usart4RspSendChar(tempdata); //! data form uav to pai
		 }
		 else                         //! not sure which one 
		 {
		    while(telemetryrxFifo.pop(tempdata))
		    {
			   usart1UsbSendChar(tempdata); //! data from uav to usb
		       usart4RspSendChar(tempdata); //! data from uav to pai
		    }  		  
		 }
         
         while(usart1rxFifo.pop(tempdata)) usart2UavSendChar(tempdata); //! data form usb to uav
         while(usart4rxFifo.pop(tempdata)) usart2UavSendChar(tempdata); //! data form pai to uav				 
	  }	
      
	  if(g_eeGeneral.ftpReady) //! firmware update
      {
	     ftpProcess();
	     g_eeGeneral.ftpReady = 0;
      }	
	  
	  uint8_t runtime = CoGetOSTime() - start; 
#if defined APPLE_DEBUG
	  lcd_ShowNum(35, 60, WHITE, 18, 2, runtime, 0);    
#endif
	  if(runtime < COMN_TASK_PERIOD_TICKS) 
	  {
		 CoTickDelay(COMN_TASK_PERIOD_TICKS - runtime);
	  }		  
	   
	  //! CoTickDelay(2);//! 4ms for now 
  } 
}



void tasksStart()
{
  CoInitOS();
   
  mixerTaskId = CoCreateTask(mixerTask, NULL, 1, &mixerStack.stack[MIXER_STACK_SIZE-1], MIXER_STACK_SIZE);  //! default priority 5
  menusTaskId = CoCreateTask(menusTask, NULL, 2, &menusStack.stack[MENUS_STACK_SIZE-1], MENUS_STACK_SIZE);  //! default priority 10  
  commnTaskId = CoCreateTask(commnTask, NULL, 3, &commnStack.stack[COMMN_STACK_SIZE-1], COMMN_STACK_SIZE);  //! default priority 11  

  mixerMutex = CoCreateMutex();
  CoStartOS();
}







