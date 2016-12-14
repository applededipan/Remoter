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


#define HAPTIC_RDX           0XD0
#define HAPTIC_RDY           0X90

#define HAPTIC_DESELECT()    GPIO_SetBits(HAPTIC_GPIO,   HAPTIC_GPIO_PIN_CS)
#define HAPTIC_SELECT()      GPIO_ResetBits(HAPTIC_GPIO, HAPTIC_GPIO_PIN_CS)


void hapticInit(void)
{
  /* Configure I/O for Haptic Chip select */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = HAPTIC_GPIO_PIN_CS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(HAPTIC_GPIO, &GPIO_InitStructure);
  
  HAPTIC_DESELECT(); 
}


void hapticOn(void)
{
  HAPTIC_SELECT();	
}


void hapticOff(void)
{
  HAPTIC_DESELECT();	
}

/************************************************
    func: 读取一个坐标值
     cmd: 坐标	 
  return: 返回坐标值
************************************************/
static uint16_t hapticRead(uint8_t cmd)
{
	uint16_t coordinate;
	HAPTIC_SELECT();
    spiReadWriteByte(cmd);
	delay_01us(30);
	coordinate = spiReadWriteByte(0);
	coordinate <<= 8;
	coordinate |= spiReadWriteByte(0);
	coordinate >>= 4;
	HAPTIC_DESELECT();	
	
	return coordinate&0x0fff;
}


static uint16_t hapticFilterRead(uint8_t cmd)
{
    uint16_t i, j;
    uint16_t buf[15];
    uint16_t sum=0;
    uint16_t temp;
    for(i=0;i<15;i++)
    {                
        buf[i]=hapticRead(cmd);     
    }                   
    for(i=0;i<14; i++)//排序
    {
        for(j=i+1;j<15;j++)
        {
            if(buf[i]>buf[j])//升序排列
            {
                temp=buf[i];
                buf[i]=buf[j];
                buf[j]=temp;
            }
        }
    }     
    sum=0;
    for(i=5;i<10;i++)sum+=buf[i];
    temp=sum/(15-2*5);
    return temp;   
} 



/************************************************
    func: 读取X,Y的坐标值
     x,y: 读到的坐标值	 
  return: 1：成功 0：失败
************************************************/
uint8_t hapticReadXY(uint16_t *x, uint16_t *y)
{
	uint16_t xtemp = hapticFilterRead(HAPTIC_RDX);
	uint16_t ytemp = hapticFilterRead(HAPTIC_RDY);
	*x = xtemp;
	*y = ytemp;
	
	return 1;
}



















