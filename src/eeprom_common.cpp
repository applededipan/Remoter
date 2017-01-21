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

#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include "opentx.h"
#include "timers.h"

uint8_t   s_eeDirtyMsk;
tmr10ms_t s_eeDirtyTime10ms;

void eeDirty(uint8_t msk)
{
  s_eeDirtyMsk |= msk;
  s_eeDirtyTime10ms = get_tmr10ms() ;
}

uint8_t eeFindEmptyModel(uint8_t id, bool down)
{
  uint8_t i = id;
  for (;;) {
    i = (MAX_MODELS + (down ? i+1 : i-1)) % MAX_MODELS;
    if (!eeModelExists(i)) break;
    if (i == id) return 0xff; // no free space in directory left
  }
  return i;
}

void selectModel(uint8_t sub)
{
#if !defined(COLORLCD)
  displayPopup(STR_LOADINGMODEL);
#endif
  saveTimers();
  eeCheck(true); // force writing of current model data before this is changed
  g_eeGeneral.currModel = sub;
  eeDirty(EE_GENERAL);
  eeLoadModel(sub);
}


#if defined(CPUARM)
ModelHeader modelHeaders[MAX_MODELS];
void eeLoadModelHeaders(void)
{
  for (uint32_t i=0; i<MAX_MODELS; i++) 
  {
      eeLoadModelHeader(i, &modelHeaders[i]);
  }
}
#endif


void eeReadAll(void)
{
    if (!eepromOpen() || !eeLoadGeneral()) 
	{
        eeErase(true);
    }
    else 
	{
        eeLoadModelHeaders();
    }

    stickMode = g_eeGeneral.stickMode;

#if defined(CPUARM)
    for (uint8_t i=0; languagePacks[i]!=NULL; i++) 
	{
        if (!strncmp(g_eeGeneral.ttsLanguage, languagePacks[i]->id, 2)) 
		{
           currentLanguagePackIdx = i;
           currentLanguagePack = languagePacks[i];
        }
    }
#endif

#if !defined(CPUARM)
  eeLoadModel(g_eeGeneral.currModel);
#endif
}




//! functions below are added by apple

/*******************************************************
 * @brief write a single joyScale data into eeprom   added by apple:02/07
*******************************************************/
void eeWriteSingleJoyScale(uint8_t index, uint16_t value)
{
   eepromWriteBlock((uint8_t*)&value, E_ADDR_JOYHOLD+(index*2), sizeof(value));
}


/*******************************************************
 * @brief read a single joyScale data from eeprom   added by apple:02/07
*******************************************************/
uint16_t eeReadSingleJoyScale(uint8_t index)
{
   uint16_t temp;
   eepromReadBlock((uint8_t*)&temp, E_ADDR_JOYHOLD+(index*2), sizeof(temp));
   return temp;   
}


/*******************************************************
 * @brief write joyScale datas into eeprom   added by apple:02/07
*******************************************************/
//! STICK SCALE : --> 200 -->  850 --> 1050 --> 1800 -->
//! TRIM  SCALE : --> 850 --> 1000 --> 1050 --> 1200 -->
//!
//!               anaIn(4)                                                        anaIn(5)
//!   *** H ******************** L ***                                *** H ******************** L ***
//!     (1200)                 ( 850)                                   (1200)                 ( 850)


//!               anaIn(1)                                                        anaIn(2)
//!                  *                                                               *
//!                  *                                                               *
//!                  *                                                               *
//!                  L ( 200)                                                        H (1800) 
//!                  *                                                               *
//!                  *                                                               *
//!                  *                                                               *
//!                 ***                                                             ***
//!   *** H ******************** L ***    anaIn(0)                     *** L ******************* H ***     anaIn(3)
//!     (1800)      ***        ( 200)                                  ( 200)       ***       (1800) 
//!                  *                                                               *
//!                  *                                                               *
//!                  *                                                               *
//!                  H (1800)                                                        L ( 200) 
//!                  *                                                               *
//!                  *                                                               *
//!                  *                                                               *


void eeWriteJoyScale(void)
{
    uint16_t joyScale[50] = {0};
    joyScale[0]  = g_eeGeneral.joyscale.threshold;	
	
	joyScale[1]  = g_eeGeneral.joyscale.ele_max;
	joyScale[2]  = g_eeGeneral.joyscale.ele_cen;
	joyScale[3]  = g_eeGeneral.joyscale.ele_min;
    
	joyScale[4]  = g_eeGeneral.joyscale.rud_max;
	joyScale[5]  = g_eeGeneral.joyscale.rud_cen;
	joyScale[6]  = g_eeGeneral.joyscale.rud_min;
    
	joyScale[7]  = g_eeGeneral.joyscale.thr_max;
	joyScale[8]  = g_eeGeneral.joyscale.thr_cen;
	joyScale[9]  = g_eeGeneral.joyscale.thr_min;
    
	joyScale[10] = g_eeGeneral.joyscale.ail_max;
	joyScale[11] = g_eeGeneral.joyscale.ail_cen;
	joyScale[12] = g_eeGeneral.joyscale.ail_min;
    
	joyScale[13] = g_eeGeneral.joyscale.ltrm_max;
	joyScale[14] = g_eeGeneral.joyscale.ltrm_cen;
	joyScale[15] = g_eeGeneral.joyscale.ltrm_min;
    
	joyScale[16] = g_eeGeneral.joyscale.rtrm_max;
	joyScale[17] = g_eeGeneral.joyscale.rtrm_cen;
	joyScale[18] = g_eeGeneral.joyscale.rtrm_min;
	
	eepromWriteBlock((uint8_t*)joyScale, E_ADDR_JOYHOLD, sizeof(joyScale));	
	
	// uint16_t joyScale[50] = {0};
    // joyScale[0]  = 25;	
	
	// joyScale[1]  = 1900;
	// joyScale[2]  = 997;
	// joyScale[3]  = 50;
	
	// joyScale[4]  = 1900;
	// joyScale[5]  = 993;
	// joyScale[6]  = 95;
	
	// joyScale[7]  = 1870;
	// joyScale[8]  = 930;
	// joyScale[9]  = 50;
	
	// joyScale[10] = 1910;
	// joyScale[11] = 1010;
	// joyScale[12] = 100;
	
	// joyScale[13] = 1230;
	// joyScale[14] = 1020;
	// joyScale[15] = 810;
	
	// joyScale[16] = 1220;
	// joyScale[17] = 1020;
	// joyScale[18] = 815;
	
	// eepromWriteBlock((uint8_t*)joyScale, E_ADDR_JOYHOLD, sizeof(joyScale));	
}



/*******************************************************
 * @brief load joyScale info from eeprom to ram   added by apple:02/07
*******************************************************/
void eeReadJoyScale(void)
{
	uint16_t joyScale[50] = {0};
    eepromReadBlock((uint8_t*)joyScale, E_ADDR_JOYHOLD, sizeof(joyScale));	
	
	g_eeGeneral.joyscale.threshold = joyScale[0];
	
	g_eeGeneral.joyscale.ele_max = joyScale[1];  //! ele
	g_eeGeneral.joyscale.ele_cen = joyScale[2];
	g_eeGeneral.joyscale.ele_min = joyScale[3];
	
	g_eeGeneral.joyscale.rud_max = joyScale[4];  //! rud
	g_eeGeneral.joyscale.rud_cen = joyScale[5];
	g_eeGeneral.joyscale.rud_min = joyScale[6];
	
	g_eeGeneral.joyscale.thr_max = joyScale[7];  //! thr
	g_eeGeneral.joyscale.thr_cen = joyScale[8];
	g_eeGeneral.joyscale.thr_min = joyScale[9];
	
	g_eeGeneral.joyscale.ail_max = joyScale[10]; //! ail
	g_eeGeneral.joyscale.ail_cen = joyScale[11];
	g_eeGeneral.joyscale.ail_min = joyScale[12];
	
    g_eeGeneral.joyscale.ltrm_max = joyScale[13]; //! ltrm
	g_eeGeneral.joyscale.ltrm_cen = joyScale[14];
	g_eeGeneral.joyscale.ltrm_min = joyScale[15];
	
	g_eeGeneral.joyscale.rtrm_max = joyScale[16]; //! rtrm
	g_eeGeneral.joyscale.rtrm_cen = joyScale[17];
	g_eeGeneral.joyscale.rtrm_min = joyScale[18];		
}

