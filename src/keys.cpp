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

uint8_t s_evt;
struct t_inactivity inactivity = {0};

#if defined(CPUARM)
uint8_t getEvent(bool trim)
{
  uint8_t evt = s_evt;
  int8_t k = EVT_KEY_MASK(s_evt) - TRM_BASE;
  bool trim_evt = (k>=0 && k<8);

  if (trim == trim_evt) 
  {
    s_evt = 0;
    return evt;
  }
  else 
  {
    return 0;
  }
}
#endif

#define KEY_LONG_DELAY 32

Key keys[NUM_KEYS];

void Key::input(bool val)
{
  uint8_t t_vals = m_vals ;
  t_vals <<= 1 ;
  if (val) t_vals |= 1; //portbit einschieben
  m_vals = t_vals ;

  m_cnt++;

  if (m_state && m_vals==0) 
  {  
    //gerade eben sprung auf 0
    if (m_state != KSTATE_KILLED) 
	{
      putEvent(EVT_KEY_BREAK(key()));
    }
    m_cnt   = 0;
    m_state = KSTATE_OFF;
  }
  switch(m_state)
  {
    case KSTATE_OFF:
      if (m_vals == FFVAL) 
	  { 
        //gerade eben sprung auf ff
        m_state = KSTATE_START;
        m_cnt   = 0;
      }
      break;
      //fallthrough
    case KSTATE_START:
      putEvent(EVT_KEY_FIRST(key()));
      inactivity.counter = 0;
      m_state   = KSTATE_RPTDELAY;
      m_cnt     = 0;
      break;

    case KSTATE_RPTDELAY: // gruvin: delay state before first key repeat
      if (m_cnt == KEY_LONG_DELAY) 
	  {
        putEvent(EVT_KEY_LONG(key()));
      }
      if (m_cnt == 40) 
	  {
        m_state = 16;
        m_cnt = 0;
      }
      break;

    case 16:
    case 8:
    case 4:
    case 2:
      if (m_cnt >= 48)  
	  { 
        //3 6 12 24 48 pulses in every 480ms
        m_state >>= 1;
        m_cnt     = 0;
      }
      // no break
    case 1:
      if ((m_cnt & (m_state-1)) == 0) 
	  {
        putEvent(EVT_KEY_REPT(key()));
      }
      break;

    case KSTATE_PAUSE: //pause
      if (m_cnt >= 64)      
	  {
        m_state = 8;
        m_cnt   = 0;
      }
      break;

    case KSTATE_KILLED: //killed
      break;
  }
}

EnumKeys Key::key() const 
{ 
  return static_cast<EnumKeys>(this - keys);
}

void pauseEvents(uint8_t event)
{
  event = EVT_KEY_MASK(event);
  if (event < (int)DIM(keys)) keys[event].pauseEvents();
}

void killEvents(uint8_t event)
{
#if defined(ROTARY_ENCODER_NAVIGATION)
  if (event == EVT_ROTARY_LONG) 
  {
    killEvents(BTN_REa + NAVIGATION_RE_IDX());
  }
  else
#endif
  {
    event = EVT_KEY_MASK(event);
    if (event < (int)DIM(keys)) keys[event].killEvents();
  }
}

#if defined(CPUARM)
bool clearKeyEvents()
{
  // loop until all keys are up
  #if !defined(BOOT)
  tmr10ms_t start = get_tmr10ms();
  #endif

  while (keyDown()) 
  {
    wdt_feed();
    #if !defined(BOOT)
    if ((get_tmr10ms() - start) >= 300) 
	{  
      // wait no more than 3 seconds
      //timeout expired, at least one key stuck
      return false;
    }
    #endif
  }

  memclear(keys, sizeof(keys));
  putEvent(0);
  return true;
}



////////////////////////////////////////////////
//! added by apple
void keyprocess(void)
{
	static uint32_t keyLast;
    static uint8_t  timeLast;
	static uint8_t  keyflag=0;//该键是否按下且一直没有释放
	
	if(keyDown())//有键按下
	{
       if(keyLast != keyDown())//第一次按下
	   {
		   timeLast = g_ms100; //记录下第一次按下时间
		   keyLast = keyDown();//记录下按键值			   
	   }
	   else//不是第一次按下
	   {
		   if(keyflag == 0)
		   {
		     if((g_ms100-timeLast)>= 2)//20ms去抖
		     {
			   g_eeGeneral.key = keyLast;
			   timeLast = g_ms100;
			   keyflag = 1;
		     }
		     else if((g_ms100-timeLast)<0 && (g_ms100-timeLast+100)>=2)//20ms去抖
		     {
			   g_eeGeneral.key = keyLast;
			   timeLast = g_ms100;
			   keyflag = 1;
		     }				 
		   }
		   else{return;}				   
	   }		
	}
    else
	{
		g_eeGeneral.key = 0;
		keyLast = 0;
		keyflag = 0;
	}		
}




#endif   // #if defined(CPUARM)


