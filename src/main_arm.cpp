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
#include "telemetry/mavlink.h"


uint8_t requiredSpeakerVolume = 255;
uint8_t requestScreenshot = false;

void checkEeprom()
{
  if (!usbPlugged()) 
  {
    if (eepromIsWriting())     eepromWriteProcess();
    else if(TIME_TO_WRITE())   eeCheck(false);
  }
}


bool inPopupMenu = false;


void perMain()
{
  // annotated by apple
  // checkEeprom();
  // writeLogs();
  // checkBattery();

	 	
/********** added by apple ********************/ 
	convert_attitude();
   // displayNavigation(); 
   // displayAttitude(mavData.attitude.pitch, mavData.attitude.roll, mavData.attitude.yaw, mavData.hud.alt, mavData.sysStatus.volBat);
   // displayJoystick(g_rcChannel); //!显示手操杆

/**********************************************/  
//annotated by apple
/*   
#if defined(LUA)
  // TODO better lua stopwatch
  uint32_t t0 = get_tmr10ms();
  static uint32_t lastLuaTime = 0;
  uint16_t interval = (lastLuaTime == 0 ? 0 : (t0 - lastLuaTime));
  lastLuaTime = t0;
  if (interval > maxLuaInterval) 
  {
    maxLuaInterval = interval;
  }

  // run Lua scripts that don't use LCD (to use CPU time while LCD DMA is running)
  luaTask(0, RUN_MIX_SCRIPT | RUN_FUNC_SCRIPT | RUN_TELEM_BG_SCRIPT, false);

  t0 = get_tmr10ms() - t0;
  if (t0 > maxLuaDuration) 
  {
    maxLuaDuration = t0;
  }
#endif //#if defined(LUA)

  // wait for LCD DMA to finish before continuing, because code from this point 
  // is allowed to change the contents of LCD buffer
  // WARNING: make sure no code above this line does any change to the LCD display buffer!
  //


  // get event
  uint8_t evt;
  if(menuEvent) 
  {
      // we have a popupMenuActive entry or exit event 
      menuVerticalPosition = (menuEvent == EVT_ENTRY_UP) ? menuVerticalPositions[menuLevel] : 0;
      menuHorizontalPosition = 0;
      evt = menuEvent;
      if (menuEvent == EVT_ENTRY_UP) 
	  {
         TRACE("menuEvent EVT_ENTRY_UP");
      }
      else if (menuEvent == EVT_MENU_UP)
	  {
         TRACE("menuEvent EVT_MENU_UP");
      }
      else if (menuEvent == EVT_ENTRY) 
	  {
         TRACE("menuEvent EVT_ENTRY");
      }
      else 
	  {
         TRACE("menuEvent 0x%02x", menuEvent);
      }
      menuEvent = 0;
      AUDIO_MENUS();
   }
   else 
   {
      evt = getEvent(false);
      if (evt && (g_eeGeneral.backlightMode & e_backlight_mode_keys)) backlightOn(); // on keypress turn the light on
      checkBacklight();
   }

  if (warningText) 
  {
    // show warning on top of the normal menus
    handleGui(0); // suppress events, they are handled by the warning
    DISPLAY_WARNING(evt);
  }
  else if (popupMenuNoItems > 0) 
  {
     // popup menu is active display it on top of normal menus 
     handleGui(0); // suppress events, they are handled by the popup
     if (!inPopupMenu) 
	 {
        TRACE("Popup Menu started");
        inPopupMenu = true;
     }
     const char * result = displayPopupMenu(evt);
     if (result) 
	 {
         TRACE("popupMenuHandler(%s)", result);
         popupMenuHandler(result);
         putEvent(EVT_MENU_UP);
      // todo should we handle this event immediately??
      // handleGui(EVT_MENU_UP)
     }
  }
  else 
  {
    // normal menus
     if (inPopupMenu)
	 {
         TRACE("Popup Menu ended");
         inPopupMenu = false;
     }
     handleGui(evt);
  }

  lcdRefresh();


#if defined(PCBTARANIS)
  if (requestScreenshot) 
  {
      requestScreenshot = false;
      writeScreenshot();
  }
#endif 
*/


}
