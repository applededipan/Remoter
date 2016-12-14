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

#ifndef keys_h
#define keys_h


//added by apple
#define OPENTX_MANUAL  0x0001   //#1
#define OPENTX_AUTO    0x0002   //#2
#define OPENTX_HOME    0x0004   //#4
#define OPENTX_CHOSE   0x0008   //#8
#define OPENTX_ENTER   0x0010   //#16
#define OPENTX_POWER   0x0020   //#32
#define OPENTX_TL1     0x0080   //#128
#define OPENTX_TL2     0x0040   //#32
#define OPENTX_TR1     0x0100   //#256
#define OPENTX_TR2     0x0200   //#512




enum EnumKeys {	

  KEYS_MANUAL,//0 //added by apple
  KEYS_AUTO,  //1 //added by apple
  KEYS_HOME,  //2 //added by apple
  KEYS_CHOSE, //3 //added by apple
  KEYS_ENTER, //4 //added by apple
  KEYS_POWER, //5 //added by apple
  KEYS_TL1,   //6 //added by apple
  KEYS_TL2,   //7 //added by apple
  KEYS_TR1,   //8 //added by apple
  KEYS_TR2,   //9 //added by apple	
  
  NUM_KEYS, 
  
  KEY_MENU,  
  KEY_EXIT,  
  KEY_ENTER, 
  KEY_PAGE,  
  KEY_PLUS,  
  KEY_MINUS, 

  TRM_BASE,    
  
  SW_BASE=NUM_KEYS,
#if defined(PCBTARANIS)
  SW_SA0=SW_BASE,
  SW_SA1,
  SW_SA2,
  SW_SB0,
  SW_SB1,
  SW_SB2,
  SW_SC0,
  SW_SC1,
  SW_SC2,
  SW_SD0,
  SW_SD1,
  SW_SD2,
  SW_SE0,
  SW_SE1,
  SW_SE2,
  SW_SF0,
  SW_SF1,
  SW_SF2,
  SW_SG0,
  SW_SG1,
  SW_SG2,
  SW_SH0,
  SW_SH1,
  SW_SH2,
#endif
};


#define EVT_KEY_MASK(e)      ((e) & 0x1f)

#define _MSK_KEY_BREAK       0x20
#define _MSK_KEY_REPT        0x40
#define _MSK_KEY_FIRST       0x60
#define _MSK_KEY_LONG        0x80

#define EVT_KEY_BREAK(key)   ((key)|_MSK_KEY_BREAK)
#define EVT_KEY_FIRST(key)   ((key)|_MSK_KEY_FIRST)
#define EVT_KEY_REPT(key)    ((key)|_MSK_KEY_REPT)
#define EVT_KEY_LONG(key)    ((key)|_MSK_KEY_LONG)

#define IS_KEY_BREAK(evt)    (((evt)&0xe0) == _MSK_KEY_BREAK)
#define IS_KEY_FIRST(evt)    (((evt)&0xe0) == _MSK_KEY_FIRST)
#define IS_KEY_LONG(evt)     (((evt)&0xe0) == _MSK_KEY_LONG)
#define IS_KEY_REPT(evt)     (((evt)&0xe0) == _MSK_KEY_REPT)

#define EVT_ENTRY            0xbf
#define EVT_ENTRY_UP         0xbe
#define EVT_MENU_UP          0xbd

#if defined(PCBTARANIS)
  #define EVT_ROTARY_BREAK       EVT_KEY_BREAK(KEY_ENTER)
  #define EVT_ROTARY_LONG        EVT_KEY_LONG(KEY_ENTER)
  #define IS_ROTARY_LEFT(evt)   (evt==EVT_KEY_FIRST(KEY_MINUS) || evt==EVT_KEY_REPT(KEY_MINUS))
  #define IS_ROTARY_RIGHT(evt)  (evt==EVT_KEY_FIRST(KEY_PLUS)  || evt==EVT_KEY_REPT(KEY_PLUS))
  #define IS_ROTARY_UP(evt)     (evt==EVT_KEY_FIRST(KEY_PLUS)  || evt==EVT_KEY_REPT(KEY_PLUS))
  #define IS_ROTARY_DOWN(evt)   (evt==EVT_KEY_FIRST(KEY_MINUS) || evt==EVT_KEY_REPT(KEY_MINUS))
  #define IS_ROTARY_BREAK(evt)  (evt==EVT_KEY_BREAK(KEY_ENTER))
  #define IS_ROTARY_LONG(evt)   (evt==EVT_KEY_LONG(KEY_ENTER))
  #define IS_ROTARY_EVENT(evt)  (0)
  #define CASE_EVT_ROTARY_BREAK /*case EVT_KEY_BREAK(KEY_ENTER):*/
  #define CASE_EVT_ROTARY_LONG  /*case EVT_KEY_LONG(KEY_ENTER): */
  #define CASE_EVT_ROTARY_LEFT  case EVT_KEY_FIRST(KEY_MOVE_DOWN): case EVT_KEY_REPT(KEY_MOVE_DOWN):
  #define CASE_EVT_ROTARY_RIGHT case EVT_KEY_FIRST(KEY_MOVE_UP): case EVT_KEY_REPT(KEY_MOVE_UP):
#endif

class Key
{
#define FILTERBITS      4
#define FFVAL          ((1<<FILTERBITS)-1)

#define KSTATE_OFF      0
#define KSTATE_RPTDELAY 95 // gruvin: delay state before key repeating starts
#define KSTATE_START    97
#define KSTATE_PAUSE    98
#define KSTATE_KILLED   99

  private:
    uint8_t m_vals;   // key debounce?  4 = 40ms
    uint8_t m_cnt;
    uint8_t m_state;
  public:
    void input(bool val);
    void pauseEvents() { m_state = KSTATE_PAUSE; m_cnt = 0; }
    void killEvents()  { m_state = KSTATE_KILLED; }
    bool state()       { return m_vals > 0; }	
    EnumKeys key() const;
};

extern Key keys[NUM_KEYS];

extern uint8_t s_evt;

#define putEvent(evt) s_evt = evt

void pauseEvents(uint8_t enuk);
void killEvents(uint8_t enuk);

#if defined(CPUARM)
  bool    clearKeyEvents();
  uint8_t getEvent(bool trim=false);
#endif

uint32_t keyDown();
void keyprocess(void);
#endif
