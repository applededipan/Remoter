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
#include "timers.h"



#if defined(COLORLCD)
#elif defined(PCBTARANIS)
  const pm_uchar asterisk_lbm[] PROGMEM = {
    #include "bitmaps/Taranis/asterisk.lbm"
  };
#endif

EEGeneral  g_eeGeneral;
ModelData  g_model;
RC_CHANNEL g_rcChannel;

#if defined(SDCARD)
Clipboard clipboard;
#endif

#if defined(PCBTARANIS) && defined(SDCARD)
uint8_t modelBitmap[MODEL_BITMAP_SIZE];
void loadModelBitmap(char *name, uint8_t *bitmap)
{
  uint8_t len = zlen(name, LEN_BITMAP_NAME);
  if (len > 0) {
    char lfn[] = BITMAPS_PATH "/xxxxxxxxxx.bmp";
    strncpy(lfn+sizeof(BITMAPS_PATH), name, len);
    strcpy(lfn+sizeof(BITMAPS_PATH)+len, BITMAPS_EXT);
    if (bmpLoad(bitmap, lfn, MODEL_BITMAP_WIDTH, MODEL_BITMAP_HEIGHT) == 0) {
      return;
    }
  }

#if !defined(COLORLCD)
  // In all error cases, we set the default logo
  memcpy(bitmap, logo_taranis, MODEL_BITMAP_SIZE);
#endif
}
#endif


uint8_t unexpectedShutdown = 0;

uint8_t stickMode;

#if defined(OVERRIDE_CHANNEL_FUNCTION)
safetych_t safetyCh[NUM_CHNOUT];
#endif

union ReusableBuffer reusableBuffer;

const pm_uint8_t bchout_ar[] PROGMEM = {
    0x1B, 0x1E, 0x27, 0x2D, 0x36, 0x39,
    0x4B, 0x4E, 0x63, 0x6C, 0x72, 0x78,
    0x87, 0x8D, 0x93, 0x9C, 0xB1, 0xB4,
    0xC6, 0xC9, 0xD2, 0xD8, 0xE1, 0xE4 };

uint8_t channel_order(uint8_t x)
{
  return ( ((pgm_read_byte(bchout_ar + g_eeGeneral.templateSetup) >> (6-(x-1) * 2)) & 3 ) + 1 );
}


//! mode1 rud ele thr ail
//! mode2 rud thr ele ail
//! mode3 ail ele thr rud
//! mode4 ail thr ele rud

const pm_uint8_t modn12x3[] PROGMEM = {
    0, 1, 2, 3,
    0, 2, 1, 3,
    3, 1, 2, 0,
    3, 2, 1, 0 };

volatile tmr10ms_t g_tmr10ms;

#if defined(CPUARM)
volatile uint8_t rtc_count = 0;
uint32_t watchdogTimeout = 0;

void watchdogSetTimeout(uint32_t timeout)
{
  watchdogTimeout = timeout;
}
#endif






//! execute this function per 10ms
void per10ms()
{  
  g_tmr10ms++;
  if(++g_ms100 == 100)           //! Update global Date/Time every 100 per10ms cycles 
  {
    g_rtcTime++;
    g_ms100 = 0;
	wdt_feed();                  //! feed watchdog per 1000ms added by apple 26/07	
  }
  sdPoll10ms();                         //! must be called per 10ms for sd card    
  getBatVoltage(&g_eeGeneral.vBattery); //! get the voltage of RC battery just put it here to make sure run per 10ms
}



/*******************************************************
 * @ 10ms system timer  added by apple 29/08/2016
*******************************************************/
OS_TCID systemTimer_10ms;  
void systemTimer_10msCallBack(void)
{
#if defined (VTOL_MODE_CONTROL)
  setVtolMode(OPENTX_TR2);    //! vtol mode control  use channel 8
#elif defined (THROW_CONTROL)	
  setThrowMode(OPENTX_TR2);   //! throw mode control  use channel 8
#endif	
  
  mavlinkSendMessage();       //! send messages to uav
  
  displayNavigation();
  
  displayAttitude(mavData.attitude.pitch, mavData.attitude.roll, mavData.attitude.yaw, mavData.hud.alt, mavData.sysStatus.volBat);
  
  view_information(true);
  
  menusProcess(OPENTX_CHOSE, OPENTX_ENTER);  

}



/*******************************************************
 * @ 100ms system timer  added by apple 29/08/2016
*******************************************************/
OS_TCID systemTimer_100ms; //! 100ms timer / 10Hz
void systemTimer_100msCallBack(void)
{
  powerLowWarn(240, 2, 10, OPENTX_POWER);	
  
  powerLowShutdown(5);                                   //! if bat is lower than 5, then shutdown the system 
 
  if(mavData.mavStatus.health < 30)
  {
	 mavData.mavStatus.health++;
	 mavData.radioStatus.rssi = 99 - 3.3*mavData.mavStatus.health;		 
     if(mavData.mavStatus.health == 30) mavlinkReset(); //! must be 30	  
  }	

  if(mavData.mavStatus.raspiHealth++ > 100) mavData.mavStatus.pdlState = 0;  //! check if raspi comn is ok or not 
}




FlightModeData *flightModeAddress(uint8_t idx)
{
  return &g_model.flightModeData[idx];
}

ExpoData *expoAddress(uint8_t idx )
{
  return &g_model.expoData[idx];
}

MixData *mixAddress(uint8_t idx)
{
  return &g_model.mixData[idx];
}

LimitData *limitAddress(uint8_t idx)
{
  return &g_model.limitData[idx];
}

#if defined(CPUM64)
void memclear(void *ptr, uint8_t size)
{
  memset(ptr, 0, size);
}
#endif

void generalDefault()
{
  memclear(&g_eeGeneral, sizeof(g_eeGeneral));
  g_eeGeneral.version  = EEPROM_VER;
  g_eeGeneral.variant = EEPROM_VARIANT;
  g_eeGeneral.contrast = 25;

#if defined(PCBTARANIS)
  g_eeGeneral.potsConfig = 0x05;    // S1 and S2 = pots with detent
  g_eeGeneral.slidersConfig = 0x03; // LS and RS = sliders with detent
#endif

#if defined(PCBTARANIS)
  g_eeGeneral.switchConfig = 0x00007bff; // 6x3POS, 1x2POS, 1xTOGGLE
#endif

#if defined(PCBTARANIS)
  // NI-MH 7.2V
  g_eeGeneral.vBatWarn = 65;
  g_eeGeneral.vBatMin = -30;
  g_eeGeneral.vBatMax = -40;
#endif

#if defined(DEFAULT_MODE)
  g_eeGeneral.stickMode = DEFAULT_MODE-1;
#endif

#if defined(PCBTARANIS)
  g_eeGeneral.templateSetup = 17; /* TAER */
#endif

#if !defined(CPUM64)
  g_eeGeneral.backlightMode = e_backlight_mode_all;
  g_eeGeneral.lightAutoOff = 2;
  g_eeGeneral.inactivityTimer = 10;
#endif

#if defined(CPUARM)
  g_eeGeneral.wavVolume = 2;
  g_eeGeneral.backgroundVolume = 1;
#endif

#if defined(CPUARM)
  for (int i=0; i<NUM_STICKS; ++i) {
    g_eeGeneral.trainer.mix[i].mode = 2;
    g_eeGeneral.trainer.mix[i].srcChn = channel_order(i+1) - 1;
    g_eeGeneral.trainer.mix[i].studWeight = 100;
  }
#endif

  g_eeGeneral.chkSum = 0xFFFF;
}

uint16_t evalChkSum()
{
  uint16_t sum = 0;
  const int16_t *calibValues = (const int16_t *) &g_eeGeneral.calib[0];
  for (int i=0; i<12; i++)
    sum += calibValues[i];
  return sum;
}

#if defined(VIRTUALINPUTS)
void clearInputs()
{
  memset(g_model.expoData, 0, sizeof(g_model.expoData)); // clear all expos
}

void defaultInputs()
{
  clearInputs();

  for (int i=0; i<NUM_STICKS; i++) {
    uint8_t stick_index = channel_order(i+1);
    ExpoData *expo = expoAddress(i);
    expo->srcRaw = MIXSRC_Rud - 1 + stick_index;
    expo->curve.type = CURVE_REF_EXPO;
    expo->chn = i;
    expo->weight = 100;
    expo->mode = 3; // TODO constant
#if defined(TRANSLATIONS_CZ)
    for (int c=0; c<4; c++) {
      g_model.inputNames[i][c] = char2idx(STR_INPUTNAMES[1+4*(stick_index-1)+c]);
    }
    g_model.inputNames[i][4] = '\0';
#else
    for (int c=0; c<3; c++) {
      g_model.inputNames[i][c] = char2idx(STR_VSRCRAW[2+4*stick_index+c]);
    }
    g_model.inputNames[i][3] = '\0';
#endif
  }
  eeDirty(EE_MODEL);
}
#endif

#if defined(TEMPLATES)
inline void applyDefaultTemplate()
{
  applyTemplate(TMPL_SIMPLE_4CH); // calls eeDirty internally
}
#else
void applyDefaultTemplate()
{
#if defined(VIRTUALINPUTS)
  defaultInputs(); // calls eeDirty internally
#else
  eeDirty(EE_MODEL);
#endif

  for (int i=0; i<NUM_STICKS; i++) {
    MixData *mix = mixAddress(i);
    mix->destCh = i;
    mix->weight = 100;
#if defined(VIRTUALINPUTS)
    mix->srcRaw = i+1;
#else
    mix->srcRaw = MIXSRC_Rud - 1 + channel_order(i+1);
#endif
  }
}
#endif

#if defined(CPUARM)
void checkModelIdUnique(uint8_t index, uint8_t module)
{
  uint8_t modelId = g_model.header.modelId[module];
  if (modelId != 0) {
    for (uint8_t i=0; i<MAX_MODELS; i++) {
      if (i != index) {
        for (uint8_t j=0; j<NUM_MODULES; j++) {
          if (modelId == modelHeaders[i].modelId[j]) {
            POPUP_WARNING(STR_MODELIDUSED);
            return;
          }
        }
      }
    }
  }
}
#endif

#if defined(SDCARD)
bool isFileAvailable(const char * filename)
{
  return f_stat(filename, 0) == FR_OK;
}
#endif

void modelDefault(uint8_t id)
{
  memset(&g_model, 0, sizeof(g_model));

  applyDefaultTemplate();

#if defined(LUA)
  if (isFileAvailable(WIZARD_PATH "/" WIZARD_NAME)) {
    f_chdir(WIZARD_PATH);
    luaExec(WIZARD_NAME);
  }
#endif

#if defined(PCBTARANIS)
  g_model.moduleData[INTERNAL_MODULE].type = MODULE_TYPE_XJT;
#endif

#if defined(CPUARM)
  for (int i=0; i<NUM_MODULES; i++) {
    modelHeaders[id].modelId[i] = g_model.header.modelId[i] = id+1;
  }
  checkModelIdUnique(id, 0);
#endif

#if defined(CPUARM) && defined(FLIGHT_MODES) && defined(GVARS)
  for (int p=1; p<MAX_FLIGHT_MODES; p++) {
    for (int i=0; i<MAX_GVARS; i++) {
      g_model.flightModeData[p].gvars[i] = GVAR_MAX+1;
    }
  }
#endif

#if defined(MAVLINK)
  g_model.mavlink.rc_rssi_scale = 15;
  g_model.mavlink.pc_rssi_en = 1;
#endif
}

#if defined(VIRTUALINPUTS)
bool isInputRecursive(int index)
{
  ExpoData * line = expoAddress(0);
  for (int i=0; i<MAX_EXPOS; i++, line++) {
    if (line->chn > index)
      break;
    else if (line->chn < index)
      continue;
    else if (line->srcRaw >= MIXSRC_FIRST_LOGICAL_SWITCH)
      return true;
  }
  return false;
}
#endif

#if defined(AUTOSOURCE)
int8_t getMovedSource(GET_MOVED_SOURCE_PARAMS)
{
  int8_t result = 0;
  static tmr10ms_t s_move_last_time = 0;

#if defined(VIRTUALINPUTS)
  static int16_t inputsStates[MAX_INPUTS];
  if (min <= MIXSRC_FIRST_INPUT) {
    for (uint8_t i=0; i<MAX_INPUTS; i++) {
      if (abs(anas[i] - inputsStates[i]) > 512) {
        if (!isInputRecursive(i)) {
          result = MIXSRC_FIRST_INPUT+i;
          break;
        }
      }
    }
  }
#endif

  static int16_t sourcesStates[NUM_STICKS+NUM_POTS];
  if (result == 0) {
    for (uint8_t i=0; i<NUM_STICKS+NUM_POTS; i++) {
      if (abs(calibratedStick[i] - sourcesStates[i]) > 512) {
        result = MIXSRC_Rud+i;
        break;
      }
    }
  }

  bool recent = ((tmr10ms_t)(get_tmr10ms() - s_move_last_time) > 10);
  if (recent) {
    result = 0;
  }

  if (result || recent) {
#if defined(VIRTUALINPUTS)
    memcpy(inputsStates, anas, sizeof(inputsStates));
#endif
    memcpy(sourcesStates, calibratedStick, sizeof(sourcesStates));
  }

  s_move_last_time = get_tmr10ms();
  return result;
}
#endif

#if defined(FLIGHT_MODES)
uint8_t getFlightMode()
{
  for (uint8_t i=1; i<MAX_FLIGHT_MODES; i++) {
    FlightModeData *phase = &g_model.flightModeData[i];
    if (phase->swtch && getSwitch(phase->swtch)) {
      return i;
    }
  }
  return 0;
}
#endif

trim_t getRawTrimValue(uint8_t phase, uint8_t idx)
{
  FlightModeData *p = flightModeAddress(phase);
#if defined(PCBSTD)
  return (((trim_t)p->trim[idx]) << 2) + ((p->trim_ext >> (2*idx)) & 0x03);
#else
  return p->trim[idx];
#endif
}

int getTrimValue(uint8_t phase, uint8_t idx)
{
#if defined(VIRTUALINPUTS)
  int result = 0;
  for (uint8_t i=0; i<MAX_FLIGHT_MODES; i++) {
    trim_t v = getRawTrimValue(phase, idx);
    if (v.mode == TRIM_MODE_NONE) {
      return result;
    }
    else {
      unsigned int p = v.mode >> 1;
      if (p == phase || phase == 0) {
        return result + v.value;
      }
      else {
        phase = p;
        if (v.mode % 2 != 0) {
          result += v.value;
        }
      }
    }
  }
  return 0;
#else
  return getRawTrimValue(getTrimFlightPhase(phase, idx), idx);
#endif
}

#if defined(VIRTUALINPUTS)
bool setTrimValue(uint8_t phase, uint8_t idx, int trim)
{
  for (uint8_t i=0; i<MAX_FLIGHT_MODES; i++) {
    trim_t & v = flightModeAddress(phase)->trim[idx];
    if (v.mode == TRIM_MODE_NONE)
      return false;
    unsigned int p = v.mode >> 1;
    if (p == phase || phase == 0) {
      v.value = trim;
      break;
    }
    else if (v.mode % 2 == 0) {
      phase = p;
    }
    else {
      v.value = limit<int>(TRIM_EXTENDED_MIN, trim - getTrimValue(p, idx), TRIM_EXTENDED_MAX);
      break;
    }
  }
  eeDirty(EE_MODEL);
  return true;
}
#endif

#if !defined(VIRTUALINPUTS)
uint8_t getTrimFlightPhase(uint8_t phase, uint8_t idx)
{
  for (uint8_t i=0; i<MAX_FLIGHT_MODES; i++) {
    if (phase == 0) return 0;
    trim_t trim = getRawTrimValue(phase, idx);
    if (trim <= TRIM_EXTENDED_MAX) return phase;
    uint8_t result = trim-TRIM_EXTENDED_MAX-1;
    if (result >= phase) result++;
    phase = result;
  }
  return 0;
}
#endif

#if defined(ROTARY_ENCODERS)
uint8_t getRotaryEncoderFlightPhase(uint8_t idx)
{
  uint8_t phase = mixerCurrentFlightMode;
  for (uint8_t i=0; i<MAX_FLIGHT_MODES; i++) {
    if (phase == 0) return 0;
    int16_t value = flightModeAddress(phase)->rotaryEncoders[idx];
    if (value <= ROTARY_ENCODER_MAX) return phase;
    uint8_t result = value-ROTARY_ENCODER_MAX-1;
    if (result >= phase) result++;
    phase = result;
  }
  return 0;
}

int16_t getRotaryEncoder(uint8_t idx)
{
  return flightModeAddress(getRotaryEncoderFlightPhase(idx))->rotaryEncoders[idx];
}

void incRotaryEncoder(uint8_t idx, int8_t inc)
{
  g_rotenc[idx] += inc;
  int16_t *value = &(flightModeAddress(getRotaryEncoderFlightPhase(idx))->rotaryEncoders[idx]);
  *value = limit((int16_t)-1024, (int16_t)(*value + (inc * 8)), (int16_t)+1024);
  eeDirty(EE_MODEL);
}
#endif

#if defined(GVARS)

#if defined(PCBSTD)
  #define SET_GVAR_VALUE(idx, phase, value) \
    (GVAR_VALUE(idx, phase) = value, eeDirty(EE_MODEL))
#else
  #define SET_GVAR_VALUE(idx, phase, value) \
    GVAR_VALUE(idx, phase) = value; \
    eeDirty(EE_MODEL); \
    if (g_model.gvars[idx].popup) { \
      s_gvar_last = idx; \
      s_gvar_timer = GVAR_DISPLAY_TIME; \
    }
#endif

#if defined(PCBSTD)
int16_t getGVarValue(int16_t x, int16_t min, int16_t max)
{
  if (GV_IS_GV_VALUE(x, min, max)) {
    int8_t idx = GV_INDEX_CALCULATION(x, max);
    int8_t mul = 1;

    if (idx < 0) {
      idx = -1-idx;
      mul = -1;
    }

    x = GVAR_VALUE(idx, -1) * mul;
  }

  return limit(min, x, max);
}

void setGVarValue(uint8_t idx, int8_t value)
{
  if (GVAR_VALUE(idx, -1) != value) {
    SET_GVAR_VALUE(idx, -1, value);
  }
}
#else
uint8_t s_gvar_timer = 0;
uint8_t s_gvar_last = 0;

uint8_t getGVarFlightPhase(uint8_t phase, uint8_t idx)
{
  for (uint8_t i=0; i<MAX_FLIGHT_MODES; i++) {
    if (phase == 0) return 0;
    int16_t val = GVAR_VALUE(idx, phase); // TODO phase at the end everywhere to be consistent!
    if (val <= GVAR_MAX) return phase;
    uint8_t result = val-GVAR_MAX-1;
    if (result >= phase) result++;
    phase = result;
  }
  return 0;
}

int16_t getGVarValue(int16_t x, int16_t min, int16_t max, int8_t phase)
{
  if (GV_IS_GV_VALUE(x, min, max)) {
    int8_t idx = GV_INDEX_CALCULATION(x, max);
    int8_t mul = 1;

    if (idx < 0) {
      idx = -1-idx;
      mul = -1;
    }

    x = GVAR_VALUE(idx, getGVarFlightPhase(phase, idx)) * mul;
  }
  return limit(min, x, max);
}

void setGVarValue(uint8_t idx, int16_t value, int8_t phase)
{
  phase = getGVarFlightPhase(phase, idx);
  if (GVAR_VALUE(idx, phase) != value) {
    SET_GVAR_VALUE(idx, phase, value);
  }
}
#endif

#endif

#if defined(CPUARM)
getvalue_t convert16bitsTelemValue(source_t channel, ls_telemetry_value_t value)
{
  return value;
}

getvalue_t convert8bitsTelemValue(source_t channel, ls_telemetry_value_t value)
{
  return value;
}

#if defined(FRSKY)
ls_telemetry_value_t minTelemValue(source_t channel)
{
  return 0;
}

ls_telemetry_value_t maxTelemValue(source_t channel)
{
  return 30000;
}
#endif

ls_telemetry_value_t max8bitsTelemValue(source_t channel)
{
  return 30000;
}

#elif defined(FRSKY)
ls_telemetry_value_t maxTelemValue(uint8_t channel)
{
  switch (channel) {
    case TELEM_FUEL:
    case TELEM_RSSI_TX:
    case TELEM_RSSI_RX:
      return 100;
    case TELEM_HDG:
      return 180;
    default:
      return 255;
  }
}
#endif

#define INAC_STICKS_SHIFT   6
#define INAC_SWITCHES_SHIFT 8
bool inputsMoved()
{
  uint8_t sum = 0;
  for (uint8_t i=0; i<NUM_STICKS; i++)
    sum += anaIn(i) >> INAC_STICKS_SHIFT;
  for (uint8_t i=0; i<NUM_SWITCHES; i++)
    sum += getValue(MIXSRC_FIRST_SWITCH+i) >> INAC_SWITCHES_SHIFT;

  if (abs((int8_t)(sum-inactivity.sum)) > 1) {
    inactivity.sum = sum;
    return true;
  }
  else {
    return false;
  }
}

void checkBacklight()
{
  static uint8_t tmr10ms ;

  uint8_t x = g_blinkTmr10ms;
  if (tmr10ms != x) {
    tmr10ms = x;
    if (inputsMoved()) {
      inactivity.counter = 0;
      if (g_eeGeneral.backlightMode & e_backlight_mode_sticks)
        backlightOn();
    }

    bool backlightOn = (g_eeGeneral.backlightMode == e_backlight_mode_on || lightOffCounter || isFunctionActive(FUNCTION_BACKLIGHT));
    if (flashCounter) backlightOn = !backlightOn;
    if (backlightOn)
      BACKLIGHT_ON();
    else
      BACKLIGHT_OFF();
  }
}

void backlightOn()
{
  lightOffCounter = ((uint16_t)g_eeGeneral.lightAutoOff*250) << 1;
}

#if MENUS_LOCK == 1
bool readonly = true;
bool readonlyUnlocked()
{
  if (readonly) {
    POPUP_WARNING(STR_MODS_FORBIDDEN);
    return false;
  }
  else {
    return true;
  }
}
#endif

#if defined(SPLASH)
void doSplash()
{
  if (SPLASH_NEEDED()) {
    displaySplash();

    getADC(); // init ADC array

    inputsMoved();

    tmr10ms_t tgtime = get_tmr10ms() + SPLASH_TIMEOUT;

    while (tgtime > get_tmr10ms()) {

#if defined(CPUARM)
      CoTickDelay(1);
#endif

      getADC();

#if defined(FSPLASH)
      // Splash is forced, we can't skip it
      if (!(g_eeGeneral.splashMode & 0x04)) {
#endif

      if (keyDown() || inputsMoved()) return;

#if defined(FSPLASH)
      }
#endif
/*       if (pwrCheck() == e_power_off) {
        return;
      } */
      checkBacklight();
    }
  }
}
#else
#define Splash()
#define doSplash()
#endif

#if defined(PCBTARANIS)
void checkFailsafe()
{
  for (int i=0; i<NUM_MODULES; i++) {
    if (IS_MODULE_XJT(i)) {
      ModuleData & moduleData = g_model.moduleData[i];
      if (HAS_RF_PROTOCOL_FAILSAFE(moduleData.rfProtocol) && moduleData.failsafeMode == FAILSAFE_NOT_SET) {
        ALERT(STR_FAILSAFEWARN, STR_NO_FAILSAFE, AU_ERROR);
        break;
      }
    }
  }
}
#endif

#if defined(GUI)
void checkAll()
{
#if defined(EEPROM_RLC)
  checkLowEEPROM();
#endif

  checkTHR();
  checkSwitches();
  checkFailsafe();

#if defined(CPUARM)
  if (g_model.displayChecklist && modelHasNotes()) {
    pushModelNotes();
  }
#endif

#if defined(CPUARM)
  if (!clearKeyEvents()) {
    displayPopup(STR_KEYSTUCK);
    tmr10ms_t tgtime = get_tmr10ms() + 500;
    while (tgtime != get_tmr10ms()) {
      CoTickDelay(1);
      wdt_feed();
    }
  }
#else    // #if defined(CPUARM)
  clearKeyEvents();
#endif   // #if defined(CPUARM)

  START_SILENCE_PERIOD();
}
#endif // GUI



#if defined(EEPROM_RLC)
void checkLowEEPROM()
{
  if (g_eeGeneral.disableMemoryWarning) return;
  if (EeFsGetFree() < 100) {
    ALERT(STR_EEPROMWARN, STR_EEPROMLOWMEM, AU_ERROR);
  }
}
#endif

void checkTHR()
{
  uint8_t thrchn = ((g_model.thrTraceSrc==0) || (g_model.thrTraceSrc>NUM_POTS)) ? THR_STICK : g_model.thrTraceSrc+NUM_STICKS-1;
  // throttle channel is either the stick according stick mode (already handled in evalInputs)
  // or P1 to P3;
  // in case an output channel is choosen as throttle source (thrTraceSrc>NUM_POTS) we assume the throttle stick is the input
  // no other information available at the moment, and good enough to my option (otherwise too much exceptions...)
}

void checkAlarm() // added by Gohst
{
  if (g_eeGeneral.disableAlarmWarning) {
    return;
  }

  if (IS_SOUND_OFF()) {
    ALERT(STR_ALARMSWARN, STR_ALARMSDISABLED, AU_ERROR);
  }
}

void alert(const pm_char * t, const pm_char *s MESSAGE_SOUND_ARG)
{
  MESSAGE(t, s, STR_PRESSANYKEY, sound);


  while(1)
  {
    SIMU_SLEEP(1);

    if (keyDown()) return;  // wait for key release

    checkBacklight();

    wdt_feed();
/*     if (pwrCheck() == e_power_off) {
      boardOff(); // turn power off now
    } */

  }
}

#if defined(GVARS)
int8_t trimGvar[NUM_STICKS] = { -1, -1, -1, -1 };
#endif

#if defined(CPUARM)
void checkTrims()
{
  uint8_t event = getEvent(true);
  if (event && !IS_KEY_BREAK(event)) {
    int8_t k = EVT_KEY_MASK(event) - TRM_BASE;
#else
uint8_t checkTrim(uint8_t event)
{
  int8_t k = EVT_KEY_MASK(event) - TRM_BASE;
  if (k>=0 && k<8 && !IS_KEY_BREAK(event)) {
#endif
    // LH_DWN LH_UP LV_DWN LV_UP RV_DWN RV_UP RH_DWN RH_UP
    uint8_t idx = CONVERT_MODE((uint8_t)k/2);
    uint8_t phase;
    int before;
    bool thro;

#if defined(CPUARM)
    trimsDisplayTimer = 200; // 2 seconds
    trimsDisplayMask |= (1<<idx);
#endif

#if defined(GVARS)
    if (TRIM_REUSED(idx)) {
      phase = getGVarFlightPhase(mixerCurrentFlightMode, trimGvar[idx]);
      before = GVAR_VALUE(trimGvar[idx], phase);
      thro = false;
    }
    else {
      phase = getTrimFlightPhase(mixerCurrentFlightMode, idx);
#if defined(VIRTUALINPUTS)
      before = getTrimValue(phase, idx);
#else
      before = getRawTrimValue(phase, idx);
#endif
      thro = (idx==THR_STICK && g_model.thrTrim);
    }
#else
    phase = getTrimFlightPhase(mixerCurrentFlightMode, idx);
#if defined(VIRTUALINPUTS)
    before = getTrimValue(phase, idx);
#else
    before = getRawTrimValue(phase, idx);
#endif
    thro = (idx==THR_STICK && g_model.thrTrim);
#endif
    int8_t trimInc = g_model.trimInc + 1;
    int8_t v = (trimInc==-1) ? min(32, abs(before)/4+1) : (1 << trimInc); // TODO flash saving if (trimInc < 0)
    if (thro) v = 4; // if throttle trim and trim trottle then step=4
    int16_t after = (k&1) ? before + v : before - v;   // positive = k&1
#if defined(CPUARM)
    uint8_t beepTrim = 0;
#else
    bool beepTrim = false;
#endif
    for (int16_t mark=TRIM_MIN; mark<=TRIM_MAX; mark+=TRIM_MAX) {
      if ((mark!=0 || !thro) && ((mark!=TRIM_MIN && after>=mark && before<mark) || (mark!=TRIM_MAX && after<=mark && before>mark))) {
        after = mark;
        beepTrim = (mark == 0 ? 1 : 2);
      }
    }

    if ((before<after && after>TRIM_MAX) || (before>after && after<TRIM_MIN)) {
      if (!g_model.extendedTrims || TRIM_REUSED(idx)) after = before;
    }

    if (after < TRIM_EXTENDED_MIN) {
      after = TRIM_EXTENDED_MIN;
    }
    if (after > TRIM_EXTENDED_MAX) {
      after = TRIM_EXTENDED_MAX;
    }

#if defined(GVARS)
    if (TRIM_REUSED(idx)) {
      SET_GVAR_VALUE(trimGvar[idx], phase, after);
    }
    else
#endif
    {
#if defined(VIRTUALINPUTS)
      if (!setTrimValue(phase, idx, after)) {
        // we don't play a beep, so we exit now the function
        return;
      }
#else
      setTrimValue(phase, idx, after);
#endif
    }

#if defined(AUDIO)
    // toneFreq higher/lower according to trim position
    // limit the frequency, range -125 to 125 = toneFreq: 19 to 101
    if (after > TRIM_MAX)
      after = TRIM_MAX;
    if (after < TRIM_MIN)
      after = TRIM_MIN;
#if defined(CPUARM)
    after <<= 3;
    after += 120*16;
#else
    after >>= 2;
    after += 60;
#endif
#endif

    if (beepTrim) {
      if (beepTrim == 1) {
        AUDIO_TRIM_MIDDLE(after);
        pauseEvents(event);
      }
      else {
        AUDIO_TRIM_END(after);
        killEvents(event);
      }
    }
    else {
      AUDIO_TRIM(event, after);
    }
#if !defined(CPUARM)
    return 0;
#endif
  }
#if !defined(CPUARM)
  return event;
#endif
}

uint16_t s_anaFilt[NUMBER_ANALOG_ADC1];

#if defined(JITTER_MEASURE)
JitterMeter<uint16_t> rawJitter[NUMBER_ANALOG_ADC1];
JitterMeter<uint16_t> avgJitter[NUMBER_ANALOG_ADC1];
tmr10ms_t jitterResetTime = 0;
#define JITTER_MEASURE_ACTIVE()   (menuHandlers[menuLevel] == menuGeneralDiagAna)
#endif  // defined(JITTER_MEASURE)



#if !defined(SIMU)
uint16_t anaIn(uint8_t chan)
{
  return s_anaFilt[chan]; 
}

//! NUMBER_ANALOG_ADC1

#if defined(CPUARM)
void getADC(void)
{
  uint16_t temp[NUMBER_ANALOG_ADC1] = { 0 };

  for(uint32_t i=0; i<4; i++) //! 四次求平均值
  {
      adcRead();
      for (uint32_t x=0; x<NUMBER_ANALOG_ADC1; x++) 
	  {
          temp[x] += getAnalogValue(x);
      }
  }

  for(uint32_t x=0; x<NUMBER_ANALOG_ADC1; x++) 
  {
    uint16_t v = temp[x] >> 3;

#if defined(VIRTUALINPUTS)
    if(calibrationState) 
	{
      v = temp[x] >> 1;
    }
#if defined(JITTER_FILTER)   
    else 
	{
      // jitter filter
      uint16_t diff = (v > s_anaFilt[x]) ? (v - s_anaFilt[x]) : (s_anaFilt[x] - v);
      if (diff < 10) 
	  {
        // apply filter
        v = (7 * s_anaFilt[x] + v) >> 3;
      }
    }
#endif

#if defined(JITTER_MEASURE)
    if(JITTER_MEASURE_ACTIVE()) 
	{
      avgJitter[x].measure(v);
    }
#endif
    StepsCalibData * calib = (StepsCalibData *) &g_eeGeneral.calib[x];
    if(!calibrationState && IS_POT_MULTIPOS(x) && calib->count>0 && calib->count<XPOTS_MULTIPOS_COUNT) {
      uint8_t vShifted = (v >> 4);
      s_anaFilt[x] = 2*RESX;
      for(int i=0; i<calib->count; i++) 
	  {
        if(vShifted < calib->steps[i]) 
		{
          s_anaFilt[x] = i*2*RESX/calib->count;
          break;
        }
      }
    }
    else 
#endif
    {
      s_anaFilt[x] = v; //! 滤波之后的数据
    }
  }
}
#endif
#endif // SIMU

uint8_t g_vbat100mV = 0;
uint16_t lightOffCounter;
uint8_t flashCounter = 0;

uint16_t sessionTimer;
uint16_t s_timeCumThr;    // THR in 1/16 sec
uint16_t s_timeCum16ThrP; // THR% in 1/16 sec

uint8_t  trimsCheckTimer = 0;

#if defined(CPUARM)
uint8_t trimsDisplayTimer = 0;
uint8_t trimsDisplayMask = 0;
#endif

void flightReset()
{
  // we don't reset the whole audio here (the tada.wav would be cut, if a prompt is queued before FlightReset, it should be played)
  // TODO check if the vario / background music are stopped correctly if switching to a model which doesn't have these functions enabled

  if(!IS_MANUAL_RESET_TIMER(0)) 
  {
    timerReset(0);
  }

#if TIMERS > 1
  if(!IS_MANUAL_RESET_TIMER(1)) 
  {
    timerReset(1);
  }
#endif

#if TIMERS > 2
  if(!IS_MANUAL_RESET_TIMER(2)) 
  {
    timerReset(2);
  }
#endif

#if defined(FRSKY)
  telemetryReset();
#endif

  s_mixer_first_run_done = false;

  START_SILENCE_PERIOD();

  RESET_THR_TRACE();
}

#if defined(THRTRACE)
uint8_t  s_traceBuf[MAXTRACE];
#if LCD_W >= 255
  int16_t  s_traceWr;
  int16_t  s_traceCnt;
#else
  uint8_t  s_traceWr;
  int16_t  s_traceCnt;
#endif
uint8_t  s_cnt_10s;
uint16_t s_cnt_samples_thr_10s;
uint16_t s_sum_samples_thr_10s;
#endif

FORCEINLINE void evalTrims()
{
  uint8_t phase = mixerCurrentFlightMode;
  for (uint8_t i=0; i<NUM_STICKS; i++) {
    // do trim -> throttle trim if applicable
    int16_t trim = getTrimValue(phase, i);
#if !defined(VIRTUALINPUTS)
    if (i==THR_STICK && g_model.thrTrim) {
      int16_t trimMin = g_model.extendedTrims ? TRIM_EXTENDED_MIN : TRIM_MIN;
      trim = (((g_model.throttleReversed)?(int32_t)(trim+trimMin):(int32_t)(trim-trimMin)) * (RESX-anas[i])) >> (RESX_SHIFT+1);
    }
#endif
    if (trimsCheckTimer > 0) {
      trim = 0;
    }

    trims[i] = trim*2;
  }
}

#if !defined(PCBSTD)
uint8_t mSwitchDuration[1+NUM_ROTARY_ENCODERS] = { 0 };
#define CFN_PRESSLONG_DURATION   100
#endif












uint8_t s_mixer_first_run_done = false;

void doMixerCalculations()
{
  //tmr10ms_t tmr10ms = get_tmr10ms();

  getADC();                      //!
  keyprocess();                  //!
  joystickConvert();             //!
  channelUpdate();               //! 
  powerShutdown();               //! 
  setFlightMode(OPENTX_MANUAL, OPENTX_AUTO, OPENTX_HOME); //! 
  //setGimbalMode(OPENTX_TR1);     //!  

  s_mixer_first_run_done = true;
}





void opentxStart()
{
  doSplash();

#if defined(DEBUG_TRACE_BUFFER)
  trace_event(trace_start, 0x12345678);
#endif 


#if defined(CPUARM)
  eeLoadModel(g_eeGeneral.currModel);
#endif

#if defined(GUI)
  checkAlarm();
  checkAll();
#endif

#if defined(GUI)
  if (g_eeGeneral.chkSum != evalChkSum()) 
  {
    chainMenu(menuFirstCalib);
  }
#endif

}




#if defined(CPUARM) || defined(CPUM2560)
void opentxClose()
{
  AUDIO_BYE();

#if defined(LUA)
  luaClose();
#endif

#if defined(SDCARD)
  closeLogs();
#endif

#if defined(HAPTIC)
hapticOff();
#endif

  saveTimers();

#if defined(CPUARM)
  for (int i=0; i<MAX_SENSORS; i++) {
    TelemetrySensor & sensor = g_model.telemetrySensors[i];
    if (sensor.type == TELEM_TYPE_CALCULATED) {
      if (sensor.persistent && sensor.persistentValue != telemetryItems[i].value) {
        sensor.persistentValue = telemetryItems[i].value;
        eeDirty(EE_MODEL);
      }
      else if (!sensor.persistent) {
        sensor.persistentValue = 0;
        eeDirty(EE_MODEL);
      }
    }
  }
#endif


#if defined(PCBTARANIS)
  if (g_model.potsWarnMode == POTS_WARN_AUTO) {
    for (int i=0; i<NUM_POTS; i++) {
      if (!(g_model.potsWarnEnabled & (1 << i))) {
        SAVE_POT_POSITION(i);
      }
    }
    eeDirty(EE_MODEL);
  }
#endif

  g_eeGeneral.unexpectedShutdown = 0;

  eeDirty(EE_GENERAL);
  eeCheck(true);

#if defined(CPUARM)
  while (IS_PLAYING(ID_PLAY_BYE)){
    CoTickDelay(10);
  }

  CoTickDelay(50);
#endif

#if defined(SDCARD)
  sdDone();
#endif
}
#endif


void checkBattery()
{
  static uint8_t counter = 0;
#if defined(GUI) && !defined(COLORLCD)
  // TODO not the right menu I think ...
  if(menuHandlers[menuLevel] == menuGeneralDiagAna) 
  {
    g_vbat100mV = 0;
    counter = 0;
  }
#endif
  if(counter-- == 0) 
  {
    counter = 10;
    int32_t instant_vbat = anaIn(TX_VOLTAGE);
#if defined(PCBTARANIS)
    instant_vbat = (instant_vbat + instant_vbat*(g_eeGeneral.txVoltageCalibration)/128) * BATT_SCALE;
    instant_vbat >>= 11;
    instant_vbat += 2; // because of the diode
#endif
    static uint8_t  s_batCheck;
    static uint16_t s_batSum;
#if defined(VOICE)
    s_batCheck += 8;
#else
    s_batCheck += 32;
#endif

    s_batSum += instant_vbat;

    if(g_vbat100mV == 0) 
	{
      g_vbat100mV = instant_vbat;
      s_batSum = 0;
      s_batCheck = 0;
    }
#if defined(VOICE)
    else if(!(s_batCheck & 0x3f)) 
	{
#else
    else if(s_batCheck == 0) 
	{
#endif
      g_vbat100mV = s_batSum / 8;
      s_batSum = 0;
#if defined(VOICE)
      if (s_batCheck != 0) {
        // no alarms
      }
      else
#endif
      if(IS_TXBATT_WARNING() && g_vbat100mV>50) 
	  {
        AUDIO_TX_BATTERY_LOW();
      }
    }
  }
}


#if defined(VIRTUALINPUTS)
  #define INSTANT_TRIM_MARGIN 10 /* around 1% */
#else
  #define INSTANT_TRIM_MARGIN 15 /* around 1.5% */
#endif

void instantTrim()
{
#if defined(VIRTUALINPUTS)
  int16_t  anas_0[NUM_INPUTS];
  evalInputs(e_perout_mode_notrainer | e_perout_mode_nosticks);
  memcpy(anas_0, anas, sizeof(anas_0));
#endif

  evalInputs(e_perout_mode_notrainer);

  for (uint8_t stick=0; stick<NUM_STICKS; stick++) {
    if (stick!=THR_STICK) {
      // don't instant trim the throttle stick
      uint8_t trim_phase = getTrimFlightPhase(mixerCurrentFlightMode, stick);
#if defined(VIRTUALINPUTS)
      int16_t delta = 0;
      for (int e=0; e<MAX_EXPOS; e++) {
        ExpoData * ed = expoAddress(e);
        if (!EXPO_VALID(ed)) break; // end of list
        if (ed->srcRaw-MIXSRC_Rud == stick) {
          delta = anas[ed->chn] - anas_0[ed->chn];
          break;
        }
      }
#else
      int16_t delta = anas[stick];
#endif
      if (abs(delta) >= INSTANT_TRIM_MARGIN) {
        int16_t trim = limit<int16_t>(TRIM_EXTENDED_MIN, (delta + trims[stick]) / 2, TRIM_EXTENDED_MAX);
        setTrimValue(trim_phase, stick, trim);
      }
    }
  }

  eeDirty(EE_MODEL);
  AUDIO_WARNING2();
}

void copySticksToOffset(uint8_t ch)
{
  pauseMixerCalculations();
  int32_t zero = (int32_t)channelOutputs[ch];

  evalFlightModeMixes(e_perout_mode_nosticks+e_perout_mode_notrainer, 0);
  int32_t val = chans[ch];
  LimitData *ld = limitAddress(ch);
  limit_min_max_t lim = LIMIT_MIN(ld);
  if (val < 0) {
    val = -val;
    lim = LIMIT_MIN(ld);
  }
#if defined(CPUARM)
  zero = (zero*256000 - val*lim) / (1024*256-val);
#else
  zero = (zero*25600 - val*lim) / (26214-val);
#endif
  ld->offset = (ld->revert ? -zero : zero);
  resumeMixerCalculations();
  eeDirty(EE_MODEL);
}

void copyTrimsToOffset(uint8_t ch)
{
  int16_t zero;

  pauseMixerCalculations();

  evalFlightModeMixes(e_perout_mode_noinput, 0); // do output loop - zero input sticks and trims
  zero = applyLimits(ch, chans[ch]);

  evalFlightModeMixes(e_perout_mode_noinput-e_perout_mode_notrims, 0); // do output loop - only trims

  int16_t output = applyLimits(ch, chans[ch]) - zero;
  int16_t v = g_model.limitData[ch].offset;
  if (g_model.limitData[ch].revert) output = -output;
#if defined(CPUARM)
  v += (output * 125) / 128;
#else
  v += output;
#endif
  g_model.limitData[ch].offset = limit((int16_t)-1000, (int16_t)v, (int16_t)1000); // make sure the offset doesn't go haywire

  resumeMixerCalculations();
  eeDirty(EE_MODEL);
}

void moveTrimsToOffsets() // copy state of 3 primary to subtrim
{
  int16_t zeros[NUM_CHNOUT];

  pauseMixerCalculations();

  evalFlightModeMixes(e_perout_mode_noinput, 0); // do output loop - zero input sticks and trims
  for (uint8_t i=0; i<NUM_CHNOUT; i++) {
    zeros[i] = applyLimits(i, chans[i]);
  }

  evalFlightModeMixes(e_perout_mode_noinput-e_perout_mode_notrims, 0); // do output loop - only trims

  for (uint8_t i=0; i<NUM_CHNOUT; i++) {
    int16_t output = applyLimits(i, chans[i]) - zeros[i];
    int16_t v = g_model.limitData[i].offset;
    if (g_model.limitData[i].revert) output = -output;
#if defined(CPUARM)
    v += (output * 125) / 128;
#else
    v += output;
#endif
    g_model.limitData[i].offset = limit((int16_t)-1000, (int16_t)v, (int16_t)1000); // make sure the offset doesn't go haywire
  }

  // reset all trims, except throttle (if throttle trim)
  for (uint8_t i=0; i<NUM_STICKS; i++) {
    if (i!=THR_STICK || !g_model.thrTrim) {
      int16_t original_trim = getTrimValue(mixerCurrentFlightMode, i);
      for (uint8_t phase=0; phase<MAX_FLIGHT_MODES; phase++) {
#if defined(VIRTUALINPUTS)
        trim_t trim = getRawTrimValue(phase, i);
        if (trim.mode / 2 == phase)
          setTrimValue(phase, i, trim.value - original_trim);
#else
        trim_t trim = getRawTrimValue(phase, i);
        if (trim <= TRIM_EXTENDED_MAX)
          setTrimValue(phase, i, trim - original_trim);
#endif
      }
    }
  }

  resumeMixerCalculations();

  eeDirty(EE_MODEL);
  AUDIO_WARNING2();
}

#if defined(ROTARY_ENCODERS)
  volatile rotenc_t g_rotenc[ROTARY_ENCODERS] = {0};
#elif defined(ROTARY_ENCODER_NAVIGATION)
  volatile rotenc_t g_rotenc[1] = {0};
#endif








#define OPENTX_INIT_ARGS

void opentxInit(OPENTX_INIT_ARGS)
{
  globalDatelInit();

  eeReadJoyScale();
  
  displayInit(); //!   
  
  systemTimer_10ms = CoCreateTmr(TMR_TYPE_PERIODIC, 5, 5, systemTimer_10msCallBack);         //! 100Hz creat system timer：1 = 2ms
  CoStartTmr(systemTimer_10ms);                                                              //! start the 10ms system timer  
  
  systemTimer_100ms = CoCreateTmr(TMR_TYPE_PERIODIC, 50, 50, systemTimer_100msCallBack);     //! 10Hz  creat system timer：1 = 2ms
  CoStartTmr(systemTimer_100ms);                                                             //! start the 100ms system timer  
  

  
  
  
//annotaged by apple 
//eeReadAll();  
/* #if defined(CPUARM)
  if (UNEXPECTED_SHUTDOWN()) 
  {
    unexpectedShutdown = 1;
  }
#endif

#if MENUS_LOCK == 1
  getMovedSwitch();
  if(TRIMS_PRESSED() && g_eeGeneral.switchUnlockStates==switches_states) 
  {
    readonly = false;
  }
#endif

  if(g_eeGeneral.backlightMode != e_backlight_mode_off) backlightOn(); // on Tx start turn the light on

  if(UNEXPECTED_SHUTDOWN()) 
  {
#if defined(CPUARM)//annotated by apple
    eeLoadModel(g_eeGeneral.currModel);
#endif
  }
  else 
  {
    opentxStart();//annotated by apple
  }

#if defined(CPUARM)
  if(!g_eeGeneral.unexpectedShutdown) 
  {
    g_eeGeneral.unexpectedShutdown = 1;
    eeDirty(EE_GENERAL);
  }
#endif
*/

  backLightEnable(100, 100); //! backlight enable
  wdt_enable(WDTO_1500MS); 
}



/***************************main app start here **************************************************************/
#if !defined(SIMU)
int main(void)
{
  // G: The WDT remains active after a WDT reset -- at maximum clock speed. So it's
  // important to disable it before commencing with system initialisation (or
  // we could put a bunch more wdt_feed()s in. But I don't like that approach
  // during boot up.)

  wdt_disable();
  boardInit();
  stackPaint();

  sei(); //! interrupts needed for telemetryInit and eeReadAll.

  tasksStart();
}
#endif // !SIMU
/*********************************main app end here********************************************************************/















/*******************************************************
 * @ globalDatelInit()
*******************************************************/

void globalDatelInit(void)
{
  memset(&g_rcChannel, 1500, sizeof(g_rcChannel)); 
  g_eeGeneral.flightMode = 1000;
  g_eeGeneral.vtolMode = 1000; //! init vtol mode value
  g_eeGeneral.throwMode = 1000; 
  g_eeGeneral.joyscale.ele_max  = ELE_MAX; //! 必须在eeReadJoyScale()之前初始化，因为初始化赋值和读取eeprom的值都是用的统一全局变量 
  g_eeGeneral.joyscale.ele_min  = ELE_MIN;    
  g_eeGeneral.joyscale.rud_max  = RUD_MAX;
  g_eeGeneral.joyscale.rud_min  = RUD_MIN;   
  g_eeGeneral.joyscale.thr_max  = THR_MAX;
  g_eeGeneral.joyscale.thr_min  = THR_MIN;   
  g_eeGeneral.joyscale.ail_max  = AIL_MAX;
  g_eeGeneral.joyscale.ail_min  = AIL_MIN;   
  g_eeGeneral.joyscale.ltrm_max = LTRM_MAX;
  g_eeGeneral.joyscale.ltrm_min = LTRM_MIN;    
  g_eeGeneral.joyscale.rtrm_max = RTRM_MAX;
  g_eeGeneral.joyscale.rtrm_min = RTRM_MIN; 
  
  mavlinkDateInit();  
  
}









/*******************************************************
 * @ set the flightmode :the value is 
 
 * @ param: keyManual: apply which key to set the uav into manual mode
 * @ param: keyAuto:   apply which key to set the uav into auto mode
 * @ param: keyRtl:    apply which key to set the uav into return to home mode
 
 * @ manual: 1000  (uav work without gps)
 * @ auto  : 1500  (uav work with gps)
 * @ return: 2000  (return to home)
*******************************************************/
void setFlightMode(uint32_t keyManual, uint32_t keyAuto, uint32_t keyRtl)
{
 
  if(g_eeGeneral.key == keyManual)  //! manual mode
  {
	g_eeGeneral.flightMode = 1400;
	g_eeGeneral.key = 0;	
  } 
  else if(g_eeGeneral.key == keyAuto)    //! auto mode
  {
	g_eeGeneral.flightMode = 1700;
	g_eeGeneral.key = 0; 	
  }  
  else if(g_eeGeneral.key == keyRtl)    //! return to home
  {
    g_eeGeneral.flightMode = 2000;
	g_eeGeneral.key = 0;	
  }
  else 
  {
	  return; //! 
  }
}




/*******************************************************
 * @ set the vtolMode :the value is 1000 or 2000 use channel 8
 
 * @ param: keyVtol:  apply which key to set the vtol mode
*******************************************************/
void setVtolMode(uint32_t keyVtol)
{
  if(g_eeGeneral.key == keyVtol)  
  {
	if(g_eeGeneral.vtolMode != 1000)
	{
	  g_eeGeneral.vtolMode = 1000;
	}
	else
	{
	  g_eeGeneral.vtolMode = 2000;
	}
   
    g_eeGeneral.key = 0;	
  } 
}



/*******************************************************
 * @ set the throw mode :the value is 1000 or 2000 use channel 8
 
 * @ param: keyThrow:  apply which key to set the throw mode
*******************************************************/
void setThrowMode(uint32_t keyThrow)
{
  if(g_eeGeneral.key == keyThrow)  
  {
	if(g_eeGeneral.throwMode != 1000)
	{
	  g_eeGeneral.throwMode = 1000;
	}
	else
	{
	  g_eeGeneral.throwMode = 2000;
	}
	
    g_eeGeneral.key = 0;	
  }  
}



/*******************************************************
 * @ set the gimbal mode :the value is 
 * @ mode1: 1000  
 * @ mode2: 1500  
 * @ mode3: 2000  
 * @ param: key: apply which key to set the gimbal mode 
*******************************************************/
void setGimbalMode(uint32_t key)
{
  if(g_eeGeneral.key == key)  //! set gimbal mode
  {

	  if(g_eeGeneral.gimbalMode < 1500)
	  {
		  g_eeGeneral.gimbalMode = 1500;
	  }	  
	  else if(g_eeGeneral.gimbalMode == 1500) 
	  {
		  g_eeGeneral.gimbalMode = 2000; 
	  }	 
	  else 
	  {
		  g_eeGeneral.gimbalMode = 1000;
	  } 

	  g_eeGeneral.key = 0;	  
  }	
}



/*******************************************************
 * @ convert anaIn[] to standard joystick output 1000 -- 2000
 * @ ele  anaIn[1]  升降舵：俯仰角    
 * @ rud  anaIn[0]  方向舵：航向角
 * @ thr  anaIn[2]  油门 
 * @ ail  anaIn[3]  副翼：  横滚角
 * @ ltrm anaIn[4]  左滚轮
 * @ rtrm anaIn[5]  右滚轮
*******************************************************/
void joystickConvert(void)
{  
   int16_t temp;
   
   //! ELE
   temp = anaIn(1);
   if(temp>g_eeGeneral.joyscale.ele_max)      temp = g_eeGeneral.joyscale.ele_max;
   else if(temp<g_eeGeneral.joyscale.ele_min) temp = g_eeGeneral.joyscale.ele_min; 
   
   if(temp<(g_eeGeneral.joyscale.ele_min+g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.ele = 2000;
   }
   else if(temp<(g_eeGeneral.joyscale.ele_cen-g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.ele = temp*500/(g_eeGeneral.joyscale.ele_min-g_eeGeneral.joyscale.ele_cen) - 500*g_eeGeneral.joyscale.ele_min/(g_eeGeneral.joyscale.ele_min-g_eeGeneral.joyscale.ele_cen) + 2000;	   
   }
   else if(temp<(g_eeGeneral.joyscale.ele_cen+g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.ele = 1500;  
   }
   else if(temp<(g_eeGeneral.joyscale.ele_max-g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.ele = temp*500/(g_eeGeneral.joyscale.ele_cen-g_eeGeneral.joyscale.ele_max) + 500*g_eeGeneral.joyscale.ele_max/(g_eeGeneral.joyscale.ele_max-g_eeGeneral.joyscale.ele_cen) + 1000;
   }	
   else
   {
	 g_eeGeneral.joystick.ele = 1000;  
   }	
  
   //! RUD
   temp = anaIn(0);
   if(temp>g_eeGeneral.joyscale.rud_max)      temp = g_eeGeneral.joyscale.rud_max;
   else if(temp<g_eeGeneral.joyscale.rud_min) temp = g_eeGeneral.joyscale.rud_min; 
   
   if(temp<(g_eeGeneral.joyscale.rud_min+g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.rud = 2000;
   }
   else if(temp<(g_eeGeneral.joyscale.rud_cen-g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.rud = temp*500/(g_eeGeneral.joyscale.rud_min-g_eeGeneral.joyscale.rud_cen) - 500*g_eeGeneral.joyscale.rud_min/(g_eeGeneral.joyscale.rud_min-g_eeGeneral.joyscale.rud_cen) + 2000;	   
   }
   else if(temp<(g_eeGeneral.joyscale.rud_cen+g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.rud = 1500;  
   }
   else if(temp<(g_eeGeneral.joyscale.rud_max-g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.rud = temp*500/(g_eeGeneral.joyscale.rud_cen-g_eeGeneral.joyscale.rud_max) + 500*g_eeGeneral.joyscale.rud_max/(g_eeGeneral.joyscale.rud_max-g_eeGeneral.joyscale.rud_cen) + 1000;
   }	
   else
   {
	 g_eeGeneral.joystick.rud = 1000;  
   }
  
   //! THR
   temp = anaIn(2);
   if(temp>g_eeGeneral.joyscale.thr_max)      temp = g_eeGeneral.joyscale.thr_max;
   else if(temp<g_eeGeneral.joyscale.thr_min) temp = g_eeGeneral.joyscale.thr_min; 
   
   if(temp>(g_eeGeneral.joyscale.thr_max-g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.thr = 2000;
   }
   else if(temp>(g_eeGeneral.joyscale.thr_cen+g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.thr = temp*500/(g_eeGeneral.joyscale.thr_cen-g_eeGeneral.joyscale.thr_min) + 500*g_eeGeneral.joyscale.thr_cen/(g_eeGeneral.joyscale.thr_min-g_eeGeneral.joyscale.thr_cen) + 1500;	   
   }
   else if(temp>(g_eeGeneral.joyscale.thr_cen-g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.thr = 1500;
   }
   else if(temp>(g_eeGeneral.joyscale.thr_min+g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.thr = temp*500/(g_eeGeneral.joyscale.thr_max-g_eeGeneral.joyscale.thr_cen) + 500*g_eeGeneral.joyscale.thr_cen/(g_eeGeneral.joyscale.thr_cen-g_eeGeneral.joyscale.thr_max) + 1500;
   }	
   else
   {
	 g_eeGeneral.joystick.thr = 1000;   
   }

   //! AIL
   temp = anaIn(3);
   if(temp>g_eeGeneral.joyscale.ail_max)      temp = g_eeGeneral.joyscale.ail_max;
   else if(temp<g_eeGeneral.joyscale.ail_min) temp = g_eeGeneral.joyscale.ail_min; 
   
   if(temp>(g_eeGeneral.joyscale.ail_max-g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.ail = 2000;
   }
   else if(temp>(g_eeGeneral.joyscale.ail_cen+g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.ail = temp*500/(g_eeGeneral.joyscale.ail_cen-g_eeGeneral.joyscale.ail_min) + 500*g_eeGeneral.joyscale.ail_cen/(g_eeGeneral.joyscale.ail_min-g_eeGeneral.joyscale.ail_cen) + 1500;	   
   }
   else if(temp>(g_eeGeneral.joyscale.ail_cen-g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.ail = 1500;
   }
   else if(temp>(g_eeGeneral.joyscale.ail_min+g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.ail = temp*500/(g_eeGeneral.joyscale.ail_max-g_eeGeneral.joyscale.ail_cen) + 500*g_eeGeneral.joyscale.ail_cen/(g_eeGeneral.joyscale.ail_cen-g_eeGeneral.joyscale.ail_max) + 1500;
   }	
   else
   {
	 g_eeGeneral.joystick.ail = 1000;   
   }	
 
   //! LTRM
   temp = anaIn(4);
   if(temp>g_eeGeneral.joyscale.ltrm_max)      temp = g_eeGeneral.joyscale.ltrm_max;
   else if(temp<g_eeGeneral.joyscale.ltrm_min) temp = g_eeGeneral.joyscale.ltrm_min; 
   
   if(temp>(g_eeGeneral.joyscale.ltrm_max-g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.ltrm = 2000;
   }
   else if(temp>(g_eeGeneral.joyscale.ltrm_cen+g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.ltrm = temp*500/(g_eeGeneral.joyscale.ltrm_cen-g_eeGeneral.joyscale.ltrm_min) + 500*g_eeGeneral.joyscale.ltrm_cen/(g_eeGeneral.joyscale.ltrm_min-g_eeGeneral.joyscale.ltrm_cen) + 1500;	   
   }
   else if(temp>(g_eeGeneral.joyscale.ltrm_cen-g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.ltrm = 1500;
   }
   else if(temp>(g_eeGeneral.joyscale.ltrm_min+g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.ltrm = temp*500/(g_eeGeneral.joyscale.ltrm_max-g_eeGeneral.joyscale.ltrm_cen) + 500*g_eeGeneral.joyscale.ltrm_cen/(g_eeGeneral.joyscale.ltrm_cen-g_eeGeneral.joyscale.ltrm_max) + 1500;
   }	
   else
   {
	 g_eeGeneral.joystick.ltrm = 1000;   
   }   

   //! RTRM  
   temp = anaIn(5);
   if(temp>g_eeGeneral.joyscale.rtrm_max)      temp = g_eeGeneral.joyscale.rtrm_max;
   else if(temp<g_eeGeneral.joyscale.rtrm_min) temp = g_eeGeneral.joyscale.rtrm_min; 
   
   if(temp>(g_eeGeneral.joyscale.rtrm_max-g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.rtrm = 2000;
   }
   else if(temp>(g_eeGeneral.joyscale.rtrm_cen+g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.rtrm = temp*500/(g_eeGeneral.joyscale.rtrm_cen-g_eeGeneral.joyscale.rtrm_min) + 500*g_eeGeneral.joyscale.rtrm_cen/(g_eeGeneral.joyscale.rtrm_min-g_eeGeneral.joyscale.rtrm_cen) + 1500;	   
   }
   else if(temp>(g_eeGeneral.joyscale.rtrm_cen-g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.rtrm = 1500;
   }
   else if(temp>(g_eeGeneral.joyscale.rtrm_min+g_eeGeneral.joyscale.threshold))
   {
	 g_eeGeneral.joystick.rtrm = temp*500/(g_eeGeneral.joyscale.rtrm_max-g_eeGeneral.joyscale.rtrm_cen) + 500*g_eeGeneral.joyscale.rtrm_cen/(g_eeGeneral.joyscale.rtrm_cen-g_eeGeneral.joyscale.rtrm_max) + 1500;
   }	
   else
   {
	 g_eeGeneral.joystick.rtrm = 1000;   
   } 
   
   //! Limiter
   if(g_eeGeneral.joystick.ele>2000)  g_eeGeneral.joystick.ele = 2000;
   if(g_eeGeneral.joystick.ele<1000)  g_eeGeneral.joystick.ele = 1000;
   if(g_eeGeneral.joystick.rud>2000)  g_eeGeneral.joystick.rud = 2000;
   if(g_eeGeneral.joystick.rud<1000)  g_eeGeneral.joystick.rud = 1000;
   if(g_eeGeneral.joystick.thr>2000)  g_eeGeneral.joystick.thr = 2000;
   if(g_eeGeneral.joystick.thr<1000)  g_eeGeneral.joystick.thr = 1000;
   if(g_eeGeneral.joystick.ail>2000)  g_eeGeneral.joystick.ail = 2000;
   if(g_eeGeneral.joystick.ail<1000)  g_eeGeneral.joystick.ail = 1000;	
   if(g_eeGeneral.joystick.ltrm>2000) g_eeGeneral.joystick.ltrm = 2000;
   if(g_eeGeneral.joystick.ltrm<1000) g_eeGeneral.joystick.ltrm = 1000;
   if(g_eeGeneral.joystick.rtrm>2000) g_eeGeneral.joystick.rtrm = 2000;
   if(g_eeGeneral.joystick.rtrm<1000) g_eeGeneral.joystick.rtrm = 1000;	 

}



/*******************************************************
 * @ update 8 channel values 1000 -- 2000
 * @ chan1: ele   升降舵   
 * @ chan2: rud   方向舵
 * @ chan3: thr   油门 
 * @ chan4: ail   副翼
 * @ chan5:       飞行模式
 * @ chan6: ltrm  左滚轮
 * @ chan7: rtrm  右滚轮
 * @ chan8: 
 * @ chan9: 
 * @ chan10:
 * @ chan11:
 * @ chan12:
 * @ chan13:
 * @ chan14:
*******************************************************/
void channelUpdate(void)
{   
   g_rcChannel.chan1 = g_eeGeneral.joystick.ele;
   g_rcChannel.chan2 = g_eeGeneral.joystick.rud;
   g_rcChannel.chan3 = g_eeGeneral.joystick.thr;
   g_rcChannel.chan4 = g_eeGeneral.joystick.ail;
   
   g_rcChannel.chan5 = g_eeGeneral.flightMode;
   g_rcChannel.chan6 = g_eeGeneral.joystick.ltrm;
   g_rcChannel.chan7 = g_eeGeneral.joystick.rtrm;
#if defined (VTOL_MODE_CONTROL)
   g_rcChannel.chan8 = g_eeGeneral.vtolMode;     //! vtol mode control  use channel 8
#elif defined (THROW_CONTROL)	
   g_rcChannel.chan8 = g_eeGeneral.throwMode;    //! throw mode control  use channel 8
#endif	
   g_rcChannel.chan9 = 1500;
   g_rcChannel.chan10 = 1500;
   g_rcChannel.chan11 = 1500;
   g_rcChannel.chan12 = 1500;
   g_rcChannel.chan13 = 1500;
   g_rcChannel.chan14 = 1500; 
}







//annotated by apple : i think we will refer to  the functions later, but we do not use it right now!!!
/* #if defined(PCBTARANIS) && defined(REV9E)
#define PWR_PRESS_SHUTDOWN             300 // 3s

const pm_uchar bmp_shutdown[] PROGMEM = {
  #include "../../bitmaps/Taranis/shutdown.lbm"
};

uint32_t pwr_press_time = 0;

uint32_t pwrPressedDuration()
{
  if (pwr_press_time == 0) {
    return 0;
  }
  else {
    return get_tmr10ms() - pwr_press_time;
  }
}

uint32_t pwrCheck()
{
  enum PwrCheckState {
    PWR_CHECK_ON,
    PWR_CHECK_OFF,
    PWR_CHECK_PAUSED,
  };

  static uint8_t pwr_check_state = PWR_CHECK_ON;

  if (pwr_check_state == PWR_CHECK_OFF) {
    return e_power_off;
  }
  else if (pwrPressed()) {
    if (pwr_check_state == PWR_CHECK_PAUSED) {
      // nothing
    }
    else if (pwr_press_time == 0) {
      pwr_press_time = get_tmr10ms();
    }
    else {
      if (get_tmr10ms() - pwr_press_time > PWR_PRESS_SHUTDOWN) {
#if defined(SHUTDOWN_CONFIRMATION)
        while (1) {
          lcd_clear();
          POPUP_CONFIRMATION("Confirm Shutdown");
          uint8_t evt = getEvent(false);
          DISPLAY_WARNING(evt);
          lcdRefresh();
          if (warningResult == true) {
            pwr_check_state = PWR_CHECK_OFF;
            return e_power_off;
          }
          else if (!warningText) {
            // shutdown has been cancelled
            pwr_check_state = PWR_CHECK_PAUSED;
            return e_power_on;
          }
        }
#else
        haptic.play(15, 3, PLAY_NOW);
        pwr_check_state = PWR_CHECK_OFF;
        return e_power_off;
#endif
      }
      else {
        unsigned index = pwrPressedDuration() / (PWR_PRESS_SHUTDOWN / 4);
        lcd_clear();
        lcd_bmp(76, 2, bmp_shutdown, index*60, 60);
        lcdRefresh();
        return e_power_press;
      }
    }
  }
  else {
    pwr_check_state = PWR_CHECK_ON;
    pwr_press_time = 0;
  }

  return e_power_on;
}
#endif */
