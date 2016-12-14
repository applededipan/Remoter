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

#ifndef _BOARD_TARANIS_H_
#define _BOARD_TARANIS_H_

#include "stddef.h"
#include "../../global.h"

#if defined(__cplusplus) && !defined(SIMU)
extern "C" {
#endif

#if defined(REVPLUS)
  #include "STM32F2xx_StdPeriph_Lib_V1.1.0/Libraries/CMSIS/Device/ST/STM32F2xx/Include/stm32f2xx.h"
  #include "STM32F2xx_StdPeriph_Lib_V1.1.0/Libraries/STM32F2xx_StdPeriph_Driver/inc/stm32f2xx_rcc.h"
  #include "STM32F2xx_StdPeriph_Lib_V1.1.0/Libraries/STM32F2xx_StdPeriph_Driver/inc/stm32f2xx_gpio.h"
  #include "STM32F2xx_StdPeriph_Lib_V1.1.0/Libraries/STM32F2xx_StdPeriph_Driver/inc/stm32f2xx_adc.h"
  #include "STM32F2xx_StdPeriph_Lib_V1.1.0/Libraries/STM32F2xx_StdPeriph_Driver/inc/stm32f2xx_spi.h"
  #include "STM32F2xx_StdPeriph_Lib_V1.1.0/Libraries/STM32F2xx_StdPeriph_Driver/inc/stm32f2xx_i2c.h"
  #include "STM32F2xx_StdPeriph_Lib_V1.1.0/Libraries/STM32F2xx_StdPeriph_Driver/inc/stm32f2xx_rtc.h"
  #include "STM32F2xx_StdPeriph_Lib_V1.1.0/Libraries/STM32F2xx_StdPeriph_Driver/inc/stm32f2xx_pwr.h"
  #include "STM32F2xx_StdPeriph_Lib_V1.1.0/Libraries/STM32F2xx_StdPeriph_Driver/inc/stm32f2xx_dma.h"
  #include "STM32F2xx_StdPeriph_Lib_V1.1.0/Libraries/STM32F2xx_StdPeriph_Driver/inc/stm32f2xx_usart.h"
  #include "STM32F2xx_StdPeriph_Lib_V1.1.0/Libraries/STM32F2xx_StdPeriph_Driver/inc/stm32f2xx_flash.h"
  #include "STM32F2xx_StdPeriph_Lib_V1.1.0/Libraries/STM32F2xx_StdPeriph_Driver/inc/stm32f2xx_dbgmcu.h"   
  #include "STM32F2xx_StdPeriph_Lib_V1.1.0/Libraries/STM32F2xx_StdPeriph_Driver/inc/stm32f2xx_fsmc.h"
  #include "STM32F2xx_StdPeriph_Lib_V1.1.0/Libraries/STM32F2xx_StdPeriph_Driver/inc/misc.h"
#endif

#if !defined(SIMU)
  #include "usbd_cdc_core.h"
  #include "usbd_msc_core.h"
  #include "usbd_hid_core.h"
  #include "usbd_usr.h"
  #include "usbd_desc.h"
  #include "usb_conf.h"
  #include "usbd_conf.h"
#endif

#include "hal.h"
#include "aspi.h"

#if defined(__cplusplus) && !defined(SIMU)
}
#endif

#define FLASHSIZE          0x80000
#define BOOTLOADER_SIZE    0x8000
#define FIRMWARE_ADDRESS   0x08000000

#define PERI1_FREQUENCY    30000000
#define PERI2_FREQUENCY    60000000

#define TIMER_MULT_APB1    2
#define TIMER_MULT_APB2    2

#define PIN_MODE_MASK      0x0003
#define PIN_INPUT          0x0000
#define PIN_OUTPUT         0x0001
#define PIN_PERIPHERAL     0x0002
#define PIN_ANALOG         0x0003

#define PIN_PULL_MASK      0x000C
#define PIN_PULLUP         0x0004

#define PIN_PORT_MASK      0x0700
#define PIN_PORTA          0x0000
#define PIN_PORTB          0x0100
#define PIN_PORTC          0x0200
#define PIN_PORTD          0x0300
#define PIN_PORTE          0x0400
#define PIN_PORTF          0x0500

#define PIN_PERI_MASK      0x00F0
#define PIN_PER_1          0x0010
#define PIN_PER_2          0x0020
#define PIN_PER_3          0x0030
#define PIN_PER_5          0x0050
#define PIN_PER_6          0x0060
#define PIN_PER_8          0x0080

#define PIN_SPEED_MASK     0x6000
#define PIN_OS25           0x2000
#define PIN_OS50           0x4000
#define PIN_OS100          0x6000

void configure_pins( uint32_t pins, uint16_t config );

#define strcpy_P strcpy
#define strcat_P strcat

extern uint16_t sessionTimer;

#define SLAVE_MODE()         (g_model.trainerMode == TRAINER_MODE_SLAVE)


#define TRAINER_CONNECTED()  (GPIO_ReadInputDataBit(TRAINER_GPIO_DETECT, TRAINER_GPIO_PIN_DETECT) == Bit_RESET)


#ifdef __cplusplus
extern "C" {
#endif
void delaysInit(void);
void delay_01us(uint16_t nb);
#ifdef __cplusplus
}
#endif








/****************************************************************************/
// Pulses driver
#define INTERNAL_MODULE_ON()      GPIO_SetBits(INTMODULE_GPIO_PWR, INTMODULE_GPIO_PIN_PWR)
#define INTERNAL_MODULE_OFF()     GPIO_ResetBits(INTMODULE_GPIO_PWR, INTMODULE_GPIO_PIN_PWR)
#define IS_INTERNAL_MODULE_ON()   (GPIO_ReadInputDataBit(INTMODULE_GPIO_PWR, INTMODULE_GPIO_PIN_PWR) == Bit_SET)
void init_no_pulses(uint32_t port);
void disable_no_pulses(uint32_t port);
void init_ppm( uint32_t module_index );
void disable_ppm( uint32_t module_index );
void set_external_ppm_parameters(uint32_t idleTime, uint32_t delay, uint32_t positive);
#if defined(TARANIS_INTERNAL_PPM)
  void set_internal_ppm_parameters(uint32_t idleTime, uint32_t delay, uint32_t positive);
#endif
void init_pxx( uint32_t module_index );
void disable_pxx( uint32_t module_index );
void init_dsm2( uint32_t module_index );
void disable_dsm2( uint32_t module_index );
void init_crossfire( uint32_t module_index );
void disable_crossfire( uint32_t module_index );






/****************************************************************************/
// Trainer driver
void init_trainer_ppm(void);
void stop_trainer_ppm(void);
void init_trainer_capture(void);
void stop_trainer_capture(void);
void init_cppm_on_heartbeat_capture(void);
void stop_cppm_on_heartbeat_capture(void);
void init_sbus_on_heartbeat_capture(void);
void stop_sbus_on_heartbeat_capture(void);
void set_trainer_ppm_parameters(uint32_t idleTime, uint32_t delay, uint32_t positive);







/****************************************************************************/
// Keys driver
void     keysInit(void);
void     ledsInit(void);
uint32_t readKeys(void);
uint32_t readTrims(void);
void     readKeysAndTrims(void);
#define TRIMS_PRESSED()           
#define KEYS_PRESSED()             (readKeys())




/****************************************************************************/
//Power driver 
void powerInit(void);
void powerOn(void);
void powerOff(void);
void powerStartup(void);
void powerShutdown(void);
void getBatVoltage(uint8_t *battery);
void powerLowWarn(uint16_t x, uint16_t y, uint8_t bat, uint32_t keyMute);
void powerLowShutdown(uint8_t bat);
uint8_t powerCheck(void);
void systemReboot(void);
uint8_t getMiddleValue(uint8_t a, uint8_t b, uint8_t c);
#define UNEXPECTED_SHUTDOWN()   (g_eeGeneral.unexpectedShutdown)


/****************************************************************************/
//Beep driver
void beepInit(void);
void beepActive(uint8_t state);



/****************************************************************************/
//Motor driver
void motorInit(void);
void motorActive(uint8_t state);



/****************************************************************************/
// WDT driver
#if !defined(SIMU)
#define wdt_disable()
#define wdt_enable(x)   watchdogInit(1500)
#define wdt_feed()      IWDG->KR = 0xAAAA
#define WAS_RESET_BY_SOFTWARE()               (RCC->CSR & RCC_CSR_SFTRSTF)
#define WAS_RESET_BY_WATCHDOG()               (RCC->CSR & (RCC_CSR_WDGRSTF | RCC_CSR_WWDGRSTF))
#define WAS_RESET_BY_WATCHDOG_OR_SOFTWARE()   (RCC->CSR & (RCC_CSR_WDGRSTF | RCC_CSR_WWDGRSTF | RCC_CSR_SFTRSTF))

void watchdogInit(unsigned int duration);
#endif




/****************************************************************************/
// ADC driver

#define NUMBER_ANALOG_ADC1      8
#define BATT_SCALE              150

void adcInit(void);

#if defined(__cplusplus) && !defined(SIMU)
extern "C" {
#endif







/****************************************************************************/


#if defined(REVPLUS)

#define backlightEnable()       
#define backlightDisable()       
#define isBacklightEnable()      ((BACKLIGHT_TIMER->CCR4 != 0) || (BACKLIGHT_TIMER->CCR2 != 0)) 
#endif
 



/****************************************************************************/
// USB driver
int  usbPlugged(void);
void usbInit(void);
void usbDeInit(void);
void usbSerialPutc(uint8_t c);

#if defined(__cplusplus) && !defined(SIMU)
}
#endif



/****************************************************************************/
// Debug driver
void debugPutc(const char c);



/****************************************************************************/
//usart1/usb driver
void usart1UsbInit(uint32_t baudrate);
void usart1UsbStop(void);
void usart1UsbSendChar(uint8_t data);
void usart1UsbSendBuffer(uint8_t *buffer, uint16_t count);
uint8_t usart1UsbPlugged(void);


/****************************************************************************/
//usart2/uav driver
void usart2UavInit(uint32_t baudrate);
void usart2UavStop(void);
void usart2UavSendChar(uint8_t data);
void usart2UavSendBuffer(uint8_t *buffer, uint16_t count);



/****************************************************************************/
//usart3/bth driver
void usart3BthInit(uint32_t baudrate);
void usart3BthWriteWakeup(void);
void usart3BthWakeup(void);
void usart3BthSendChar(uint8_t data);
void usart3BthSendBuffer(uint8_t *buffer, uint16_t count);
int  usart3BthRead(void * buffer, int len);
uint8_t usart3BthReady();



/****************************************************************************/
//usart4/rsp driver
void usart4RspInit(uint32_t baudrate);
void usart4RspStop(void);
void usart4RspSendChar(uint8_t data);
void usart4RspSendBuffer(uint8_t *buffer, uint16_t count);



/****************************************************************************/






//LCD driver
//added by apple
// !"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijklmnopqrstuvwxyz{|}~

// macros about colors
#define   RGB565(color)   ((((color) >> 19) & 0x1f) << 11)|((((color) >> 10) & 0x3f) << 5)|(((color) >> 3) & 0x1f)

#define   BACKCOLOR       0x0000              //背景色
#define   LINEBLACK       0x3186              //分隔线
#define   WHITE           0xFFFF              //白色
#define   LIGHTWHITE      0xE6FB              //浅白色
#define   BLACK           0x0000              //黑色
#define   LIGHTBLACK      0x8410              //浅黑色 
#define   RED             0xF800              //红色
#define   BLUE            0x001F              //蓝色
#define   YELLOW          0xFFE0              //黄色
#define   BROWN 		  0XBC40              //棕色
#define   BRRED 		  0XFC07              //棕红色
#define   GRAY  		  RGB565(0x808080)    //灰色
#define   DARKBLUE        0X01CF	//深蓝色
#define   LIGHTBLUE       0X7D7C	//浅蓝色  
#define   GRAYBLUE        0X5458    //灰蓝色
#define   LIGHTGREEN      0X841F    //浅绿色
#define   DARKGREEN       RGB565(0x006400)    //深绿
#define   GREEN           RGB565(0x66FF00)    //明绿

 










//! Backlight driver 

void lcdInit(void);
void lcdOff(void);
void lcdSleep(void);
void delayms(uint16_t ms);
void lcdScrolDisable(void);
void lcdScrolScreen(uint16_t line);
void lcd_ClearScreen(uint16_t color);
void lcd_SetCursor(uint16_t x, uint16_t y);
void lcd_DrawPoint(uint16_t x, uint16_t y, uint16_t color);
void lcd_DrawLine(uint16_t xstart, uint16_t ystart, uint16_t xend, uint16_t yend, uint16_t color);
void lcd_DrawLineDiff(uint16_t xstart, uint16_t ystart, int16_t xdiff, int16_t ydiff, uint16_t color);
void lcd_DrawCicle(uint16_t xc, uint16_t yc, uint8_t r, uint16_t color);
void lcd_DrawFillCicle(uint16_t xc, uint16_t yc, uint8_t r, uint16_t color);
void lcd_DrawRectangle(uint16_t xstart, uint16_t ystart, uint16_t xend, uint16_t yend, uint16_t color);
void lcd_DrawRectangleDiff(uint16_t xstart, uint16_t ystart, int16_t xdiff, int16_t ydiff, uint16_t color);
void lcd_DrawFillRectangle(uint16_t xstart, uint16_t ystart, uint16_t xend, uint16_t yend, uint16_t color);
void lcd_DrawFillRectangleDiff(uint16_t xstart, uint16_t ystart, int16_t xdiff, int16_t ydiff, uint16_t color);
void lcd_ShowChar(uint16_t x, uint16_t y, uint16_t color, uint8_t size, unsigned char c);
void lcd_ShowCharBackColor(uint16_t x, uint16_t y, uint16_t color, uint8_t size, unsigned char c, uint16_t backcolor);
void lcd_ShowString(uint16_t x, uint16_t y, uint16_t color, uint8_t size, const unsigned char *p);
void lcd_ShowStringBackcolor(uint16_t x, uint16_t y, uint16_t color, uint8_t size, const unsigned char *p, uint16_t backcolor);
void lcd_ShowNum(uint16_t x, uint16_t y, uint16_t color, uint8_t size, uint8_t len, int32_t num, uint8_t flag);
void lcd_ShowNumBackColor(uint16_t x, uint16_t y, uint16_t color, uint8_t size, uint8_t len, int32_t num, uint8_t flag, uint16_t backcolor);
void lcd_ShowFloat(uint16_t x, uint16_t y, uint16_t color, uint8_t size, uint8_t len, float num);
void lcd_DrawBmp(uint16_t x, uint16_t y, uint16_t dx, uint16_t dy, const unsigned char *p);
void lcd_DrawBmp_256(uint16_t x, uint16_t y, uint16_t dx, uint16_t dy, const unsigned char *p);
void lcd_DrawCharge(uint16_t x, uint16_t y, uint16_t charge);
void lcd_ShowTriCursor(uint16_t x, uint16_t y, uint8_t dir, uint16_t color);
void lcd_ShowCircle(uint16_t x, uint16_t y, uint16_t color);
void backLightEnable(uint8_t state, uint16_t ms);



//GUI driver
void displayInit(void);
void displayLogo(void);
void displayCell(void);
void displayComStep(void);
void displayStartmenu(void);
void displayNavigation(void);
void displayArmed(uint16_t x, uint16_t y);
void displayMiniCell(uint16_t x, uint16_t y);
void displayRunState(uint16_t x, uint16_t y, uint8_t value);
void displayGps(uint16_t x, uint16_t y, uint8_t gps, uint8_t fixType);
void displayTime(uint16_t x, uint16_t y, uint8_t size);
void displayRssi(uint16_t x, uint16_t y, uint8_t wifiRssi, uint8_t dataRssi);
void displayPilotTypeFlightmode(uint16_t x, uint16_t y);
void displayJoystick(RC_CHANNEL channels);
void displayAttitude(int16_t pitch, int16_t roll, int16_t heading, int32_t alt, int8_t batRemain);


void displayUavType(uint16_t x, uint16_t y);
void displayContectState(uint16_t x, uint16_t y);
void displayUavFlightMode(uint16_t x, uint16_t y);
void displayUpdating(uint16_t x, uint16_t y);

void displayTest(void);

void view_information(uint16_t id);


void menusProcess(uint32_t keyChoice, uint32_t keyEnter);
void menu_update(uint8_t page);
void menu_main(uint8_t state);
void menu_debug(uint8_t state);
void menu_version(uint8_t state);
void menu_failsafe(uint8_t state);
void menu_flightmode(uint8_t state);
void menu_gimbal(uint8_t state);
void menu_camera(uint8_t state);
void menu_radio(uint8_t state);
void menu_battery(uint8_t state);
void menu_calibJoystick(uint8_t state);
void menu_rcChannels(uint8_t state);
/****************************************************************************/








/****************************************************************************/
//I2C & EEPROM driver M24512

#define E_ADDR_UPDATE         0  //! 0x55: do firmware update  
                                 //! 0x11: has just finished update and jump across powerStartup()
                                 //! others: normal operation	
							 
#define E_ADDR_JOYHOLD        1  //! bytes: sizeof(JoystickScale ~ 40bytes)


void    i2cInit(void);
void    eepromReadBlock(uint8_t * buffer, uint32_t address, uint32_t size);
void    eepromWriteBlock(uint8_t * buffer, uint32_t address, uint32_t size);
uint8_t eepromM24512Check(void);


/****************************************************************************/











/****************************************************************************/
//SPI Driver

void spiInit(void);
void spiSetSpeed(uint8_t speed);
void eraseSector(uint32_t sector);
void writeFlash(uint32_t *address, uint32_t *buffer);
void unlockFlash(void);
void lockFlash(void);
void writeFlash(uint32_t * address, uint32_t * buffer);
uint8_t  spiReadWriteByte(uint8_t data);
uint32_t isFirmwareStart(const void * buffer);
uint32_t isBootloaderStart(const void * buffer);



//SPI:FLASH Driver
#define FLASH_PAGESIZE            256

#define F_ADDR_PICSTARTMENU       0     //! pic 的地址为0-307200 ：1200page=300KB  分配5个Block block0 - block4

#define F_ADDR_FIXED              323072//! 768   占用768个字节 即3个page
#define F_ADDR_QUADROTOR          323840//! 768   占用768个字节 即3个page
#define F_ADDR_HEXAROTOR          324608//! 768   占用768个字节 即3个page
#define F_ADDR_OCTOROTOR          325376//! 768   占用768个字节 即3个page
#define F_ADDR_HELI               326144//! 768   占用768个字节 即3个page
#define F_ADDR_QGC                326912//! 768   占用768个字节 即3个page           

#define F_ADDR_CHAR_24            327680//! 4560  第5个block首地址(block从0开始算)
#define F_ADDR_CHAR_32            393216//! 6080  第6个block首地址
#define F_ADDR_CHAR_36            458752//! 10260 第7个block首地址
#define F_ADDR_CHAR_42            524288//! 11970 第8个block首地址
                                
#define F_ADDR_ARMED              589824//! 1024  占用1024个字节  即4个page //第9个block首地址 
#define F_ADDR_DISARMED           590848//! 1024  占用1024个字节  即4个page

#define F_ADDR_GPS                655360//! 1280  占用1280个字节  即5个page //第10个block首地址 
#define F_ADDR_RSSI               656640//! 1280  占用1280个字节  即5个page

void spiFlashInit(void);
void spiFlashDownloadFont(void);
void spiFlashEraseChip(void); //! not useful yet
void spiFlashEraseSector(uint32_t addr_sect);
void spiFlashEraseBlock32(uint32_t addr_block_32);
void spiFlashEraseBlock64(uint32_t addr_block_64); 
void spiFlashWritePage(uint8_t * buffer, uint32_t writeaddr, uint16_t nbytes);
void spiFlashRead(uint8_t * buffer, uint32_t readaddr, uint16_t nbytes);
uint16_t spiFlashReadID(void);

void readOutFlash(void);

//SPI: HAPTIC 
void hapticOn(void);
void hapticOff(void);
void hapticInit(void);
uint8_t hapticReadXY(uint16_t *x, uint16_t *y);






#if !defined(SIMU) || defined(SIMU_DISKIO)
uint32_t sdIsHC(void);
uint32_t sdGetSpeed(void);
#define SD_IS_HC()              (sdIsHC())
#define SD_GET_SPEED()          (sdGetSpeed())
#define SD_GET_FREE_BLOCKNR()   (sdGetFreeSectors())
#else	
#define SD_IS_HC()              (0)
#define SD_GET_SPEED()          (0)
#endif

uint8_t sdInit(void);
void    sdDone(void);
void    sdPoll10ms(void);
uint32_t sdMounted(void);


/****************************************************************************/








/****************************************************************************/
// Audio driver
#define VOLUME_LEVEL_MAX  23
#define VOLUME_LEVEL_DEF  12
void audioInit(void) ;
void audioEnd(void) ;
void dacStart(void);
void dacStop(void);
void setSampleRate(uint32_t frequency);
extern const int8_t volumeScale[];
/****************************************************************************/







/****************************************************************************/
#define USART_FLAG_ERRORS (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)

#endif
