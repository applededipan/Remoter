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

#include <string.h>
#include "board_taranis.h"
#include "../../thirdparty/FatFs/diskio.h"
#include "../../thirdparty/FatFs/ff.h"
#include "../../thirdparty/CoOS/kernel/CoOS.h"
#include "hal.h"
#include "../../debug.h"
#include "../../serial.h"

//! Definitions for MMC/SDC command 
#define CMD0    (0x40+0)        //! GO_IDLE_STATE 
#define CMD1    (0x40+1)        //! SEND_OP_COND (MMC) 
#define ACMD41  (0xC0+41)       //! SEND_OP_COND (SDC) 
#define CMD8    (0x40+8)        //! SEND_IF_COND 
#define CMD9    (0x40+9)        //! SEND_CSD 
#define CMD10   (0x40+10)       //! SEND_CID 
#define CMD12   (0x40+12)       //! STOP_TRANSMISSION 
#define ACMD13  (0xC0+13)       //! SD_STATUS (SDC) 
#define CMD16   (0x40+16)       //! SET_BLOCKLEN 
#define CMD17   (0x40+17)       //! READ_SINGLE_BLOCK 
#define CMD18   (0x40+18)       //! READ_MULTIPLE_BLOCK 
#define CMD23   (0x40+23)       //! SET_BLOCK_COUNT (MMC) 
#define ACMD23  (0xC0+23)       //! SET_WR_BLK_ERASE_COUNT (SDC) 
#define CMD24   (0x40+24)       //! WRITE_BLOCK 
#define CMD25   (0x40+25)       //! WRITE_MULTIPLE_BLOCK 
#define CMD55   (0x40+55)       //! APP_CMD 
#define CMD58   (0x40+58)       //! READ_OCR 
#define CMD59   (0x40+59)


#define BOOL   bool
#define FALSE  false
#define TRUE   true

//! Card type flags (CardType) 
#define CT_MMC              0x01
#define CT_SD1              0x02
#define CT_SD2              0x04
#define CT_SDC             (CT_SD1|CT_SD2)
#define CT_BLOCK            0x08

//! SPI: SD driver  
#define SD_SELECT()        GPIO_ResetBits(SD_GPIO, SD_GPIO_PIN_CS)    //! MMC CS = L 
#define SD_DESELECT()      GPIO_SetBits(SD_GPIO, SD_GPIO_PIN_CS)      //! MMC CS = H 


/****************************************************************
  * @brief  Lock / unlock functions   
****************************************************************/
#if !defined(BOOT)

static OS_MutexID ioMutex;
volatile int mutexCheck = 0;

int ff_cre_syncobj(BYTE vol, _SYNC_t *mutex)
{
  *mutex = ioMutex;
  return 1;
}

int ff_req_grant(_SYNC_t mutex)
{
  return CoEnterMutexSection(mutex) == E_OK;
}

void ff_rel_grant(_SYNC_t mutex)
{
  CoLeaveMutexSection(mutex);
}

int ff_del_syncobj(_SYNC_t mutex)
{
  return 1;
}
#endif

static const DWORD socket_state_mask_cp = (1 << 0);
static const DWORD socket_state_mask_wp = (1 << 1);

static volatile DSTATUS Stat = STA_NOINIT;      //! Disk status */
static volatile DWORD Timer1, Timer2;           //! 100Hz decrement timers */

BYTE CardType;                                  //! Card type flags */

enum SPEED_SETTING {INTERFACE_SLOW, INTERFACE_FAST};



/****************************************************************
  * @brief  set interface speed   
****************************************************************/
static void interfaceSpeed(enum SPEED_SETTING speed)
{
  DWORD tmp;
  tmp = SPI->CR1;
  if(speed == INTERFACE_SLOW) 
  {  
    tmp = (tmp | SPI_BaudRatePrescaler_128);//! Set slow clock (100k-400k) 
  }
  else 
  {
    tmp = (tmp & ~SPI_BaudRatePrescaler_128) | SPI_SPI_BaudRatePrescaler;//! Set fast clock (depends on the CSD) 
  }
  SPI->CR1 = tmp;
}



static inline DWORD socket_is_write_protected(void)
{
  return 0; //! fake not protected 
}



static inline DWORD socket_is_empty(void)
{
  return 0;//return !SD_CARD_PRESENT(); //! fake inserted 
}



static int chk_power(void)
{
  return 1; //! fake powered 
}



/****************************************************************
  * @brief  Alternative macro to receive data fas  
****************************************************************/
#define rcvr_spi_m(dst)  *(dst)=spiReadWriteByte(0xff)



/****************************************************************
  * @brief  Wait for sdcard ready 

  * @return 0xFF: response ok     others: wrong  
****************************************************************/
static uint8_t spiCardReady(void)
{
  uint8_t res;

  Timer2 = 50;                 //! Wait for ready in timeout of 500ms 
  spiReadWriteByte(0xFF);
  do 
  {
    res = spiReadWriteByte(0xFF);
  } while ((res != 0xFF) && Timer2);

  return res;
}



/****************************************************************
  * @brief  Reset spi   
****************************************************************/
static void spiReset(void)
{
  for(int n=0; n<520; ++n) 
  {
    spiReadWriteByte(0xFF);  
  }
  TRACE_SD_CARD_EVENT(1, sd_spi_reset, 0);
}



/****************************************************************
  * @brief  Deselect the card and release SPI bus   
****************************************************************/
static void releaseSpi(void)
{
  SD_DESELECT();
  spiReadWriteByte(0xFF);
}



#ifdef SD_USE_DMA
/****************************************************************
  * @brief  Transmit/Receive Block using DMA (Platform dependent. STM32 here) 
  
  * @param  receive  FALSE for buff->SPI, TRUE for SPI->buff
  * @param *buff     receive TRUE  : 512 byte data block to be transmitted receive FALSE : Data buffer to store received data
  * @param  btr      receive TRUE  : Byte count (must be multiple of 2)receive FALSE : Byte count (must be 512)
****************************************************************/
static void stDmaTransfer(BOOL receive, const BYTE *buff, UINT btr)
{
  DMA_InitTypeDef DMA_InitStructure;
  WORD rw_workbyte[] = {0xffff};

  DMA_DeInit(SD_DMA_Stream_SPI_RX);
  DMA_DeInit(SD_DMA_Stream_SPI_TX);
  
  //! shared DMA configuration values between SPI2 RX & TX 
  DMA_InitStructure.DMA_Channel = SD_DMA_Channel_SPI;        //! the same channel
  DMA_InitStructure.DMA_PeripheralBaseAddr = (DWORD)(&(SPI->DR));
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_BufferSize = btr;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

  //! separate RX & TX
  if (receive) 
  {
    DMA_InitStructure.DMA_Memory0BaseAddr = (DWORD)buff;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_Init(SD_DMA_Stream_SPI_RX, &DMA_InitStructure);
    DMA_InitStructure.DMA_Memory0BaseAddr = (DWORD)rw_workbyte;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_Init(SD_DMA_Stream_SPI_TX, &DMA_InitStructure);
  }
  else 
  {
#if _FS_READONLY == 0
    DMA_InitStructure.DMA_Memory0BaseAddr = (DWORD)rw_workbyte;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_Init(SD_DMA_Stream_SPI_RX, &DMA_InitStructure);
    DMA_InitStructure.DMA_Memory0BaseAddr = (DWORD)buff;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_Init(SD_DMA_Stream_SPI_TX, &DMA_InitStructure);
#endif
  }

  //! Enable DMA Channels 
  DMA_Cmd(SD_DMA_Stream_SPI_RX, ENABLE);
  DMA_Cmd(SD_DMA_Stream_SPI_TX, ENABLE);

  //! Enable SPI TX/RX request 
  SPI_I2S_DMACmd(SPI, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);

  while (DMA_GetFlagStatus(SD_DMA_Stream_SPI_TX, SD_DMA_FLAG_SPI_TC_TX) == RESET) { ; }
  while (DMA_GetFlagStatus(SD_DMA_Stream_SPI_RX, SD_DMA_FLAG_SPI_TC_RX) == RESET) { ; }

  //! Disable DMA Channels 
  DMA_Cmd(SD_DMA_Stream_SPI_RX, DISABLE);
  DMA_Cmd(SD_DMA_Stream_SPI_TX, DISABLE);

  //! Disable SPI RX/TX request 
  SPI_I2S_DMACmd(SPI, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, DISABLE);
}
#endif /* SD_USE_DMA */



/****************************************************************
  * @brief  Power Control and interface-initialization (Platform dependent)   
****************************************************************/
static void spiCardPowerOn(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  //! Configure I/O for Flash Chip select 
  GPIO_InitStructure.GPIO_Pin   = SD_GPIO_PIN_CS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(SD_GPIO, &GPIO_InitStructure);

  SD_DESELECT();
}



/****************************************************************
  * @brief  Power Control and interface-initialization (Platform dependent)   
****************************************************************/
static void spiCardPowerOff(void)
{	
  if (!(Stat & STA_NOINIT)) 
  {
    SD_SELECT();
    spiCardReady();
    releaseSpi();
  }
  Stat |= STA_NOINIT; //! Set STA_NOINIT 
}


/****************************************************************
  * @brief  receive a data packet from MMC

  * @param   *buff: Data buffer to store received data
  * @param     len: Byte count (must be multiple of 4)
  * @return   
****************************************************************/
static BOOL spiCardReceiveData(uint8_t *buff, uint16_t len)
{
  uint8_t token;
  Timer1 = 10;
  do 
  {                                         //! Wait for data packet in timeout of 100ms 
    token = spiReadWriteByte(0xFF);
  } while((token == 0xFF) && Timer1);
  if(token != 0xFE) 
  {
    TRACE_SD_CARD_EVENT(1, sd_rcvr_datablock, ((uint32_t)(Timer1) << 24) + ((uint32_t)(len) << 8) + token);
    spiReset();
    return FALSE;                                      //! If not valid data token, return with error 
  }
#if defined(SD_USE_DMA)
  stDmaTransfer(TRUE, buff, len);
#else
  do 
  {                                         //! Receive the data block into buffer 
    rcvr_spi_m(buff++);
    rcvr_spi_m(buff++);
    rcvr_spi_m(buff++);
    rcvr_spi_m(buff++);
  }while(len -= 4);
#endif /* SD_USE_DMA */

  spiReadWriteByte(0xFF);                   //! Discard CRC 
  spiReadWriteByte(0xFF);

  return TRUE;                              //! Return with success 
}



/****************************************************************
  * @brief  Send a data packet to MMC

  * @param   *buff: //! 512 byte data block to be transmitted 
  * @param   token: //! Data/Stop token 
  * @return   
****************************************************************/
#define DATA_RESPONSE_TIMEOUT   10

static BOOL spiCardSendData(const uint8_t *buff, uint8_t token)
{
  uint8_t res;
#ifndef SD_USE_DMA
  uint8_t wc;
#endif
  if(spiCardReady() != 0xFF) 
  {
    TRACE_SD_CARD_EVENT(1, sd_xmit_datablock_wait_ready, token);
    spiReset();
    return FALSE;
  }

  spiReadWriteByte(token);                    //! transmit data token 
  if(token != 0xFD) 
  {                                           //! Is data token 
#if defined(SD_USE_DMA)
    stDmaTransfer(FALSE, buff, 512);
#else
    wc = 0;
    do 
	{                                         //! transmit the 512 byte data block to MMC 
      spiReadWriteByte(*buff++);
      spiReadWriteByte(*buff++);
    } while (--wc);
#endif /* SD_USE_DMA */

    spiReadWriteByte(0xFF);                   //! CRC (Dummy) 
    spiReadWriteByte(0xFF);

    /*
    Despite what the SD card standard says, the reality is that (at least for some SD cards)
    the Data Response byte does not come immediately after the last byte of data.
    This delay only happens very rarely, but it does happen. Typical response delay is some 10ms
    */
    Timer2 = DATA_RESPONSE_TIMEOUT;   
    do 
	{
      res = spiReadWriteByte(0xFF);           //! Receive data response 
      if((res & 0x1F) == 0x05) 
	  {
        TRACE_SD_CARD_EVENT((Timer2 != DATA_RESPONSE_TIMEOUT), sd_xmit_datablock_rcvr_spi, ((uint32_t)(Timer2)<< 16) + ((uint32_t)(res)<<8) + token);
        return TRUE;
      }
      if(res != 0xFF) 
	  {
        TRACE_SD_CARD_EVENT(1, sd_xmit_datablock_rcvr_spi, ((uint32_t)(Timer2)<<16) + ((uint32_t)(res)<< 8) + token);
        spiReset();
        return FALSE;
      }
    } while(Timer2);
    TRACE_SD_CARD_EVENT(1, sd_xmit_datablock_rcvr_spi, ((uint32_t)(Timer2)<<16) + ((uint32_t)(res)<<8) + token);
    return FALSE;
  }

  return TRUE;
}



/****************************************************************
  * @brief  Send a command packet to MMC  
  * @param  cmd: Command byte
  * @param  arg: Argument
****************************************************************/
static uint8_t spiCardSendCmd(uint8_t cmd, uint32_t arg)
{
  uint8_t n, res;
  if(cmd & 0x80) 
  {       
    cmd &= 0x7F;                                   //! ACMD<n> is the command sequence of CMD55-CMD<n> 
    res = spiCardSendCmd(CMD55, 0);
    if (res > 1) return res;
  }

  //! Select the card and wait for ready 
  SD_SELECT();
  if(spiCardReady() != 0xFF) 
  {
    TRACE_SD_CARD_EVENT(1, sd_send_cmd_wait_ready, cmd);
    spiReset();
    return 0xFF;
  }

  //! Send command packet 
  spiReadWriteByte(cmd);                           //! Start + Command index 
  spiReadWriteByte((uint8_t)(arg>>24));            //! Argument[31..24] 
  spiReadWriteByte((uint8_t)(arg>>16));            //! Argument[23..16] 
  spiReadWriteByte((uint8_t)(arg>> 8));            //! Argument[15..8] 
  spiReadWriteByte((uint8_t)arg);                  //! Argument[7..0] 
  n = 0x01;                                        //! Dummy CRC + Stop 
  if (cmd == CMD0) n = 0x95;                       //! Valid CRC for CMD0(0) 
  if (cmd == CMD8) n = 0x87;                       //! Valid CRC for CMD8(0x1AA) 
  spiReadWriteByte(n);

  //! Receive command response 
  if (cmd == CMD12) spiReadWriteByte(0xFF);        //! Skip a stuff byte when stop reading 

  n = 10;                                          //! Wait for a valid response in timeout of 10 attempts 
  do 
  {
    res = spiReadWriteByte(0xFF);
  } while((res & 0x80) && --n);

  TRACE_SD_CARD_EVENT((res > 1), sd_send_cmd_rcvr_spi, ((uint32_t)(n)<<16) + ((uint32_t)(res)<<8) + cmd);

  return res;                                      //! Return with the response value 
}







/****************************************************************
  *
  *                Public Functions
  *
****************************************************************/


/****************************************************************
  * @brief  Initialize Disk Drive 
****************************************************************/
DSTATUS diskInit(BYTE drv)                     //! Physical drive number (0) 
{
  BYTE n, cmd, ty, ocr[4];

  if (drv) return STA_NOINIT;                  //! Supports only single drive    0x01
  if (Stat & STA_NODISK)  return Stat;         //! No card in the socket 	     0x02
	  
  spiCardPowerOn();                            //! Force socket power on and initialize interface 
  interfaceSpeed(INTERFACE_SLOW);
  for (n = 10; n; n--) spiReadWriteByte(0xFF); //! 80 dummy clocks 

  ty = 0;
  if(spiCardSendCmd(CMD0, 0) == 1) 
  {                                            //! Enter Idle state 
    Timer1 = 100;                              //! Initialization timeout of 1000 milliseconds 
    if (spiCardSendCmd(CMD8, 0x1AA) == 1) 
	{                                          //! SDHC 
      for (n = 0; n < 4; n++) ocr[n] = spiReadWriteByte(0xFF);    //! Get trailing return value of R7 response 
      if (ocr[2] == 0x01 && ocr[3] == 0xAA) 
	  {                                                           //! The card can work at VDD range of 2.7-3.6V */
        while (Timer1 && spiCardSendCmd(ACMD41, 1UL << 30));      //! Wait for leaving idle state (ACMD41 with HCS bit) 
        if (Timer1 && spiCardSendCmd(CMD58, 0) == 0) 
		{                                                         //! Check CCS bit in the OCR 
          for (n = 0; n < 4; n++) ocr[n] = spiReadWriteByte(0xFF);
          ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
        }
      }
    } 
	else 
	{                                                             //! SDSC or MMC 
      if(spiCardSendCmd(ACMD41, 0) <= 1)   
	  {
        ty = CT_SD1; cmd = ACMD41;                                //! SDSC 
      } 
	  else 
	  {
        ty = CT_MMC; cmd = CMD1;                                  //! MMC 
      }
      while(Timer1 && spiCardSendCmd(cmd, 0));                    //! Wait for leaving idle state 
      if(!Timer1 || spiCardSendCmd(CMD16, 512) != 0)              //! Set R/W block length to 512 
        ty = 0;
    }
  }
  CardType = ty;
  releaseSpi();

  if(ty)
  {                                                               //! Initialization succeeded 
    Stat &= ~STA_NOINIT;                                          //! Clear STA_NOINIT 
    interfaceSpeed(INTERFACE_FAST);
  }
  else 
  {                                                               //! Initialization failed 
    spiCardPowerOff();
  }

  return Stat;
}



/****************************************************************
  * @brief  Get Disk Status 
****************************************************************/
DSTATUS getDiskStatus(BYTE drv) //! Physical drive number (0) 
{
  if(drv) return STA_NOINIT;    //! Supports only single drive (0)
  return Stat;
}



/****************************************************************
  * @brief  Read Sector(s) 
****************************************************************/
int8_t SD_ReadSectors(uint8_t *buff, uint32_t sector, uint32_t count)
{
  if(!(CardType & CT_BLOCK)) sector *= 512;      //! Convert to byte address if needed 

  if(count == 1) 
  {                                              //! Single block read 
    if(spiCardSendCmd(CMD17, sector) == 0)       
	{                                            //! READ_SINGLE_BLOCK 
      if(spiCardReceiveData(buff, 512)) 
	  {
        count = 0;
      }
    }
    else 
	{
      spiReset();
    }
  }
  else 
  {                                              //! Multiple block read 
    if(spiCardSendCmd(CMD18, sector) == 0) 
	{                                            //! READ_MULTIPLE_BLOCK 
      do 
	  {
        if(!spiCardReceiveData(buff, 512)) 
		{
          break;
        }
        buff += 512;
      } while (--count);
      spiCardSendCmd(CMD12, 0);                  //! STOP_TRANSMISSION 
    }
    else 
	{
      spiReset();
    }
  }
  releaseSpi();
  TRACE_SD_CARD_EVENT((count != 0), sd_SD_ReadSectors, (count << 24) + ((sector/((CardType & CT_BLOCK) ? 1 : 512)) & 0x00FFFFFF));

  return count ? -1 : 0;
}



/****************************************************************
  * @brief  Write Sector(s) 
****************************************************************/
int8_t SD_WriteSectors(const uint8_t *buff, uint32_t sector, uint32_t count)
{
  if(!(CardType & CT_BLOCK)) sector *= 512;      //! Convert to byte address if needed 

  if(count == 1) 
  {                                              //! Single block write 
    if(spiCardSendCmd(CMD24, sector) == 0) 
	{                                            //! WRITE_BLOCK 
      if(spiCardSendData(buff, 0xFE)) 
	  {
        count = 0;
      }
    }
    else 
	{
      spiReset();
    }
  }
  else 
  {                                              //! Multiple block write 
    if(CardType & CT_SDC) spiCardSendCmd(ACMD23, count);
    if(spiCardSendCmd(CMD25, sector) == 0) 
	{                                            //! WRITE_MULTIPLE_BLOCK 
      do 
	  {
        if(!spiCardSendData(buff, 0xFC)) break;
        buff += 512;
      } while (--count);
      if(!spiCardSendData(0, 0xFD))              //! STOP_TRAN token 
        count = 1;
    }
    else 
	{
      spiReset();
    }
  }
  releaseSpi();
  TRACE_SD_CARD_EVENT((count != 0), sd_SD_WriteSectors, (count << 24) + ((sector/((CardType & CT_BLOCK) ? 1 : 512)) & 0x00FFFFFF));

  return count ? -1 : 0;
}



/****************************************************************
  * @brief  Read disk 
  
  * @param    drv: //! default Physical drive number (0)
  * @param  *buff: //! Pointer to the data buffer to store the read data
  * @param sector: //! Start sector number (LBA)
  * @param  count: //! Sector count (1..255)
****************************************************************/
DRESULT diskRead(BYTE drv, BYTE *buff, DWORD sector, UINT count)
{
  if(drv || !count)     return RES_PARERR; //! support only physical driver number(0) and count can not be 0
  if(Stat & STA_NOINIT) return RES_NOTRDY;
  int8_t res = SD_ReadSectors(buff, sector, count);
  TRACE_SD_CARD_EVENT((res != 0), sd_disk_read, (count << 24) + (sector & 0x00FFFFFF));
  return (res != 0) ? RES_ERROR : RES_OK;
}



#if _FS_READONLY == 0
/****************************************************************
  * @brief  Write disk 
  
  * @param    drv: //! Physical drive number (0)
  * @param  *buff: //! Pointer to the data buffer to be written
  * @param sector: //! Start sector number (LBA)
  * @param  count: //! Sector count (1..255)
****************************************************************/
DRESULT diskWrite(BYTE drv, const BYTE *buff, DWORD sector, UINT count)
{
  if (drv || !count) return RES_PARERR;
  if (Stat & STA_NOINIT) return RES_NOTRDY;
  if (Stat & STA_PROTECT) return RES_WRPRT;
  int8_t res = SD_WriteSectors(buff, sector, count);
  TRACE_SD_CARD_EVENT((res != 0), sd_disk_write, (count << 24) + (sector & 0x00FFFFFF));
  return (res != 0) ? RES_ERROR : RES_OK;
}
#endif /* _READONLY == 0 */



/****************************************************************
  * @brief  Miscellaneous Functions 
  
  * @param    drv: //! Physical drive number (0)
  * @param   ctrl: //! Control code
  * @param  *buff: //! Buffer to send/receive control data  
****************************************************************/
DRESULT diskIoctl(BYTE drv, BYTE ctrl, void *buff)
{
  DRESULT res;
  BYTE n, csd[16], *ptr = (BYTE *)buff;
  WORD csize;

  if (drv) return RES_PARERR;

  res = RES_ERROR;

  if(ctrl == CTRL_POWER) 
  {
    switch (*ptr) 
	{
    case 0:                //! Sub control code == 0 (POWER_OFF) 
      if(chk_power())
        spiCardPowerOff(); //! Power off 
      res = RES_OK;
      break;
    case 1:                //! Sub control code == 1 (POWER_ON) 
      spiCardPowerOn();    //! Power on 
      res = RES_OK;
      break;
    case 2:                //! Sub control code == 2 (POWER_GET) 
      *(ptr+1) = (BYTE)chk_power();
      res = RES_OK;
      break;
    default :
      res = RES_PARERR;
    }
  }
  else 
  {
    if(Stat & STA_NOINIT) 
	{
      return RES_NOTRDY;
    }

    switch(ctrl) 
	{
    case CTRL_SYNC :                                            //! Make sure that no pending write process 
      SD_SELECT();
      if (spiCardReady() == 0xFF) 
	  {
        res = RES_OK;
      }
      else 
	  {
        TRACE_SD_CARD_EVENT(1, sd_disk_ioctl_CTRL_SYNC, 0);
      }
      break;

    case GET_SECTOR_COUNT :                                     //! Get number of sectors on the disk (DWORD) 
      if((spiCardSendCmd(CMD9, 0) == 0) && spiCardReceiveData(csd, 16)) 
	  {
        if ((csd[0] >> 6) == 1) 
		{                                                       //! SDC version 2.00 
          csize = csd[9] + ((WORD)csd[8] << 8) + 1;
          *(DWORD*)buff = (DWORD)csize << 10;
        } 
		else 
		{                                                       //! SDC version 1.XX or MMC
          n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
          csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
          *(DWORD*)buff = (DWORD)csize << (n - 9);
        }
        res = RES_OK;
      }
      else 
	  {
        TRACE_SD_CARD_EVENT(1, sd_disk_ioctl_GET_SECTOR_COUNT, 0);
      }
      break;

    case GET_SECTOR_SIZE :                                      //! Get R/W sector size (WORD) 
      *(WORD*)buff = 512;
      res = RES_OK;
      break;

    case GET_BLOCK_SIZE :                                       //! Get erase block size in unit of sector (DWORD) 
      if(CardType & CT_SD2) 
	  {                                                         //! SDC version 2.00 
        if(spiCardSendCmd(ACMD13, 0) == 0) 
		{                                                       //! Read SD status 
          spiReadWriteByte(0xFF);
          if(spiCardReceiveData(csd, 16)) 
		  {                                                     //! Read partial block 
            for (n = 64 - 16; n; n--) spiReadWriteByte(0xFF);   //! Purge trailing data 
            *(DWORD*)buff = 16UL << (csd[10] >> 4);
            res = RES_OK;
          }
        }
      } 
	  else
	  {                                                         //! SDC version 1.XX or MMC 
        if((spiCardSendCmd(CMD9, 0) == 0) && spiCardReceiveData(csd, 16)) 
		{                                                       //! Read CSD */
          if(CardType & CT_SD1) 
		  {                                                     //! SDC version 1.XX */
            *(DWORD*)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
          } 
		  else 
		  {                                                     //! MMC 
            *(DWORD*)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
          }
          res = RES_OK;
        }
      }
      break;

    case MMC_GET_TYPE :                                         //! Get card type flags (1 byte) 
      *ptr = CardType;
      res = RES_OK;
      break;

    case MMC_GET_CSD :                                          //! Receive CSD as a data block (16 bytes) 
      if((spiCardSendCmd(CMD9,0) == 0) && spiCardReceiveData(ptr, 16)) //! READ_CSD 
	  {
        res = RES_OK;
      }
      else 
	  {
        TRACE_SD_CARD_EVENT(1, sd_disk_ioctl_MMC_GET_CSD, 0);
      }
      break;

    case MMC_GET_CID :                                          //! Receive CID as a data block (16 bytes) */
      if((spiCardSendCmd(CMD10, 0) == 0) && spiCardReceiveData(ptr, 16)) //! READ_CID 
	  {
        res = RES_OK;
      }
      else 
	  {
        TRACE_SD_CARD_EVENT(1, sd_disk_ioctl_MMC_GET_CID, 0);
      }
      break;

    case MMC_GET_OCR :                                          //! Receive OCR as an R3 resp (4 bytes) 
      if(spiCardSendCmd(CMD58, 0) == 0) 
	  {                                                         //! READ_OCR 
        for (n = 4; n; n--) *ptr++ = spiReadWriteByte(0xFF);
        res = RES_OK;
      }
      else 
	  {
        TRACE_SD_CARD_EVENT(1, sd_disk_ioctl_MMC_GET_OCR, 0);
      }
      break;

    case MMC_GET_SDSTAT :                                       //! Receive SD status as a data block (64 bytes) 
      if(spiCardSendCmd(ACMD13, 0) == 0) 
	  {                                                         //! SD_STATUS 
        spiReadWriteByte(0xFF);
        if(spiCardReceiveData(ptr, 64)) 
		{
          res = RES_OK;
        }
        else 
		{
          TRACE_SD_CARD_EVENT(1, sd_disk_ioctl_MMC_GET_SDSTAT_1, 0);
        }
      }
      else 
	  {
        TRACE_SD_CARD_EVENT(1, sd_disk_ioctl_MMC_GET_SDSTAT_2, 0);
      }
      break;

    default:
      res = RES_PARERR;
    }
    releaseSpi();
  }
  return res;
}



/****************************************************************
  * @brief  Device Timer Interrupt Procedure  (Platform dependent)
  * @brief  This function must be called in period of 10ms    
****************************************************************/
void sdPoll10ms(void)
{
  static DWORD pv;
  DWORD ns;
  BYTE n, s;

  n = Timer1;                           //! 100Hz decrement timers 
  if (n) Timer1 = --n;
  n = Timer2;
  if (n) Timer2 = --n;

  ns = pv;
  pv = socket_is_empty() | socket_is_write_protected();   //! Sample socket switch 

  if (ns == pv) 
  {                                     //! Have contacts stabled? 
    s = Stat;

    if (pv & socket_state_mask_wp)      //! WP is H (write protected) 
            s |= STA_PROTECT;
    else                                //! WP is L (write enabled) 
            s &= ~STA_PROTECT;

    if (pv & socket_state_mask_cp)      //! INS = H (Socket empty) 
            s |= (STA_NODISK | STA_NOINIT);
    else                                //! INS = L (Card inserted) 
            s &= ~STA_NODISK;
    Stat = s;
  }
}

// TODO everything here should not be in the driver layer ...

FATFS g_FATFS_Obj;
#if defined(SPORT_FILE_LOG)
FIL g_telemetryFile = {0};
#endif

#if defined(BOOT)
/****************************************************************
  * @return 0: ok    1: failed 
****************************************************************/
uint8_t sdInit(void)
{
  if(f_mount(&g_FATFS_Obj, "", 1) == FR_OK) 
  {	  
    f_chdir("/");
	return 0;
  }
  else 
  {
	return 1;
  }
}
#else
uint8_t sdInit(void)
{
  ioMutex = CoCreateMutex();
  if(ioMutex >= CFG_MAX_MUTEX) 
  {
	return 1; //! sd error
  }
  if(f_mount(&g_FATFS_Obj, "", 1) == FR_OK) 
  {    
    sdGetFreeSectors(); //! call sdGetFreeSectors() now because f_getfree() takes a long time first time it's called
    referenceSystemAudioFiles();   
#if defined(SPORT_FILE_LOG)
    // annotated by apple
    // f_open(&g_telemetryFile, LOGS_PATH "/sport.log", FA_OPEN_ALWAYS | FA_WRITE);
    // if(f_size(&g_telemetryFile) > 0) 
	// {
      // f_lseek(&g_telemetryFile, f_size(&g_telemetryFile)); // append
    // }
#endif
    return 0;
  }
  else 
  {
	 return 1;
  }
}


void sdDone(void)
{
  if(sdMounted()) 
  {
    audioQueue.stopSD();
#if defined(SPORT_FILE_LOG)
    f_close(&g_telemetryFile);
#endif
    f_mount(NULL, "", 0); //! unmount SD
  }
}
#endif


uint32_t sdMounted(void)
{
  return g_FATFS_Obj.fs_type != 0;
}


uint32_t sdIsHC(void)
{
  return (CardType & CT_BLOCK);
}


uint32_t sdGetSpeed(void)
{
  return 330000;
}
























	
	
	
	


