
//added by apple:16/05/2016

#include "../../opentx.h"

// SD CMDS 	   
#define CMD0    0       //! reset
#define CMD1    1
#define CMD8    8
#define CMD9    9       //! read CSD data
#define CMD10   10      //! read CID data
#define CMD12   12      //! stop data transmition
#define CMD16   16      //! set sector size  should return 0x00
#define CMD17   17      //! read sector
#define CMD18   18      //! read multi sector
#define ACMD23  23      //! 
#define CMD24   24      //! write sector
#define CMD25   25      //! write Multi sector
#define ACMD41  41      //! shoule return 0x00
#define CMD55   55      //! shoule return 0x01
#define CMD58   58      //! read OCR data
#define CMD59   59      //! enable/disable CRC shoule return 0x00

// SD types  
#define SD_TYPE_MMC     0
#define SD_TYPE_V1      1
#define SD_TYPE_V2      2
#define SD_TYPE_V2HC    4	 

#define MSD_DATA_OK                0x05
#define MSD_DATA_CRC_ERROR         0x0B
#define MSD_DATA_WRITE_ERROR       0x0D
#define MSD_DATA_OTHER_ERROR       0xFF

#define MSD_RESPONSE_NO_ERROR      0x00
#define MSD_IN_IDLE_STATE          0x01
#define MSD_ERASE_RESET            0x02
#define MSD_ILLEGAL_COMMAND        0x04
#define MSD_COM_CRC_ERROR          0x08
#define MSD_ERASE_SEQUENCE_ERROR   0x10
#define MSD_ADDRESS_ERROR          0x20
#define MSD_PARAMETER_ERROR        0x40
#define MSD_RESPONSE_FAILURE       0xFF

#define SD_SELECT()        GPIO_ResetBits(SD_GPIO, SD_GPIO_PIN_CS)    /* MMC CS = L */
#define SD_DESELECT()      GPIO_SetBits(SD_GPIO, SD_GPIO_PIN_CS)      /* MMC CS = H */

uint8_t sdType;


/****************************************************************
  * @brief  initialize the sdcard

  * @return  0: ok         
  * @return  1: timeout
  * @return 99: no card 
****************************************************************/
uint8_t spiCardInit(void)
{
  //! Configure I/O for SD CARD Chip select 
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin   = SD_GPIO_PIN_CS;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(SD_GPIO, &GPIO_InitStructure);

  SD_DESELECT();
  
  uint8_t r1;     //! response of the sdcard
  uint16_t retry; 
  uint8_t buf[4]={0};
  
  if(spiCardSetIdleState()) return 1; //! set sdcard IDLE mode failed
  
  SD_SELECT(); //! get sdcard version info
  r1 = spiCardSendCommand(CMD8, 0x1aa, 0x87);
  if(r1 == 0x05) //! v1.0
  {
	  sdType = SD_TYPE_V1;
	  SD_DESELECT(); //! if v1.0, there is no data later after CMD8,   
	  spiReadWriteByte(0xff);
	  
	  retry = 0;
	  do
	  {
		  r1 = spiCardSendCommand(CMD55, 0, 0);
		  if(r1 == 0xff) return r1; //! continue to send if response is not 0xff
		  r1 = spiCardSendCommand(ACMD41, 0, 0);
		  retry++;
	  }while((r1!=0x00)&&(retry<400));
	  
	  //! if get response, it is SD card; otherwise it is MMC card
	  if(retry == 400)
	  {
		  retry = 0;
		  do
		  {
			  r1 = spiCardSendCommand(CMD1, 0, 0);
			  retry++;
		  }while((r1!=0x00)&&(retry<400));
		  if(retry == 400) return 1; //! MMC card init timeout
		  sdType = SD_TYPE_MMC;
	  }
	  spiReadWriteByte(0xff);
	  r1 = spiCardSendCommand(CMD59, 0, 0X95); //! disable crc
	  if(r1 != 0x00) return r1; //! cmd wrong, return r1   
	  r1 = spiCardSendCommand(CMD16,512,0X95); //! set sector size
	  if(r1 != 0x00) return r1; //! cmd wrong, return r1
  }
  else if(r1 == 0x01) //! v2.0 need to read OCR to make sure it is SD2.0 or SD2.0HC 
  {
	  buf[0] = spiReadWriteByte(0xff); //! should be 0x00
	  buf[1] = spiReadWriteByte(0xff); //! should be 0x00
	  buf[2] = spiReadWriteByte(0xff); //! should be 0x01
	  buf[3] = spiReadWriteByte(0xff); //! should be 0xAA
	  SD_DESELECT();
	  spiReadWriteByte(0xff);
	  retry = 0;
	  do
	  {
		  r1 = spiCardSendCommand(CMD55, 0, 0);
		  if(r1 != 0x01) return r1;
		  r1 = spiCardSendCommand(ACMD41, 0X40000000, 0);
		  if(retry > 200) return r1;
	  }while(r1 != 0);
	  
	  //! get OCR info, start to identify SD2.0
	  r1 = spiCardSendCommand(CMD58, 0, 0);
	  if(r1 != 0x00)
	  {
		  SD_DESELECT();
		  return r1;
	  }
	  buf[0] = spiReadWriteByte(0xff); 
	  buf[1] = spiReadWriteByte(0xff); 
	  buf[2] = spiReadWriteByte(0xff); 
	  buf[3] = spiReadWriteByte(0xff); 
      SD_DESELECT();
	  spiReadWriteByte(0xff);
	  //! check bit30(CCS), CCS = 1: SDHC  CCS = 0: SD2.0
	  if(buf[0]&0x40) sdType = SD_TYPE_V2HC;
	  else            sdType = SD_TYPE_V2;	  
  }
  
  return r1; 
}



/****************************************************************
  * @brief  wait for sdcard response
  
  * @param  response: the respose you want to get
  * @return 0: response ok others: wrong
****************************************************************/
uint8_t spiCardGetResponse(uint8_t response)
{
  uint16_t count = 0xFFF;   						  
  while((spiReadWriteByte(0XFF)!=response)&&count)count--;  	  
  if(count==0) return -1; //! failure
  else         return  0; //! ok
}
	
	
/****************************************************************
  * @brief  wait for sdcard ready

  * @return 0: response ok others: wrong
****************************************************************/	
uint8_t spiCardReady(void)
{
  uint8_t  r1 = MSD_DATA_OTHER_ERROR;
  uint32_t retry = 0;
  do
  {
     r1 = spiReadWriteByte(0xFF)&0X1F;
     if(retry == 0xFFFE) return 1; 
     retry++;
     switch(r1)
     {					   
	   case MSD_DATA_OK: r1 = MSD_DATA_OK; break;              //! receive data ok!
	   case MSD_DATA_CRC_ERROR:   return MSD_DATA_CRC_ERROR;   //! crc wrong
	   case MSD_DATA_WRITE_ERROR: return MSD_DATA_WRITE_ERROR; //! write data wrong
	   default: r1 = MSD_DATA_OTHER_ERROR; break;	           //! unknown wrong
     }   
   }while(r1 == MSD_DATA_OTHER_ERROR); //! wait while the data is wrong
   
   retry = 0;
   while(spiReadWriteByte(0XFF)==0)    //! get '0' means data writing is  not finished 
   {
	  retry++;
	  if(retry >= 0XFFFFFFFE) return 0XFF; //! occur something wrong
   };
   
   return 0; //! sdcard ready
}		
	
	
/****************************************************************
  * @brief  send a cmd to sdcard

  * @param cmd: 
  * @param arg: cmd's parameter
  * @param crc:   
  * @return  the response of sdcard
****************************************************************/	
uint8_t spiCardSendCommand(uint8_t cmd, uint32_t arg, uint8_t crc)	
{
	uint8_t r1;	
	uint8_t retry = 0;	
	
	SD_DESELECT();              //! deselect sdcard
	spiReadWriteByte(0xff);     //! wait for a while  
	
	SD_SELECT();                //! select sdcard	
	spiReadWriteByte(cmd);
	spiReadWriteByte(arg>>24);
	spiReadWriteByte(arg>>16);
	spiReadWriteByte(arg>> 8);
	spiReadWriteByte(arg);
	spiReadWriteByte(crc);
	
	while((r1=spiReadWriteByte(0xFF))==0xFF) //! wait for response or drop out
	{
		retry++;	    
		if(retry>200)break; 
	}   
	SD_DESELECT();
	spiReadWriteByte(0xFF); //! extra 8 pulses for sdcard
	return r1;	
}
	
	
/****************************************************************
  * @brief  set sdcard into idle state
 
  * @return  0: ok   others: failure	
****************************************************************/	
uint8_t spiCardSetIdleState(void)
{
	uint16_t i;
	uint8_t retry;	   	  
	for(i=0; i<0xf00; i++);	//!wait for sdcard power on 
	for(i=0; i<10; i++)spiReadWriteByte(0xFF); //! generate 74 pulses to make sure sdcard init finished 
	retry = 0;
	do //! continue to send CMD0 until receive 0x01 which means IDLE state 
	{	   
	   i = spiCardSendCommand(CMD0, 0, 0x95);
	   retry++;
	}while((i!=0x01)&&(retry<200));
	if(retry==200)return 1; 
	return 0;		
}	
	
	
	
/****************************************************************
  * @brief  read assigned bytes data and place assigned addr

  * @param   *data: 
  * @param     len: 
  * @return   0: ok   other: failed
****************************************************************/	
uint8_t spiCardReceiveData(uint8_t *data, uint16_t len)
{
	SD_SELECT();
	if(spiCardGetResponse(0XFE)) //! wait for start token 0XFE
	{
		SD_DESELECT();
		return 1;
	}
	while(len--) //! start to receive data
	{
		*data = spiReadWriteByte(0xff);
		data++;
	}
	spiReadWriteByte(0xff);
	spiReadWriteByte(0xff);
	
	return 0;
}	
	
	
	
/****************************************************************
  * @brief  get CID info 

  * @param   *cidData:  at least 16 bytes
  * @return         0: ok
  * @return         1: timeout
  * @return     other: failed
****************************************************************/	
uint8_t spiCardGetCid(uint8_t *cidData)
{
	uint8_t r1;	   
	r1 = spiCardSendCommand(CMD10,0,0xFF); //! send CMD10, read CID
	if(r1 != 0x00) return r1;  
	spiCardReceiveData(cidData,16);        //! receive 16 bytes data 
	return 0;	
}	
	
	
	
/****************************************************************
  * @brief  get CSD info 

  * @param   *csdData:  at least 16 bytes
  * @return         0: ok
  * @return         1: timeout
  * @return     other: failed
****************************************************************/	
uint8_t spiCardGetCsd(uint8_t *csdData)
{
	uint8_t r1;	   
	r1 = spiCardSendCommand(CMD9,0,0xFF); //! send CMD9, read CSD
	if(r1 != 0x00) return r1;  
	spiCardReceiveData(csdData,16);   //! receive 16 bytes data 
	return 0;	
}	
	
	
	
/****************************************************************
  * @brief  get SD card capacity(bytes) 

  * @return         0: failed
  * @return     other: exactely bytes
****************************************************************/														  
uint32_t spiCardGetCapacity(void)
{
	uint8_t  csd[16];
	uint8_t  r1;	
	uint16_t i;
	uint16_t temp;	
	uint32_t capacity;

	if(spiCardGetCsd(csd)!=0) return 0;	//! get CSD info, if failed return 0    

	if((csd[0]&0xC0)==0x40) //! SDHC card
	{									  
		capacity=((uint32_t)csd[8])<<8;
		capacity+=(uint32_t)csd[9]+1;	 
		capacity = (capacity)*1024; //! number of sectors
		capacity*=512; //! number of bytes
	}
	else
	{		    
		i = csd[6]&0x03;
		i<<=8;
		i += csd[7];
		i<<=2;
		i += ((csd[8]&0xc0)>>6);
		//! C_SIZE_MULT
		r1 = csd[9]&0x03;
		r1<<=1;
		r1 += ((csd[10]&0x80)>>7);	 
		r1+=2;//! BLOCKNR
		temp = 1;
		while(r1)
			{
			temp*=2;
			r1--;
			}
		capacity = ((uint32_t)(i+1))*((uint32_t)temp);	 
		//! READ_BL_LEN
		i = csd[5]&0x0f;
		//! BLOCK_LEN
		temp = 1;
		while(i)
			{
			temp*=2;
			i--;
			}
		//! The final result
		capacity *= (uint32_t)temp;	  
	}
	return (uint32_t)capacity;
}	
	
	
	
/****************************************************************
  * @brief  read a sector 

  * @param  sector  the addr of the sector, not the physicial addr 
  * @param *buffer  the addr to save the bytes(at least 512 bytes)
  * @return  0: ok   other: failed
****************************************************************/	
uint8_t spiCardReadSector(uint32_t sector, uint8_t *buffer)
{
	uint8_t r1;	    
  		   
	if(sdType != SD_TYPE_V2HC)
	{
		sector = sector<<9;
	} 
	r1 = spiCardSendCommand(CMD17, sector, 0); 		    
	if(r1 != 0x00) return r1; 		   							  
	r1 = spiCardReceiveData(buffer, 512);		 
	if(r1 != 0) return r1;   
	else        return  0; 
}





























	
	
	
	
	
	
	
	
	
	
	
	
	
	