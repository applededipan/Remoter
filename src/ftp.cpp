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

/****************************************************************
  * @brief  process the ftp message    added by apple 25/05/2016
   
  * @brief  note that only when write sdcard failed the smartconsole dose not response to QGC, otherwise must response to QGC!
****************************************************************/
void ftpProcess(void)
{
    Ftp  myFtp;                                  
	myFtp.network  = 0;
	myFtp.sysid    = 255;
	myFtp.cmpid    = 250;
	bool ackGood   = true;	
	uint8_t update = 0x55;
	
	myFtp.payload.hdr.size    = sizeof(uint32_t);	
	myFtp.payload.hdr.session = 0;
	myFtp.payload.hdr.opcode  = 128;
    myFtp.payload.hdr.offset  = mavData.ftp.payload.hdr.offset;
	myFtp.payload.hdr.reqOpcode = mavData.ftp.payload.hdr.opcode;
	myFtp.payload.hdr.seqNumber = mavData.ftp.payload.hdr.seqNumber + 1;
    memset(&myFtp.payload.data, 0, sizeof(myFtp.payload.data)); 	
 	*((uint32_t*)myFtp.payload.data) = mavData.ftp.payload.hdr.size;  
	
	switch(mavData.ftp.payload.hdr.opcode)
	{
		case kCmdCreateFile:			 
			 break;
		
		case kCmdWriteFile:		
             if(g_eeGeneral.firmwareUpdate != 1)
			 {
				 if(g_eeGeneral.vBattery>20) //! battery > 20, then can update
				 {
					g_eeGeneral.firmwareUpdate = 1; //! start firmware update!!!  
				 }
				 else
				 {
					ackGood = false;
					g_eeGeneral.firmwareUpdate = 2; //! firmware update rejected!!!  
				 }
			 }	
			 else
			 {   //! 根据第一次的消息判断是不是进行更新，若更新，则需地面站重发一次才正式开始更新
				 if(creatUpdateFile((uint8_t*)&mavData.ftp.payload.data, mavData.ftp.payload.hdr.offset, mavData.ftp.payload.hdr.size))
				 {
					 ackGood = false; //! write sdcard file failed!  
				 }
				 else
				 {
					 ackGood = true;  //! write sdcard file succeed!
				 }				 
			 }           			 
			 break;
		
		case kCmdResetSessions: 
		     break;
		
		case kCmdSearchVersion:
		     strcpy((char*)myFtp.payload.data, SMARTC_VERSION); //! send smartconsole version to QGC 
		     break;
	    
		case kCmdRemoveFile:
			 break;
			 
		case kCmdReboot: 
		     if(g_eeGeneral.comlinkState == COMLINK_BTH) //! if connect to bth
			 {
				g_mavlink_msg_file_transfer_protocol_send(MAVLINK_COMM_3, myFtp.network, myFtp.sysid, myFtp.cmpid, (uint8_t*)&myFtp.payload); //! response to QGC then reboot	 
			 }
			 else                                        //! default via usb
			 {
                g_mavlink_msg_file_transfer_protocol_send(MAVLINK_COMM_1, myFtp.network, myFtp.sysid, myFtp.cmpid, (uint8_t*)&myFtp.payload); //! response to QGC then reboot					 
			 }
	
			 eepromWriteBlock(&update, E_ADDR_UPDATE, 1);   //! set the update flag which means smartconsole will update when startup!
			 systemReboot();
		     break;
			 
		default: 
		     break;
	}
    //! only when wirte sdcard failed we will not response, otherwise we all have to response to the ftp message!!!
    if(ackGood) 
	{
	    if(g_eeGeneral.comlinkState == COMLINK_BTH)
		{
			g_mavlink_msg_file_transfer_protocol_send(MAVLINK_COMM_3, myFtp.network, myFtp.sysid, myFtp.cmpid, (uint8_t*)&myFtp.payload);
		}
		else
		{
			g_mavlink_msg_file_transfer_protocol_send(MAVLINK_COMM_1, myFtp.network, myFtp.sysid, myFtp.cmpid, (uint8_t*)&myFtp.payload);
		}
	}
		
}













