
 
//! this file is added by apple
#include "../../opentx.h"
#include "../../global.h"
#include "../../sdcard.h"
#include "telemetry/mavlink.h"

//! LCD WIDTH: 0 -- 479
//! LCD HIGHT: 0 -- 319

/************************************************
gui   name: displayStartmenu
      func: 显示开机画面 
	  func: first get pic from sdcard, if failed then get pic from extra flash
************************************************/
void displayStartmenu(void)
{	
	FATFS fs;         
	FIL fsrc;   
	BYTE buffer[61440];   
	UINT br;        
	FRESULT fr;
	
	if(g_eeGeneral.sdCardError) //! sdcard error
	{
		for(uint8_t i=0; i<5; i++)
		{
			spiFlashRead(buffer, F_ADDR_PICSTARTMENU+i*61440, 61440);
			lcd_DrawBmp(0, 320-64*(i+1), 480, 64, buffer);
		}	
	}
	else                             //! sdcard ok
	{
		f_mount(&fs, "0:", 0);		
		if(fr=f_open(&fsrc, "0:SmartConsole/fonts/startup.bin", FA_READ)) //! open file failed then get info from extra flash
		{
			f_mount(NULL, "0:", 0);
			for(uint8_t i=0; i<5; i++)
			{
				spiFlashRead(buffer, F_ADDR_PICSTARTMENU+i*61440, 61440);
				lcd_DrawBmp(0, 320-64*(i+1), 480, 64, buffer);
			}		   
		}
		else                                                              //! open file ok 
		{
			for(uint8_t i=0; i<5; i++)                           
			{
				fr = f_read(&fsrc, buffer, sizeof(buffer), &br);  
				
				if(fr || br == 0) break; 
				lcd_DrawBmp(0, 320-64*(i+1), 480, 64, buffer);
			}
			
			f_close(&fsrc);
			f_mount(NULL, "0:", 0);			   
		}            
	}		   

}


/************************************************
gui   name: displayInit
      func: the first time of displayLogo 
	      ：the first time of displayCustomMode 
		  ：the first time of displayMovType 
		  ：the first time of displayArmed 
		  ：the first time of displayRssi
		  : the first time of displayAttitude
		  : the first time of displayJoystick
************************************************/ 
void displayInit(void)
{
	displayLogo();	
	displayRssi(365, 23, mavData.mavStatus.pdlState, mavData.radioStatus.rssi); //! 信号强度
	displayMiniCell(410, 0);                           //! 电池电量	
	displayAttitude(0, 0, 0, 0, 0);	
}



/************************************************
gui   name: displayNavigation
      func:
	      ：displayCustomMode 
		  ：displayMovType 
		  ：displayArmed 
		  ：displayRssi
************************************************/  
void displayNavigation(void)
{	
	displayUavType(137, 3);	
	displayGps(320, 3, mavData.gpsRaw.satellites, mavData.gpsRaw.fixType);      //! GPS
	displayRssi(365, 23, mavData.mavStatus.pdlState, mavData.radioStatus.rssi); //! 信号强度
	displayMiniCell(410, 0);                                                    //! 电池电量		
	//! displayTime(380, 4, 24);	                                            //! 显示时间				
}



/************************************************
gui   name: displayLogo
      func: init all logos
************************************************/
void displayLogo(void)
{ 
	lcd_DrawRectangle(1, 0, 479, 319, LINEBLACK);     //外围
	lcd_DrawLine(0,   0, 480,  0, LINEBLACK);
	lcd_DrawLine(0,  28, 480, 28, LINEBLACK);
	lcd_DrawLine(134, 0, 134, 27, LINEBLACK); 
	lcd_DrawLine(163, 0, 163, 27, LINEBLACK); 
	lcd_DrawLine(316, 0, 316, 27, LINEBLACK); 
	lcd_DrawLine(363, 0, 363, 27, LINEBLACK); 
	lcd_DrawLine(405, 0, 405, 27, LINEBLACK); 	
	lcd_DrawRectangle(100, 163, 380, 319, LINEBLACK); //区域方框
	//! GPS logo
	uint8_t gpsLogo[968] = {0};	
	spiFlashRead(gpsLogo, F_ADDR_GPS, 968); 
	lcd_DrawBmp(320, 3, 22, 22, gpsLogo);

	//! RSSI logo
	uint8_t rssiLogo[968] = {0};	
	spiFlashRead(rssiLogo, F_ADDR_RSSI, 968); 
	lcd_DrawBmp(365, 3, 22, 22, rssiLogo);

   
	FATFS fs;         
	FIL fsrc;   
	BYTE logoBuffer[1392];          
	UINT br;
	f_mount(&fs, "0:", 0);		
	f_open(&fsrc, "0:SmartConsole/fonts/logo.bin", FA_READ); 
	f_read(&fsrc, logoBuffer, sizeof(logoBuffer), &br);  
	lcd_DrawBmp(5, 2, 29, 24, logoBuffer);
	f_close(&fsrc);
	f_mount(NULL, "0:", 0);			         
	uint8_t BrainyBEE[] = "BrainyBEE";
	lcd_ShowString(38, 7, WHITE, 18, BrainyBEE);	
}



/************************************************
gui   name: displayRunState
************************************************/
void displayRunState(uint16_t x, uint16_t y, uint8_t value)
{
	uint8_t valueLast = 20;
	
	if(value>20) value = 20;
	
	if(value != valueLast)
	{
		lcd_DrawFillRectangleDiff(x, y, 2, -21, BACKCOLOR);
		
		if(value == 0)      lcd_DrawFillRectangleDiff(x, y, 2, -8, GREEN);
		else if(value <= 2) lcd_DrawFillRectangleDiff(x, y, 2, -8-(value*2), GREEN);
		else if(value <= 5) lcd_DrawFillRectangleDiff(x, y, 2, -12-(value), YELLOW);
		else                lcd_DrawFillRectangleDiff(x, y, 2, -17-(value-5)/5, RED);  
		
		valueLast = value;      
	}

}





/************************************************
gui   name: displayUavType
      func: 显示飞机机型   四旋翼 六旋翼。。。。
	 (x,y): 显示位置
************************************************/
void displayUavType(uint16_t x, uint16_t y)
{
	static uint8_t  typeLast = MAV_TYPE_ENUM_END;
	uint8_t temp[576] = {0};	
	
	if(mavData.heartBeat.type != typeLast)
	{
		if(mavData.heartBeat.type == MAV_TYPE_FIXED_WING)     //! 固定翼
		{
			spiFlashRead(temp, F_ADDR_FIXED, 576);
			lcd_DrawBmp_256(x, y, 24, 24, temp);		
		}
		else if(mavData.heartBeat.type == MAV_TYPE_QUADROTOR) //! 四旋翼
		{
			spiFlashRead(temp, F_ADDR_QUADROTOR, 576);
			lcd_DrawBmp_256(x, y, 24, 24, temp);				
		}		
		else if(mavData.heartBeat.type == MAV_TYPE_HEXAROTOR) //! 六旋翼
		{
			spiFlashRead(temp, F_ADDR_HEXAROTOR, 576);
			lcd_DrawBmp_256(x, y, 24, 24, temp);				
		}		
		else if(mavData.heartBeat.type == MAV_TYPE_OCTOROTOR) //! 八旋翼
		{
			spiFlashRead(temp, F_ADDR_OCTOROTOR, 576);
			lcd_DrawBmp_256(x, y, 24, 24, temp);				
		}		
		else if(mavData.heartBeat.type == MAV_TYPE_HELICOPTER)//! 直升机
		{
			spiFlashRead(temp, F_ADDR_HELI, 576);
			lcd_DrawBmp_256(x, y, 24, 24, temp);				
		}
		else if(mavData.heartBeat.type == MAV_TYPE_GCS)       //! 地面站
		{
			spiFlashRead(temp, F_ADDR_QGC, 576);
			lcd_DrawBmp_256(x, y, 24, 24, temp);				
		}			
		else //! 未知机型
		{
			uint8_t logo_none[]="NA";
			lcd_ShowString(x, y, BACKCOLOR, 24, logo_none);
		}

		typeLast = mavData.heartBeat.type;		
	}

	static uint8_t  vtolLast = MAV_VTOL_STATE_UNDEFINED;

	if(mavData.heartBeat.type == MAV_TYPE_VTOL_DUOROTOR)     //! VTOL
	{
		if(mavData.sysStatus.vtolState != vtolLast)
		{
			if(mavData.sysStatus.vtolState == MAV_VTOL_STATE_FW)      //! fw
			{
				spiFlashRead(temp, F_ADDR_FIXED, 576);
				lcd_DrawBmp_256(x, y, 24, 24, temp);					
			}
			else if(mavData.sysStatus.vtolState == MAV_VTOL_STATE_MC) //! mc
			{
				spiFlashRead(temp, F_ADDR_QUADROTOR, 576);
				lcd_DrawBmp_256(x, y, 24, 24, temp);					
			}

			vtolLast = mavData.sysStatus.vtolState;			
		}

		typeLast = mavData.heartBeat.type;
	}
	
}





/************************************************
gui   name: displayUavFlightmode
      func: 显示飞行模式   MANUAL AUTO ........
	 (x,y): 显示位置
************************************************/
void displayUavFlightMode(uint16_t x, uint16_t y)
{
	static uint32_t flightmodeLast = FLIGHT_MODE_END; 
	static uint8_t  armStateLast = 0;

	if((mavData.heartBeat.custMode != flightmodeLast)||(mavData.mavStatus.armState != armStateLast))
	{
	   if(mavData.heartBeat.autopilot == MAV_AUTOPILOT_PX4) //! PX4
	   {  
            if(mavData.heartBeat.custMode == PX4_FLIGHT_MODE_MANUAL) 
            {
		      uint8_t MANUAL[] = "   MANUAL   ";			  
		      if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, MANUAL);				  
			  else                                lcd_ShowString(x-72, y, WHITE, 24, MANUAL);                              
			  GPIO_SetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
			  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
			  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);			  
	        }		  
	        else if(mavData.heartBeat.custMode == PX4_FLIGHT_MODE_ACRO)
	        {
              uint8_t ACRO[] =   "    ACRO    ";	
              if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, ACRO);				  
              else                                lcd_ShowString(x-72, y, WHITE, 24, ACRO);				  			  
			  GPIO_SetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
			  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
			  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);
	        }		  
	        else if(mavData.heartBeat.custMode == PX4_FLIGHT_MODE_STABILIZED) 
	        {
	    	  uint8_t STABILIZED[] = " STABILIZED ";
	    	  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, STABILIZED);
			  else                                lcd_ShowString(x-72, y, WHITE, 24, STABILIZED);
			  GPIO_SetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
			  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
			  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);			  
	        }  
			else if(mavData.heartBeat.custMode == PX4_FLIGHT_MODE_RATTITUDE)	
			{
			  uint8_t RATTITUDE[] =  " RATTITUDE  ";		  
	    	  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-66, y, GREEN, 24, RATTITUDE);
			  else                                lcd_ShowString(x-66, y, WHITE, 24, RATTITUDE);
			  GPIO_SetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
			  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
			  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);			  
			}
	        else if(mavData.heartBeat.custMode == PX4_FLIGHT_MODE_PAUSEFLIGHT)
	        {
	    	  uint8_t PAUSEFLIGHT[]= "PAUSEFLIGHT";
		      uint8_t NONE[]=        "            ";
		      lcd_ShowString(x-72, y, BACKCOLOR, 24, NONE);				  
	    	  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-66, y, GREEN, 24, PAUSEFLIGHT); 
			  else                                lcd_ShowString(x-66, y, WHITE, 24, PAUSEFLIGHT);
			  GPIO_SetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
			  GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
			  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);			  
	        } 
	        else if(mavData.heartBeat.custMode == PX4_FLIGHT_MODE_MISSION)  
	        {
	    	  uint8_t MISSION[] = "MISSION";
		      uint8_t NONE[]="            ";
		      lcd_ShowString(x-72, y, BACKCOLOR, 24, NONE);				  
	    	  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-42, y, GREEN, 24, MISSION);
			  else                                lcd_ShowString(x-42, y, WHITE, 24, MISSION);
			  GPIO_SetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
			  GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
			  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);
	        }
            else if(mavData.heartBeat.custMode == PX4_FLIGHT_MODE_RTL)
			{
	    	  uint8_t RTL[] = "   RETURN   ";				  
	    	  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, RTL);
			  else                                lcd_ShowString(x-72, y, WHITE, 24, RTL);
			  GPIO_SetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);
			  GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
			  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);				
			}
            else if(mavData.heartBeat.custMode == PX4_FLIGHT_MODE_ALTITUDE)
			{
	    	  uint8_t ALT[] = "  ALTITUDE  ";			  
	    	  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, ALT);
			  else                                lcd_ShowString(x-72, y, WHITE, 24, ALT);
			  GPIO_SetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
			  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
			  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);				
			}
            else if(mavData.heartBeat.custMode == PX4_FLIGHT_MODE_POSITION)
			{
	    	  uint8_t POS[] = "  POSITION  ";			  
	    	  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, POS);
			  else                                lcd_ShowString(x-72, y, WHITE, 24, POS);
			  GPIO_SetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
			  GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
			  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);				
			}       
            else if(mavData.heartBeat.custMode == PX4_FLIGHT_MODE_LANDING) 
			{
	    	  uint8_t LANDING[] = "LANDING";			  
		      uint8_t NONE[]="            ";
		      lcd_ShowString(x-72, y, BACKCOLOR, 24, NONE);				  
	    	  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-42, y, GREEN, 24, LANDING);
			  else                                lcd_ShowString(x-42, y, WHITE, 24, LANDING);
			  GPIO_SetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
			  GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
			  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);				
			}
            else if(mavData.heartBeat.custMode == PX4_FLIGHT_MODE_RTGS) 
			{
	    	  uint8_t RTGS[] = "    RTGS    ";				  
	    	  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, RTGS);
			  else                                lcd_ShowString(x-72, y, WHITE, 24, RTGS);
			  GPIO_SetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);
			  GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
			  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);			
			}            
            else if(mavData.heartBeat.custMode == FLIGHT_MODE_END) //! 空模式，与unknown模式不同
			{
	    	  uint8_t END[] = "            ";			  
	    	  lcd_ShowString(x-72, y, GREEN, 24, END);
			  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
			  GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
			  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);				
			}           			
            else 
	        {
              uint8_t UNKNOWN[] = "UNKNOWNMODE";				
		      uint8_t NONE[]="            ";
		      lcd_ShowString(x-72, y, BACKCOLOR, 24, NONE);	
	    	  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-66, y, GREEN, 24, UNKNOWN);
			  else                                lcd_ShowString(x-66, y, WHITE, 24, UNKNOWN);			  
		      GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);
		      GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
		      GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);			  
	        }		  	  		   
	   }
	   else if(mavData.heartBeat.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA) //! APM
	   {
		    //! apmcopter: QUADROTOR   HEXAROTOR   OCTOROTOR
			if((mavData.heartBeat.type==MAV_TYPE_QUADROTOR)||(mavData.heartBeat.type==MAV_TYPE_HEXAROTOR)||(mavData.heartBeat.type==MAV_TYPE_OCTOROTOR))
			{
				if(mavData.heartBeat.custMode == APMCOPTER_FLIGHT_MODE_STABILIZE) 
				{
				  uint8_t STABILIZE[] = "STABILIZE";
		          uint8_t NONE[]="            ";
		          lcd_ShowString(x-72, y, BACKCOLOR, 24, NONE);					  
				  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-54, y, GREEN, 24, STABILIZE);
				  else                                lcd_ShowString(x-54, y, WHITE, 24, STABILIZE);
				  GPIO_SetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);				  
				}		  
				else if(mavData.heartBeat.custMode == APMCOPTER_FLIGHT_MODE_ACRO)
				{
				  uint8_t ACRO[] = "    ACRO    ";						  
				  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, ACRO);
				  else                                lcd_ShowString(x-72, y, WHITE, 24, ACRO);
				  GPIO_SetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);
				}		  
				else if(mavData.heartBeat.custMode == APMCOPTER_FLIGHT_MODE_AUTO) 
				{
				  uint8_t AUTO[] = "    AUTO    ";				  
				  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, AUTO);
				  else                                lcd_ShowString(x-72, y, WHITE, 24, AUTO);
				  GPIO_SetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);
				}  
				else if(mavData.heartBeat.custMode == APMCOPTER_FLIGHT_MODE_LOITER)
				{
				  uint8_t LOITER[]= "   LOITER   ";				  
				  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, LOITER);
				  else                                lcd_ShowString(x-72, y, WHITE, 24, LOITER);
				  GPIO_SetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);				  
				}
				else if(mavData.heartBeat.custMode == APMCOPTER_FLIGHT_MODE_OF_LOITER)
				{
				  uint8_t OF_LOITER[]= "OF_LOITER";
		          uint8_t NONE[]="            ";
		          lcd_ShowString(x-72, y, BACKCOLOR, 24, NONE);					  
				  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-54, y, GREEN, 24, OF_LOITER); 
				  else                                lcd_ShowString(x-54, y, WHITE, 24, OF_LOITER); 
				  GPIO_SetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);				  
				}				
				else if(mavData.heartBeat.custMode == APMCOPTER_FLIGHT_MODE_ALT_HOLD)
				{
				  uint8_t ALT_HOLD[]= "  ALT_HOLD  ";				  
				  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, ALT_HOLD); 
				  else                                lcd_ShowString(x-72, y, WHITE, 24, ALT_HOLD);
				  GPIO_SetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);				  
				}
                else if(mavData.heartBeat.custMode == APMCOPTER_FLIGHT_MODE_RTL)
				{
				  uint8_t RTL[]= "   RETURN   ";				  
				  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, RTL);
				  else                                lcd_ShowString(x-72, y, WHITE, 24, RTL);
				  GPIO_SetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);
				  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);				  
				}
                else if(mavData.heartBeat.custMode == APMCOPTER_FLIGHT_MODE_POSITION)
				{
				  uint8_t POSITION[]= "  POSITION  ";				  
				  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, POSITION);
				  else                                lcd_ShowString(x-72, y, WHITE, 24, POSITION);
				  GPIO_SetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);				  
				}	
                else if(mavData.heartBeat.custMode == APMCOPTER_FLIGHT_MODE_POS_HOLD)
				{
				  uint8_t POS_HOLD[]= "  POS_HOLD  ";				  
				  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, POS_HOLD);
				  else                                lcd_ShowString(x-72, y, WHITE, 24, POS_HOLD);
				  GPIO_SetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);				  
				}
				else if(mavData.heartBeat.custMode == FLIGHT_MODE_END) //! 空模式，与unknown模式不同
				{
				  uint8_t END[] = "            ";			  
				  lcd_ShowString(x-72, y, GREEN, 24, END);
				  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);				
				}					
				else 
				{
				   //! other flightmodes
				   uint8_t UNKNOWN[] = "UNKNOWNMODE";
				   uint8_t NONE[]="            ";
		           lcd_ShowString(x-72, y, BACKCOLOR, 24, NONE);
				   if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-66, y, GREEN, 24, UNKNOWN);
				   else                                lcd_ShowString(x-66, y, WHITE, 24, UNKNOWN);				   
				   GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);
				   GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				   GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);				   
				}				
			}		  
            //! apmplane
			else if(mavData.heartBeat.type == MAV_TYPE_FIXED_WING)
			{
				if(mavData.heartBeat.custMode == APMPLANE_FLIGHT_MODE_MANUAL) 
				{
				  uint8_t MANUAL[] = "   MANUAL   ";					  
				  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, MANUAL);
				  else                                lcd_ShowString(x-72, y, WHITE, 24, MANUAL);
				  GPIO_SetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);
				}	
				else if(mavData.heartBeat.custMode == APMPLANE_FLIGHT_MODE_CIRCLE)
				{
				  uint8_t CIRCLE[]= "   CIRCLE   ";					  
				  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, CIRCLE); 
				  else                                lcd_ShowString(x-72, y, WHITE, 24, CIRCLE);
				  GPIO_SetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);				  
				} 				
				else if(mavData.heartBeat.custMode == APMPLANE_FLIGHT_MODE_STABILIZE)
				{
				  uint8_t STABILIZE[] = "STABILIZE";
		          uint8_t NONE[]="            ";
		          lcd_ShowString(x-72, y, BACKCOLOR, 24, NONE);					  
				  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-54, y, GREEN, 24, STABILIZE);
				  else                                lcd_ShowString(x-54, y, WHITE, 24, STABILIZE);
				  GPIO_SetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);				  
				}
				else if(mavData.heartBeat.custMode == APMPLANE_FLIGHT_MODE_TRAINING)
				{
				  uint8_t TRAINING[]= "  TRAINING  ";				  
				  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, TRAINING); 
				  else                                lcd_ShowString(x-72, y, WHITE, 24, TRAINING);
				  GPIO_SetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);				  
				}
				else if(mavData.heartBeat.custMode == APMPLANE_FLIGHT_MODE_ACRO)
				{
				  uint8_t ACRO[]= "    ACRO    ";				  
				  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, ACRO); 
				  else                                lcd_ShowString(x-72, y, WHITE, 24, ACRO);
				  GPIO_SetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);				  
				}
				else if(mavData.heartBeat.custMode == APMPLANE_FLIGHT_MODE_FLY_BY_WIRE_A)
				{
				  uint8_t FBWA[]= "    FBWA    ";				  
				  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, FBWA);
				  else                                lcd_ShowString(x-72, y, WHITE, 24, FBWA);
				  GPIO_SetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);				  
				}
				else if(mavData.heartBeat.custMode == APMPLANE_FLIGHT_MODE_FLY_BY_WIRE_B)
				{
				  uint8_t FBWB[]= "    FBWB    ";					  
				  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, FBWB);
				  else                                lcd_ShowString(x-72, y, WHITE, 24, FBWB);
				  GPIO_SetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);				  
				}
				else if(mavData.heartBeat.custMode == APMPLANE_FLIGHT_MODE_CRUISE)
				{
				  uint8_t CRUISE[]= "   CRUISE   ";				  
				  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, CRUISE); 
				  else                                lcd_ShowString(x-72, y, WHITE, 24, CRUISE);
				  GPIO_SetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);				  
				}
				else if(mavData.heartBeat.custMode == APMPLANE_FLIGHT_MODE_AUTOTUNE)
				{
				  uint8_t AUTOTUNE[]= "  AUTOTUNE  ";				  
				  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, AUTOTUNE); 
				  else                                lcd_ShowString(x-72, y, WHITE, 24, AUTOTUNE);
				  GPIO_SetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);				  
				}
				else if(mavData.heartBeat.custMode == APMPLANE_FLIGHT_MODE_AUTO) 
				{
				  uint8_t AUTO[] = "    AUTO    ";				  
				  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, AUTO);
				  else                                lcd_ShowString(x-72, y, WHITE, 24, AUTO);
				  GPIO_SetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);
				}		
				else if(mavData.heartBeat.custMode == APMPLANE_FLIGHT_MODE_RTL)
				{
				  uint8_t RETURN[]= "   RETURN   ";				  
				  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, RETURN); 
				  else                                lcd_ShowString(x-72, y, WHITE, 24, RETURN);
				  GPIO_SetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);
				  GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				}						  
				else if(mavData.heartBeat.custMode == APMPLANE_FLIGHT_MODE_LOITER)
				{
				  uint8_t LOITER[]= "   LOITER   ";				  
				  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, LOITER); 
				  else                                lcd_ShowString(x-72, y, WHITE, 24, LOITER);
				  GPIO_SetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);				  
				} 
				else if(mavData.heartBeat.custMode == APMPLANE_FLIGHT_MODE_GUIDED)
				{
				  uint8_t GUIDED[]= "   GUIDED   ";				  
				  if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, GUIDED); 
				  else                                lcd_ShowString(x-72, y, WHITE, 24, GUIDED); 
				  GPIO_SetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);				  
				}
				else if(mavData.heartBeat.custMode == FLIGHT_MODE_END) //! 空模式，与unknown模式不同
				{
				  uint8_t END[] = "            ";			  
				  lcd_ShowString(x-72, y, GREEN, 24, END);
				  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
				  GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				  GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);				
				}					
				else 
				{
				   //! other flightmodes
				   uint8_t UNKNOWN[] = "UNKNOWNMODE";				   
				   uint8_t NONE[]="            ";
		           lcd_ShowString(x-72, y, BACKCOLOR, 24, NONE);
				   if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-66, y, GREEN, 24, UNKNOWN); 
				   else                                lcd_ShowString(x-66, y, WHITE, 24, UNKNOWN); 				   
				   GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);
				   GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
				   GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);					
				}				
			}
			else
			{
			   //! 其他机型
			   // uint8_t UNKNOWN[] = "UNKNOWNTYPE";			   
			   // uint8_t NONE[]="            ";
		       // lcd_ShowString(x-72, y, BACKCOLOR, 24, NONE);
			   // if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-66, y, GREEN, 24, UNKNOWN); 
			   // else                                lcd_ShowString(x-66, y, WHITE, 24, UNKNOWN); 			   
			   // GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);
			   // GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
			   // GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);				
			}          			
	   }	   
	   else //! 其他飞控
	   {
		  // uint8_t UNKNOWN[] = " UNKNOWNUAV ";		   
		  // if(mavData.mavStatus.armState == ARMSTATE_ARMED) lcd_ShowString(x-72, y, GREEN, 24, UNKNOWN); 
		  // else                                lcd_ShowString(x-72, y, WHITE, 24, UNKNOWN); 		  
		  // GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);
		  // GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
		  // GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);		   
	   }
       flightmodeLast = mavData.heartBeat.custMode;     
       armStateLast = mavData.mavStatus.armState;  	   
	}		
}




/************************************************
gui   name: displayArmed
      func: 显示是否解除武装
     (x,y): 起始坐标
armState: : armed  disarmed
************************************************/
void displayArmed(uint16_t x, uint16_t y)
{
	static uint8_t armStateLast = 255;
	
	if(mavData.mavStatus.armState != armStateLast)
	{
		uint8_t temp[800] = {0};
		
		if(mavData.mavStatus.armState == ARMSTATE_ARMED) 
		{
			spiFlashRead(temp, F_ADDR_ARMED, 800); lcd_DrawBmp(x, y, 20, 20, temp);		
		}	
		else //! disarmed
		{
			spiFlashRead(temp, F_ADDR_DISARMED, 800); lcd_DrawBmp(x, y, 20, 20, temp);		  
		}	
		
		armStateLast = mavData.mavStatus.armState;	  
	}
}



/************************************************
gui   name: displayCell
      func: 显示电池电量图标
     (bat): 电量大小 0--100
************************************************/
const uint8_t batError = 15;

void displayCell(void)
{
	delayms(500);

	uint32_t sum = 0;	
	int32_t temp = 0;
	
	for(uint8_t i = 0; i < 100; i++)
	{
		delayms(2); //! delay for 2ms
		getADC();
#if defined(BAT18650)
		temp = anaIn(6)*0.67 - 567;
#else	
		temp = anaIn(6)-900;
#endif		
		if(temp > 100)     temp = 100;
		else if(temp <= 0)temp = 0;
		sum += temp;
	}	
	uint8_t bat = sum/100;
	if(bat > 100) bat = 100;	

	g_eeGeneral.vBattery = bat;

	if(bat > batError)
	{
		lcd_DrawRectangle(120, 105, 360, 195, WHITE); //! 框
		lcd_DrawFillRectangle(360, 130, 380, 170, WHITE); //! 正极
		lcd_DrawFillRectangle(124, 110, 126+bat*2.3, 190, WHITE); //! 进度
	}
	else
	{
		lcd_DrawRectangle(120, 105, 360, 195, RED);
		lcd_DrawFillRectangle(360, 130, 380, 170, RED); //! 正极
		lcd_DrawFillRectangle(124, 110, 126+bat*2.3, 190, RED);			
	}  
}
 
 
/************************************************
gui   name: displayMiniCell
      func: 显示小电池电量图标,长：36 宽15
	 (x,y): 电池坐标 
	anaIn(7): 0 -- 1500 大于等于1000表示正在充电 
************************************************/
void displayMiniCell(uint16_t x, uint16_t y)
{
	static gtime_t timeLast = 255;
	static uint8_t  count = 0;
	static uint8_t batLast = 255; //! 初始值非0解决充电时上电伊始不显示电池轮廓的问题	

	if(anaIn(7) > 1100) //! charging
	{	
		if(g_rtcTime != timeLast) //! 一秒更新一次
		{ 
		
			if(batLast != 0) //! 充电过程只执行了一次
			{
				batLast = 0; //! 解决由充电切换到未充电时是否显示当前电量格数的问题			   
				lcd_DrawFillRectangleDiff(x+2, y+8, 29, 12, BACKCOLOR); //! 清除掉之前电池电量的格数	
				lcd_DrawRectangleDiff(x, y+6, 32, 15, RED); //! 电池轮廓
				lcd_DrawFillRectangleDiff(x+32, y+9, 4, 9, RED); //! 正极图标				  
			}
			
			switch(count)
			{
				case 0: lcd_DrawFillRectangleDiff(x+2,  y+8, 3, 11, WHITE); count++; break;
				case 1: lcd_DrawFillRectangleDiff(x+7,  y+8, 3, 11, WHITE); count++; break;
				case 2: lcd_DrawFillRectangleDiff(x+12, y+8, 3, 11, WHITE); count++; break;
				case 3: lcd_DrawFillRectangleDiff(x+17, y+8, 3, 11, WHITE); count++; break;
				case 4: lcd_DrawFillRectangleDiff(x+22, y+8, 3, 11, WHITE); count++; break;
				case 5: lcd_DrawFillRectangleDiff(x+27, y+8, 3, 11, WHITE); count++; break;
				default : lcd_DrawFillRectangleDiff(x+2,  y+8,29, 11, BACKCOLOR); count = 0;
			}
			
			lcd_ShowNumBackColor(x+39, y+7, WHITE, 18, 2, g_eeGeneral.vBattery, 0, BACKCOLOR);
			lcd_ShowChar(x+39+19, y+4, WHITE, 24, '%');		   
			timeLast = g_rtcTime;			
		}
		
	    //! 显示充电logo
	    lcd_DrawCharge(x+22, y+8, anaIn(7));		
	}	
	else //! 未充电状态
	{
        if(count != 0) //! 下次充电时显示的动态格数从0开始
		{
			count = 0;
			batLast = 255;
			lcd_DrawFillRectangleDiff(x+2, y+8, 29, 12, BACKCOLOR);
		}
        		
	    if(g_eeGeneral.vBattery != batLast) 
	    {  		
			if(g_eeGeneral.vBattery > batError)
			{
				lcd_DrawFillRectangleDiff(x+2, y+8, 29, 12, BACKCOLOR);
				lcd_DrawRectangleDiff(x, y+6, 32, 15, WHITE); //! 轮廓
				lcd_DrawFillRectangleDiff(x+32, y+9, 4, 9, WHITE); //! 正极
				lcd_DrawFillRectangleDiff(x+2,  y+8, g_eeGeneral.vBattery/3.6, 11, WHITE); //! 格数
			}
			else
			{
				lcd_DrawFillRectangleDiff(x+2, y+8, 29, 12, BACKCOLOR);
				lcd_DrawRectangleDiff(x, y+6, 32, 15, RED); //! 轮廓
				lcd_DrawFillRectangleDiff(x+32, y+9, 4, 9, RED); //! 正极
				lcd_DrawFillRectangleDiff(x+2,  y+8, g_eeGeneral.vBattery/3.6, 11, RED); //! 格数
			}
			
			lcd_ShowNumBackColor(x+39, y+7, WHITE, 18, 2, g_eeGeneral.vBattery, 0, BACKCOLOR);
			lcd_ShowChar(x+39+18, y+4, WHITE, 24, '%');
			batLast = g_eeGeneral.vBattery;				
	    }			
	}
}



/************************************************
gui   name: displayGps
      func: 显示GPS个数和定位状态
	   gps: gps个数
   fixType：3：3D定位
************************************************/
void displayGps(uint16_t x, uint16_t y, uint8_t gps, uint8_t fixType)
{
	static uint8_t gpsLast = 255;
		
	if(gps != gpsLast)
	{
		if(gps < 10) 
		{	
			if(fixType == 3) //! 3D fix
			{
				lcd_ShowChar(x+33, y+5, BACKCOLOR, 18, ' '); //! clear the second number
				lcd_ShowNumBackColor(x+23, y+5, GREEN, 18, 1, gps, 0, BACKCOLOR);			   
			}
			else
			{
				lcd_ShowChar(x+33, y+5, WHITE, 18, ' '); //! clear the second number
				lcd_ShowNumBackColor(x+23, y+5, WHITE, 18, 1, gps, 0, BACKCOLOR);			   
			}			
		}
		else
		{
			if(fixType == 3)
			{
				lcd_ShowNumBackColor(x+23, y+5, GREEN, 18, 2, gps, 0, BACKCOLOR);				   
			}
			else
			{
				lcd_ShowNumBackColor(x+23, y+5, WHITE, 18, 2, gps, 0, BACKCOLOR);			   
			}

		} 
		
		gpsLast = gps;
	}			
}



/************************************************
gui   name: displayRssi
      func: 显示图传和数传信号强度
     (x,y): 起始坐标
	  size: 字体大小
  wifiRssi: 0 -- 255： 128： 树莓派启动成功
                       144： 遥控器端图传(Master)启动成功
                       176： 飞控端图传(Slave)连接成功
                       240： 摄像头端(Target)连接成功
  dataRssi: 0 -- 100/255
************************************************/ 
void displayRssi(uint16_t x, uint16_t y, uint8_t wifiRssi, uint8_t dataRssi)
{
	static uint8_t wifiRssiLast = 255;
	static uint8_t dataRssiLast = 255;
  
    if(wifiRssi != wifiRssiLast)
	{
		if(wifiRssi == 0)          //! PDDL startup failed
		{
			lcd_DrawFillRectangleDiff(x+22,   y-11, 1, -1, RED);
			lcd_DrawFillRectangleDiff(x+22+4, y-11, 1, -3, RED);
			lcd_DrawFillRectangleDiff(x+22+8, y-11, 1, -5, RED);
			lcd_DrawFillRectangleDiff(x+22+12,y-11, 1, -7, RED);		
		}	
        else if(wifiRssi == 128)   //! PDDL startup failed
        {
            lcd_DrawFillRectangleDiff(x+22,   y-11, 1, -1, GREEN);
            lcd_DrawFillRectangleDiff(x+22+4, y-11, 1, -3, RED);
            lcd_DrawFillRectangleDiff(x+22+8, y-11, 1, -5, RED);
            lcd_DrawFillRectangleDiff(x+22+12,y-11, 1, -7, RED);			
        }
        else if(wifiRssi == 144)	
        {
            lcd_DrawFillRectangleDiff(x+22,   y-11, 1, -1, GREEN);
            lcd_DrawFillRectangleDiff(x+22+4, y-11, 1, -3, GREEN);
            lcd_DrawFillRectangleDiff(x+22+8, y-11, 1, -5, RED);
            lcd_DrawFillRectangleDiff(x+22+12,y-11, 1, -7, RED);			
        }
        else if(wifiRssi == 176)
        {
            lcd_DrawFillRectangleDiff(x+22,   y-11, 1, -1, GREEN);
            lcd_DrawFillRectangleDiff(x+22+4, y-11, 1, -3, GREEN);
            lcd_DrawFillRectangleDiff(x+22+8, y-11, 1, -5, GREEN);
            lcd_DrawFillRectangleDiff(x+22+12,y-11, 1, -7, RED);			
        }
        else if(wifiRssi == 240)
        {
            lcd_DrawFillRectangleDiff(x+22,   y-11, 1, -1, GREEN);
            lcd_DrawFillRectangleDiff(x+22+4, y-11, 1, -3, GREEN);
            lcd_DrawFillRectangleDiff(x+22+8, y-11, 1, -5, GREEN);
            lcd_DrawFillRectangleDiff(x+22+12,y-11, 1, -7, GREEN);	
        }
        else
        {
            
        } 
		
        wifiRssiLast = wifiRssi;       
	}

	if(abs(dataRssi-dataRssiLast) > 10) //! datarssi
	{
		if(dataRssi <= 5)
		{
			lcd_DrawFillRectangleDiff(x+22,   y, 1, -1, LIGHTBLACK);
			lcd_DrawFillRectangleDiff(x+22+4, y, 1, -3, LIGHTBLACK);
			lcd_DrawFillRectangleDiff(x+22+8, y, 1, -5, LIGHTBLACK);
			lcd_DrawFillRectangleDiff(x+22+12,y, 1, -7, LIGHTBLACK);		
		}
		else if((dataRssi>5)&&(dataRssi<=30))     
		{
			lcd_DrawFillRectangleDiff(x+22,   y, 1, -1, WHITE);
			lcd_DrawFillRectangleDiff(x+22+4, y, 1, -3, LIGHTBLACK);
			lcd_DrawFillRectangleDiff(x+22+8, y, 1, -5, LIGHTBLACK);
			lcd_DrawFillRectangleDiff(x+22+12,y, 1, -7, LIGHTBLACK);			
		}
		else if((dataRssi>30)&&(dataRssi<=55))	
		{
			lcd_DrawFillRectangleDiff(x+22,   y, 1, -1, WHITE);
			lcd_DrawFillRectangleDiff(x+22+4, y, 1, -3, WHITE);
			lcd_DrawFillRectangleDiff(x+22+8, y, 1, -5, LIGHTBLACK);
			lcd_DrawFillRectangleDiff(x+22+12,y, 1, -7, LIGHTBLACK);			
		}
		else if((dataRssi>55)&&(dataRssi<=80))
		{
			lcd_DrawFillRectangleDiff(x+22,   y, 1, -1, WHITE);
			lcd_DrawFillRectangleDiff(x+22+4, y, 1, -3, WHITE);
			lcd_DrawFillRectangleDiff(x+22+8, y, 1, -5, WHITE);
			lcd_DrawFillRectangleDiff(x+22+12,y, 1, -7, LIGHTBLACK);			
		}
		else 
		{
			lcd_DrawFillRectangleDiff(x+22,   y, 1, -1, WHITE);
			lcd_DrawFillRectangleDiff(x+22+4, y, 1, -3, WHITE);
			lcd_DrawFillRectangleDiff(x+22+8, y, 1, -5, WHITE);
			lcd_DrawFillRectangleDiff(x+22+12,y, 1, -7, WHITE);	
		}
		
		dataRssiLast = dataRssi;
	}	
}
 
 
 
/************************************************
gui   name: displayTime
      func: 显示当前时间
     (x,y): 起始坐标
	  size: 字体大小
************************************************/
void displayTime(uint16_t x, uint16_t y, uint8_t size)
{
    struct gtm t;	
	static struct gtm tlast;		
	rtcGetTime(&t);
		
	if(t.tm_sec != tlast.tm_sec)    //! 时间更新时更新显示   只显示时间
    {      
		tlast.tm_sec = t.tm_sec;
		uint16_t x1 = x;
		uint16_t x2 = x+size;
		uint16_t x3 = x2+size/2;
		uint16_t x4 = x3+size;
		uint16_t x5 = x4+size/2;

		if(t.tm_hour != tlast.tm_hour) lcd_ShowNum (x1,  y, WHITE, size, 2, t.tm_hour, 1);//	
		lcd_ShowChar(x2,  y, WHITE, size, ':');
		if(t.tm_min != tlast.tm_min)   lcd_ShowNum (x3,  y, WHITE, size, 2, t.tm_min, 1);
		lcd_ShowChar(x4,  y, WHITE, size, ':');
		lcd_ShowNum (x5,  y, WHITE, size, 2, t.tm_sec, 1);		   
    }	
}



 
/************************************************
gui   name: displayComStep
      func: 显示USB或PAI连接质量
************************************************/
void displayComStep(void)
{
	// to do 		
}


 
/************************************************
gui   name: displayJoystick
      func: 显示飞控返回来的控制通道的值  
 * @ chan1: ele  升降舵：俯仰角   
 * @ chan2: rud  方向舵：航向角    
 * @ chan3: thr  油门              
 * @ chan4: ail  副翼：  横滚角  
 * @ in:  1000 -- 2000 
************************************************/
void displayJoystick(RC_CHANNEL channels) //!RC_CHANNEL channels
{
    const uint8_t width = 15;
	const uint8_t longth= 64;
	
	static int8_t chan1Last = 10;
	static int8_t chan2Last = 10;
	static int8_t chan3Last = 10;
	static int8_t chan4Last = 10;
	
	static bool excuted = false;
	
	if(!excuted)
	{
		excuted = true;
		lcd_DrawRectangleDiff(102, 154, width, longth, LIGHTWHITE);//左侧边小矩形框
		lcd_DrawRectangleDiff(363, 154, width, longth, LIGHTWHITE);//右侧边小矩形框
		lcd_DrawFillRectangleDiff(103, 155, width-2, longth-2, BACKCOLOR);
		lcd_DrawFillRectangleDiff(364, 155, width-2, longth-2, BACKCOLOR);

		lcd_DrawRectangleDiff(119, 203, longth, width, LIGHTWHITE);//左侧下小矩形框
		lcd_DrawRectangleDiff(298, 203, longth, width, LIGHTWHITE);//右侧下小矩形框
		lcd_DrawFillRectangleDiff(120, 204, longth-2, width-2, BACKCOLOR);
		lcd_DrawFillRectangleDiff(299, 204, longth-2, width-2, BACKCOLOR);	   
	}

	int8_t chan1 = (channels.chan1-1500)*3/50;
	int8_t chan2 = (channels.chan2-1500)*3/50;
	int8_t chan3 = (channels.chan3-1500)*3/50;
	int8_t chan4 = (channels.chan4-1500)*3/50;	   
   
	if(chan1 != chan1Last)
	{
		lcd_DrawFillRectangleDiff(104, 186, width-4, -chan1Last, BACKCOLOR);
		lcd_DrawFillRectangleDiff(104, 186, width-4, -chan1, GREEN);
		chan1Last = chan1;	   
	}
	
	if(chan2 != chan2Last)
	{
		lcd_DrawFillRectangleDiff(151, 205, chan2Last, width-4, BACKCOLOR);	
		lcd_DrawFillRectangleDiff(151, 205, chan2, width-4, GREEN);
		chan2Last = chan2;
	}
	
    if(chan3 != chan3Last)
	{
		lcd_DrawFillRectangleDiff(365, 186, width-4, -chan3Last, BACKCOLOR);
		lcd_DrawFillRectangleDiff(365, 186, width-4, -chan3, GREEN);	   
		chan3Last = chan3;	   
	}
	
	if(chan4 != chan4Last)
	{
		lcd_DrawFillRectangleDiff(330, 205, chan4Last, width-4, BACKCOLOR);		
		lcd_DrawFillRectangleDiff(330, 205, chan4, width-4, GREEN);
		chan4Last = chan4;
	}
}
 
 



/************************************************
menu  name: displayAttitude
      func: 显示飞行器姿态
	 pitch: 俯仰角
      roll: 横滚角
   heading: 航向角
 batRemain: 电池电压  0%:0   100%:100  -1:estimating  
************************************************/ 
void displayAttitude(int16_t pitch, int16_t roll, int16_t heading, int32_t alt, int8_t batRemain)
{
	const uint16_t upBackColor = BLUE;
	const uint16_t downBackColor = BROWN;
	
	static bool attitude_painted = false;	
	//矩形框中心点坐标                       //矩形坐标：(100, 28)   (380, 28)
	const uint16_t xpoint = 240;             //矩形坐标：(100,160)   (380,160)
	const uint16_t ypoint = 115;		
	if(!attitude_painted)
	{
		attitude_painted = true;
		//背景色
		lcd_DrawFillRectangle(xpoint-140, ypoint-85, xpoint+140,  ypoint-15, upBackColor);
		lcd_DrawFillRectangle(xpoint-140, ypoint-15, xpoint+140,  ypoint+46, downBackColor); //! 
		//轮廓线
	    lcd_DrawLine(100,29, 100,220, LINEBLACK);//左竖线
	    lcd_DrawLine(380,29, 380,220, LINEBLACK);//右竖线
	    lcd_DrawLine(100,162,380,162, LINEBLACK);//底横线 
		lcd_DrawLine(100+3,100,380-3,100, LIGHTWHITE);//中横线:地平线	
        //顶部航向刻度线
        lcd_DrawLineDiff(140,43, 0, 7, LIGHTWHITE);
        lcd_DrawLineDiff(190,43, 0, 7, LIGHTWHITE);
        lcd_DrawLineDiff(240,43, 0, 7, LIGHTWHITE);
        lcd_DrawLineDiff(290,43, 0, 7, LIGHTWHITE);	
        lcd_DrawLineDiff(340,43, 0, 7, LIGHTWHITE);	
        lcd_DrawLine(100+1,50,378,50, LIGHTWHITE);
	    //大矩形的长宽及其起始点相对中点位置
	    uint8_t width = 60;
	    uint8_t longth= 100;
	    uint8_t relative_x = 120;
        uint8_t relative_y = 70;		
		lcd_DrawRectangleDiff(xpoint-relative_x, ypoint+5-relative_y, width, longth, LIGHTWHITE);
		lcd_DrawRectangleDiff(xpoint+relative_x-width, ypoint+5-relative_y, width, longth, LIGHTWHITE);	
		//大矩形刻度线
		for(uint8_t i=1; i<8; i++)//左大矩形刻度线
		{
			lcd_DrawLineDiff(xpoint-relative_x+54, ypoint+5-relative_y+i*12.5, 5, 0, LIGHTWHITE);
		}
		for(uint8_t i=1; i<8; i++)//右大矩形刻度线
		{
			lcd_DrawLineDiff(xpoint+relative_x-width, ypoint+5-relative_y+i*12.5, 6, 0, LIGHTWHITE);
		}
		
	    //小矩形的长宽及其起始点相对中点位置
	    width = 18;
	    longth= 80;
	    relative_x = 138;
        relative_y = 60;		
		lcd_DrawRectangleDiff(xpoint-relative_x, ypoint+5-relative_y, width, longth, LIGHTWHITE);//左侧小矩形框
		lcd_DrawRectangleDiff(xpoint+relative_x-width, ypoint+5-relative_y, width, longth, LIGHTWHITE);//右侧小矩形框
        lcd_DrawFillRectangleDiff(xpoint+relative_x-width+1, ypoint+5-relative_y+1, width-2, longth-2, BACKCOLOR);//右侧填充背景色			
	}
	
	
	/********pitch***********************************************************/
	static int16_t pitchLast = 90;
	//限定角度值
	if(pitch >= 90) pitch = 90;
	else if(pitch <= -90) pitch = -90;
	if(pitch != pitchLast)
	{		
		//清除上次痕迹
        if(pitchLast >= 0)
		{
			lcd_ShowTriCursor(170, 100-pitchLast*0.44, 1, upBackColor);//left		
		}
		else
		{
			lcd_ShowTriCursor(170, 100-pitchLast*0.44, 1, downBackColor);//left			
		}
        if(pitchLast<=13 && pitchLast>=-13)
        {	
			lcd_DrawFillRectangleDiff(170-9, 100-6, 9, 5, upBackColor);
			lcd_DrawFillRectangleDiff(170-9, 100+1 ,9, 5, downBackColor);
			lcd_DrawLineDiff(170-9, 100, 9, 0, LIGHTWHITE);            			
		}		
		lcd_ShowTriCursor(170, 100-pitch*0.44, 1, RED);//left
        
        //左侧方框内显示俯仰角的值
        lcd_ShowNumBackColor(122,  100-7, LIGHTWHITE, 18, 4, pitch, 0, BACKCOLOR); 
        lcd_DrawRectangleDiff(122, 100-8 ,37, 15, LIGHTWHITE); 
        
		pitchLast = pitch;			
	}
		
	
	/********roll***********************************************************/	
    static int16_t rollLast = 180;
	if(roll >= 180) roll = 180;
	else if(roll <= -180) roll = -180;
    
    if(roll != rollLast)
    {  //中心点坐标(240, 100)	  
		if((100-sin(rollLast/57.3)*30)>100) 
		{
			lcd_DrawLine(240-cos(rollLast/57.3)*30, 100-sin(rollLast/57.3)*30, 240-cos(rollLast/57.3)*15, 100-sin(rollLast/57.3)*15, downBackColor);   
		}
		else 
		{
			lcd_DrawLine(240-cos(rollLast/57.3)*30, 100-sin(rollLast/57.3)*30, 240-cos(rollLast/57.3)*15, 100-sin(rollLast/57.3)*15, upBackColor); 
		}  	  

		if((100-cos(rollLast/57.3)*15)>100) 
		{
			lcd_DrawLine(240+sin(rollLast/57.3)*15, 100-cos(rollLast/57.3)*15, 240+sin(rollLast/57.3)*30, 100-cos(rollLast/57.3)*30, downBackColor);      
		}
		else 
		{
			lcd_DrawLine(240+sin(rollLast/57.3)*15, 100-cos(rollLast/57.3)*15, 240+sin(rollLast/57.3)*30, 100-cos(rollLast/57.3)*30, upBackColor);   
		} 

		if((100+sin(rollLast/57.3)*15)>100) 
		{
			lcd_DrawLine(240+cos(rollLast/57.3)*15, 100+sin(rollLast/57.3)*15, 240+cos(rollLast/57.3)*30, 100+sin(rollLast/57.3)*30, downBackColor); 
		}
		else 
		{
			lcd_DrawLine(240+cos(rollLast/57.3)*15, 100+sin(rollLast/57.3)*15, 240+cos(rollLast/57.3)*30, 100+sin(rollLast/57.3)*30, upBackColor);	  
		}

		lcd_DrawCicle(240, 100, 15, WHITE);
		lcd_DrawLineDiff(240-30,100,60,  0, LIGHTWHITE);
		lcd_DrawLine(240-cos(roll/57.3)*30, 100-sin(roll/57.3)*30, 240-cos(roll/57.3)*15, 100-sin(roll/57.3)*15, WHITE);   
		lcd_DrawLine(240+sin(roll/57.3)*15, 100-cos(roll/57.3)*15, 240+sin(roll/57.3)*30, 100-cos(roll/57.3)*30, WHITE);   
		lcd_DrawLine(240+cos(roll/57.3)*15, 100+sin(roll/57.3)*15, 240+cos(roll/57.3)*30, 100+sin(roll/57.3)*30, WHITE); 		  

		rollLast = roll;	  
     }
		
	/********heading***********************************************************/
	static int16_t headingLast = 360;
	static uint8_t headingStep = 0; //! for delay about 1s
	if(heading != headingLast)
	{
		headingStep = 0;       	
		if((heading-headingLast)<0)
		{	
			lcd_ShowTriCursor(240-54, 37, 0, RED);
			lcd_ShowTriCursor(240+55, 37, 1, LIGHTBLUE);
		}
		else 
		{
			lcd_ShowTriCursor(240-54, 37, 0, LIGHTBLUE);
			lcd_ShowTriCursor(240+55, 37, 1, RED);		   
		}
	   
		lcd_DrawFillRectangleDiff(240-15, 30, 32, 15, upBackColor);//清除上次的痕迹   
		if(heading == 0)        lcd_ShowCharBackColor(240-5, 30, WHITE, 18, 'N', upBackColor); 
		else if(heading == 90)  lcd_ShowCharBackColor(240-5, 30, WHITE, 18, 'E', upBackColor);		   
		else if(heading == 180) lcd_ShowCharBackColor(240-5, 30, WHITE, 18, 'S', upBackColor);		      
		else if(heading == 270) lcd_ShowCharBackColor(240-5, 30, WHITE, 18, 'W', upBackColor);		   	   
		else if(heading<10)     lcd_ShowNumBackColor(240-5,  30, WHITE, 18, 1, heading, 0, upBackColor);		   
		else if(heading<99)     lcd_ShowNumBackColor(240-9,  30, WHITE, 18, 2, heading, 0, upBackColor);		   
		else                    lcd_ShowNumBackColor(240-13, 30, WHITE, 18, 3, heading, 0, upBackColor);	

		headingLast = heading;
	}
	else
	{
		if(headingStep++ == 100)
		{
			lcd_ShowTriCursor(240-54, 37, 0, LIGHTBLUE);
			lcd_ShowTriCursor(240+55, 37, 1, LIGHTBLUE);			
		}
		
	}


    /********alt***********************************************************/
    static int32_t altLast = 1000;
	if(alt > 1000) alt = 1000;
	else if(alt < -100) alt = -100;
	if(abs(alt-altLast) > 5)
	{
		//基本范围为-100 -- 0 -- +1000  刻度范围为100: -40--+40  系数: *0.04 或 0.4
		//右侧方框内显示高度的值
		lcd_ShowNumBackColor(321,  100-7, LIGHTWHITE, 18, 4, alt, 0, BACKCOLOR);
		lcd_DrawRectangleDiff(321, 100-8 ,37, 15, LIGHTWHITE);
		//清除上次痕迹
		if(altLast >= 0)  lcd_ShowTriCursor(310, 100-altLast*0.04, 0, upBackColor); //right
		else		      lcd_ShowTriCursor(310, 100-altLast*0.4, 0, downBackColor); //right

		if(altLast*0.04<=6 && altLast*0.4>=-6)
		{	
			lcd_DrawFillRectangleDiff(310, 100-6, 9, 5, upBackColor);
			lcd_DrawFillRectangleDiff(310, 100+1 ,9, 5, downBackColor);
			lcd_DrawLineDiff(310, 100, 9, 0, LIGHTWHITE);            			
		}

		if(alt >= 0) lcd_ShowTriCursor(310, 100-alt*0.04, 0, RED); //right	
		else         lcd_ShowTriCursor(310, 100-alt*0.4,  0, RED); //right   

		altLast = alt;	   	
	}

	
    /********batRemain***********************************************************/  
	static uint8_t battery_remainingLast = 200;	
    if(abs(batRemain-battery_remainingLast) >= 3) //去除干扰
	{  
		switch(batRemain/9)
		{
			case 0: 
				for(uint8_t i=0; i<1; i++) lcd_DrawFillRectangleDiff(363, 133-7*i, 12, 4, RED); 
				for(uint8_t i=1; i<11;i++) lcd_DrawFillRectangleDiff(363, 133-7*i, 12, 4, BACKCOLOR);
				break;

			case 1: 
				for(uint8_t i=0; i<2; i++) lcd_DrawFillRectangleDiff(363, 133-7*i, 12, 4, RED);			        
				for(uint8_t i=2; i<11;i++) lcd_DrawFillRectangleDiff(363, 133-7*i, 12, 4, BACKCOLOR);
				break;

			case 2: 
				for(uint8_t i=0; i<3; i++) lcd_DrawFillRectangleDiff(363, 133-7*i, 12, 4, RED);			        
				for(uint8_t i=3; i<11;i++) lcd_DrawFillRectangleDiff(363, 133-7*i, 12, 4, BACKCOLOR);
				break;

			case 3: 
				for(uint8_t i=0; i<4; i++) lcd_DrawFillRectangleDiff(363, 133-7*i, 12, 4, RED);			        
				for(uint8_t i=4; i<11;i++) lcd_DrawFillRectangleDiff(363, 133-7*i, 12, 4, BACKCOLOR);
				break;

			case 4: 
				for(uint8_t i=0; i<5; i++) lcd_DrawFillRectangleDiff(363, 133-7*i, 12, 4, RED);			        
				for(uint8_t i=5; i<11;i++) lcd_DrawFillRectangleDiff(363, 133-7*i, 12, 4, BACKCOLOR);
				break;

			case 5: 
				for(uint8_t i=0; i<6; i++) lcd_DrawFillRectangleDiff(363, 133-7*i, 12, 4, RED);			        
				for(uint8_t i=6; i<11;i++) lcd_DrawFillRectangleDiff(363, 133-7*i, 12, 4, BACKCOLOR);
				break;

			case 6: 
				for(uint8_t i=0; i<7; i++) lcd_DrawFillRectangleDiff(363, 133-7*i, 12, 4, GREEN);			        
				for(uint8_t i=7; i<11;i++) lcd_DrawFillRectangleDiff(363, 133-7*i, 12, 4, BACKCOLOR);
				break;

			case 7: 
				for(uint8_t i=0; i<8; i++) lcd_DrawFillRectangleDiff(363, 133-7*i, 12, 4, GREEN);			        
				for(uint8_t i=8; i<11;i++) lcd_DrawFillRectangleDiff(363, 133-7*i, 12, 4, BACKCOLOR);
				break;

			case 8: 
				for(uint8_t i=0; i<9; i++) lcd_DrawFillRectangleDiff(363, 133-7*i, 12, 4, GREEN);			        
				for(uint8_t i=9; i<11;i++) lcd_DrawFillRectangleDiff(363, 133-7*i, 12, 4, BACKCOLOR);
				break;

			case 9: 
				for(uint8_t i=0; i<10; i++) lcd_DrawFillRectangleDiff(363, 133-7*i, 12, 4, GREEN);			        
				for(uint8_t i=10; i<11;i++) lcd_DrawFillRectangleDiff(363, 133-7*i, 12, 4, BACKCOLOR);
				break;

			case 10: 
				for(uint8_t i=0; i<11; i++) lcd_DrawFillRectangleDiff(363, 133-7*i, 12, 4, GREEN);			        
				break;				
		}
		
		battery_remainingLast = batRemain;
	}
}


/************************************************
gui   name: displayAirSpeed
      func: 
	 (x,y):  
************************************************/
void displayAirSpeed(uint16_t x, uint16_t y, float airSpeed)
{
	uint8_t logo[] = "AIRSPD";
	const uint8_t charsize = 24;
	const uint8_t valuelength = 5;

	lcd_ShowString(x, y, WHITE, charsize, logo);

	static float airSpeedLast = 50.0;
	if(fabs(airSpeed-airSpeedLast) > 0.5f)
	{
		lcd_ShowFloat(x+(sizeof(logo)-1-valuelength)*charsize/4, y+25, WHITE, charsize, valuelength, airSpeed);	
		airSpeedLast = airSpeed;	  
	}

}


/************************************************
gui   name: displayGroundSpeed
      func: 
	 (x,y):  
************************************************/
void displayGroundSpeed(uint16_t x, uint16_t y, float groundSpeed)
{
	uint8_t logo[] = "GRDSPD";
	const uint8_t charsize = 24;
	const uint8_t valuelength = 5;

	lcd_ShowString(x, y, WHITE, charsize, logo);

	static float groundSpeedLast = 50.0;
	if(fabs(groundSpeed-groundSpeedLast) > 0.5f)
	{
		lcd_ShowFloat(x+(sizeof(logo)-1-valuelength)*charsize/4, y+25, WHITE, charsize, valuelength, groundSpeed);	
		groundSpeedLast = groundSpeed;	  
	}   
}


/************************************************
gui   name: displayClimb
      func: 
	 (x,y):  
************************************************/
void displayClimb(uint16_t x, uint16_t y, float climb)
{
	uint8_t logo[] = "CLIMB";
	const uint8_t charsize = 24;
	const uint8_t valuelength = 5;

	lcd_ShowString(x, y, WHITE, charsize, logo);

	static float climbLast = 50.0;
	if(fabs(climb-climbLast) > 0.1f)
	{
		lcd_ShowFloat(x+(sizeof(logo)-1-valuelength)*charsize/4, y+25, WHITE, charsize, valuelength, climb);	
		climbLast = climb;	  
	}
}


/************************************************
gui   name: displayThrottle
      func: 
	 (x,y):  
************************************************/
void displayThrottle(uint16_t x, uint16_t y, uint16_t throttle)
{
	uint8_t logo[] = "THROTTLE";
	const uint8_t charsize = 24;
	const uint8_t valuelength = 3;

	lcd_ShowString(x, y, WHITE, charsize, logo);

	static uint16_t throttleLast = 50.0;
	if(throttle != throttleLast)
	{
		lcd_ShowNum(x+(sizeof(logo)-1-valuelength)*charsize/4, y+25, WHITE, charsize, valuelength, throttle, 0);	
		throttleLast = throttle;	  
	}
}



/************************************************
gui   name: displayUpdating
      func: 显示在线更新状态 
	 (x,y):  
************************************************/
void displayUpdating(uint16_t x, uint16_t y)
{
	static uint8_t updateLast = 255;
	if(g_eeGeneral.firmwareUpdate != updateLast)
	{
		if(g_eeGeneral.firmwareUpdate == FIRMWARE_UPDATE) //! updating!!!
		{
			uint8_t UPDATING[]= "  UPDATING  ";					  
			lcd_ShowString(x-72, y, RED, 24, UPDATING); 		
		}	
		else if(g_eeGeneral.firmwareUpdate == FIRMWARE_REJECT) //! update rejected!!!
		{
			uint8_t REJECTED[]= "  REJECTED  ";					  
			lcd_ShowString(x-72, y, RED, 24, REJECTED);		  
		}
		
		updateLast = g_eeGeneral.firmwareUpdate;	  
	}

}



/************************************************
gui   name: displayContectState
      func: 显示数传连接状态 
	 (x,y):  
************************************************/
void displayContectState(uint16_t x, uint16_t y)
{
	if(mavData.mavStatus.health == 30)     //! uncontected!
	{
		uint8_t UNCONNTECTED[]= "UNCONNTECTED";		
		lcd_ShowString(x-72, y, RED, 24, UNCONNTECTED);			
	}
}




/************************************************
gui   name: convert_attitude
      func: convert attitude from radian to degree    
************************************************/
void convert_attitude(void)
{
	float pitchTemp = mavData.attitude.pitch_rad;
	float rollTemp  = mavData.attitude.roll_rad;
	float yawTemp   = mavData.attitude.yaw_rad;
	
	if(mavData.sysStatus.vtolState == MAV_VTOL_STATE_FW)   
	{   // fw mode
		float Me[3][3] = {0}, Him[3][3] = {0};
		dcm_from_euler(Me, pitchTemp, rollTemp, yawTemp);
		dcm_from_euler(Him, 1.57f, 0.0f, 0.0f);

		float self[3][3] = {0}, other[3][3] = {0}, res[3][3] = {0};
		memcpy(self, Me, sizeof(Me));
		memcpy(other, (Him), sizeof(Him));
		memset(res, 0, sizeof(res)); 

		for(uint8_t i = 0; i < 3; i++) 
		{
			for(uint8_t k = 0; k < 3; k++) 
			{
				for(uint8_t j = 0; j < 3; j++) 
				{
					res[i][k] += self[i][j] * other[j][k];
				}
			}
		}
		dcm_to_euler(&pitchTemp, &rollTemp, &yawTemp, res);
		mavData.attitude.pitch = pitchTemp * 57.3;        
		mavData.attitude.roll  = rollTemp  * 57.3;
		mavData.attitude.yaw   = (yawTemp < 0) ? yawTemp*57.3+360 : yawTemp*57.3;        
	}
	else if(mavData.sysStatus.vtolState == MAV_VTOL_STATE_MC) 
	{   // mc mode
		mavData.attitude.roll  = rollTemp  * 57.3;
		mavData.attitude.pitch = pitchTemp * 57.3;
		mavData.attitude.yaw   = (yawTemp < 0) ? yawTemp*57.3+360 : yawTemp*57.3;
	}
	else 
	{
		mavData.attitude.roll  = rollTemp  * 57.3;
		mavData.attitude.pitch = pitchTemp * 57.3;
		mavData.attitude.yaw   = (yawTemp < 0) ? yawTemp*57.3+360 : yawTemp*57.3;		
	}			
	
}





/************************************************
gui   name: view_information
      func: 随时显示当前重要信息，每100ms更新一次 
	    id: information id   
************************************************/
void view_information(uint16_t id)
{
	const uint16_t x = 240;
	const uint16_t y = 3;

	static uint8_t count = 0;

	if((count++ >= 10)||(g_eeGeneral.firmwareUpdate == 1)) count = 0; //! 10Hz and if do update then view_information 
  
	switch(count)
	{
		case 0:
			displayUpdating(x, y);
			break;		  	
		case 1:
			displayUavFlightMode(x, y);
			break;	
		case 2:
			displayContectState(x, y);
			break;	
		case 3:
			break;	
		case 4:
			break;	
		case 5:
			break;	
		case 6:
			break;	
		case 7:
			break;	
		case 8:
			break;	
		case 9:
			break;	
	}

}













 
void displayTest(void)
{  	

   #if defined APPLE_DEBUG

   lcd_ShowNum(40, 150,  RED, 24, 3, mavData.mavStatus.health, 0);   	   
    // lcd_ShowNum(400, 30, RED, 24, 4, anaIn(1), 0);
    // lcd_ShowNum(400, 60, RED, 24, 4, anaIn(0), 0);
    // lcd_ShowNum(400, 90, RED, 24, 4, anaIn(2), 0);
    // lcd_ShowNum(400,120, RED, 24, 4, anaIn(3), 0);
    // lcd_ShowNum(400,150, RED, 24, 4, anaIn(4), 0);
    // lcd_ShowNum(400,180, RED, 24, 4, anaIn(5), 0);
    // lcd_ShowNum(400,210, RED, 24, 4, anaIn(6), 0);
    // lcd_ShowNum(400,240, RED, 24, 4, anaIn(7), 0);	
	
    // lcd_ShowNum(40, 30, RED, 18, 4, g_eeGeneral.joyscale.ele_max, 0);
    // lcd_ShowNum(40, 50, RED, 18, 4, g_eeGeneral.joyscale.ele_cen, 0);
    // lcd_ShowNum(40, 70, RED, 18, 4, g_eeGeneral.joyscale.ele_min, 0);
    // lcd_ShowNum(40, 90, RED, 18, 4, g_eeGeneral.joyscale.rud_max, 0);
    // lcd_ShowNum(40,110, RED, 18, 4, g_eeGeneral.joyscale.rud_cen, 0);
    // lcd_ShowNum(40,130, RED, 18, 4, g_eeGeneral.joyscale.rud_min, 0);
    // lcd_ShowNum(40,150, RED, 18, 4, g_eeGeneral.joyscale.thr_max, 0);
    // lcd_ShowNum(40,170, RED, 18, 4, g_eeGeneral.joyscale.thr_cen, 0);
    // lcd_ShowNum(40,190, RED, 18, 4, g_eeGeneral.joyscale.thr_min, 0);
    // lcd_ShowNum(40,210, RED, 18, 4, g_eeGeneral.joyscale.ail_max, 0);
    // lcd_ShowNum(40,230, RED, 18, 4, g_eeGeneral.joyscale.ail_cen, 0);
    // lcd_ShowNum(40,250, RED, 18, 4, g_eeGeneral.joyscale.ail_min, 0);
    // lcd_ShowNum(40,270, RED, 18, 4, g_eeGeneral.joyscale.ltrm_max, 0);
    // lcd_ShowNum(40,290, RED, 18, 4, g_eeGeneral.joyscale.ltrm_cen, 0);
    // lcd_ShowNum(40,310, RED, 18, 4, g_eeGeneral.joyscale.ltrm_min, 0);
	 //lcd_ShowNum(40,90, RED, 24, 2, g_eeGeneral.key, 0);   
       // lcd_ShowNum(40,200, RED, 24, 3, mavData.mavStatus.pdlState, 0);
	   // lcd_ShowNum(40,90, RED, 24, 2, g_eeGeneral.menuCurrent, 0);
	// lcd_ShowNum(40,60, RED, 24, 4, g_eeGeneral.joystick.ltrm, 0);
    // lcd_ShowNum(40,90, RED, 24, 4, g_eeGeneral.joystick.rtrm, 0);	
	
	// lcd_ShowNum(40,150, RED, 24, 4, g_eeGeneral.joystick.ele, 0);
    // lcd_ShowNum(40,180, RED, 24, 4, g_eeGeneral.joystick.rud, 0);
    // lcd_ShowNum(40,210, RED, 24, 4, g_eeGeneral.joystick.thr, 0);
    // lcd_ShowNum(40,240, RED, 24, 4, g_eeGeneral.joystick.ail, 0);
	// menuFlightMode();
    lcd_ShowNum(40, 120,  RED, 24, 3, mavData.heartBeat.type, 0);
    lcd_ShowNum(40, 150,  RED, 24, 3, mavData.sysStatus.vtolState, 0);
	 
    //displayBar(200, 186, 0, 14, g_eeGeneral.joystick.baseEle*0.3, BACKCOLOR);
	// lcd_ShowNum(250, 120,  RED, 24, 4, mavData.mav_mode_v.custMode, 0);	 
	// lcd_ShowNum(350,  30,  RED, 24, 4, mavData.radioStatus.remrssi, 0);
    //displayBar(220, 200, 0, 20, g_eeGeneral.joystick.baseEle*0.3, GREEN);
	
   #endif
}
 

 

 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 