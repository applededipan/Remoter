


//! this file is added by apple
#include "../../opentx.h"
#include "../../global.h"
#include "telemetry/mavlink.h"
#include "string.h"


/************************************************
gui   name: menusProcess(uint32_t keyChoice, uint32_t keyEnter) 
************************************************/
void menusProcess(uint32_t keyChoice, uint32_t keyEnter)
{
    if(g_eeGeneral.key == keyChoice) //! 选择键按下
    {
       g_eeGeneral.key = 0;
       if(g_eeGeneral.menus[g_eeGeneral.menuCurrent].subState == SUB_STATE_NONE)   //! 如果当前菜单为浏览模式则切换到下一个菜单
       {
           g_eeGeneral.menuCurrent++; 
           g_eeGeneral.menus[g_eeGeneral.menuCurrent].subState = SUB_STATE_INIT;   //! 将切换前的菜单的子状态复位           
           if(g_eeGeneral.menuCurrent == MENU_NUMBER) 
		   {
			   g_eeGeneral.menuCurrent = MENU_MAIN;  
			   g_eeGeneral.menus[g_eeGeneral.menuCurrent].subState = SUB_STATE_INIT;
		   }			   
       }          
       else
       {
           
       }
     
    }
    // else if(g_eeGeneral.key == keyEnter) //! 确认键按下
    // {
	   // g_eeGeneral.key = 0;
       // if(g_eeGeneral.menus[g_eeGeneral.menuCurrent].subState == SUB_STATE_NONE)
       // {
           // g_eeGeneral.menus[g_eeGeneral.menuCurrent].subState = SUB_STATE_START; 
       // }
           
    // }
    
    menu_update(g_eeGeneral.menuCurrent);
}



/************************************************
gui   name: menu_update(uint8_t page)
      func: 切换显示各子菜单，互斥，每10ms更新一次 
     event: 
************************************************/
void menu_update(uint8_t page)
{  
  switch(page)
  {
	case  MENU_MAIN:
          menu_main(g_eeGeneral.menus[MENU_MAIN].subState);
          break;		  	
	case  MENU_CALIBJOYSTICK:
	      menu_calibJoystick(g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState);
          break;	
	case  MENU_RCCHANNELS:
          menu_rcChannels(g_eeGeneral.menus[MENU_RCCHANNELS].subState);
          break;	
	case  MENU_GIMBAL:
          menu_gimbal(g_eeGeneral.menus[MENU_GIMBAL].subState);
          break;	
	case  MENU_CAMERA:
          menu_camera(g_eeGeneral.menus[MENU_CAMERA].subState);
          break;	
	case  MENU_FAILSAFE:
          menu_failsafe(g_eeGeneral.menus[MENU_FAILSAFE].subState);
          break;	
	case  MENU_DEBUG:
          menu_debug(g_eeGeneral.menus[MENU_DEBUG].subState);
          break;	
	case  MENU_VERSION:
          menu_version(g_eeGeneral.menus[MENU_VERSION].subState);
          break;	
	case  MENU_FLIGHTMODE:
          menu_flightmode(g_eeGeneral.menus[MENU_FLIGHTMODE].subState);
          break;	
	case  MENU_BATTERY:
          menu_battery(g_eeGeneral.menus[MENU_BATTERY].subState);
          break;
    case  MENU_RADIO:
          menu_radio(g_eeGeneral.menus[MENU_RADIO].subState);          
          break;          
  }

}



/************************************************
      name: showTitle() 显示不同菜单的名字，只在该区域内有效
      func:  
************************************************/
#define TITLECOLOR  LIGHTBLACK
static void showTitle(uint16_t x, uint16_t y, uint16_t color, const unsigned char *p, uint16_t backcolor)
{
   lcd_DrawFillRectangle(102, 165, 378, 188, backcolor);
   lcd_ShowStringBackcolor(x, y, color, 24, p, backcolor); 
}


/************************************************
      name: menuClear()，对菜单清屏，只在该区域内有效
      func:  
************************************************/
static void menuClear(uint16_t color)
{
   lcd_DrawFillRectangle(101, 189, 379, 318, color); //! 清屏     
}



/************************************************
menu  name: menu_main
      func:  
************************************************/
void menu_main(uint8_t state)
{
   const uint16_t x = 240;        //! 区域坐标：(240-140, 162)(240+140, 162)
   const uint16_t y = 200;        //! 区域坐标：(240-140, 320)(240+140, 320)  
   
   uint8_t logo[]= "MAINMENU";
   switch(state)
   {
       case SUB_STATE_INIT:
            menuClear(BACKCOLOR); //! 清屏                              
            showTitle(x-(sizeof(logo)-1)*6, y-35, WHITE, logo, TITLECOLOR);
            g_eeGeneral.menus[MENU_MAIN].subState = SUB_STATE_NONE;
            break;
       case SUB_STATE_START:
            showTitle(x-(sizeof(logo)-1)*6, y-35, WHITE, logo, GREEN);
            break;
       case MAIN_STATE_EXIT:
            break;
       case SUB_STATE_NONE:
            break;            
   }
}



/************************************************
menu  name: menu_failsafe
      func:  
************************************************/
void menu_failsafe(uint8_t state)
{
   const uint16_t x = 240;        //! 区域坐标：(240-140, 162)(240+140, 162)
   const uint16_t y = 200;        //! 区域坐标：(240-140, 320)(240+140, 320)
   
   uint8_t logo[]= "FAILSAFE"; 
   
   switch(state)
   {
       case SUB_STATE_INIT:
            menuClear(BACKCOLOR); //! 清屏 
            showTitle(x-(sizeof(logo)-1)*6, y-35, WHITE, logo, TITLECOLOR);
            g_eeGeneral.menus[MENU_FAILSAFE].subState = SUB_STATE_NONE;
            break;
       case SUB_STATE_START:
            showTitle(x-(sizeof(logo)-1)*6, y-35, WHITE, logo, GREEN);
            break;
       case FAILSAFE_STATE_EXIT:
            break;
       case SUB_STATE_NONE:
            break;
   }   
}



/************************************************
menu  name: menu_gimbal
      func:  
************************************************/
void menu_gimbal(uint8_t state)
{
   const uint16_t x = 240;        //! 区域坐标：(240-140, 162)(240+140, 162)
   const uint16_t y = 200;        //! 区域坐标：(240-140, 320)(240+140, 320)
   
   uint8_t logo[]= "GIMBAL"; 
   
   switch(state)
   {
       case SUB_STATE_INIT:
            menuClear(BACKCOLOR); //! 清屏
            showTitle(x-(sizeof(logo)-1)*6, y-35, WHITE, logo, TITLECOLOR);
            g_eeGeneral.menus[MENU_GIMBAL].subState = SUB_STATE_NONE;
            break;
       case SUB_STATE_START:
            showTitle(x-(sizeof(logo)-1)*6, y-35, WHITE, logo, GREEN);
            break;
       case GIMBAL_STATE_EXIT:
            break;
       case SUB_STATE_NONE:
            break;
   }  

}




/************************************************
menu  name: menu_camera
      func:  
************************************************/
void menu_camera(uint8_t state)
{
   const uint16_t x = 240;        //! 区域坐标：(240-140, 162)(240+140, 162)
   const uint16_t y = 200;        //! 区域坐标：(240-140, 320)(240+140, 320)
   
   uint8_t logo[]= "CAMERA"; 
   uint8_t zoom[]= "ZOOM ENABLE";
   
   switch(state)
   {
       case SUB_STATE_INIT:
            menuClear(BACKCOLOR); //! 清屏 
            showTitle(x-(sizeof(logo)-1)*6, y-35, WHITE, logo, TITLECOLOR);
			#if defined(DIGICAM_ZONE_IN_OUT)			
			lcd_ShowString(x-130, y, LIGHTWHITE, 24, zoom);
            #endif	
            g_eeGeneral.menus[MENU_CAMERA].subState = SUB_STATE_NONE;
            break;
       case SUB_STATE_START:
            showTitle(x-(sizeof(logo)-1)*6, y-35, WHITE, logo, GREEN);
            break;
       case CAMERA_STATE_EXIT:
            break;
       case SUB_STATE_NONE:
            break;
   }    

}



/************************************************
menu  name: menu_version
      func:  
************************************************/
void menu_version(uint8_t state)
{
   const uint16_t x = 240;        //! 区域坐标：(240-140, 162)(240+140, 162)
   const uint16_t y = 200;        //! 区域坐标：(240-140, 320)(240+140, 320)
  
   uint8_t logo[]= "VERSION";
   uint8_t thow[]= "THROW ENABLE";
   uint8_t mav1[]= "MAVLINK1.0";
   uint8_t mav2[]= "MAVLINK2.0";
   
   switch(state)
   {
       case SUB_STATE_INIT:
            menuClear(BACKCOLOR); //! 清屏
            showTitle(x-(sizeof(logo)-1)*6, y-35, WHITE, logo, TITLECOLOR);		
			#if defined(THROW_CONTROL)			
			lcd_ShowString(x-130, y, LIGHTWHITE, 24, thow);
            #endif
			#if defined(MAVLINK_1)			
			lcd_ShowString(x-130, y+30, LIGHTWHITE, 24, mav1);
			#else
			lcd_ShowString(x-130, y+30, LIGHTWHITE, 24, mav2);	
            #endif			
            g_eeGeneral.menus[MENU_VERSION].subState = SUB_STATE_NONE;
            break;
       case SUB_STATE_START:
            showTitle(x-(sizeof(logo)-1)*6, y-35, WHITE, logo, GREEN);
            break;
       case VERSION_STATE_EXIT:
            break;
       case SUB_STATE_NONE:
            break;
   }  
   
}



/************************************************
menu  name: menu_debug
      func:  
************************************************/
void menu_debug(uint8_t state)
{
   const uint16_t x = 240;        //! 区域坐标：(240-140, 162)(240+140, 162)
   const uint16_t y = 200;        //! 区域坐标：(240-140, 320)(240+140, 320)

   uint8_t logo[]= "DEBUG"; 
   
   switch(state)
   {
       case SUB_STATE_INIT:
            menuClear(BACKCOLOR); //! 清屏 
            showTitle(x-(sizeof(logo)-1)*6, y-35, WHITE, logo, TITLECOLOR);
            g_eeGeneral.menus[MENU_DEBUG].subState = SUB_STATE_NONE;
            break;
       case SUB_STATE_START:
            showTitle(x-(sizeof(logo)-1)*6, y-35, WHITE, logo, GREEN);
            break;
       case DEBUG_STATE_EXIT:
            break;
       case SUB_STATE_NONE:
            break;
   }    
   
}



/************************************************
menu  name: menu_flightmode
      func:  
************************************************/
void menu_flightmode(uint8_t state)
{
   const uint16_t x = 240;        //! 区域坐标：(240-140, 162)(240+140, 162)
   const uint16_t y = 200;        //! 区域坐标：(240-140, 320)(240+140, 320)
  
   uint8_t logo[]= "FLIGHTMODE"; 
   
   switch(state)
   {
       case SUB_STATE_INIT:
            menuClear(BACKCOLOR); //! 清屏 
            showTitle(x-(sizeof(logo)-1)*6, y-35, WHITE, logo, TITLECOLOR);
            g_eeGeneral.menus[MENU_FLIGHTMODE].subState = SUB_STATE_NONE;
            break;
       case SUB_STATE_START:
            showTitle(x-(sizeof(logo)-1)*6, y-35, WHITE, logo, GREEN);
            break;
       case FLIGHTMODE_STATE_EXIT:
            break;
       case SUB_STATE_NONE:
            break;
   }    

}




/************************************************
menu  name: menu_battery
      func:  
************************************************/
void menu_battery(uint8_t state)
{
   const uint16_t x = 240;        //! 区域坐标：(240-140, 162)(240+140, 162)
   const uint16_t y = 200;        //! 区域坐标：(240-140, 320)(240+140, 320)
   
   uint8_t logo[]= "BATTERY";
   
   switch(state)
   {
       case SUB_STATE_INIT:
            menuClear(BACKCOLOR); //! 清屏 
            showTitle(x-(sizeof(logo)-1)*6, y-35, WHITE, logo, TITLECOLOR);
            g_eeGeneral.menus[MENU_BATTERY].subState = SUB_STATE_NONE;
            break;
       case SUB_STATE_START:
            showTitle(x-(sizeof(logo)-1)*6, y-35, WHITE, logo, GREEN);
            break;
       case BATTERY_STATE_EXIT:
            break;
       case SUB_STATE_NONE:
            break;
   }    

}




/************************************************
menu  name: menu_radio
      func:  
************************************************/
void menu_radio(uint8_t state)
{
   const uint16_t x = 240;        //! 区域坐标：(240-140, 162)(240+140, 162)
   const uint16_t y = 200;        //! 区域坐标：(240-140, 320)(240+140, 320)
 
   uint8_t logo[] = "RADIO";
   uint8_t type[] = "Type: PMP";
   uint8_t mode[] = "Mode: Master";
   uint8_t addr[] = "Addr: 20160821";
   uint8_t rate[] = "Rate: 115200"; 
   
   switch(state)
   {
       case SUB_STATE_INIT:
            menuClear(BACKCOLOR); //! 清屏
            showTitle(x-(sizeof(logo)-1)*6, y-35, WHITE, logo, TITLECOLOR);
            lcd_ShowString(x-130,    y, LIGHTWHITE, 24, type);
            lcd_ShowString(x-130, y+30, LIGHTWHITE, 24, mode);           
            lcd_ShowString(x-130, y+60, LIGHTWHITE, 24, addr);  
            lcd_ShowString(x-130, y+90, LIGHTWHITE, 24, rate);               
            g_eeGeneral.menus[MENU_RADIO].subState = SUB_STATE_NONE;
            break;
       case SUB_STATE_START:
            showTitle(x-(sizeof(logo)-1)*6, y-35, WHITE, logo, GREEN);
            break;
       case RADIO_STATE_EXIT:
            break;
       case SUB_STATE_NONE:
            break;
   } 

}



/************************************************
menu  name: menu_calibJoystick
      func: 校准遥控器  每10ms执行一次
************************************************/
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

void menu_calibJoystick(uint8_t state)
{ 
   const uint16_t x = 240;        //! 区域坐标：(240-140, 200)(240+140, 200)
   const uint16_t y = 200;        //! 区域坐标：(240-140, 320)(240+140, 320)
   const uint16_t timers = 499;   //! 采样摇杆次数

   static uint16_t count = 0;
   static uint16_t error = 0; 
   
   uint8_t logo[]= "CALIBJOYSTICK";              

    switch(state)
    { 
     case SUB_STATE_INIT:
          menuClear(BACKCOLOR); //! 清屏
          showTitle(x-(sizeof(logo)-1)*6, y-35, WHITE, logo, TITLECOLOR);           
          lcd_DrawCicle(x-70, y+70, 45, LINEBLACK);
          lcd_DrawCicle(x+70, y+70, 45, LINEBLACK);
          
          lcd_ShowCircle(x-70, y+70-35, LINEBLACK); 
          lcd_ShowCircle(x-70, y+70+35, LINEBLACK);
          lcd_ShowCircle(x-70-35, y+70, LINEBLACK);   
          lcd_ShowCircle(x-70+35, y+70, LINEBLACK); 
          lcd_ShowCircle(x-70   , y+70, LINEBLACK); 
          lcd_ShowCircle(x+70, y+70-35, LINEBLACK);
          lcd_ShowCircle(x+70, y+70+35, LINEBLACK); 
          lcd_ShowCircle(x+70-35, y+70, LINEBLACK);  
          lcd_ShowCircle(x+70+35, y+70, LINEBLACK);
          lcd_ShowCircle(x+70   , y+70, LINEBLACK);
          
          lcd_ShowTriCursor(x-70-45, y+45-35,  0, LINEBLACK);  
          lcd_ShowTriCursor(x-70+45, y+45-35,  1, LINEBLACK);
          lcd_ShowCircle(x-70, y+45-35, LINEBLACK);
          lcd_ShowTriCursor(x+70-45, y+45-35,  0, LINEBLACK); 
          lcd_ShowTriCursor(x+70+45, y+45-35,  1, LINEBLACK); 
          lcd_ShowCircle(x+70, y+45-35, LINEBLACK); 
          g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = SUB_STATE_NONE;          
          break;     
     case SUB_STATE_START:
          showTitle(x-(sizeof(logo)-1)*6, y-35, WHITE, logo, GREEN);     
          g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_THRESHOLD;      
          break;          
     case CALIB_STATE_THRESHOLD:
          g_eeGeneral.joyscale.threshold = 50;
          g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_STICK_LEFT_UP;           
          break;        
     case CALIB_STATE_STICK_LEFT_UP:
          if(count%40 == 0)      lcd_ShowCircle(x-70, y+70-35, WHITE);          //! 左上
          else if(count%40 == 20) lcd_ShowCircle(x-70, y+70-35, BACKCOLOR); 
          count++; //!          
          if(anaIn(1)<=g_eeGeneral.joyscale.ele_min) g_eeGeneral.joyscale.ele_min = anaIn(1); 
          //else error++;         
          if(count>timers) //! 采样500次，大概是5s
          {
             if(error>timers/3) //! calib failed!
             {
                lcd_ShowCircle(x-70, y+70-35, RED);
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_ERROR;                                
             }
             else
             {
                lcd_ShowCircle(x-70, y+70-35, WHITE); //! OK
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_STICK_LEFT_DOWN;                     
             }
             count = 0;
             error = 0;          
          }
          break;          
     case CALIB_STATE_STICK_LEFT_DOWN:
          if(count%40 == 0)      lcd_ShowCircle(x-70, y+70+35, WHITE);          //! 左下
          else if(count%40 == 20) lcd_ShowCircle(x-70, y+70+35, BACKCOLOR);
          count++;
          if(anaIn(1)>=g_eeGeneral.joyscale.ele_max) g_eeGeneral.joyscale.ele_max = anaIn(1);
          //else error++;           
          if(count>timers) //! 采样500次，大概是5s
          {
             if(error>timers/3) //! calib failed!
             {
                lcd_ShowCircle(x-70, y+70+35, RED);
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_ERROR;                
             }
             else
             {
                lcd_ShowCircle(x-70, y+70+35, WHITE); //! OK
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_STICK_LEFT_LEFT;                 
             }
             error = 0;
             count = 0;              
          }          
          break;          
     case CALIB_STATE_STICK_LEFT_LEFT:
          if(count%40 == 0)      lcd_ShowCircle(x-70-35, y+70, WHITE);          //! 左左
          else if(count%40 == 20) lcd_ShowCircle(x-70-35, y+70, BACKCOLOR); 
          count++;
          if(anaIn(0)>=g_eeGeneral.joyscale.rud_max) g_eeGeneral.joyscale.rud_max = anaIn(0);  
          //else error++;          
          if(count>timers) //! 采样500次，大概是5s
          {
             if(error>timers/3) //! calib failed!
             {
                lcd_ShowCircle(x-70-35, y+70, RED);
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_ERROR;
             }
             else
             {
                lcd_ShowCircle(x-70-35, y+70, WHITE); //! OK   
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_STICK_LEFT_RIGHT;               
             } 
             error = 0;
             count = 0;             
          }           
          break;          
     case CALIB_STATE_STICK_LEFT_RIGHT:
          if(count%40 == 0)      lcd_ShowCircle(x-70+35, y+70, WHITE);          //! 左右
          else if(count%40 == 20) lcd_ShowCircle(x-70+35, y+70, BACKCOLOR); 
          count++;
          if(anaIn(0)<=g_eeGeneral.joyscale.rud_min) g_eeGeneral.joyscale.rud_min = anaIn(0);  
          //else error++;          
          if(count>timers) //! 采样500次，大概是5s
          {
             if(error>timers/3)
             {
                lcd_ShowCircle(x-70+35, y+70, RED);
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_ERROR;
             }
             else
             {
                lcd_ShowCircle(x-70+35, y+70, WHITE); //! OK  
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_STICK_LEFT_CENTER;                  
             } 
             error = 0;
             count = 0;            
          }          
          break;          
     case CALIB_STATE_STICK_LEFT_CENTER: 
          if(count%40 == 0)      lcd_ShowCircle(x-70   , y+70, WHITE);       //! 左中 
          else if(count%40 == 20) lcd_ShowCircle(x-70   , y+70, BACKCOLOR); 
          count++;
          if((anaIn(1)<=g_eeGeneral.joyscale.ele_max-600)&&(anaIn(1)>=g_eeGeneral.joyscale.ele_min+600)) g_eeGeneral.joyscale.ele_cen = anaIn(1); 
          else error++;
          if((anaIn(0)<=g_eeGeneral.joyscale.rud_max-600)&&(anaIn(0)>=g_eeGeneral.joyscale.rud_min+600)) g_eeGeneral.joyscale.rud_cen = anaIn(0); 
          //else error++;          
          if(count>timers) //! 采样500次，大概是5s
          {
             if(error>timers/3)
             {
                lcd_ShowCircle(x-70   , y+70, RED);
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_ERROR;
             }
             else
             {
                lcd_ShowCircle(x-70, y+70, WHITE);    //! OK
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_STICK_RIGHT_UP;                 
             }  
             error = 0;
             count = 0;             
          }          
          break;          
     case CALIB_STATE_STICK_RIGHT_UP: 
          if(count%40 == 0)      lcd_ShowCircle(x+70, y+70-35, WHITE);       //! 右上
          else if(count%40 == 20) lcd_ShowCircle(x+70, y+70-35, BACKCOLOR);
          count++;    
          if(anaIn(2)>=g_eeGeneral.joyscale.thr_max) g_eeGeneral.joyscale.thr_max = anaIn(2); 
          //else error++;          
          if(count>timers) //! 采样500次，大概是5s
          {
             if(error>timers/3)
             {
                lcd_ShowCircle(x+70, y+70-35, RED);
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_ERROR;
             }
             else
             {
                lcd_ShowCircle(x+70, y+70-35, WHITE); //! OK 
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_STICK_RIGHT_DOWN;                  
             } 
             error = 0;
             count = 0;             
          }          
          break;          
     case CALIB_STATE_STICK_RIGHT_DOWN:     
          if(count%40 == 0)      lcd_ShowCircle(x+70, y+70+35, WHITE);          //! 右下
          else if(count%40 == 20) lcd_ShowCircle(x+70, y+70+35, BACKCOLOR); 
          count++; 
          if(anaIn(2)<=g_eeGeneral.joyscale.thr_min) g_eeGeneral.joyscale.thr_min = anaIn(2);
          //else error++;          
          if(count>timers) //! 采样500次，大概是5s
          {
             if(error>timers/3)
             {
                lcd_ShowCircle(x+70, y+70+35, RED);
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_ERROR;
             }
             else
             {
                lcd_ShowCircle(x+70, y+70+35, WHITE); //! OK 
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_STICK_RIGHT_LEFT;                  
             }
             error = 0;    
             count = 0;             
          }          
          break;         
     case CALIB_STATE_STICK_RIGHT_LEFT:     
          if(count%40 == 0)      lcd_ShowCircle(x+70-35, y+70, WHITE);       //! 右左
          else if(count%40 == 20) lcd_ShowCircle(x+70-35, y+70, BACKCOLOR);
          count++;
          if(anaIn(3)<=g_eeGeneral.joyscale.ail_min) g_eeGeneral.joyscale.ail_min = anaIn(3); 
          //else error++;          
          if(count>timers) //! 采样500次，大概是5s
          {
             if(error>timers/3)
             {
                lcd_ShowCircle(x+70-35, y+70, RED);
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_ERROR;
             }
             else
             {
                lcd_ShowCircle(x+70-35, y+70, WHITE); //! OK 
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_STICK_RIGHT_RIGHT;                 
             }  
             error = 0;
             count = 0;             
          }          
          break;         
     case CALIB_STATE_STICK_RIGHT_RIGHT:
          if(count%40 == 0)      lcd_ShowCircle(x+70+35, y+70, WHITE);       //! 右右
          else if(count%40 == 20) lcd_ShowCircle(x+70+35, y+70, BACKCOLOR); 
          count++;
          if(anaIn(3)>=g_eeGeneral.joyscale.ail_max) g_eeGeneral.joyscale.ail_max = anaIn(3);
          //else error++;          
          if(count>timers) //! 采样500次，大概是5s
          {
             if(error>timers/3)
             {
                lcd_ShowCircle(x+70+35, y+70, RED);
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_ERROR;
             }
             else
             {
                lcd_ShowCircle(x+70+35, y+70, WHITE); //! OK 
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_STICK_RIGHT_CENTER;                 
             }   
             error = 0;
             count = 0;             
          }           
          break;          
     case CALIB_STATE_STICK_RIGHT_CENTER:
          if(count%40 == 0)      lcd_ShowCircle(x+70   , y+70, WHITE);       //! 右中
          else if(count%40 == 20) lcd_ShowCircle(x+70   , y+70, BACKCOLOR);
          count++; 
          if((anaIn(2)<=g_eeGeneral.joyscale.thr_max-600)&&(anaIn(2)>=g_eeGeneral.joyscale.thr_min+600)) g_eeGeneral.joyscale.thr_cen = anaIn(2);
          //else error++;          
          if((anaIn(3)<=g_eeGeneral.joyscale.ail_max-600)&&(anaIn(3)>=g_eeGeneral.joyscale.ail_min+600)) g_eeGeneral.joyscale.ail_cen = anaIn(3);
          //else error++;          
          if(count>timers) //! 采样500次，大概是5s
          {
             if(error>timers/3)
             {
                lcd_ShowCircle(x+70   , y+70, RED);
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_ERROR;
             }
             else
             {
                lcd_ShowCircle(x+70   , y+70, WHITE); //! OK
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_TRIM_LEFT_LEFT;                 
             } 
             error = 0;
             count = 0;             
          }           
          break;          
     case CALIB_STATE_TRIM_LEFT_LEFT:
          if(count%40 == 0)     lcd_ShowTriCursor(x-70-45, y+45-35,  0, WHITE);        //! 左左箭头
          else if(count%40 == 20) lcd_ShowTriCursor(x-70-45, y+45-35,  0, BACKCOLOR); 
          count++; 
          if(anaIn(4)>=g_eeGeneral.joyscale.ltrm_max) g_eeGeneral.joyscale.ltrm_max = anaIn(4); 
          //else error++;          
          if(count>timers) //! 采样500次，大概是5s
          {
             if(error>timers/3)
             {
                lcd_ShowTriCursor(x-70-45, y+45-35,  0, RED);
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_ERROR;                
             }
             else
             {
                lcd_ShowTriCursor(x-70-45, y+45-35,  0, WHITE); //! OK
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_TRIM_LEFT_RIGHT;                   
             } 
             error = 0;
             count = 0;             
          }          
          break;           
     case CALIB_STATE_TRIM_LEFT_RIGHT:
          if(count%40 == 0)     lcd_ShowTriCursor(x-70+45, y+45-35,  1, WHITE);            //! 左右箭头
          else if(count%40 == 20) lcd_ShowTriCursor(x-70+45, y+45-35,  1, BACKCOLOR); 
          count++; 
          if(anaIn(4)<=g_eeGeneral.joyscale.ltrm_min) g_eeGeneral.joyscale.ltrm_min = anaIn(4); 
          //else error++;          
          if(count>timers) //! 采样500次，大概是5s
          {
             if(error>timers/3)
             {
                lcd_ShowTriCursor(x-70+45, y+45-35,  1, RED);
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_ERROR;                
             }
             else
             {
                lcd_ShowTriCursor(x-70+45, y+45-35,  1, WHITE); //! OK
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_TRIM_LEFT_CENTER;                  
             }  
             error = 0;
             count = 0;             
          }          
          break;          
     case CALIB_STATE_TRIM_LEFT_CENTER:
          if(count%40 == 0)     lcd_ShowCircle(x-70, y+45-35, WHITE);                      //! 左中 
          else if(count%40 == 20) lcd_ShowCircle(x-70, y+45-35, BACKCOLOR);
          count++; 
          if((anaIn(4)<=g_eeGeneral.joyscale.ltrm_max-150)&&(anaIn(4)>=g_eeGeneral.joyscale.ltrm_min+150)) g_eeGeneral.joyscale.ltrm_cen = anaIn(4); 
          //else error++;          
          if(count>timers) //! 采样500次，大概是5s
          {
             if(error>timers/3)
             {
                lcd_ShowCircle(x-70, y+45-35, RED);
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_ERROR;
             }
             else
             {
                lcd_ShowCircle(x-70, y+45-35, WHITE);           //! OK
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_TRIM_RIGHT_LEFT;                  
             }  
             error = 0;
             count = 0;             
          }          
          break;            
     case CALIB_STATE_TRIM_RIGHT_LEFT:
          if(count%40 == 0)     lcd_ShowTriCursor(x+70-45, y+45-35,  0, WHITE);            //! 右左箭头
          else if(count%40 == 20) lcd_ShowTriCursor(x+70-45, y+45-35,  0, BACKCOLOR);
          count++; 
          if(anaIn(5)>=g_eeGeneral.joyscale.rtrm_max) g_eeGeneral.joyscale.rtrm_max = anaIn(5);  
          //else error++;          
          if(count>timers) //! 采样500次，大概是5s
          {
             if(error>timers/3)
             {
                lcd_ShowTriCursor(x+70-45, y+45-35,  0, RED);
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_ERROR;
             }
             else
             {
                lcd_ShowTriCursor(x+70-45, y+45-35,  0, WHITE); //! OK
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_TRIM_RIGHT_RIGHT;                 
             }
             error = 0;
             count = 0;             
          }          
          break;           
     case CALIB_STATE_TRIM_RIGHT_RIGHT:
          if(count%40 == 0)     lcd_ShowTriCursor(x+70+45, y+45-35,  1, WHITE);            //! 右右箭头
          else if(count%40 == 20) lcd_ShowTriCursor(x+70+45, y+45-35,  1, BACKCOLOR);
          count++; 
          if(anaIn(5)<=g_eeGeneral.joyscale.rtrm_min) g_eeGeneral.joyscale.rtrm_min = anaIn(5);
          //else error++;          
          if(count>timers) //! 采样500次，大概是5s
          {
             if(error>timers/3)
             {
                lcd_ShowTriCursor(x+70+45, y+45-35,  1, RED); 
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_ERROR;
             }
             else
             {
                lcd_ShowTriCursor(x+70+45, y+45-35,  1, WHITE); //! OK
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_TRIM_RIGHT_CENTER;                
             } 
             error = 0;
             count = 0;             
          }           
          break;           
     case CALIB_STATE_TRIM_RIGHT_CENTER:
          if(count%40 == 0)     lcd_ShowCircle(x+70, y+45-35, WHITE);                   //! 右中 
          else if(count%40 == 20) lcd_ShowCircle(x+70, y+45-35, BACKCOLOR);          
          count++;
          if((anaIn(5)<=g_eeGeneral.joyscale.rtrm_max-150)&&(anaIn(5)>=g_eeGeneral.joyscale.rtrm_min+150)) g_eeGeneral.joyscale.rtrm_cen = anaIn(5);   
          //else error++;          
          if(count>timers) //! 采样500次，大概是5s
          {         
             if(error>timers/3)
             {
                lcd_ShowCircle(x+70, y+45-35, RED); 
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_ERROR;
             }
             else
             {
                lcd_ShowCircle(x+70, y+45-35, WHITE);        //! OK
                g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_WRITE; 
             } 
             error = 0;
             count = 0;              
          }           
          break;         
     case CALIB_STATE_ERROR:
          g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = SUB_STATE_NONE; //! 临时赋空值
          break;          
     case CALIB_STATE_WRITE: 
          eeWriteJoyScale();
          g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_READ; 
          break;          
     case CALIB_STATE_READ:     
          eeReadJoyScale();
          g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_FINISH;
          break;                    
     case CALIB_STATE_FINISH:       
          lcd_DrawCicle(x-70, y+70, 45, GREEN);
          lcd_DrawCicle(x+70, y+70, 45, GREEN);
          lcd_ShowCircle(x-70, y+70-35, GREEN);             //! 左上 
          lcd_ShowCircle(x-70, y+70+35, GREEN);             //! 左下   
          lcd_ShowCircle(x-70-35, y+70, GREEN);             //! 左左
          lcd_ShowCircle(x-70+35, y+70, GREEN);             //! 左右
          lcd_ShowCircle(x-70   , y+70, GREEN);             //! 左中 
          lcd_ShowCircle(x+70, y+70-35, GREEN);             //! 右上
          lcd_ShowCircle(x+70, y+70+35, GREEN);             //! 右下
          lcd_ShowCircle(x+70-35, y+70, GREEN);             //! 右左
          lcd_ShowCircle(x+70+35, y+70, GREEN);             //! 右右
          lcd_ShowCircle(x+70   , y+70, GREEN);             //! 右中
          lcd_ShowTriCursor(x-70-45, y+45-35,  0, GREEN);   //! 左左箭头
          lcd_ShowTriCursor(x-70+45, y+45-35,  1, GREEN);   //! 左右箭头
          lcd_ShowCircle(x-70, y+45-35, GREEN);             //! 左中
          lcd_ShowTriCursor(x+70-45, y+45-35,  0, GREEN);   //! 右左箭头
          lcd_ShowTriCursor(x+70+45, y+45-35,  1, GREEN);   //! 右右箭头
          lcd_ShowCircle(x+70, y+45-35, GREEN);             //! 右中              
          g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = CALIB_STATE_EXIT;
          break;         
     case CALIB_STATE_EXIT:
          count = 0;
          error = 0; 
          g_eeGeneral.menus[MENU_CALIBJOYSTICK].subState = SUB_STATE_NONE;          
          break; 
     case SUB_STATE_NONE:
          break;          
    }


   
   
   
   
/*    static uint8_t painted = 0; 
   const uint16_t x = 240;        //! 区域坐标：(240-140, 200)(240+140, 200)
   const uint16_t y = 200;        //! 区域坐标：(240-140, 320)(240+140, 320)
   const uint16_t timers = 499;   //! 采样摇杆次数

   static uint16_t count = 0;
   static uint16_t error = 0;   
   
   if(event == MENU_CALIBJOYSTICK) 
   {            
       if(painted == 0)
       {
          painted = 1;
          lcd_DrawFillRectangle(101, 164, 379, 318, BACKCOLOR); //清屏
          uint8_t logo[]= "CALIBRATION";          
	      lcd_ShowString(x-66, y-35, RED, 24, logo); 
          
          lcd_DrawCicle(x-70, y+70, 45, LINEBLACK);
          lcd_DrawCicle(x+70, y+70, 45, LINEBLACK);
          
          lcd_ShowCircle(x-70, y+70-35, LINEBLACK); 
          lcd_ShowCircle(x-70, y+70+35, LINEBLACK);
          lcd_ShowCircle(x-70-35, y+70, LINEBLACK);   
          lcd_ShowCircle(x-70+35, y+70, LINEBLACK); 
          lcd_ShowCircle(x-70   , y+70, LINEBLACK); 
          lcd_ShowCircle(x+70, y+70-35, LINEBLACK);
          lcd_ShowCircle(x+70, y+70+35, LINEBLACK); 
          lcd_ShowCircle(x+70-35, y+70, LINEBLACK);  
          lcd_ShowCircle(x+70+35, y+70, LINEBLACK);
          lcd_ShowCircle(x+70   , y+70, LINEBLACK);
          
          lcd_ShowTriCursor(x-70-45, y+45-35,  0, LINEBLACK);  
          lcd_ShowTriCursor(x-70+45, y+45-35,  1, LINEBLACK);
          lcd_ShowCircle(x-70, y+45-35, LINEBLACK);
          lcd_ShowTriCursor(x+70-45, y+45-35,  0, LINEBLACK); 
          lcd_ShowTriCursor(x+70+45, y+45-35,  1, LINEBLACK); 
          lcd_ShowCircle(x+70, y+45-35, LINEBLACK); 
       }
       
       switch(g_eeGeneral.calibState)
       {
         case CALIB_STATE_NONE:
              break;
              
         case CALIB_STATE_START: 
              g_eeGeneral.calibState = CALIB_STATE_THRESHOLD;      
              break;
              
         case CALIB_STATE_THRESHOLD:
              g_eeGeneral.joyscale.threshold = 25;
              g_eeGeneral.calibState = CALIB_STATE_STICK_LEFT_UP;           
              break;
              
         case CALIB_STATE_STICK_LEFT_UP:
              if(count%40 == 0)      lcd_ShowCircle(x-70, y+70-35, WHITE);          //! 左上
              else if(count%40 == 20) lcd_ShowCircle(x-70, y+70-35, BACKCOLOR); 
              count++; //!          
              if(anaIn(1)<=g_eeGeneral.joyscale.ele_min) g_eeGeneral.joyscale.ele_min = anaIn(1); 
              //else error++;         
              if(count>timers) //! 采样500次，大概是5s
              {
                 if(error>timers/3) //! calib failed!
                 {
                    lcd_ShowCircle(x-70, y+70-35, RED);
                    g_eeGeneral.calibState = CALIB_STATE_ERROR;                                
                 }
                 else
                 {
                    lcd_ShowCircle(x-70, y+70-35, WHITE); //! OK
                    g_eeGeneral.calibState = CALIB_STATE_STICK_LEFT_DOWN;                     
                 }
                 count = 0;
                 error = 0;          
              }
              break;
              
         case CALIB_STATE_STICK_LEFT_DOWN:
              if(count%40 == 0)      lcd_ShowCircle(x-70, y+70+35, WHITE);          //! 左下
              else if(count%40 == 20) lcd_ShowCircle(x-70, y+70+35, BACKCOLOR);
              count++;
              if(anaIn(1)>=g_eeGeneral.joyscale.ele_max) g_eeGeneral.joyscale.ele_max = anaIn(1);
              //else error++;           
              if(count>timers) //! 采样500次，大概是5s
              {
                 if(error>timers/3) //! calib failed!
                 {
                    lcd_ShowCircle(x-70, y+70+35, RED);
                    g_eeGeneral.calibState = CALIB_STATE_ERROR;                
                 }
                 else
                 {
                    lcd_ShowCircle(x-70, y+70+35, WHITE); //! OK
                    g_eeGeneral.calibState = CALIB_STATE_STICK_LEFT_LEFT;                 
                 }
                 error = 0;
                 count = 0;              
              }          
              break;
              
         case CALIB_STATE_STICK_LEFT_LEFT:
              if(count%40 == 0)      lcd_ShowCircle(x-70-35, y+70, WHITE);          //! 左左
              else if(count%40 == 20) lcd_ShowCircle(x-70-35, y+70, BACKCOLOR); 
              count++;
              if(anaIn(0)>=g_eeGeneral.joyscale.rud_max) g_eeGeneral.joyscale.rud_max = anaIn(0);  
              //else error++;          
              if(count>timers) //! 采样500次，大概是5s
              {
                 if(error>timers/3) //! calib failed!
                 {
                    lcd_ShowCircle(x-70-35, y+70, RED);
                    g_eeGeneral.calibState = CALIB_STATE_ERROR;
                 }
                 else
                 {
                    lcd_ShowCircle(x-70-35, y+70, WHITE); //! OK   
                    g_eeGeneral.calibState = CALIB_STATE_STICK_LEFT_RIGHT;               
                 } 
                 error = 0;
                 count = 0;             
              }           
              break;
              
         case CALIB_STATE_STICK_LEFT_RIGHT:
              if(count%40 == 0)      lcd_ShowCircle(x-70+35, y+70, WHITE);          //! 左右
              else if(count%40 == 20) lcd_ShowCircle(x-70+35, y+70, BACKCOLOR); 
              count++;
              if(anaIn(0)<=g_eeGeneral.joyscale.rud_min) g_eeGeneral.joyscale.rud_min = anaIn(0);  
              //else error++;          
              if(count>timers) //! 采样500次，大概是5s
              {
                 if(error>timers/3)
                 {
                    lcd_ShowCircle(x-70+35, y+70, RED);
                    g_eeGeneral.calibState = CALIB_STATE_ERROR;
                 }
                 else
                 {
                    lcd_ShowCircle(x-70+35, y+70, WHITE); //! OK  
                    g_eeGeneral.calibState = CALIB_STATE_STICK_LEFT_CENTER;                  
                 } 
                 error = 0;
                 count = 0;            
              }          
              break;
              
         case CALIB_STATE_STICK_LEFT_CENTER: 
              if(count%40 == 0)      lcd_ShowCircle(x-70   , y+70, WHITE);       //! 左中 
              else if(count%40 == 20) lcd_ShowCircle(x-70   , y+70, BACKCOLOR); 
              count++;
              if((anaIn(1)<=g_eeGeneral.joyscale.ele_max-600)&&(anaIn(1)>=g_eeGeneral.joyscale.ele_min+600)) g_eeGeneral.joyscale.ele_cen = anaIn(1); 
              else error++;
              if((anaIn(0)<=g_eeGeneral.joyscale.rud_max-600)&&(anaIn(0)>=g_eeGeneral.joyscale.rud_min+600)) g_eeGeneral.joyscale.rud_cen = anaIn(0); 
              //else error++;          
              if(count>timers) //! 采样500次，大概是5s
              {
                 if(error>timers/3)
                 {
                    lcd_ShowCircle(x-70   , y+70, RED);
                    g_eeGeneral.calibState = CALIB_STATE_ERROR;
                 }
                 else
                 {
                    lcd_ShowCircle(x-70, y+70, WHITE);    //! OK
                    g_eeGeneral.calibState = CALIB_STATE_STICK_RIGHT_UP;                 
                 }  
                 error = 0;
                 count = 0;             
              }          
              break;
              
         case CALIB_STATE_STICK_RIGHT_UP: 
              if(count%40 == 0)      lcd_ShowCircle(x+70, y+70-35, WHITE);       //! 右上
              else if(count%40 == 20) lcd_ShowCircle(x+70, y+70-35, BACKCOLOR);
              count++;    
              if(anaIn(2)>=g_eeGeneral.joyscale.thr_max) g_eeGeneral.joyscale.thr_max = anaIn(2); 
              //else error++;          
              if(count>timers) //! 采样500次，大概是5s
              {
                 if(error>timers/3)
                 {
                    lcd_ShowCircle(x+70, y+70-35, RED);
                    g_eeGeneral.calibState = CALIB_STATE_ERROR;
                 }
                 else
                 {
                    lcd_ShowCircle(x+70, y+70-35, WHITE); //! OK 
                    g_eeGeneral.calibState = CALIB_STATE_STICK_RIGHT_DOWN;                  
                 } 
                 error = 0;
                 count = 0;             
              }          
              break;
              
         case CALIB_STATE_STICK_RIGHT_DOWN:     
              if(count%40 == 0)      lcd_ShowCircle(x+70, y+70+35, WHITE);          //! 右下
              else if(count%40 == 20) lcd_ShowCircle(x+70, y+70+35, BACKCOLOR); 
              count++; 
              if(anaIn(2)<=g_eeGeneral.joyscale.thr_min) g_eeGeneral.joyscale.thr_min = anaIn(2);
              //else error++;          
              if(count>timers) //! 采样500次，大概是5s
              {
                 if(error>timers/3)
                 {
                    lcd_ShowCircle(x+70, y+70+35, RED);
                    g_eeGeneral.calibState = CALIB_STATE_ERROR;
                 }
                 else
                 {
                    lcd_ShowCircle(x+70, y+70+35, WHITE); //! OK 
                    g_eeGeneral.calibState = CALIB_STATE_STICK_RIGHT_LEFT;                  
                 }
                 error = 0;    
                 count = 0;             
              }          
              break;
              
         case CALIB_STATE_STICK_RIGHT_LEFT:     
              if(count%40 == 0)      lcd_ShowCircle(x+70-35, y+70, WHITE);       //! 右左
              else if(count%40 == 20) lcd_ShowCircle(x+70-35, y+70, BACKCOLOR);
              count++;
              if(anaIn(3)<=g_eeGeneral.joyscale.ail_min) g_eeGeneral.joyscale.ail_min = anaIn(3); 
              //else error++;          
              if(count>timers) //! 采样500次，大概是5s
              {
                 if(error>timers/3)
                 {
                    lcd_ShowCircle(x+70-35, y+70, RED);
                    g_eeGeneral.calibState = CALIB_STATE_ERROR;
                 }
                 else
                 {
                    lcd_ShowCircle(x+70-35, y+70, WHITE); //! OK 
                    g_eeGeneral.calibState = CALIB_STATE_STICK_RIGHT_RIGHT;                 
                 }  
                 error = 0;
                 count = 0;             
              }          
              break;
              
         case CALIB_STATE_STICK_RIGHT_RIGHT:
              if(count%40 == 0)      lcd_ShowCircle(x+70+35, y+70, WHITE);       //! 右右
              else if(count%40 == 20) lcd_ShowCircle(x+70+35, y+70, BACKCOLOR); 
              count++;
              if(anaIn(3)>=g_eeGeneral.joyscale.ail_max) g_eeGeneral.joyscale.ail_max = anaIn(3);
              //else error++;          
              if(count>timers) //! 采样500次，大概是5s
              {
                 if(error>timers/3)
                 {
                    lcd_ShowCircle(x+70+35, y+70, RED);
                    g_eeGeneral.calibState = CALIB_STATE_ERROR;
                 }
                 else
                 {
                    lcd_ShowCircle(x+70+35, y+70, WHITE); //! OK 
                    g_eeGeneral.calibState = CALIB_STATE_STICK_RIGHT_CENTER;                 
                 }   
                 error = 0;
                 count = 0;             
              }           
              break;
              
         case CALIB_STATE_STICK_RIGHT_CENTER:
              if(count%40 == 0)      lcd_ShowCircle(x+70   , y+70, WHITE);       //! 右中
              else if(count%40 == 20) lcd_ShowCircle(x+70   , y+70, BACKCOLOR);
              count++; 
              if((anaIn(2)<=g_eeGeneral.joyscale.thr_max-600)&&(anaIn(2)>=g_eeGeneral.joyscale.thr_min+600)) g_eeGeneral.joyscale.thr_cen = anaIn(2);
              //else error++;          
              if((anaIn(3)<=g_eeGeneral.joyscale.ail_max-600)&&(anaIn(3)>=g_eeGeneral.joyscale.ail_min+600)) g_eeGeneral.joyscale.ail_cen = anaIn(3);
              //else error++;          
              if(count>timers) //! 采样500次，大概是5s
              {
                 if(error>timers/3)
                 {
                    lcd_ShowCircle(x+70   , y+70, RED);
                    g_eeGeneral.calibState = CALIB_STATE_ERROR;
                 }
                 else
                 {
                    lcd_ShowCircle(x+70   , y+70, WHITE); //! OK
                    g_eeGeneral.calibState = CALIB_STATE_TRIM_LEFT_LEFT;                 
                 } 
                 error = 0;
                 count = 0;             
              }           
              break;
              
         case CALIB_STATE_TRIM_LEFT_LEFT:
              if(count%40 == 0)     lcd_ShowTriCursor(x-70-45, y+45-35,  0, WHITE);        //! 左左箭头
              else if(count%40 == 20) lcd_ShowTriCursor(x-70-45, y+45-35,  0, BACKCOLOR); 
              count++; 
              if(anaIn(4)>=g_eeGeneral.joyscale.ltrm_max) g_eeGeneral.joyscale.ltrm_max = anaIn(4); 
              //else error++;          
              if(count>timers) //! 采样500次，大概是5s
              {
                 if(error>timers/3)
                 {
                    lcd_ShowTriCursor(x-70-45, y+45-35,  0, RED);
                    g_eeGeneral.calibState = CALIB_STATE_ERROR;                
                 }
                 else
                 {
                    lcd_ShowTriCursor(x-70-45, y+45-35,  0, WHITE); //! OK
                    g_eeGeneral.calibState = CALIB_STATE_TRIM_LEFT_RIGHT;                   
                 } 
                 error = 0;
                 count = 0;             
              }          
              break; 
              
         case CALIB_STATE_TRIM_LEFT_RIGHT:
              if(count%40 == 0)     lcd_ShowTriCursor(x-70+45, y+45-35,  1, WHITE);            //! 左右箭头
              else if(count%40 == 20) lcd_ShowTriCursor(x-70+45, y+45-35,  1, BACKCOLOR); 
              count++; 
              if(anaIn(4)<=g_eeGeneral.joyscale.ltrm_min) g_eeGeneral.joyscale.ltrm_min = anaIn(4); 
              //else error++;          
              if(count>timers) //! 采样500次，大概是5s
              {
                 if(error>timers/3)
                 {
                    lcd_ShowTriCursor(x-70+45, y+45-35,  1, RED);
                    g_eeGeneral.calibState = CALIB_STATE_ERROR;                
                 }
                 else
                 {
                    lcd_ShowTriCursor(x-70+45, y+45-35,  1, WHITE); //! OK
                    g_eeGeneral.calibState = CALIB_STATE_TRIM_LEFT_CENTER;                  
                 }  
                 error = 0;
                 count = 0;             
              }          
              break;
              
         case CALIB_STATE_TRIM_LEFT_CENTER:
              if(count%40 == 0)     lcd_ShowCircle(x-70, y+45-35, WHITE);                      //! 左中 
              else if(count%40 == 20) lcd_ShowCircle(x-70, y+45-35, BACKCOLOR);
              count++; 
              if((anaIn(4)<=g_eeGeneral.joyscale.ltrm_max-150)&&(anaIn(4)>=g_eeGeneral.joyscale.ltrm_min+150)) g_eeGeneral.joyscale.ltrm_cen = anaIn(4); 
              //else error++;          
              if(count>timers) //! 采样500次，大概是5s
              {
                 if(error>timers/3)
                 {
                    lcd_ShowCircle(x-70, y+45-35, RED);
                    g_eeGeneral.calibState = CALIB_STATE_ERROR;
                 }
                 else
                 {
                    lcd_ShowCircle(x-70, y+45-35, WHITE);           //! OK
                    g_eeGeneral.calibState = CALIB_STATE_TRIM_RIGHT_LEFT;                  
                 }  
                 error = 0;
                 count = 0;             
              }          
              break;  
              
         case CALIB_STATE_TRIM_RIGHT_LEFT:
              if(count%40 == 0)     lcd_ShowTriCursor(x+70-45, y+45-35,  0, WHITE);            //! 右左箭头
              else if(count%40 == 20) lcd_ShowTriCursor(x+70-45, y+45-35,  0, BACKCOLOR);
              count++; 
              if(anaIn(5)>=g_eeGeneral.joyscale.rtrm_max) g_eeGeneral.joyscale.rtrm_max = anaIn(5);  
              //else error++;          
              if(count>timers) //! 采样500次，大概是5s
              {
                 if(error>timers/3)
                 {
                    lcd_ShowTriCursor(x+70-45, y+45-35,  0, RED);
                    g_eeGeneral.calibState = CALIB_STATE_ERROR;
                 }
                 else
                 {
                    lcd_ShowTriCursor(x+70-45, y+45-35,  0, WHITE); //! OK
                    g_eeGeneral.calibState = CALIB_STATE_TRIM_RIGHT_RIGHT;                 
                 }
                 error = 0;
                 count = 0;             
              }          
              break; 
              
         case CALIB_STATE_TRIM_RIGHT_RIGHT:
              if(count%40 == 0)     lcd_ShowTriCursor(x+70+45, y+45-35,  1, WHITE);            //! 右右箭头
              else if(count%40 == 20) lcd_ShowTriCursor(x+70+45, y+45-35,  1, BACKCOLOR);
              count++; 
              if(anaIn(5)<=g_eeGeneral.joyscale.rtrm_min) g_eeGeneral.joyscale.rtrm_min = anaIn(5);
              //else error++;          
              if(count>timers) //! 采样500次，大概是5s
              {
                 if(error>timers/3)
                 {
                    lcd_ShowTriCursor(x+70+45, y+45-35,  1, RED); 
                    g_eeGeneral.calibState = CALIB_STATE_ERROR;
                 }
                 else
                 {
                    lcd_ShowTriCursor(x+70+45, y+45-35,  1, WHITE); //! OK
                    g_eeGeneral.calibState = CALIB_STATE_TRIM_RIGHT_CENTER;                
                 } 
                 error = 0;
                 count = 0;             
              }           
              break; 
              
         case CALIB_STATE_TRIM_RIGHT_CENTER:
              if(count%40 == 0)     lcd_ShowCircle(x+70, y+45-35, WHITE);                   //! 右中 
              else if(count%40 == 20) lcd_ShowCircle(x+70, y+45-35, BACKCOLOR);          
              count++;
              if((anaIn(5)<=g_eeGeneral.joyscale.rtrm_max-150)&&(anaIn(5)>=g_eeGeneral.joyscale.rtrm_min+150)) g_eeGeneral.joyscale.rtrm_cen = anaIn(5);   
              //else error++;          
              if(count>timers) //! 采样500次，大概是5s
              {         
                 if(error>timers/3)
                 {
                    lcd_ShowCircle(x+70, y+45-35, RED); 
                    g_eeGeneral.calibState = CALIB_STATE_ERROR;
                 }
                 else
                 {
                    lcd_ShowCircle(x+70, y+45-35, WHITE);        //! OK
                    g_eeGeneral.calibState = CALIB_STATE_WRITE; 
                 } 
                 error = 0;
                 count = 0;              
              }           
              break; 
              
         case CALIB_STATE_ERROR:
              g_eeGeneral.calibState = CALIB_STATE_NONE; //! 临时赋空值
              break; 
              
         case CALIB_STATE_WRITE: 
              eeWriteJoyScale();
              g_eeGeneral.calibState = CALIB_STATE_READ; 
              break;
              
         case CALIB_STATE_READ:     
              eeReadJoyScale();
              g_eeGeneral.calibState = CALIB_STATE_FINISH;
              break;          
              
         case CALIB_STATE_FINISH:       
              lcd_DrawCicle(x-70, y+70, 45, GREEN);
              lcd_DrawCicle(x+70, y+70, 45, GREEN);
              lcd_ShowCircle(x-70, y+70-35, GREEN);             //! 左上 
              lcd_ShowCircle(x-70, y+70+35, GREEN);             //! 左下   
              lcd_ShowCircle(x-70-35, y+70, GREEN);             //! 左左
              lcd_ShowCircle(x-70+35, y+70, GREEN);             //! 左右
              lcd_ShowCircle(x-70   , y+70, GREEN);             //! 左中 
              lcd_ShowCircle(x+70, y+70-35, GREEN);             //! 右上
              lcd_ShowCircle(x+70, y+70+35, GREEN);             //! 右下
              lcd_ShowCircle(x+70-35, y+70, GREEN);             //! 右左
              lcd_ShowCircle(x+70+35, y+70, GREEN);             //! 右右
              lcd_ShowCircle(x+70   , y+70, GREEN);             //! 右中
              lcd_ShowTriCursor(x-70-45, y+45-35,  0, GREEN);   //! 左左箭头
              lcd_ShowTriCursor(x-70+45, y+45-35,  1, GREEN);   //! 左右箭头
              lcd_ShowCircle(x-70, y+45-35, GREEN);             //! 左中
              lcd_ShowTriCursor(x+70-45, y+45-35,  0, GREEN);   //! 右左箭头
              lcd_ShowTriCursor(x+70+45, y+45-35,  1, GREEN);   //! 右右箭头
              lcd_ShowCircle(x+70, y+45-35, GREEN);             //! 右中              
              g_eeGeneral.calibState = CALIB_STATE_NONE;
              break;
              
         case CALIB_STATE_EXIT:
              break;         
       }
   }
   else
   {
       count = 0;
       error = 0;
       painted = 0;       
       g_eeGeneral.calibState = CALIB_STATE_NONE;
   } */
}





/************************************************
menu  name: menu_rcChannels
      func: 显示14个输入或输出通道的值 
************************************************/
void menu_rcChannels(uint8_t state)
{   
   static uint8_t painted = 0;  
   RC_CHANNEL channels = g_rcChannel;
   const uint16_t x = 240; //! 中心点坐标
   const uint16_t y = 200;	
   static RC_CHANNEL channelsLast = {1100, 1100, 1100, 1100, 1100, 1100, 1100, 1100, 1100, 1100, 1100, 1100, 1100, 1100};

   uint8_t logo[]= "CHANNELS";
   
   switch(state)
   {
       case SUB_STATE_INIT:
            menuClear(BACKCOLOR); //! 清屏 
            showTitle(x-(sizeof(logo)-1)*6, y-35, WHITE, logo, TITLECOLOR); 
            memset(&channelsLast, 1000, sizeof(channelsLast));   //!            
            g_eeGeneral.menus[MENU_RCCHANNELS].subState = SUB_STATE_START;
            break;
       case SUB_STATE_START:
            g_eeGeneral.menus[MENU_RCCHANNELS].subState = SUB_STATE_NONE;
            break;
       case RCCHANNELS_STATE_EXIT:
            break;
       case SUB_STATE_NONE:
            if(channels.chan1!=channelsLast.chan1)
            {
              int8_t chan1 = (channels.chan1-1500)/25*2;         //! sacle: -40 -- +40
              int8_t chan1Last = (channelsLast.chan1-1500)/25*2; //! sacle: -40 -- +40
              lcd_DrawFillRectangle(x-130-2, y+70-2-chan1Last, x-130+2, y+70+2-chan1Last, BACKCOLOR); 
              lcd_DrawFillRectangle(x-130-2, y+70-2-chan1, x-130+2, y+70+2-chan1, LIGHTWHITE);
              lcd_DrawLine(x-130,y+70-40, x-130,y+70+40, LIGHTWHITE);          
              channelsLast.chan1 = channels.chan1;     

              lcd_DrawLine(x-130, y+70-40, x+130, y+70-40, LIGHTWHITE); //! 中轴线 
              lcd_DrawLine(x-130,    y+70, x+130,    y+70, LIGHTWHITE); //! 中轴线       
              lcd_DrawLine(x-130, y+70+40, x+130, y+70+40, LIGHTWHITE); //! 中轴线            
            }

            if(channels.chan2!=channelsLast.chan2)
            {
              int8_t chan2 = (channels.chan2-1500)/25*2; //! sacle: -40 -- +40
              int8_t chan2Last = (channelsLast.chan2-1500)/25*2; //! sacle: -40 -- +40
              lcd_DrawFillRectangle(x-110-2, y+70-2-chan2Last, x-110+2, y+70+2-chan2Last, BACKCOLOR); 
              lcd_DrawFillRectangle(x-110-2, y+70-2-chan2, x-110+2, y+70+2-chan2, LIGHTWHITE);	  
              channelsLast.chan2 = channels.chan2;
              lcd_DrawLine(x-110,y+70-40, x-110,y+70+40, LIGHTWHITE);

              lcd_DrawLine(x-130, y+70-40, x+130, y+70-40, LIGHTWHITE); //! 中轴线 
              lcd_DrawLine(x-130,    y+70, x+130,    y+70, LIGHTWHITE); //! 中轴线       
              lcd_DrawLine(x-130, y+70+40, x+130, y+70+40, LIGHTWHITE); //! 中轴线            
            }

            if(channels.chan3!=channelsLast.chan3)
            {
              int8_t chan3 = (channels.chan3-1500)/25*2; //! sacle: -40 -- +40
              int8_t chan3Last = (channelsLast.chan3-1500)/25*2; //! sacle: -40 -- +40
              lcd_DrawFillRectangle(x-90-2, y+70-2-chan3Last, x-90+2, y+70+2-chan3Last, BACKCOLOR); 
              lcd_DrawFillRectangle(x-90-2, y+70-2-chan3, x-90+2, y+70+2-chan3, LIGHTWHITE);	  
              channelsLast.chan3 = channels.chan3;
              lcd_DrawLine(x-90, y+70-40, x-90, y+70+40, LIGHTWHITE);

              lcd_DrawLine(x-130, y+70-40, x+130, y+70-40, LIGHTWHITE); //! 中轴线 
              lcd_DrawLine(x-130,    y+70, x+130,    y+70, LIGHTWHITE); //! 中轴线       
              lcd_DrawLine(x-130, y+70+40, x+130, y+70+40, LIGHTWHITE); //! 中轴线            
            }

            if(channels.chan4!=channelsLast.chan4)
            {
              int8_t chan4 = (channels.chan4-1500)/25*2; //! sacle: -40 -- +40
              int8_t chan4Last = (channelsLast.chan4-1500)/25*2; //! sacle: -40 -- +40           
              lcd_DrawFillRectangle(x-70-2, y+70-2-chan4Last, x-70+2, y+70+2-chan4Last, BACKCOLOR); 
              lcd_DrawFillRectangle(x-70-2, y+70-2-chan4, x-70+2, y+70+2-chan4, LIGHTWHITE);	  
              channelsLast.chan4 = channels.chan4;
              lcd_DrawLine(x-70, y+70-40, x-70, y+70+40, LIGHTWHITE);

              lcd_DrawLine(x-130, y+70-40, x+130, y+70-40, LIGHTWHITE); //! 中轴线 
              lcd_DrawLine(x-130,    y+70, x+130,    y+70, LIGHTWHITE); //! 中轴线       
              lcd_DrawLine(x-130, y+70+40, x+130, y+70+40, LIGHTWHITE); //! 中轴线            
            }

            if(channels.chan5!=channelsLast.chan5)
            {
              int8_t chan5 = (channels.chan5-1500)/25*2; //! sacle: -40 -- +40
              int8_t chan5Last = (channelsLast.chan5-1500)/25*2; //! sacle: -40 -- +40
              lcd_DrawFillRectangle(x-50-2, y+70-2-chan5Last, x-50+2, y+70+2-chan5Last, BACKCOLOR);   
              lcd_DrawFillRectangle(x-50-2, y+70-2-chan5, x-50+2, y+70+2-chan5, LIGHTWHITE);	  
              channelsLast.chan5 = channels.chan5;
              lcd_DrawLine(x-50, y+70-40, x-50, y+70+40, LIGHTWHITE);

              lcd_DrawLine(x-130, y+70-40, x+130, y+70-40, LIGHTWHITE); //! 中轴线 
              lcd_DrawLine(x-130,    y+70, x+130,    y+70, LIGHTWHITE); //! 中轴线       
              lcd_DrawLine(x-130, y+70+40, x+130, y+70+40, LIGHTWHITE); //! 中轴线            
            }

            if(channels.chan6!=channelsLast.chan6)
            {
              int8_t chan6 = (channels.chan6-1500)/25*2; //! sacle: -40 -- +40
              int8_t chan6Last = (channelsLast.chan6-1500)/25*2; //! sacle: -40 -- +40           
              lcd_DrawFillRectangle(x-30-2, y+70-2-chan6Last, x-30+2, y+70+2-chan6Last, BACKCOLOR); 
              lcd_DrawFillRectangle(x-30-2, y+70-2-chan6, x-30+2, y+70+2-chan6, LIGHTWHITE);	  
              channelsLast.chan6 = channels.chan6;
              lcd_DrawLine(x-30, y+70-40, x-30, y+70+40, LIGHTWHITE); 

              lcd_DrawLine(x-130, y+70-40, x+130, y+70-40, LIGHTWHITE); //! 中轴线 
              lcd_DrawLine(x-130,    y+70, x+130,    y+70, LIGHTWHITE); //! 中轴线       
              lcd_DrawLine(x-130, y+70+40, x+130, y+70+40, LIGHTWHITE); //! 中轴线            
            }

            if(channels.chan7!=channelsLast.chan7)
            {
              int8_t chan7 = (channels.chan7-1500)/25*2; //! sacle: -40 -- +40
              int8_t chan7Last = (channelsLast.chan7-1500)/25*2; //! sacle: -40 -- +40           
              lcd_DrawFillRectangle(x-10-2, y+70-2-chan7Last, x-10+2, y+70+2-chan7Last, BACKCOLOR);
              lcd_DrawFillRectangle(x-10-2, y+70-2-chan7, x-10+2, y+70+2-chan7, LIGHTWHITE);	  
              channelsLast.chan7 = channels.chan7;
              lcd_DrawLine(x-10, y+70-40, x-10, y+70+40, LIGHTWHITE);

              lcd_DrawLine(x-130, y+70-40, x+130, y+70-40, LIGHTWHITE); //! 中轴线 
              lcd_DrawLine(x-130,    y+70, x+130,    y+70, LIGHTWHITE); //! 中轴线       
              lcd_DrawLine(x-130, y+70+40, x+130, y+70+40, LIGHTWHITE); //! 中轴线            
            }

            if(channels.chan8!=channelsLast.chan8)
            {
              int8_t chan8 = (channels.chan8-1500)/25*2; //! sacle: -40 -- +40
              int8_t chan8Last = (channelsLast.chan8-1500)/25*2; //! sacle: -40 -- +40           
              lcd_DrawFillRectangle(x+10-2, y+70-2-chan8Last, x+10+2, y+70+2-chan8Last, BACKCOLOR); 
              lcd_DrawFillRectangle(x+10-2, y+70-2-chan8, x+10+2, y+70+2-chan8, LIGHTWHITE);	  
              channelsLast.chan8 = channels.chan8;
              lcd_DrawLine(x+10, y+70-40, x+10, y+70+40, LIGHTWHITE);

              lcd_DrawLine(x-130, y+70-40, x+130, y+70-40, LIGHTWHITE); //! 中轴线 
              lcd_DrawLine(x-130,    y+70, x+130,    y+70, LIGHTWHITE); //! 中轴线       
              lcd_DrawLine(x-130, y+70+40, x+130, y+70+40, LIGHTWHITE); //! 中轴线            
            }   

            if(channels.chan9!=channelsLast.chan9)
            {
              int8_t chan9 = (channels.chan9-1500)/25*2; //! sacle: -40 -- +40
              int8_t chan9Last = (channelsLast.chan9-1500)/25*2; //! sacle: -40 -- +40           
              lcd_DrawFillRectangle(x+30-2, y+70-2-chan9Last, x+30+2, y+70+2-chan9Last, BACKCOLOR); 
              lcd_DrawFillRectangle(x+30-2, y+70-2-chan9, x+30+2, y+70+2-chan9, LIGHTWHITE);	  
              channelsLast.chan9 = channels.chan9;
              lcd_DrawLine(x+30, y+70-40, x+30, y+70+40, LIGHTWHITE);

              lcd_DrawLine(x-130, y+70-40, x+130, y+70-40, LIGHTWHITE); //! 中轴线 
              lcd_DrawLine(x-130,    y+70, x+130,    y+70, LIGHTWHITE); //! 中轴线       
              lcd_DrawLine(x-130, y+70+40, x+130, y+70+40, LIGHTWHITE); //! 中轴线            
            }

            if(channels.chan10!=channelsLast.chan10)
            {
              int8_t chan10 = (channels.chan10-1500)/25*2; //! sacle: -40 -- +40
              int8_t chan10Last = (channelsLast.chan10-1500)/25*2; //! sacle: -40 -- +40           
              lcd_DrawFillRectangle(x+50-2, y+70-2-chan10Last, x+50+2, y+70+2-chan10Last, BACKCOLOR); 
              lcd_DrawFillRectangle(x+50-2, y+70-2-chan10, x+50+2, y+70+2-chan10, LIGHTWHITE);	  
              channelsLast.chan10 = channels.chan10;
              lcd_DrawLine(x+50, y+70-40, x+50, y+70+40, LIGHTWHITE);

              lcd_DrawLine(x-130, y+70-40, x+130, y+70-40, LIGHTWHITE); //! 中轴线 
              lcd_DrawLine(x-130,    y+70, x+130,    y+70, LIGHTWHITE); //! 中轴线       
              lcd_DrawLine(x-130, y+70+40, x+130, y+70+40, LIGHTWHITE); //! 中轴线            
            }

            if(channels.chan11!=channelsLast.chan11)
            {
              int8_t chan11 = (channels.chan11-1500)/25*2; //! sacle: -40 -- +40
              int8_t chan11Last = (channelsLast.chan11-1500)/25*2; //! sacle: -40 -- +40           
              lcd_DrawFillRectangle(x+70-2, y+70-2-chan11Last, x+70+2, y+70+2-chan11Last, BACKCOLOR); 
              lcd_DrawFillRectangle(x+70-2, y+70-2-chan11, x+70+2, y+70+2-chan11, LIGHTWHITE);	  
              channelsLast.chan11 = channels.chan11;
              lcd_DrawLine(x+70, y+70-40, x+70, y+70+40, LIGHTWHITE);

              lcd_DrawLine(x-130, y+70-40, x+130, y+70-40, LIGHTWHITE); //! 中轴线 
              lcd_DrawLine(x-130,    y+70, x+130,    y+70, LIGHTWHITE); //! 中轴线       
              lcd_DrawLine(x-130, y+70+40, x+130, y+70+40, LIGHTWHITE); //! 中轴线            
            }

            if(channels.chan12!=channelsLast.chan12)
            {
              int8_t chan12 = (channels.chan12-1500)/25*2; //! sacle: -40 -- +40
              int8_t chan12Last = (channelsLast.chan12-1500)/25*2; //! sacle: -40 -- +40           
              lcd_DrawFillRectangle(x+90-2, y+70-2-chan12Last, x+90+2, y+70+2-chan12Last, BACKCOLOR); 
              lcd_DrawFillRectangle(x+90-2, y+70-2-chan12, x+90+2, y+70+2-chan12, LIGHTWHITE);	  
              channelsLast.chan12 = channels.chan12;
              lcd_DrawLine(x+90, y+70-40, x+90, y+70+40, LIGHTWHITE);

              lcd_DrawLine(x-130, y+70-40, x+130, y+70-40, LIGHTWHITE); //! 中轴线 
              lcd_DrawLine(x-130,    y+70, x+130,    y+70, LIGHTWHITE); //! 中轴线       
              lcd_DrawLine(x-130, y+70+40, x+130, y+70+40, LIGHTWHITE); //! 中轴线            
            }

            if(channels.chan13!=channelsLast.chan13)
            {
              int8_t chan13 = (channels.chan13-1500)/25*2; //! sacle: -40 -- +40
              int8_t chan13Last = (channelsLast.chan13-1500)/25*2; //! sacle: -40 -- +40           
              lcd_DrawFillRectangle(x+110-2, y+70-2-chan13Last, x+110+2, y+70+2-chan13Last, BACKCOLOR);   
              lcd_DrawFillRectangle(x+110-2, y+70-2-chan13, x+110+2, y+70+2-chan13, LIGHTWHITE);	  
              channelsLast.chan13 = channels.chan13;
              lcd_DrawLine(x+110,y+70-40, x+110,y+70+40, LIGHTWHITE);

              lcd_DrawLine(x-130, y+70-40, x+130, y+70-40, LIGHTWHITE); //! 中轴线 
              lcd_DrawLine(x-130,    y+70, x+130,    y+70, LIGHTWHITE); //! 中轴线       
              lcd_DrawLine(x-130, y+70+40, x+130, y+70+40, LIGHTWHITE); //! 中轴线            
            }

            if(channels.chan14!=channelsLast.chan14)
            {
              int8_t chan14 = (channels.chan14-1500)/25*2; //! sacle: -40 -- +40
              int8_t chan14Last = (channelsLast.chan14-1500)/25*2; //! sacle: -40 -- +40           
              lcd_DrawFillRectangle(x+130-2, y+70-2-chan14Last, x+130+2, y+70+2-chan14Last, BACKCOLOR); 
              lcd_DrawFillRectangle(x+130-2, y+70-2-chan14, x+130+2, y+70+2-chan14, LIGHTWHITE);	  
              channelsLast.chan14 = channels.chan14;
              lcd_DrawLine(x+130,y+70-40, x+130,y+70+40, LIGHTWHITE); 

              lcd_DrawLine(x-130, y+70-40, x+130, y+70-40, LIGHTWHITE); //! 中轴线 
              lcd_DrawLine(x-130,    y+70, x+130,    y+70, LIGHTWHITE); //! 中轴线       
              lcd_DrawLine(x-130, y+70+40, x+130, y+70+40, LIGHTWHITE); //! 中轴线  
            } 
       
            break;
   }  
}





























