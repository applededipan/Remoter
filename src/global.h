

#ifndef _GLOBAL_H_
#define _GLOBAL_H_ 

#include <stdio.h>

#define  COMMAX  30  //! the comstep to eval the input from usb or rsp 

#define   ELE_MAX  1800
#define   ELE_MIN  200
#define   RUD_MAX  1800
#define   RUD_MIN  200
#define   THR_MAX  1800
#define   THR_MIN  200
#define   AIL_MAX  1800
#define   AIL_MIN  200
#define   LTRM_MAX 1800
#define   LTRM_MIN 200
#define   RTRM_MAX 1800
#define   RTRM_MIN 200


typedef struct
{
  int16_t   comStep;
  uint16_t  comCut;
}COMDATA;

extern COMDATA comData;

typedef struct 
{
  uint16_t chan1;
  uint16_t chan2;
  uint16_t chan3;
  uint16_t chan4;
  uint16_t chan5;
  uint16_t chan6;
  uint16_t chan7;
  uint16_t chan8;
  uint16_t chan9;
  uint16_t chan10;
  uint16_t chan11;
  uint16_t chan12;
  uint16_t chan13;
  uint16_t chan14;
}RC_CHANNEL;

extern RC_CHANNEL g_rcChannel;



enum DATASTREAMACK
{
    ACK_NO,
	ACK_OK,
};

enum ARMSTATE
{
    ARMSTATE_DISARMED,
	ARMSTATE_ARMED,
};


#define SUB_STATE_INIT  0     //! 第一次切换到该菜单，执行初始化的操作
#define SUB_STATE_START 1     //! 表示进入了该菜单
#define SUB_STATE_NONE  255   //! 菜单执行完初始化操作后跳转到该状态等待进一步操作（若菜单的作用是实时显示变化的数据则不需跳转到该状态）

enum CALIB_STATE
{
    CALIB_STATE_THRESHOLD = 2,    
	CALIB_STATE_STICK_LEFT_UP, 
	CALIB_STATE_STICK_LEFT_DOWN,
	CALIB_STATE_STICK_LEFT_LEFT,
	CALIB_STATE_STICK_LEFT_RIGHT,
	CALIB_STATE_STICK_LEFT_CENTER,
	CALIB_STATE_STICK_RIGHT_UP,
	CALIB_STATE_STICK_RIGHT_DOWN,
	CALIB_STATE_STICK_RIGHT_LEFT,
	CALIB_STATE_STICK_RIGHT_RIGHT,
	CALIB_STATE_STICK_RIGHT_CENTER,
	CALIB_STATE_TRIM_LEFT_LEFT,
    CALIB_STATE_TRIM_LEFT_RIGHT,
    CALIB_STATE_TRIM_LEFT_CENTER,
    CALIB_STATE_TRIM_RIGHT_LEFT,
    CALIB_STATE_TRIM_RIGHT_RIGHT,
    CALIB_STATE_TRIM_RIGHT_CENTER,
    CALIB_STATE_ERROR,
    CALIB_STATE_WRITE,
    CALIB_STATE_READ,    
    CALIB_STATE_FINISH,
    CALIB_STATE_EXIT,
};


enum MAIN_STATE
{
    MAIN_STATE_EXIT = 2,    
};


enum GIMBAL_STATE
{
    GIMBAL_STATE_EXIT = 2,   
};


enum RCCHANNELS_STATE
{
    RCCHANNELS_STATE_EXIT = 2,   
};


enum CAMERA_STATE
{
    CAMERA_STATE_EXIT = 2,   
};


enum FAILSAFE_STATE
{
    FAILSAFE_STATE_EXIT = 2,    
};


enum FLIGHTMODE_STATE
{
    FLIGHTMODE_STATE_EXIT = 2,   
};


enum BATTERY_STATE
{
    BATTERY_STATE_EXIT = 2,    
};


enum RADIO_STATE
{
    RADIO_STATE_EXIT = 2,    
};



enum VERSION_STATE
{
    VERSION_STATE_EXIT = 2,  
};


enum DEBUG_STATE
{
    DEBUG_STATE_EXIT = 2,   
};



enum MENUS
{
   MENU_MAIN = 0,
   MENU_GIMBAL,
   MENU_CAMERA,
   MENU_FAILSAFE,
   MENU_DEBUG,
   MENU_VERSION,
   MENU_FLIGHTMODE,
   MENU_BATTERY,
   MENU_RADIO,
   MENU_RCCHANNELS,   
   MENU_CALIBJOYSTICK,   
   MENU_NUMBER,   
};


enum COMLINK_STATUS 
{
  COMLINK_NAN = 0,	
  COMLINK_USB = 1,
  COMLINK_RSP = 2,
  COMLINK_BTH = 3,
};













#endif