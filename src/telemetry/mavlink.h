/*
 * Authors (alphabetical order)
 * - Andre Bernet <bernet.andre@gmail.com>
 * - Andreas Weitl
 * - Bertrand Songis <bsongis@gmail.com>
 * - Bryan J. Rentoul (Gruvin) <gruvin@gmail.com>
 * - Cameron Weeks <th9xer@gmail.com>
 * - Erez Raviv
 * - Gabriel Birkus
 * - Gerard Valade <gerard.valade@gmail.com>
 * - Jean-Pierre Parisy
 * - Karl Szmutny
 * - Michael Blandford
 * - Michal Hlavinka
 * - Pat Mackenzie
 * - Philip Moss
 * - Rienk de Jong
 * - Rob Thomson
 * - Romolo Manfredini <romolo.manfredini@gmail.com>
 * - Thomas Husterer
 *
 * Adapted from mavlink for ardupilot
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
 

#ifndef _MAVLINK_H_
#define _MAVLINK_H_


#define UAV_SYSTEM_ID       1  //!PIXHAWK and APM firmware both default are 1
#define UAV_COMPON_ID       1  //!PIXHAWK and APM firmware both default are 1
#define QGCONTROL_ID      255  //!QGC 
#define RASPI_SYSTEM_ID   250  //!RASPI
#define OPENTX_SYSTEM_ID  255 
#define OPENTX_COMPON_ID  250


#define mavlinkDateInit  mavlinkReset
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
//#define MAVLINK_COMM_NUM_BUFFERS 1

#include "opentx.h"

#define MAVLINK_1

#if defined MAVLINK_1
	#include "GCS_MAVLink/v1.0/mavlink_types.h"
	mavlink_system_t mavlink_system = {OPENTX_SYSTEM_ID, OPENTX_COMPON_ID};
	#include "GCS_MAVLink/v1.0/common/mavlink.h"
	#include "GCS_MAVLink/v1.0/ardupilotmega/mavlink.h"
#else
	#include "GCS_MAVLink/v2.0/mavlink_types.h"
	mavlink_system_t mavlink_system = {OPENTX_SYSTEM_ID, OPENTX_COMPON_ID};
	#include "GCS_MAVLink/v2.0/common/mavlink.h"
	#include "GCS_MAVLink/v2.0/ardupilotmega/mavlink.h"	
#endif

//! macros below may be useful sometime later, so do not delete it !!!  apple
//! #define MAVLINK_START_UART_SEND(chan,len) 
//! #define MAVLINK_END_UART_SEND(chan,len) 
//! #define MAVLINK_SEND_UART_BYTES(chan,buf,len)  usart2UavSendBuffer(buf, len)
#define MAVLINK_SEND_UART_BYTES



struct MavlinkStatus
{
	uint8_t  health;         //! 0：connected!     30: unconnect!    1--29: signal strength
	uint8_t  armState;       //! 
	uint8_t  pdlState;       //! 图传状态 
    uint8_t  raspiHealth;
	uint8_t  dataStreamAck;  //! 数据流请求回应
    uint8_t  dataReadyUsb;   //! usart1UsbPort message ready  1: ok   0: not ok 
    uint8_t  dataReadyRsp;   //! usart4RspPort message ready
    uint8_t  dataReadyBth;   //! usart3BthPort message ready

};

struct HeartBeat   
{
	uint8_t  sysid;     //! sender's sysid
	uint8_t  cmpid;     //! sender's cmpid
	uint8_t  type;      //! Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
	uint8_t  autopilot; //! Autopilot type class. defined in MAV_AUTOPILOT ENUM
	uint8_t  baseMode;  //! System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
	uint8_t  sysStatus; //! System status flag, see MAV_STATUS ENUM
	uint32_t custMode;  //! A bitfield for use for autopilot-specific flags
	uint8_t  version;
};


struct SystemStatus   
{
	uint16_t volBat;    //!   1 = 1 millivolt
	int16_t  curBat;    //!   1 = 10 milliampere      -1: does not measure the current
	int8_t   batRemain; //!   0%: 0,  100%: 100,      -1: estimating the remaining battery_remaining
	uint16_t dropRate;  //!   0%: 0,  100%: 10,000
    uint8_t  vtolState; //!   The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration
    uint8_t  landState; //!   The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown
};


struct BatteryStatus
{
	int32_t  current_consumed;  //! Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
	int32_t  energy_consumed;   //! Consumed energy, in 100*Joules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate
    int16_t  temperature;       //! Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
    uint16_t voltages[10];      //! Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery should have the UINT16_MAX value.
	int16_t  current_battery;   //! Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
    uint8_t  id;                //! Battery ID
	uint8_t  battery_function;  //! Function of the battery
	uint8_t  type;              //! Type (chemistry) of the battery
	int8_t   battery_remaining; //! Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery	
};


struct RadioStatus
{
    uint8_t rssi;       //!  local signal strength
    uint8_t remrssi;    //!  remote signal strength	
};

typedef RC_CHANNEL ServoOutput;

struct GpsRaw
{
   uint8_t satellites;
   uint8_t fixType;
};


struct Attitude  
{
	float roll;          
	float pitch;          
	float yaw;           
	float roll_rad;       
	float pitch_rad;
	float yaw_rad;
	
	bool update;
};


struct GlobalPosition
{
	int32_t lat;
	int32_t lon;            
	int32_t alt;              
	int32_t relative_alt;  
};


struct VrfHud  
{
	float    airspeed;
	float    groundspeed;
	float    alt;          
	float    climb;
	int16_t  heading;	     
	uint16_t throttle;     
};


struct RequestHeader
{
	uint16_t seqNumber;      //! sequence number for message
	uint8_t  session;        //! Session id for read and write commands
	uint8_t  opcode;         //! Command opcode
	uint8_t  size;           //! Size of data
	uint8_t  reqOpcode;      //! Request opcode returned in kRspAck, kRspNak message
	uint8_t  burstComplete;  //! Only used if req_opcode=kCmdBurstReadFile - 1: set of burst packets complete, 0: More burst packets coming.
	uint8_t  padding;        //! 32 bit aligment padding
	uint32_t offset;         //! Offsets for List and Read commands
};	


struct Payload
{
   RequestHeader  hdr;
   uint8_t data[sizeof(((mavlink_file_transfer_protocol_t*)0)->payload) - sizeof(RequestHeader)];			
};	


struct Ftp
{
    uint8_t network;  //! Network ID   (0 for broadcast)
    uint8_t sysid;    //! System ID    (0 for broadcast)
    uint8_t cmpid;    //! Component ID (0 for broadcast)
	Payload payload;
};


struct MavData
{
	Ftp             ftp;	
	GpsRaw          gpsRaw;
	VrfHud          hud;
	Attitude        attitude;	
	HeartBeat       heartBeat;
	ServoOutput     servoOutput;	
    RadioStatus     radioStatus;
	SystemStatus    sysStatus;	
	MavlinkStatus   mavStatus;
	BatteryStatus	batStatus;
	GlobalPosition  globalPos;
  
};

MavData  mavData;  //! defined by apple

void mavlinkReset(void);
void mavlinkReceiver(mavlink_channel_t chan, uint8_t c);
void px4_set_mode_rtl(uint8_t target_system);
void px4_set_mode_acro(uint8_t target_system);   //need
void px4_set_mode_manual(uint8_t target_system); //need
void px4_set_mode_mission(uint8_t target_system);//need
void px4_set_mode_takeoff(uint8_t target_system);
void px4_set_mode_landing(uint8_t target_system);
void px4_set_mode_altitude(uint8_t target_system);
void px4_set_mode_position(uint8_t target_system);
void px4_set_mode_offboard(uint8_t target_system);
void px4_set_mode_stabilized(uint8_t target_system);
void px4_set_mode_readyflight(uint8_t target_system);
void px4_set_mode_pauseflight(uint8_t target_system);//need

void apmcopter_set_mode_rtl(uint8_t target_system);
void apmcopter_set_mode_acro(uint8_t target_system);
void apmcopter_set_mode_auto(uint8_t target_system);
void apmcopter_set_mode_loiter(uint8_t target_system);
void apmcopter_set_mode_alt_hold(uint8_t target_system);
void apmcopter_set_mode_stabilize(uint8_t target_system);

void apmplane_set_mode_rtl(uint8_t target_system);
void apmplane_set_mode_auto(uint8_t target_system);
void apmplane_set_mode_manual(uint8_t target_system);
void apmplane_set_mode_stabilize(uint8_t target_system);
void apmplane_set_mode_circle(uint8_t target_system);
void apmplane_set_mode_training(uint8_t target_system);
void apmplane_set_mode_acro(uint8_t target_system);
void apmplane_set_mode_loiter(uint8_t target_system);
void apmplane_set_mode_guided(uint8_t target_system);


void g_mavlink_msg_set_mode_send(uint8_t target_system, uint8_t baseMode, uint32_t custMode);
void g_mavlink_msg_manual_control_send(uint8_t target_system, int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons);
void g_mavlink_msg_heartbeat_send(mavlink_channel_t chan, uint8_t type, uint8_t autopilot, uint8_t baseMode, uint32_t custMode, uint8_t sysStatus);
void g_mavlink_msg_request_data_stream_send(uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop);											   													 
void g_mavlink_msg_set_local_position_setpoint_send(uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, float x, float y, float z, float yaw);		
void g_mavlink_msg_rc_channels_scaled_send(uint32_t time_boot_ms, uint8_t port, int16_t chan1, int16_t chan2, int16_t chan3, int16_t chan4, int16_t chan5, int16_t chan6, int16_t chan7, int16_t chan8, uint8_t rssi);										   										   																		  
void g_mavlink_msg_rc_channels_override_send(uint16_t chan1, uint16_t chan2, uint16_t chan3, uint16_t chan4, uint16_t chan5, uint16_t chan6, uint16_t chan7, uint16_t chan8);										 							 
void g_mavlink_msg_command_long_send(uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7);														   
void g_mavlink_msg_file_transfer_protocol_send(mavlink_channel_t chan, uint8_t target_network, uint8_t target_system, uint8_t target_component, uint8_t *payload);
void g_mavlink_msg_camera_trigger_send(uint32_t seq);


void mavlinkSendMessage(void);
void autopilot_send_heartbeat(void);
void autopilot_set_digicam_zoom(uint32_t keyZoomIn, uint32_t keyZoomOut);
void autopilot_gimbal_control_send(uint16_t pitch, uint16_t roll, uint16_t yaw, uint16_t gimbalMode);
void autopilot_set_armed_disarmed(RC_CHANNEL channels);
void autopilot_joystick_command_send(RC_CHANNEL channels);
/***********************************************************************************************/



enum FTP_OPCODE
{
	kCmdNone,				//! ignored, always acked
	kCmdTerminateSession,	//! Terminates open Read session
	kCmdResetSessions,		//! Terminates all open Read sessions
	kCmdListDirectory,		//! List files in <path> from <offset>
	kCmdOpenFileRO,			//! Opens file at <path> for reading, returns <session>
	kCmdReadFile,			//! Reads <size> bytes from <offset> in <session>
	kCmdCreateFile,			//! Creates file at <path> for writing, returns <session>
	kCmdWriteFile,			//! Writes <size> bytes to <offset> in <session>
	kCmdRemoveFile,			//! Remove file at <path>
	kCmdCreateDirectory,	//! Creates directory at <path>
	kCmdRemoveDirectory,	//! Removes Directory at <path>, must be empty
	kCmdOpenFileWO,			//! Opens file at <path> for writing, returns <session>
	kCmdTruncateFile,		//! Truncate file at <path> to <offset> length
	kCmdRename,				//! Rename <path1> to <path2>
	kCmdCalcFileCRC32,		//! Calculate CRC32 for file at <path>
	kCmdBurstReadFile,      //! Burst download session file
	kCmdSearchVersion,      //! search for firmware version by wireless connection
	kCmdReboot = 100,       //! reBoot
	kRspAck = 128,          //! Ack response
	kRspNak,                //! Nak response
	kCmdTestNoAck,          //! ignored, ack not sent back, should timeout waiting for ack
};

/************************************************************************/
//! enums about px4 custMode below are added by apple

//! MANUAL: 
//!       Fixed wing aircraft/ rovers/ boats:
//! 	  Multirotors:
//! 	    ACRO:
//! 		RATTITUDE:
//! 		ANGLE:
//! ASSISTED:
//!         ALTCTL:
//! 	    POSCTL:
//! AUTO:
//!         AUTO_LOITER:
//!         AUTO_RTL:
//!         AUTO_MISSION:	 
//! OFFBOARD:	  

/************************************************************************/
#ifndef PX4_CUSTOM_MODE_H_
#define PX4_CUSTOM_MODE_H_

enum PX4_CUSTOM_MAIN_MODE{
	 PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
	 PX4_CUSTOM_MAIN_MODE_ALTCTL,
	 PX4_CUSTOM_MAIN_MODE_POSCTL,
	 PX4_CUSTOM_MAIN_MODE_AUTO,      
	 PX4_CUSTOM_MAIN_MODE_ACRO,
	 PX4_CUSTOM_MAIN_MODE_OFFBOARD,
	 PX4_CUSTOM_MAIN_MODE_STABILIZED,
	 PX4_CUSTOM_MAIN_MODE_RATTITUDE
};

enum PX4_CUSTOM_SUB_MODE_AUTO{
	 PX4_CUSTOM_SUB_MODE_AUTO_READY = 1,
	 PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,
	 PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
	 PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
	 PX4_CUSTOM_SUB_MODE_AUTO_RTL,
	 PX4_CUSTOM_SUB_MODE_AUTO_LAND,
	 PX4_CUSTOM_SUB_MODE_AUTO_RTGS
};

//PX4_FLIGHT_MODES
#define PX4_FLIGHT_MODE_MANUAL          PX4_CUSTOM_MAIN_MODE_MANUAL<<16                                        //! belong to manual
#define PX4_FLIGHT_MODE_ACRO            PX4_CUSTOM_MAIN_MODE_ACRO<<16                                          //! belong to manual
#define PX4_FLIGHT_MODE_STABILIZED      PX4_CUSTOM_MAIN_MODE_STABILIZED<<16                                    //! belong to manual
#define PX4_FLIGHT_MODE_RATTITUDE       PX4_CUSTOM_MAIN_MODE_RATTITUDE<<16                                     //! belong to manual
#define PX4_FLIGHT_MODE_ALTITUDE        PX4_CUSTOM_MAIN_MODE_ALTCTL<<16                                        //! belong to assist
#define PX4_FLIGHT_MODE_POSITION        PX4_CUSTOM_MAIN_MODE_POSCTL<<16                                        //! belong to assist     
#define PX4_FLIGHT_MODE_OFFBOARD        PX4_CUSTOM_MAIN_MODE_OFFBOARD<<16
#define PX4_FLIGHT_MODE_READYFLIGHT   ((PX4_CUSTOM_MAIN_MODE_AUTO<<16)|(PX4_CUSTOM_SUB_MODE_AUTO_READY<<24))   //! belong to auto
#define PX4_FLIGHT_MODE_TAKEOFF       ((PX4_CUSTOM_MAIN_MODE_AUTO<<16)|(PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF<<24)) //! belong to auto
#define PX4_FLIGHT_MODE_PAUSEFLIGHT   ((PX4_CUSTOM_MAIN_MODE_AUTO<<16)|(PX4_CUSTOM_SUB_MODE_AUTO_LOITER<<24))  //! belong to auto
#define PX4_FLIGHT_MODE_MISSION       ((PX4_CUSTOM_MAIN_MODE_AUTO<<16)|(PX4_CUSTOM_SUB_MODE_AUTO_MISSION<<24)) //! belong to auto
#define PX4_FLIGHT_MODE_RTL           ((PX4_CUSTOM_MAIN_MODE_AUTO<<16)|(PX4_CUSTOM_SUB_MODE_AUTO_RTL<<24))     //! belong to auto
#define PX4_FLIGHT_MODE_LANDING       ((PX4_CUSTOM_MAIN_MODE_AUTO<<16)|(PX4_CUSTOM_SUB_MODE_AUTO_LAND<<24))    //! belong to auto
#define PX4_FLIGHT_MODE_RTGS          ((PX4_CUSTOM_MAIN_MODE_AUTO<<16)|(PX4_CUSTOM_SUB_MODE_AUTO_RTGS<<24))    //! belong to auto

#define FLIGHT_MODE_END                 255 //! added by apple for mavlink default mode or reset mode
#endif /* PX4_CUSTOM_MODE_H_ */


/************************************************************************/
// enums about apm custMode below are added by apple
/************************************************************************/
#ifndef APM_FLIGHT_MODE_H_
#define APM_FLIGHT_MODE_H_

enum APMCOPTER_FLIGHT_MODE{
	 APMCOPTER_FLIGHT_MODE_STABILIZE, //自稳模式  no gps
	 APMCOPTER_FLIGHT_MODE_ACRO,      //特技模式  no gps
	 APMCOPTER_FLIGHT_MODE_ALT_HOLD,  //定高模式  no gps
	 APMCOPTER_FLIGHT_MODE_AUTO,      //自动模式  need gps
	 APMCOPTER_FLIGHT_MODE_GUIDED,    //引导模式  
	 APMCOPTER_FLIGHT_MODE_LOITER,    //留待模式  need gps
	 APMCOPTER_FLIGHT_MODE_RTL,       //返航模式
	 APMCOPTER_FLIGHT_MODE_CIRCLE,    //绕圈模式
	 APMCOPTER_FLIGHT_MODE_POSITION,  //位置模式
	 APMCOPTER_FLIGHT_MODE_LAND,      //降落模式
	 APMCOPTER_FLIGHT_MODE_OF_LOITER, //悬停模式
	 APMCOPTER_FLIGHT_MODE_DRIFT,     //漂移模式
	 APMCOPTER_FLIGHT_MODE_SPORT,     //运动模式
	 APMCOPTER_FLIGHT_MODE_FLIP,      //
	 APMCOPTER_FLIGHT_MODE_AUTOTUNE,  //
	 APMCOPTER_FLIGHT_MODE_POS_HOLD,  //定点模式
	 APMCOPTER_FLIGHT_MODE_BRAKE      //
};

enum APMPLANE_FLIGHT_MODE{
	 APMPLANE_FLIGHT_MODE_MANUAL = 0,       //0手动模式  no gps 
	 APMPLANE_FLIGHT_MODE_CIRCLE = 1,       //1绕圈模式  no gps
	 APMPLANE_FLIGHT_MODE_STABILIZE = 2,    //2自稳模式  no gps
	 APMPLANE_FLIGHT_MODE_TRAINING  = 3,    //3训练模式  no gps
	 APMPLANE_FLIGHT_MODE_ACRO = 4,         //4特技模式  no gps  
	 APMPLANE_FLIGHT_MODE_FLY_BY_WIRE_A = 5,//5线性A增稳 no gps
	 APMPLANE_FLIGHT_MODE_FLY_BY_WIRE_B = 6,//6线性B增稳 no gps
	 APMPLANE_FLIGHT_MODE_CRUISE = 7,       //7巡航模式  need gps   
	 APMPLANE_FLIGHT_MODE_AUTOTUNE = 8,     //8          need gps
	 APMPLANE_FLIGHT_MODE_AUTO =10,         //10自动模式 need gps
	 APMPLANE_FLIGHT_MODE_RTL = 11,         //11返航模式 need gps  
	 APMPLANE_FLIGHT_MODE_LOITER = 12,      //12定点模式 need gps
	 APMPLANE_FLIGHT_MODE_GUIDED=15,        //15引导模式 need gps
	 APMPLANE_FLIGHT_MODE_INITIALIZING, 
};
#endif

#endif















