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

#include "telemetry/mavlink.h"


/*******************************************************************************
 * @brief Reset basic Mavlink varables
*******************************************************************************/

void mavlinkReset(void) 
{
	mavlink_status_t* p_status = mavlink_get_channel_status(MAVLINK_COMM_0);
	p_status->current_rx_seq = 0;
	p_status->current_tx_seq = 0;
	memset(&mavData, 0, sizeof(mavData));
	mavData.mavStatus.health = 30; 
	mavData.heartBeat.type      = MAV_TYPE_ENUM_END;      //! default value is !0
	mavData.heartBeat.custMode  = FLIGHT_MODE_END;        //! default value is !0
	mavData.heartBeat.autopilot = MAV_AUTOPILOT_ENUM_END; //! default value is !0
}


/*******************************************************************************
 * @MSGID: MAVLINK_MSG_ID_HEARTBEAT 0
 
 * @type         Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @autopilot    Autopilot type / class. defined in MAV_AUTOPILOT ENUM
 * @baseMode     System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
 * @custMode     A bitfield for use for autopilot-specific flags.
 * @sysStatus    System status flag, see MAV_STATE ENUM
*******************************************************************************/
static inline void handle_message_heartbeat(const mavlink_message_t *msg) 
{
    if(msg->sysid != UAV_SYSTEM_ID) return;
    mavData.mavStatus.health = 0; //! UAVport receive a heartBeat message ok!
	
	mavData.heartBeat.type      = mavlink_msg_heartbeat_get_type(msg);
	mavData.heartBeat.autopilot = mavlink_msg_heartbeat_get_autopilot(msg);
	mavData.heartBeat.baseMode  = mavlink_msg_heartbeat_get_base_mode(msg);
	mavData.heartBeat.custMode  = mavlink_msg_heartbeat_get_custom_mode(msg);
	mavData.heartBeat.sysStatus = mavlink_msg_heartbeat_get_system_status(msg);
	mavData.heartBeat.version   = mavlink_msg_heartbeat_get_mavlink_version(msg);
	
	mavData.heartBeat.sysid = msg->sysid;
	mavData.heartBeat.cmpid = msg->compid;
	
	//! get the armed or disarmed state from mavData.heartBeat.baseMode
	if(mavData.heartBeat.baseMode & MAV_MODE_FLAG_SAFETY_ARMED) 
    {
	  mavData.mavStatus.armState = 1; //! armed	  
    }  	
    else 
    {
	  mavData.mavStatus.armState = 0; //! disarmed
    } 
}



/*******************************************************************************
 * @MSGID: MAVLINK_MSG_ID_SYS_STATUS 1
 
 * @volBat       Battery voltage, in millivolts (1 = 1 millivolt)
 * @curBat       Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 * @batRemain    Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery 
 * @dropRate     Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
*******************************************************************************/
static inline void handle_message_sys_status(const mavlink_message_t *msg) 
{   
    if(msg->sysid != UAV_SYSTEM_ID) return;
	mavData.mavStatus.health = 0; //! UAVport receive a heartBeat message ok!
	
	mavData.sysStatus.volBat    = mavlink_msg_sys_status_get_voltage_battery(msg); 
	mavData.sysStatus.curBat    = mavlink_msg_sys_status_get_current_battery(msg);
	mavData.sysStatus.batRemain = mavlink_msg_sys_status_get_battery_remaining(msg);
	mavData.sysStatus.dropRate  = mavlink_msg_sys_status_get_drop_rate_comm(msg);
}



/*******************************************************************************
 * @MSGID: MAVLINK_MSG_ID_PING 4

 * @param seq              PING sequence
 * @param target_system    0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @param target_component 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
*******************************************************************************/ 
static inline void handle_message_ping(const mavlink_message_t *msg)
{

}


 
/*******************************************************************************
 * @MSGID: MAVLINK_MSG_ID_GPS_RAW_INT 24
 
 * @fix_type  0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @lat       Latitude (WGS84), in degrees * 1E7
 * @lon       Longitude (WGS84), in degrees * 1E7
 * @alt       Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
 * @eph       GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
 * @epv       GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
 * @vel       GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
 * @cog       Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @satellites  Number of satellites visible. If unknown, set to 255
*******************************************************************************/
static inline void handle_message_gps_raw_int(const mavlink_message_t *msg)
{
    if(msg->sysid != UAV_SYSTEM_ID) return;
	
	mavData.gpsRaw.satellites = mavlink_msg_gps_raw_int_get_satellites_visible(msg);
	mavData.gpsRaw.fixType    = mavlink_msg_gps_raw_int_get_fix_type(msg);
}



/*******************************************************************************
 * @MSGID: MAVLINK_MSG_ID_SCALED_PRESSURE 29
 
 * @press_abs   Absolute pressure (hectopascal)
 * @press_diff  Differential pressure 1 (hectopascal)
 * @temperature Temperature measurement (0.01 degrees celsius)
*******************************************************************************/
static inline void handle_message_scaled_pressure(const mavlink_message_t *msg)
{
    if(msg->sysid != UAV_SYSTEM_ID) return;
}



/*******************************************************************************
 * @MSGID: MAVLINK_MSG_ID_ATTITUDE 30
 
 * @roll       Roll angle  (rad, -pi..+pi)
 * @pitch      Pitch angle (rad, -pi..+pi)
 * @yaw        Yaw angle   (rad, -pi..+pi)
 * @rollspeed  Roll angular speed    (rad/s)
 * @pitchspeed Pitch angular speed   (rad/s)
 * @yawspeed   Yaw angular speed     (rad/s)
 * @寮у害杞崲鎴愯搴﹀叕寮忥細 瑙掑害=180/蟺脳寮у害  = 57.3*寮у害
*******************************************************************************/
static inline void handle_message_attitude(const mavlink_message_t *msg)
{
    if(msg->sysid != UAV_SYSTEM_ID) return;
    if(mavData.mavStatus.health>10) mavData.mavStatus.health--; //! 璇ユ秷鎭鐜囧お楂橈紝鏀跺埌璇ユ秷鎭笉鑳戒唬琛ㄤ俊鍙峰緢濂斤紝閫氳繃health鑷噺鐨勬柟寮忚〃绀鸿娑堟伅瀵逛簬淇″彿妫?娴嬬殑鏉冮噸
	
	mavData.attitude.pitch_rad = mavlink_msg_attitude_get_pitch(msg);
	mavData.attitude.roll_rad  = mavlink_msg_attitude_get_roll(msg);
	mavData.attitude.yaw_rad   = mavlink_msg_attitude_get_yaw(msg);
}



/*******************************************************************************
 * @MSGID: MAVLINK_MSG_ID_GLOBAL_POSITION_INT 33
 
 * @lat Latitude, expressed as * 1E7
 * @lon Longitude, expressed as * 1E7
 * @alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
*******************************************************************************/
static inline void handle_message_global_position_int(const mavlink_message_t *msg)
{
    if(msg->sysid != UAV_SYSTEM_ID) return;
	
	mavData.globalPos.lat = mavlink_msg_global_position_int_get_lat(msg) / 1E7;
	mavData.globalPos.lon = mavlink_msg_global_position_int_get_lon(msg) / 1E7;
	mavData.globalPos.alt = mavlink_msg_global_position_int_get_alt(msg);
	mavData.globalPos.relative_alt = mavlink_msg_global_position_int_get_relative_alt(msg);
}



/*******************************************************************************
 * @MSGID: MAVLINK_MSG_ID_SERVO_OUTPUT_RAW 36
 
 * @chan1   Servo output 1 value, in microseconds. 1000--2000
 * @chan2   Servo output 2 value, in microseconds. 1000--2000
 * @chan3   Servo output 3 value, in microseconds. 1000--2000
 * @chan4   Servo output 4 value, in microseconds. 1000--2000
 * @chan5   Servo output 5 value, in microseconds. 1000--2000
 * @chan6   Servo output 6 value, in microseconds. 1000--2000
 * @chan7   Servo output 7 value, in microseconds. 1000--2000
 * @chan8   Servo output 8 value, in microseconds. 1000--2000
*******************************************************************************/
static inline void handle_message_servo_output_raw(const mavlink_message_t *msg) 
{
    if(msg->sysid != UAV_SYSTEM_ID) return;
	
    mavData.servoOutput.chan1 = mavlink_msg_servo_output_raw_get_servo1_raw(msg);
	mavData.servoOutput.chan2 = mavlink_msg_servo_output_raw_get_servo2_raw(msg);
	mavData.servoOutput.chan3 = mavlink_msg_servo_output_raw_get_servo3_raw(msg);
	mavData.servoOutput.chan4 = mavlink_msg_servo_output_raw_get_servo4_raw(msg);
	mavData.servoOutput.chan5 = mavlink_msg_servo_output_raw_get_servo5_raw(msg);
	mavData.servoOutput.chan6 = mavlink_msg_servo_output_raw_get_servo6_raw(msg);
	mavData.servoOutput.chan7 = mavlink_msg_servo_output_raw_get_servo7_raw(msg);
	mavData.servoOutput.chan8 = mavlink_msg_servo_output_raw_get_servo8_raw(msg);
}



/*******************************************************************************
 * @MSGID: MAVLINK_MSG_ID_RC_CHANNELS 65
 
 * @rssi Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
*******************************************************************************/
static inline void handle_message_rc_channels(const mavlink_message_t *msg) 
{
    if(msg->sysid != UAV_SYSTEM_ID) return;	
	
    //mavData.radioStatus.rssi = mavlink_msg_rc_channels_get_rssi(msg);
}



/*******************************************************************************
 * @MSGID: MAVLINK_MSG_ID_VFR_HUD 74
 
 * @airspeed    Current airspeed in m/s
 * @groundspeed Current ground speed in m/s
 * @heading     Current heading in degrees, in compass units (0..360, 0=north)
 * @throttle    Current throttle setting in integer percent, 0 to 100
 * @alt         Current altitude (MSL), in meters
 * @climb       Current climb rate in meters/second
*******************************************************************************/
static inline void handle_message_vfr_hud(const mavlink_message_t *msg) 
{
    if(msg->sysid != UAV_SYSTEM_ID) return;
	
	mavData.hud.alt	        = mavlink_msg_vfr_hud_get_alt(msg);
	mavData.hud.climb	    = mavlink_msg_vfr_hud_get_climb(msg);
	mavData.hud.heading     = mavlink_msg_vfr_hud_get_heading(msg);
	mavData.hud.throttle	= mavlink_msg_vfr_hud_get_throttle(msg);
	mavData.hud.airspeed    = mavlink_msg_vfr_hud_get_airspeed(msg);
	mavData.hud.groundspeed = mavlink_msg_vfr_hud_get_groundspeed(msg);

	mavData.mavStatus.dataStreamAck = ACK_OK; //! ACK_OK means received the ack of ask for data stream request	
}


/*******************************************************************************
 * @MSGID: MAVLINK_MSG_ID_HIL_CONTROLS 91
 
 * @roll_ailerons  Control output  -1 .. 1
 * @pitch_elevator Control output  -1 .. 1
 * @yaw_rudder     Control output  -1 .. 1
 * @throttle Throttle               0 .. 1
 * @aux1 Aux 1, -1 .. 1
 * @aux2 Aux 2, -1 .. 1
 * @aux3 Aux 3, -1 .. 1
 * @aux4 Aux 4, -1 .. 1
 * @mode System mode (MAV_MODE)
 * @nav_mode Navigation mode (MAV_NAV_MODE) 
*******************************************************************************/
static inline void handle_message_hil_controls(const mavlink_message_t *msg) 
{
    if(msg->sysid != UAV_SYSTEM_ID) return;
}


/*******************************************************************************
 * @MSGID: MAVLINK_MSG_ID_RADIO_STATUS 109                          same as 166
 
 * @rssi     local signal strength
 * @remrssi  remote signal strength
 * @txbuf    how full the tx buffer is as a percentage
 * @noise    background noise level
 * @remnoise remote background noise level
 * @rxerrors receive errors
 * @fixed    count of error corrected packets
*******************************************************************************/
static inline void handle_message_radio_status(const mavlink_message_t *msg) 
{
    if(msg->sysid != UAV_SYSTEM_ID) return;
	
	//mavData.radioStatus.rssi    = mavlink_msg_radio_status_get_rssi(msg);
	//mavData.radioStatus.remrssi = mavlink_msg_radio_status_get_remrssi(msg);
}


/*******************************************************************************
 * @MSGID: MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL 110   
 
 * @payload          Variable length payload. 
*******************************************************************************/
static inline void handle_message_file_transfer_protocol(const mavlink_message_t *msg) 
{	
	if(msg->sysid != QGCONTROL_ID) return;  //! message must from QGC 
	mavlink_file_transfer_protocol_t data;
	mavlink_msg_file_transfer_protocol_decode(msg, &data);	
	Payload* payload = (Payload*)&data.payload[0];	
	
	mavData.ftp.sysid = mavlink_msg_file_transfer_protocol_get_target_system(msg);
	if(mavData.ftp.sysid != OPENTX_SYSTEM_ID) return; //! see if this message is for smartconsole			
	
    mavData.ftp.payload.hdr.seqNumber = payload->hdr.seqNumber;
	mavData.ftp.payload.hdr.opcode    = payload->hdr.opcode;
	mavData.ftp.payload.hdr.offset    = payload->hdr.offset; 
    mavData.ftp.payload.hdr.size	  = payload->hdr.size;
	memcpy(&mavData.ftp.payload.data, &payload->data, sizeof(mavData.ftp.payload.data));
	
	g_eeGeneral.ftpReady = 1;	
}


/*******************************************************************************
 * @MSGID: MAVLINK_MSG_ID_BATTERY_STATUS 147  
 
 * @param id                Battery ID
 * @param battery_function  Function of the battery
 * @param type              Type (chemistry) of the battery
 * @param temperature       Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
 * @param voltages Battery  voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery should have the UINT16_MAX value.
 * @param current_battery   Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 * @param current_consumed  Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
 * @param energy_consumed   Consumed energy, in 100*Joules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate
 * @param battery_remaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
*******************************************************************************/
static inline void handle_message_battery_status(const mavlink_message_t* msg)
{
	if(msg->sysid != UAV_SYSTEM_ID) return;
	
	mavData.batStatus.current_consumed  = mavlink_msg_battery_status_get_current_consumed(msg);
	mavData.batStatus.energy_consumed   = mavlink_msg_battery_status_get_energy_consumed(msg);
	mavData.batStatus.battery_remaining = mavlink_msg_battery_status_get_battery_remaining(msg);     
	mavData.batStatus.current_battery   = mavlink_msg_battery_status_get_current_battery(msg);
	mavData.batStatus.battery_function  = mavlink_msg_battery_status_get_battery_function(msg);
	mavData.batStatus.temperature       = mavlink_msg_battery_status_get_temperature(msg);
	mavData.batStatus.type	            = mavlink_msg_battery_status_get_type(msg);
	mavData.batStatus.id                = mavlink_msg_battery_status_get_id(msg);	
    mavlink_msg_battery_status_get_voltages(msg, (uint16_t *)&mavData.batStatus.voltages);	
}



/*******************************************************************************
 * @MSGID: MAVLINK_MSG_ID_EXTENDED_SYS_STATE 245
 
 * @brief Send a extended_sys_state message
 * @param chan MAVLink channel to send the message
 *
 * @param vtol_state The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration.
 * @param landed_state The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
*******************************************************************************/
static inline void handle_message_extended_sys_state(const mavlink_message_t* msg)
{
    if(msg->sysid != UAV_SYSTEM_ID) return;
    
    mavData.sysStatus.vtolState = mavlink_msg_extended_sys_state_get_vtol_state(msg);
    mavData.sysStatus.landState = mavlink_msg_extended_sys_state_get_landed_state(msg);
}



/*******************************************************************************
 * @MSGID: MAVLINK_MSG_ID_STATUSTEXT 253
 
 * @details Processes the mavlink status messages. This message contains a
 * @severity and a message. The severity is an enum difined by MAV_SEVERITY also
 * @see RFC-5424 for the severity levels.
 * @The original message is maximum 50 characters and is without termination
 * @character. For readablity on the 9x the only the first 15 (LEN_STATUSTEXT)
 * @characters are used. To get the full message you can use the commented
 * @funtion below.
*******************************************************************************/
static inline void handle_message_statustext(const mavlink_message_t* msg) 
{
    if(msg->sysid != UAV_SYSTEM_ID) return;
}



/*******************************************************************************
 * @MSGID: MAVLINK_MSG_ID_DEBUG 254
 * @brief  this message come from raspi:pdlState  state of rpi&pddl, if rpi works ok, smartconsole will receive this message per 2s 
 * 1xxxxxxx锛歳aspi ok
 * 11xxxxxx: pddl  ok

*******************************************************************************/
static inline void handle_message_debug(const mavlink_message_t* msg)
{
	if(msg->sysid != RASPI_SYSTEM_ID) return; //! message must from RASPI_SYSTEM_ID
	mavData.mavStatus.pdlState = mavlink_msg_debug_get_ind(msg);
    mavData.mavStatus.raspiHealth = 0;
    
}




/*******************************************************************************
 * @ 
*******************************************************************************/
static inline void handleMessage(mavlink_message_t *msg) 
{    
	switch (msg->msgid) 
	{
	case MAVLINK_MSG_ID_HEARTBEAT:                        
		 handle_message_heartbeat(msg);
		 break;
    case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
	     handle_message_file_transfer_protocol(msg);
         break;			 
	case MAVLINK_MSG_ID_SYS_STATUS:                       
		 handle_message_sys_status(msg);
		 break;
	case MAVLINK_MSG_ID_BATTERY_STATUS:
	     handle_message_battery_status(msg);
		 break;
	case MAVLINK_MSG_ID_RC_CHANNELS:
	     handle_message_rc_channels(msg);
		 break;		 
	case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:                  
		 handle_message_servo_output_raw(msg);  
		 break;
    case MAVLINK_MSG_ID_GPS_RAW_INT:
	     handle_message_gps_raw_int(msg);
		 break;
	case MAVLINK_MSG_ID_RADIO_STATUS:
		 handle_message_radio_status(msg);
		 break;
	case MAVLINK_MSG_ID_VFR_HUD:                          
		 handle_message_vfr_hud(msg);
		 break;
	case MAVLINK_MSG_ID_ATTITUDE:                     
	     handle_message_attitude(msg);
	     break;
	case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:          
	     handle_message_global_position_int(msg);
	     break;
	case MAVLINK_MSG_ID_SCALED_PRESSURE:                  
	     handle_message_scaled_pressure(msg); 
         break;	        
    case MAVLINK_MSG_ID_DEBUG:
         handle_message_debug(msg);	
		 break;
    case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:           //! for vtol state
         handle_message_extended_sys_state(msg);
         break;         
	case MAVLINK_MSG_ID_PING:
	     handle_message_ping(msg);
         break;		 
	}

}


/******void mavlinkReceiver(mavlink_channel_t chan, uint8_t c)*****************
 * @brief Mavlink message parser
 
 * @details Parses the characters into a mavlink message.
 * @Case statement parses each character as it is recieved.
 * @attention One big change form the 0.9 to 1.0 version is the MAVLINK_CRC_EXTRA. 
 * @This requires the mavlink_message_crcs array of 256 bytes.
 * @todo create dot for the statemachine
*******************************************************************************/

void mavlinkReceiver(mavlink_channel_t chan, uint8_t c) 
{
	if(chan == MAVLINK_COMM_0)      //! date from usart2UavPort
	{
		static mavlink_message_t  m_mavlink_message_0;
		static mavlink_message_t* p_rxmsg_0 = &m_mavlink_message_0;
		mavlink_status_t* p_status_0 = mavlink_get_channel_status(MAVLINK_COMM_0);	
		if(mavlink_parse_char(MAVLINK_COMM_0, c, p_rxmsg_0, p_status_0))
		{	
			handleMessage(p_rxmsg_0);	
		}					
	}
	else if(chan == MAVLINK_COMM_1) //! date from usart1UsbPort only parse file_transfer_protocol message
	{
		static mavlink_message_t  m_mavlink_message_1;
		static mavlink_message_t* p_rxmsg_1 = &m_mavlink_message_1;
		mavlink_status_t* p_status_1 = mavlink_get_channel_status(MAVLINK_COMM_1);	
		if(mavlink_parse_char(MAVLINK_COMM_1, c, p_rxmsg_1, p_status_1))
		{	
            // mavData.mavStatus.dataReadyUsb = 1;  //! received a complete message
            handleMessage(p_rxmsg_1);	
		}				
	}
	else if(chan == MAVLINK_COMM_2) //! date from usart4RspPort only parse debug message
	{
		static mavlink_message_t  m_mavlink_message_2;
		static mavlink_message_t* p_rxmsg_2 = &m_mavlink_message_2;
		mavlink_status_t* p_status_2 = mavlink_get_channel_status(MAVLINK_COMM_2);	
		if(mavlink_parse_char(MAVLINK_COMM_2, c, p_rxmsg_2, p_status_2)) 
		{
            // mavData.mavStatus.dataReadyRsp = 1;	//! received a complete message
			handleMessage(p_rxmsg_2);	
		}
		
	}
	else if(chan == MAVLINK_COMM_3) //! date from usart3BthPort
	{
		static mavlink_message_t  m_mavlink_message_3;
		static mavlink_message_t* p_rxmsg_3 = &m_mavlink_message_3;
		mavlink_status_t* p_status_3 = mavlink_get_channel_status(MAVLINK_COMM_3);	
		if(mavlink_parse_char(MAVLINK_COMM_3, c, p_rxmsg_3, p_status_3)) 
		{
			handleMessage(p_rxmsg_3);	//! mavData.mavStatus.dataReadyBth = 1;		
		}		
	}
	else
	{
		//! nothing to do 
	}
}







/************************************************************************************************/
//! mavlink send messages functions are added by apple 
/************************************************************************************************/

/*******************************************************
 * @messageID #0
 * @brief Send a heartbeat message
 * 
 * @chan        The MAVLink channel this message will be sent over
 * @type        Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @autopilot   Autopilot type / class. defined in MAV_AUTOPILOT ENUM
 * @baseMode    System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
 * @custMode    A bitfield for use for autopilot-specific flags.
 * @sysStatus   System status flag, see MAV_STATE ENUM
*******************************************************/
void g_mavlink_msg_heartbeat_send(mavlink_channel_t chan, uint8_t type, uint8_t autopilot, uint8_t baseMode, uint32_t custMode, uint8_t sysStatus)
{
	mavlink_msg_heartbeat_send(chan, type, autopilot, baseMode, custMode, sysStatus);
}


/*******************************************************
 * @messageID #11
 * @brief Send a set_mode message
 * 
 * @target_system  The system setting the mode
 * @baseMode       The new base mode
 * @custMode       The new autopilot-specific mode. This field can be ignored by an autopilot. 
*******************************************************/
void g_mavlink_msg_set_mode_send(uint8_t target_system, uint8_t baseMode, uint32_t custMode)
{
    mavlink_channel_t chan = MAVLINK_COMM_0;
	mavlink_msg_set_mode_send(chan,  target_system,  baseMode,  custMode);
}


/*******************************************************
 * @messageID #34
 * @brief Send a rc_channels_scaled message
 * 
 * @time_boot_ms Timestamp (milliseconds since system boot)
 * @port    Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
 * @chan1   RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
 * @chan2   RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
 * @chan3   RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
 * @chan4   RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
 * @chan5   RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
 * @chan6   RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
 * @chan7   RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
 * @chan8   RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
 * @rssi    Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
*******************************************************/
void g_mavlink_msg_rc_channels_scaled_send(uint32_t time_boot_ms, uint8_t port, int16_t chan1, int16_t chan2, int16_t chan3, int16_t chan4, int16_t chan5, int16_t chan6, int16_t chan7, int16_t chan8, uint8_t rssi)
{
    mavlink_channel_t chan = MAVLINK_COMM_0;
    mavlink_msg_rc_channels_scaled_send(chan, time_boot_ms, port, chan1, chan2, chan3, chan4, chan5, chan6, chan7, chan8, rssi);	
}


/*******************************************************
 * @messageID #66
 * @brief Send a request_data_stream message
 *
 * @req_stream_id    The ID of the requested data stream
 * @req_message_rate The requested message rate
 * @start_stop       1 to start sending, 0 to stop sending. 
*******************************************************/
void g_mavlink_msg_request_data_stream_send(uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop)
{
    if(mavData.mavStatus.dataStreamAck == ACK_NO) 
	{
	   mavlink_channel_t chan = MAVLINK_COMM_0;		
       mavlink_msg_request_data_stream_send(chan, UAV_SYSTEM_ID, UAV_COMPON_ID, req_stream_id, req_message_rate, start_stop);		
	}	
}


/*******************************************************
 * @messageID #69
 * @brief Send a manual_control message
 * 
 * @target  The system to be controlled.
 * @x       X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
 * @y       Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
 * @z       Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative thrust.
 * @r       R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
 * @buttons A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
*******************************************************/
void g_mavlink_msg_manual_control_send(uint8_t target_system, int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons)
{
	mavlink_channel_t chan = MAVLINK_COMM_0;
	mavlink_msg_manual_control_send(chan, target_system, x, y, z, r, buttons);
}


/*******************************************************
 * @messageID #70
 * @brief Send a rc_channels_override message
 * 
 * @chan1_raw  ail   RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @chan2_raw  ele   RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @chan3_raw  thr   RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @chan4_raw  rud   RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @chan5_raw        RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @chan6_raw        RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @chan7_raw        RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @chan8_raw        RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
*******************************************************/
void g_mavlink_msg_rc_channels_override_send(uint16_t chan1, //! ail
											 uint16_t chan2, //! ele
											 uint16_t chan3, //! thr
											 uint16_t chan4, //! rud
											 uint16_t chan5, 
											 uint16_t chan6, 
											 uint16_t chan7, 
											 uint16_t chan8)
{
	mavlink_channel_t chan = MAVLINK_COMM_0;
	mavlink_msg_rc_channels_override_send(chan, UAV_SYSTEM_ID, UAV_COMPON_ID, chan1, chan2, chan3, chan4, chan5, chan6, chan7, chan8);
}


/*******************************************************
 * @messageID #76
 * @brief Send a command_long message
 * 
 * @target_system    System which should execute the command
 * @target_component Component which should execute the command, 0 for all components
 
 * @command          Command ID, as defined by MAV_CMD enum.
 * @confirmation     0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param1           Parameter 1, as defined by MAV_CMD enum.
 * @param2           Parameter 2, as defined by MAV_CMD enum.
 * @param3           Parameter 3, as defined by MAV_CMD enum.
 * @param4           Parameter 4, as defined by MAV_CMD enum.
 * @param5           Parameter 5, as defined by MAV_CMD enum.
 * @param6           Parameter 6, as defined by MAV_CMD enum.
 * @param7           Parameter 7, as defined by MAV_CMD enum.
*******************************************************/
void g_mavlink_msg_command_long_send(uint8_t  target_system, 
                                     uint8_t  target_component, 
									 uint16_t command, 
									 uint8_t  confirmation, 
									 float    param1, 
									 float    param2, 
									 float    param3, 
									 float    param4, 
									 float    param5, 
									 float    param6, 
									 float    param7)
{
	mavlink_channel_t chan = MAVLINK_COMM_0;	
	mavlink_msg_command_long_send(chan, target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7);
}


/*******************************************************
 * @messageID #110
 * @brief Send a mavlink_msg_id_file_transfer_protocol message
 * 
 * @chan               The MAVLink channel this message will be sent over
 * @target_network     Network ID (0 for broadcast)
 * @target_system      System ID (0 for broadcast)
 * @target_component   Component ID (0 for broadcast)
 * @payload            Variable length payload. 
 *******************************************************/
void g_mavlink_msg_file_transfer_protocol_send(mavlink_channel_t chan, 
                                               uint8_t target_network, 
											   uint8_t target_system, 
											   uint8_t target_component, 
											   uint8_t *payload)
{	
	mavlink_msg_file_transfer_protocol_send(chan, target_network, target_system, target_component, payload);
}



/*******************************************************
 * @messageID #112
 * @brief Send a camera_trigger message

 * @param seq Image frame sequence
 * seq:
       0X00: zoom stop
	   0X80: zoom in
	   0XA0: zoom out
 *******************************************************/
void g_mavlink_msg_camera_trigger_send(uint32_t seq)
{
	mavlink_channel_t chan = MAVLINK_COMM_0;
	mavlink_msg_camera_trigger_send(chan, 0, seq);
}
 






/*******************************************************
 * @brief set px4/apm flight mode
*******************************************************/

/*******************************************************
 * @PX4_FLIGHT_MODE_MANUAL
*******************************************************/
void px4_set_mode_manual(uint8_t target_system)
{  
   if(mavData.heartBeat.baseMode & MAV_MODE_FLAG_SAFETY_ARMED)
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED|MAV_MODE_FLAG_SAFETY_ARMED, PX4_FLIGHT_MODE_MANUAL);	   
   }
   else
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_FLIGHT_MODE_MANUAL);   
   }
}

/*******************************************************
 * @PX4_FLIGHT_MODE_ACRO
*******************************************************/
void px4_set_mode_acro(uint8_t target_system)
{
   if(mavData.mavStatus.armState)
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED|MAV_MODE_FLAG_SAFETY_ARMED, PX4_FLIGHT_MODE_ACRO);	   
   }
   else
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_FLIGHT_MODE_ACRO);	   
   }
}

/*******************************************************
 * @PX4_FLIGHT_MODE_STABILIZED
*******************************************************/
void px4_set_mode_stabilized(uint8_t target_system)
{
   if(mavData.mavStatus.armState)
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED|MAV_MODE_FLAG_SAFETY_ARMED, PX4_FLIGHT_MODE_STABILIZED);	   
   }
   else
   {
      g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_FLIGHT_MODE_STABILIZED);	   
   }
}

/*******************************************************
 * @PX4_FLIGHT_MODE_ALTITUDE
*******************************************************/
void px4_set_mode_altitude(uint8_t target_system)
{
   if(mavData.mavStatus.armState)
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED|MAV_MODE_FLAG_SAFETY_ARMED, PX4_FLIGHT_MODE_ALTITUDE);	   
   }
   else
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_FLIGHT_MODE_ALTITUDE);	   
   }
}

/*******************************************************
 * @PX4_FLIGHT_MODE_POSITION
*******************************************************/
void px4_set_mode_position(uint8_t target_system)
{
   if(mavData.mavStatus.armState)
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED|MAV_MODE_FLAG_SAFETY_ARMED, PX4_FLIGHT_MODE_POSITION);	   
   }
   else
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_FLIGHT_MODE_POSITION);	   
   }	
}

/*******************************************************
 * @PX4_FLIGHT_MODE_OFFBOARD
*******************************************************/
void px4_set_mode_offboard(uint8_t target_system)
{
   if(mavData.mavStatus.armState)
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED|MAV_MODE_FLAG_SAFETY_ARMED, PX4_FLIGHT_MODE_OFFBOARD);	   
   }
   else
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_FLIGHT_MODE_OFFBOARD);	   
   }
}

/*******************************************************
 * @PX4_FLIGHT_MODE_READYFLIGHT
*******************************************************/
void px4_set_mode_readyflight(uint8_t target_system)
{
   if(mavData.mavStatus.armState)
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED|MAV_MODE_FLAG_SAFETY_ARMED, PX4_FLIGHT_MODE_READYFLIGHT);	   
   }
   else
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_FLIGHT_MODE_READYFLIGHT);	   
   }
}

/*******************************************************
 * @PX4_FLIGHT_MODE_TAKEOFF
*******************************************************/
void px4_set_mode_takeoff(uint8_t target_system)
{
   if(mavData.mavStatus.armState)
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED|MAV_MODE_FLAG_SAFETY_ARMED, PX4_FLIGHT_MODE_TAKEOFF);	   
   }
   else
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_FLIGHT_MODE_TAKEOFF);	   
   }	
}

/*******************************************************
 * @PX4_FLIGHT_MODE_PAUSEFLIGHT
*******************************************************/
void px4_set_mode_pauseflight(uint8_t target_system)
{
   if(mavData.mavStatus.armState)
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED|MAV_MODE_FLAG_SAFETY_ARMED, PX4_FLIGHT_MODE_PAUSEFLIGHT);	   
   }
   else
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_FLIGHT_MODE_PAUSEFLIGHT);	   
   }
}

/*******************************************************
 * @PX4_FLIGHT_MODE_MISSION
*******************************************************/
void px4_set_mode_mission(uint8_t target_system)
{
   if(mavData.mavStatus.armState)
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED|MAV_MODE_FLAG_SAFETY_ARMED, PX4_FLIGHT_MODE_MISSION);	   
   }
   else
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_FLIGHT_MODE_MISSION);	   
   }	
}

/*******************************************************
 * @PX4_FLIGHT_MODE_RTL
*******************************************************/
void px4_set_mode_rtl(uint8_t target_system)
{
   if(mavData.mavStatus.armState)
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED|MAV_MODE_FLAG_SAFETY_ARMED, PX4_FLIGHT_MODE_RTL);	   
   }
   else
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_FLIGHT_MODE_RTL);	   
   }
}

/*******************************************************
 * @PX4_FLIGHT_MODE_LANDING
*******************************************************/
void px4_set_mode_landing(uint8_t target_system)
{
   if(mavData.mavStatus.armState)
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED|MAV_MODE_FLAG_SAFETY_ARMED, PX4_FLIGHT_MODE_LANDING);	   
   }
   else
   {
	  g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_FLIGHT_MODE_LANDING);	   
   }
}

/*******************************************************
 * @ APMCOPTER_FLIGHT_MODE_STABILIZE
*******************************************************/
void apmcopter_set_mode_stabilize(uint8_t target_system)
{
	g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, APMCOPTER_FLIGHT_MODE_STABILIZE);
}

/*******************************************************
 * @ APMCOPTER_FLIGHT_MODE_ACRO
*******************************************************/
void apmcopter_set_mode_acro(uint8_t target_system)
{
	g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, APMCOPTER_FLIGHT_MODE_ACRO);
}

/*******************************************************
 * @ APMCOPTER_FLIGHT_MODE_ALT_HOLD
*******************************************************/
void apmcopter_set_mode_alt_hold(uint8_t target_system)
{
	g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, APMCOPTER_FLIGHT_MODE_ALT_HOLD);
}

/*******************************************************
 * @ APMCOPTER_FLIGHT_MODE_AUTO
*******************************************************/
void apmcopter_set_mode_auto(uint8_t target_system)
{
	g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, APMCOPTER_FLIGHT_MODE_AUTO);
}

/*******************************************************
 * @ APMCOPTER_FLIGHT_MODE_LOITER
*******************************************************/
void apmcopter_set_mode_loiter(uint8_t target_system)
{
	g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, APMCOPTER_FLIGHT_MODE_LOITER);
}

/*******************************************************
 * @ APMCOPTER_FLIGHT_MODE_RTL
*******************************************************/
void apmcopter_set_mode_rtl(uint8_t target_system)
{
	g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, APMCOPTER_FLIGHT_MODE_RTL);
}

/*******************************************************
 * @ APMPLANE_FLIGHT_MODE_MANUAL
*******************************************************/
void apmplane_set_mode_manual(uint8_t target_system)
{
	g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, APMPLANE_FLIGHT_MODE_MANUAL);
}

/*******************************************************
 * @ APMPLANE_FLIGHT_MODE_STABILIZE
*******************************************************/
void apmplane_set_mode_stabilize(uint8_t target_system)
{
	g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, APMPLANE_FLIGHT_MODE_STABILIZE);
}

/*******************************************************
 * @ APMPLANE_FLIGHT_MODE_AUTO
*******************************************************/
void apmplane_set_mode_auto(uint8_t target_system)
{
	g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, APMPLANE_FLIGHT_MODE_AUTO);
}

/*******************************************************
 * @ APMPLANE_FLIGHT_MODE_RTL
*******************************************************/
void apmplane_set_mode_rtl(uint8_t target_system)
{
	g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, APMPLANE_FLIGHT_MODE_RTL);
}

/*******************************************************
 * @ APMPLANE_FLIGHT_MODE_CIRCLE
*******************************************************/
void apmplane_set_mode_circle(uint8_t target_system)
{
	g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, APMPLANE_FLIGHT_MODE_CIRCLE);
}

/*******************************************************
 * @ APMPLANE_FLIGHT_MODE_TRAINING
*******************************************************/
void apmplane_set_mode_training(uint8_t target_system)
{
	g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, APMPLANE_FLIGHT_MODE_TRAINING);
}

/*******************************************************
 * @ APMPLANE_FLIGHT_MODE_ACRO
*******************************************************/
void apmplane_set_mode_acro(uint8_t target_system)
{
	g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, APMPLANE_FLIGHT_MODE_ACRO);
}

/*******************************************************
 * @ APMPLANE_FLIGHT_MODE_LOITER
*******************************************************/
void apmplane_set_mode_loiter(uint8_t target_system)
{
	g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, APMPLANE_FLIGHT_MODE_LOITER);
}

/*******************************************************
 * @ APMPLANE_FLIGHT_MODE_GUIDED
*******************************************************/
void apmplane_set_mode_guided(uint8_t target_system)
{
	g_mavlink_msg_set_mode_send(target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, APMPLANE_FLIGHT_MODE_GUIDED);
}



/*******************************************************
 * @ autopilot send heartbeat message and ask for datastream
*******************************************************/
void autopilot_send_heartbeat(void)
{
  static gtime_t timeLast = 0;
  if(timeLast != g_rtcTime) 
  {
	 timeLast = g_rtcTime;
     g_mavlink_msg_heartbeat_send(MAVLINK_COMM_0, 0, 0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, 0); //! 蹇冭烦淇℃伅闄ysid澶栧椋炴帶鏉ヨ娌℃湁鎰忎箟
  }	 
}






/*******************************************************
 * @ autopilot gimbal control command
 
 * @ pitch   
 * @ roll
 * @ yaw
 * @ gimbalMode
*******************************************************/
void autopilot_gimbal_control_send(uint16_t pitch, uint16_t roll, uint16_t yaw, uint16_t gimbalMode)
{
    g_mavlink_msg_command_long_send(UAV_SYSTEM_ID, UAV_COMPON_ID, MAV_CMD_DO_MOUNT_CONTROL, 0, pitch, roll, yaw, gimbalMode, 0, 0, MAV_MOUNT_MODE_MAVLINK_TARGETING);		       	
}





/*******************************************************
 * @ autopilot set digicam zoom in or out, each cmd sent two times 
 * @ 0X80 zoom in
 * @ 0XA0 zoom out
*******************************************************/
void autopilot_set_digicam_zoom(uint32_t keyZoomIn, uint32_t keyZoomOut)
{
	static uint8_t state = 0;
	
	if(g_eeGeneral.key == keyZoomIn)        //! keyZoomIn pressed down and set digicam zoom in
	{
	  if(state++ < 2)  g_mavlink_msg_camera_trigger_send(0X80);  //g_mavlink_msg_digicam_control_send(UAV_SYSTEM_ID, UAV_COMPON_ID, 0, 0X80, 0, 0, 0, 0, 0, 0);    
	  else state = 2;
	}
	else if(g_eeGeneral.key == keyZoomOut)  //! keyZoomOut pressed down and set digicam zoom out
	{
	  if(state++ < 2)  g_mavlink_msg_camera_trigger_send(0XA0);  //g_mavlink_msg_digicam_control_send(UAV_SYSTEM_ID, UAV_COMPON_ID, 0, 0XA0, 0, 0, 0, 0, 0, 0);   
	  else state = 2;
	} 
	else if(state != 0)                     //! keyZoomIn/Out has been pressed down and set digicam zoom stop
	{
	  if(state++ < 4)  g_mavlink_msg_camera_trigger_send(0X00);  //g_mavlink_msg_digicam_control_send(UAV_SYSTEM_ID, UAV_COMPON_ID, 0, 0X00, 0, 0, 0, 0, 0, 0);    
      else state = 0;		  		  
	}	
}





/*******************************************************
 * @brief set autopilot armed or disarmed 
 * @brief 璇ュ嚱鏁板繀椤诲湪void per10ms()鍑芥暟涓皟鐢?
 * @ ch1    (1000 -- 2000) : ele  鍗囬檷
 * @ ch2    (1000 -- 2000) : rud  鏂瑰悜
 * @ ch3    (1000 -- 2000) : thr  娌归棬
 * @ ch4    (1000 -- 2000) : ail  鍓考
*******************************************************/
void autopilot_set_armed_disarmed(RC_CHANNEL channels)
{	
   static uint16_t flagarmcount = 0; 
   static uint16_t flagdisarmcount = 0;
   
   const uint16_t chan1_min = 1100;
   const uint16_t chan2_max = 1900;
   const uint16_t chan2_min = 1100;
   const uint16_t chan3_min = 1100;    
   const uint16_t chan4_min = 1100;   
   const uint16_t chan4_max = 1900;
  
   
   if((channels.chan2>chan2_max)&&(channels.chan1<chan1_min)&&(channels.chan3<chan3_min)&&(channels.chan4<chan4_min)) //! armed 
   {
	  flagdisarmcount = 0;
	  if((flagarmcount++)==20) //! delay 2s
	  {
		 g_mavlink_msg_command_long_send(UAV_SYSTEM_ID, UAV_COMPON_ID, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0); //! 瑙ｉ攣鍗虫瑁?
		 //flagarmcount = 0;
	  }		   
   }
   else if((channels.chan2<chan2_min)&&(channels.chan1<chan1_min)&&(channels.chan3<chan3_min)&&(channels.chan4>chan4_max)) //! disarmed
   {
	  flagarmcount = 0;	
	  if((flagdisarmcount++)==20)
	  { 	
		 g_mavlink_msg_command_long_send(UAV_SYSTEM_ID, UAV_COMPON_ID, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0); //!涓婇攣鍗宠В闄ゆ瑁?
		 //flagdisarmcount = 0;
	  }			   
   }	
   else
   {
	  flagarmcount = 0;	
	  flagdisarmcount = 0;		  
   }	   
}	



/*******************************************************
 * @brief send autopilot joystick command 
 * @brief only when has received the autopilot's message the smartconsole will send this control message
 * @ 鍓嶅洓涓?氶亾鐨勯『搴忎负锛氬崌闄嶏紝鏂瑰悜锛屾补闂紝鍓考锛汸X4鍜孉PM鐨勯『搴忔湁鍖哄埆锛屼絾鍦ㄥ嚱鏁板唴閮ㄨ繘琛屼簡淇敼锛侊紒锛?
 
 * @ ch1    (1000 -- 2000) : ele  鍗囬檷
 * @ ch2    (1000 -- 2000) : rud  鏂瑰悜
 * @ ch3    (1000 -- 2000) : thr  娌归棬
 * @ ch4    (1000 -- 2000) : ail  鍓考
 * @ ch5    (1000 -- 2000)
 * @ ch6    (1000 -- 2000)
 * @ ch7    (1000 -- 2000)
 * @ ch8    (1000 -- 2000) 
*******************************************************/
void autopilot_joystick_command_send(RC_CHANNEL channels)
{
	const  uint8_t MIN = 5;   //! 闃堝??
	static uint8_t count = 0; //! 2鍒嗛
	
	static int16_t ch1Last = 0;
	static int16_t ch2Last = 0;
	static int16_t ch3Last = 0;
	static int16_t ch4Last = 0;
	static int16_t ch5Last = 0;
	static int16_t ch6Last = 0;
	static int16_t ch7Last = 0;
	static int16_t ch8Last = 0;
	
	if(count++ > 1) count = 0;
	//! 20Hz	
	if(abs(channels.chan1-ch1Last)>MIN || abs(channels.chan2-ch2Last)>MIN || abs(channels.chan3-ch3Last)>MIN || abs(channels.chan4-ch4Last)>MIN || abs(channels.chan5-ch5Last)>MIN || abs(channels.chan6-ch6Last)>MIN || abs(channels.chan7-ch7Last)>MIN || abs(channels.chan8-ch8Last)>MIN)
	{
		//! PX4
	   if(mavData.heartBeat.autopilot == MAV_AUTOPILOT_PX4)                //! PX4 inputs order: ele rud thr ail
	   {
		  //! +1000 -- +2000	
		  g_mavlink_msg_rc_channels_override_send(channels.chan1, channels.chan2, channels.chan3, channels.chan4, channels.chan5, channels.chan6, channels.chan7, channels.chan8);
	   }
	   else if(mavData.heartBeat.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA) //! APM inputs order: ail ele thr rud
	   {
		  //! +1000 -- +2000		
		  g_mavlink_msg_rc_channels_override_send(channels.chan4, channels.chan1, channels.chan3, channels.chan2, channels.chan5, channels.chan6, channels.chan7, channels.chan8);  
	   }
	   else
	   {
		  //! other autopilot
	   }
	   
	   ch1Last = channels.chan1;
	   ch2Last = channels.chan2;
	   ch3Last = channels.chan3;
	   ch4Last = channels.chan4; 
	   ch5Last = channels.chan5;
	   ch6Last = channels.chan6;
       ch7Last = channels.chan7;
       ch8Last = channels.chan8;
	}
	//! 10Hz
	else if(count) //! 瀹氭椂鍙戦?侀槻姝onnect lost
	{
	   //! PX4鎴朠XHAWK
	   if(mavData.heartBeat.autopilot == MAV_AUTOPILOT_PX4)
	   {
		  g_mavlink_msg_rc_channels_override_send(ch1Last, ch2Last, ch3Last, ch4Last, ch5Last, ch6Last, ch7Last, ch8Last);
	   }
	   else if(mavData.heartBeat.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA)//! APM
	   {
		  g_mavlink_msg_rc_channels_override_send(ch4Last, ch1Last, ch3Last, ch2Last, ch5Last, ch6Last, ch7Last, ch8Last);	   
	   }
	   else
	   {
		  //! other autopilot
	   }		   
	}
	else
	{
		//! nothing to do
	}
}



/*******************************************************************************
 * @ all datastreams are sent at 20Hz / 50ms
*******************************************************************************/
void mavlinkSendMessage(void)
{
	static uint8_t streamId = 0;
	if(streamId++ > 4) streamId = 0; //! 4
	//! all datastreams are sent based on that the smart console has connected to uav
	if((mavData.heartBeat.autopilot != MAV_AUTOPILOT_ARDUPILOTMEGA)&&(mavData.heartBeat.autopilot != MAV_AUTOPILOT_PX4)) return;
    if(g_eeGeneral.dataLost) return;
	switch(streamId)
	{   
	    
		case 0: //! in apm autopilot we should ask for datastream
		        g_mavlink_msg_request_data_stream_send(MAV_DATA_STREAM_ALL, 3, 1);  
		        break;
		
		case 1: //! send armd or disarmed messages
		        // autopilot_set_armed_disarmed(g_rcChannel);	  
		        break;		
      
		case 2: //! send joystick values
				autopilot_joystick_command_send(g_rcChannel); 	
		        break;	
        				
		case 3: 
		        #if defined (DIGICAM_ZONE_IN_OUT)	  //! digicam camera zoom_in_out control	
                autopilot_set_digicam_zoom(OPENTX_TL1, OPENTX_TL2);	
                #endif
		        break;	
		
		case 4: 	    
				break;
			
	   default: break;
	}
} 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
