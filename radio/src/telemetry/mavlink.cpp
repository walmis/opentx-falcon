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
#include "telemetry/radio_cfg_packets.hpp"

// this might need to move to the flight software
//static
mavlink_system_t mavlink_system = { 7, MAV_COMP_ID_MISSIONPLANNER };

//static uint8_t system_id = 7;
//static uint8_t component_id = 1;
//static uint8_t target_system = 7;
//static uint8_t target_component = 1;


// Mavlink message decoded Status Text
#define PARAM_NB_REPEAT 10

char mav_statustext[LEN_STATUSTEXT];
uint8_t msg_severity;
uint16_t msg_timestamp;

uint16_t mav_heartbeat = 0;
static uint8_t data_stream_start_stop = 0;

static int8_t actualbaudrateIdx = 0;
// Telemetry data hold
Telemetry_Data_t telemetry_data;
RadioConfiguration radioCfg;
RadioStatus radioStatus;

#ifdef DUMP_RX_TX
#define MAX_RX_BUFFER 16
uint8_t mavlinkRxBufferCount = 0;
uint8_t mavlinkRxBuffer[MAX_RX_BUFFER];
uint8_t mav_dump_rx = 0;
#endif

// *****************************************************

#include "serial.h"

#if defined(CPUARM)
#else
#include "serial.cpp"
void MAVLINK_rxhandler(uint8_t byte) {
	processSerialData(byte);
}

SerialFuncP RXHandler = MAVLINK_rxhandler;
#endif


/*!	\brief Reset basic Mavlink variables
 *	\todo Figure out exact function
 *
 */
void MAVLINK_reset(uint8_t warm_reset) {
	if (warm_reset && telemetry_data.status) {
		mav_statustext[0] = 0;
	}
#ifdef DUMP_RX_TX
	mavlinkRxBufferCount = 0;
	mav_dump_rx = 0;
#endif

	mavlink_status_t* p_status = mavlink_get_channel_status(MAVLINK_COMM_0);
	p_status->current_rx_seq = 0;
	p_status->current_tx_seq = 0;
	memset(&telemetry_data, 0, sizeof(telemetry_data));
	//telemetry_data.rcv_control_mode = ERROR_NUM_MODES;
	//telemetry_data.req_mode = ERROR_NUM_MODES;

	telemetry_data.type = MAV_TYPE_ENUM_END;
	telemetry_data.autopilot = MAV_AUTOPILOT_ENUM_END;
	telemetry_data.type_autopilot = MAVLINK_INVALID_TYPE;

	mav_heartbeat = 0;
	data_stream_start_stop = 0;
}

//! \brief initialize serial (see opentx.cpp)
void MAVLINK_Init(void) {
	mav_statustext[0] = 0;
	MAVLINK_reset(0);
	actualbaudrateIdx=g_eeGeneral.mavbaud;
	#if defined(PCBSKY9X)	/* PCBSKY9X means SKY9X AND 9XRPRO !! */
		#if defined(REVX)
			telemetryPortInit(0);
			telemetrySecondPortInit(Index2Baud(g_eeGeneral.mavbaud));			
			//telemetrySecondPortInit(19200);		// ok
		#else
			telemetryPortInit(Index2Baud(g_eeGeneral.mavbaud));
		#endif
	#else
	SERIAL_Init();
	#endif
}

void telemetryWakeup() {
	/* RESET protocol activity status (* symbol) on display */
	static bool initialized = 0;
	uint16_t tmr10ms = get_tmr10ms();
	#if defined(PCBSKY9X)
	uint16_t count = tmr10ms & 0x02BC; // 700*10ms ==  7 SEC
	#else
	uint8_t count = tmr10ms & 0x0f; // 15*10ms
	#endif	  

	if (!count) {
	#if defined(PCBSKY9X)
		mav_heartbeat=0;	/* reset counter */
	#else
		if(!initialized) {
			MAVLINK_reset(1);
			SERIAL_Init();
			initialized = 1;
		}
//		if (mav_heartbeat > -30) {
//			// TODO mavlink_system.sysid = g_eeGeneral.mavTargetSystem;
//			mav_heartbeat--;
//
//			if (mav_heartbeat == -30) {
//				MAVLINK_reset(1);
//				SERIAL_Init();
//			}
////			SERIAL_startTX();
//		}
	#endif	  
	  
	  
	  }
	/* --------------------------------- */
	
	#if defined(PCBSKY9X)
		#if defined(REVX)
			if (actualbaudrateIdx!=g_eeGeneral.mavbaud) {
			  telemetrySecondPortInit(Index2Baud(g_eeGeneral.mavbaud));
			  actualbaudrateIdx=g_eeGeneral.mavbaud;
			  }
			uint8_t data;
			while (telemetrySecondPortReceive(data)) {
			  processSerialData(data);
			}	
		#else
			rxPdcUsart(processSerialData);
		#endif
	#endif
}

uint32_t Index2Baud(uint8_t mavbaudIdx)
{
	switch (mavbaudIdx) {
	  //"4800  ""9600  ""14400 ""19200 ""38400 ""57600 ""76800 ""115200"
	  case 0:
		return 4800;
	  case 1:
		return 9600;
	  case 2:
		return 14400;
	  case 3:
		return 19200;
	  case 4:
		return 38400;
	  case 5:
		return 57600;
	  case 6:
		return 76800;
	  case 7:
		return 115200;
	  default:
		return 4800;
	  }
}
/*!	\brief Mavlink message parser
 *	\details Parses the characters in a mavlink message.
 *	Case statement parses each character as it is received.
 *	\attention One big change form the 0.9 to 1.0 version is the
 *	MAVLINK_CRC_EXTRA. This requires the mavlink_message_crcs array of 256 bytes.
 *	\todo create dot for the statemachine
 */
static void processSerialData(uint8_t c) {

	//! The currently decoded message
	mavlink_message_t* p_rxmsg = mavlink_get_channel_buffer(MAVLINK_COMM_0);
	//! The current decode status
	mavlink_status_t* p_status = mavlink_get_channel_status(MAVLINK_COMM_0);

	
#if defined(BLUETOOTH)
  // TODO if (g_model.bt_telemetry)
  btPushByte(c);
#endif

    if(mavlink_parse_char(MAVLINK_COMM_0, c, p_rxmsg, p_status)) {
    	handleMessage(p_rxmsg);
    }
  
}

/*!	\brief Status log message
 *	\details Processes the mavlink status messages. This message contains a
 *	severity and a message. The severity is an enum defined by MAV_SEVERITY also
 *	see RFC-5424 for the severity levels.
 *	The original message is maximum 50 characters and is without termination
 *	character. For readablity on the 9x the only the first 15 (LEN_STATUSTEXT)
 *	characters are used. To get the full message you can use the commented
 *	funtion below.
 */

static inline void REC_MAVLINK_MSG_ID_STATUSTEXT(const mavlink_message_t* msg) {
	msg_severity = mavlink_msg_statustext_get_severity(msg);
	_MAV_RETURN_char_array(msg, mav_statustext, LEN_STATUSTEXT,  1);
	msg_timestamp = get_tmr10ms();
}

/*!	\brief System status including cpu load, battery status and communication status.
 *	\details From this message we use use only the battery information. The rest
 *	is not realy of use while flying. The complete message can be decoded in to
 *	a struct with the first two commented lines.
 *  The battery votage is in mV. We devide by 100 to display tenths of volts.'
 *	\todo Add battery remaining variable in telemetry_data struct for estimated
 *	remaining battery. Decoding function already in place.
 */

static inline void REC_MAVLINK_MSG_ID_SYS_STATUS(const mavlink_message_t* msg) {
	telemetry_data.pwrbat = ((uint32_t)mavlink_msg_sys_status_get_voltage_battery(msg) *
						mavlink_msg_sys_status_get_current_battery(msg)) / 100000;

	telemetry_data.vbat = mavlink_msg_sys_status_get_voltage_battery(msg) / 100; // Voltage * 10
	telemetry_data.ibat = mavlink_msg_sys_status_get_current_battery(msg) / 10;
	telemetry_data.rem_bat = mavlink_msg_sys_status_get_battery_remaining(msg);

#ifdef MAVLINK_PARAMS
	telemetry_data.vbat_low = (getMavlinParamsValue(BATT_MONITOR) > 0)
					&& (((float) telemetry_data.vbat / 10.0) < getMavlinParamsValue(LOW_VOLT)) && (telemetry_data.vbat > 50);
#else
	telemetry_data.vbat_low = (telemetry_data.rem_bat < 10);
#endif
}

/*!	\brief Receive rc channels
 *
 */
static inline void REC_MAVLINK_MSG_ID_RC_CHANNELS_RAW(const mavlink_message_t* msg) {

}

/*!	\brief Arducopter specific radio message
 *
 */
static inline void REC_MAVLINK_MSG_ID_RADIO(const mavlink_message_t* msg) {
	if (msg->sysid != 51)		// ArduPilot/Arducopter customization
		return;

	telemetry_data.packet_drop = mavlink_msg_radio_get_rxerrors(msg);
	telemetry_data.packet_fixed = mavlink_msg_radio_get_fixed(msg);
	telemetry_data.radio_sysid = msg->sysid;
	telemetry_data.radio_compid = msg->compid;
}
static inline void REC_MAVLINK_MSG_ID_RADIO_STATUS(const mavlink_message_t* msg) {
	REC_MAVLINK_MSG_ID_RADIO(msg);
}

//! \brief Navigation output message
static inline void REC_MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT(const mavlink_message_t* msg) {
	telemetry_data.bearing = mavlink_msg_nav_controller_output_get_target_bearing(msg);
	telemetry_data.wp_dist = mavlink_msg_nav_controller_output_get_wp_dist(msg);
}

//! \brief Hud navigation message
static inline void REC_MAVLINK_MSG_ID_VFR_HUD(const mavlink_message_t* msg) {
	telemetry_data.heading = mavlink_msg_vfr_hud_get_heading(msg);
	telemetry_data.throttle = mavlink_msg_vfr_hud_get_throttle(msg);
	telemetry_data.loc_current.rel_alt = lround(mavlink_msg_vfr_hud_get_alt(msg)*10);
}

/*!	\brief Heartbeat message
 *	\details Heartbeat message is used for the following information:
 *	type and autopilot is used to determine if the UAV is an ArduPlane or Arducopter
 */
static inline void REC_MAVLINK_MSG_ID_HEARTBEAT(const mavlink_message_t* msg) {
	uint8_t type = mavlink_msg_heartbeat_get_type(msg);
	uint8_t autopilot = mavlink_msg_heartbeat_get_autopilot(msg);

	if (autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA) {
		telemetry_data.type = mavlink_msg_heartbeat_get_type(msg);
		telemetry_data.autopilot = mavlink_msg_heartbeat_get_autopilot(msg);

		telemetry_data.mav_sysid = msg->sysid;
		telemetry_data.mav_compid = msg->compid;
		telemetry_data.mode = mavlink_msg_heartbeat_get_base_mode(msg);
		telemetry_data.custom_mode = mavlink_msg_heartbeat_get_custom_mode(msg);
		telemetry_data.status = mavlink_msg_heartbeat_get_system_status(msg);

		if (type == MAV_TYPE_QUADROTOR || type == MAV_TYPE_COAXIAL
				|| type == MAV_TYPE_HELICOPTER
				|| type == MAV_TYPE_HEXAROTOR
				|| type == MAV_TYPE_OCTOROTOR
				|| type == MAV_TYPE_TRICOPTER) {
			telemetry_data.type_autopilot = MAVLINK_ARDUCOPTER;
		} else if (type == MAV_TYPE_FIXED_WING) {
			telemetry_data.type_autopilot = MAVLINK_ARDUPLANE;
		} else {
			telemetry_data.type_autopilot = MAVLINK_INVALID_TYPE;
		}


		telemetry_data.armed =
				(telemetry_data.mode & MAV_MODE_FLAG_SAFETY_ARMED) ?
						true : false;
		mav_heartbeat = get_tmr10ms();
	}

	//TODO: Process other types, gimbal... etc
}

static inline void REC_MAVLINK_MSG_ID_HIL_CONTROLS(const mavlink_message_t* msg) {
	telemetry_data.nav_mode = mavlink_msg_hil_controls_get_mode(msg);
}

/*!	\brief Process GPS raw integer message
 *	\details This message contains the following data:
 *		- fix type: 0-1: no fix, 2: 2D fix, 3: 3D fix.
 *		- Latitude, longitude in 1E7 * degrees
 *		- Altutude 1E3 * meters above MSL.
 *		- Course over ground in 100 * degrees
 *		- GPS HDOP horizontal dilution of precision in cm (m*100).
 *		- Ground speed in m/s * 100
 */
static inline void REC_MAVLINK_MSG_ID_GPS_RAW_INT(const mavlink_message_t* msg) {
	telemetry_data.fix_type = mavlink_msg_gps_raw_int_get_fix_type(msg);
	telemetry_data.loc_current.lat = mavlink_msg_gps_raw_int_get_lat(msg) / 1E7;
	telemetry_data.loc_current.lon = mavlink_msg_gps_raw_int_get_lon(msg) / 1E7;
	telemetry_data.loc_current.gps_alt = mavlink_msg_gps_raw_int_get_alt(msg) / 100;
	telemetry_data.hdop = mavlink_msg_gps_raw_int_get_eph(msg) / 10;
	telemetry_data.course = mavlink_msg_gps_raw_int_get_cog(msg) / 100;
	telemetry_data.v = mavlink_msg_gps_raw_int_get_vel(msg);
	telemetry_data.satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(msg);
}

static inline void REC_MAVLINK_MSG_ID_GLOBAL_POSITION_INT(const mavlink_message_t* msg) {

}

#ifdef MAVLINK_PARAMS
const pm_char *getParamId(uint8_t idx) {
	const pm_char *mav_params_id [((NB_PID_PARAMS / 2) + 4)]  = {
		PSTR("RATE_YAW"), // Rate Yaw
		PSTR("STB_YAW"), // Stabilize Yaw
		PSTR("RATE_PIT"), // Rate Pitch
		PSTR("STB_PIT"), // Stabilize Pitch
		PSTR("RATE_RLL"), // Rate Roll
		PSTR("STB_RLL"), // Stabilize Roll
		PSTR("THR_ALT"), // PSTR("THR_BAR"), // Altitude Hold
		PSTR("HLD_LON"), // Loiter
		PSTR("HLD_LAT"), // Loiter
		PSTR("NAV_LON"), // Nav WP
		PSTR("NAV_LAT"), // Nav WP
		PSTR("LOW_VOLT"), // Battery low voltage
		PSTR("VOLT_DIVIDER"), //
		PSTR("BATT_MONITOR"), //
		PSTR("BATT_CAPACITY")
	};
	uint8_t i;
	if (idx < NB_PID_PARAMS) {
		i = idx / 2;
	} else {
		i = idx - (NB_PID_PARAMS / 2);
	}
	return mav_params_id[i];
}

void setMavlinParamsValue(uint8_t idx, float val) {
	MavlinkParam_t *param = getParam(idx);
	if (idx < NB_PARAMS && val != param->value) {
		param->value = val;
		param->repeat = PARAM_NB_REPEAT;
		uint8_t link_idx = NB_PID_PARAMS;
		switch (idx) {
		case RATE_PIT_P:
		case RATE_PIT_I:
		case STB_PIT_P:
		case STB_PIT_I:
			link_idx = idx + 4;
			break;
		case RATE_RLL_P:
		case RATE_RLL_I:
		case STB_RLL_P:
		case STB_RLL_I:
			link_idx = idx - 4;
			break;
		case HLD_LON_P:
		case HLD_LON_I:
		case NAV_LON_P:
		case NAV_LON_I:
			link_idx = idx + 2;
			break;
		case HLD_LAT_P:
		case HLD_LAT_I:
		case NAV_LAT_P:
		case NAV_LAT_I:
			link_idx = idx - 2;
			break;
		default:
			break;
		}
		if (link_idx < NB_PID_PARAMS) {
			MavlinkParam_t *p = getParam(link_idx);
			p->value = val;
			p->repeat = PARAM_NB_REPEAT;
		}
	}
}

void putsMavlinParams(uint8_t x, uint8_t y, uint8_t idx, uint8_t att) {
	if (idx < NB_PARAMS) {
		const pm_char * s = getParamId(idx);
		char c;
		while ((c = pgm_read_byte(s++))) {
			lcd_putcAtt(x, y, (c == '_' ? ' ' : c), 0);
			x += FW;
		}
		if (idx < NB_PID_PARAMS) {
			x = 11 * FW;
			lcd_putcAtt(x, y, "PI"[idx & 0x01], att);
		}
	}
}

static inline void setParamValue(int8_t *id, float value) {
	int8_t *p_id;
	for (int8_t idx = 0; idx < NB_PARAMS; idx++) {
		const pm_char * s = getParamId(idx);
		p_id = id;
		while (1) {
			char c1 = pgm_read_byte(s++);
			if (!c1) {
				// Founded !
				uint8_t founded = !*p_id;
				if (idx < NB_PID_PARAMS) {
					p_id++;
					switch (*p_id++) {
					case 'P':
						founded = !*p_id;
						break;
					case 'I':
						founded = !*p_id;
						idx++;
						break;
					default:
						founded = 0;
						break;
					}
				}
				// test end of string char == 0 and a valid PI
				if (founded) {
					MavlinkParam_t *param = getParam(idx);
					param->repeat = 0;
					param->valid = 1;
					param->value = value;
					mav_req_params_nb_recv++;
				}
				return;
			} else if (c1 != *p_id++) {
				break;
			}
		}
		if (idx < NB_PID_PARAMS) {
			// Skip I Parameter from PID
			idx++;
		}
	}
}

static inline void REC_MAVLINK_MSG_ID_PARAM_VALUE(const mavlink_message_t* msg) {
	mavlink_param_value_t param_value;
	mavlink_msg_param_value_decode(msg, &param_value);
	char *id = param_value.param_id;
	setParamValue((int8_t*)id, param_value.param_value);
	data_stream_start_stop = 0; // stop data stream while getting params list
	watch_mav_req_params_list = mav_req_params_nb_recv < (NB_PARAMS - 5) ? 20 : 0; // stop timeout
}
#endif


#ifdef MAVLINK_PARAMS
static inline void MAVLINK_msg_param_request_list_send() {
	mavlink_channel_t chan = MAVLINK_COMM_0;
	mavlink_msg_param_request_list_send(chan, mavlink_system.sysid, mavlink_system.compid);
}

static inline void MAVLINK_msg_param_set(uint8_t idx) {
	const pm_char* s = getParamId(idx);
	int8_t buf[15];
	int8_t *p = buf;
	while (1) {
		char c = pgm_read_byte(s++);
		if (!c) {
			if (idx < NB_PID_PARAMS) {
				*p++ = '_';
				uint8_t colIdx = idx & 0x01;
				*p++ = "PI"[colIdx];
			}
			*p++ = 0;
			break;
		}
		*p++ = c;
	}
	//float param_value = ((float) telemetry_data.params[idx].pi_param[subIdx].pi_value / 100.00 + 0.005);
	float param_value = getParam(idx)->value;

	mavlink_channel_t chan = MAVLINK_COMM_0;
	uint8_t param_type = (uint8_t)MAV_PARAM_TYPE_REAL32;
	const char* param_id = (const char*)buf;
	mavlink_msg_param_set_send(chan, mavlink_system.sysid, mavlink_system.compid, param_id, param_value, param_type);

}
#endif

static inline void MAVLINK_msg_request_data_stream_pack_send(uint8_t req_stream_id, uint16_t req_message_rate,
				uint8_t start_stop) {
	mavlink_channel_t chan = MAVLINK_COMM_0;
	mavlink_msg_request_data_stream_send(chan, mavlink_system.sysid, mavlink_system.compid, req_stream_id, req_message_rate,
					start_stop);
}

//! \brief old mode switch funtion
static inline void MAVLINK_msg_action_pack_send(uint8_t action) {
//	mavlink_channel_t chan = MAVLINK_COMM_0;
//	mavlink_msg_action_send(chan, mavlink_system.sysid, mavlink_system.compid, action);
}

//! \brief old mode switch funtion
static inline void MAVLINK_msg_set_mode_send(uint8_t mode) {
	mavlink_channel_t chan = MAVLINK_COMM_0;
	mavlink_msg_set_mode_send(chan, mavlink_system.sysid, mode, 0);
}

void mavlink_handle_custom_data(mavlink_message_t* p_rxmsg) {
	uint8_t type = mavlink_msg_data16_get_type(p_rxmsg);
	uint8_t tmp[16];
	switch(type) {
	case RADIO_STATUS:
		mavlink_msg_data16_get_data(p_rxmsg, tmp);
		memcpy((uint8_t*)&radioStatus, tmp, sizeof(RadioStatus));
	break;
	case CURRENT_RADIO_CONFIGURATION:
		mavlink_msg_data16_get_data(p_rxmsg, tmp);
		RadioConfiguration* p_conf = (RadioConfiguration*)tmp;

		if(radioCfg.frequency != 0 &&
				memcmp(p_conf, (uint8_t*)&radioCfg, sizeof(RadioConfiguration)) != 0) {
			//configuration changed
			mavlink_msg_data16_send(MAVLINK_COMM_0, SET_RADIO_CONFIGURATION,
					sizeof(RadioConfiguration), (uint8_t*)&radioCfg);

		} else {

			memcpy((uint8_t*)&radioCfg, tmp, sizeof(RadioConfiguration));
		}



	break;
	}

}

static inline void handleMessage(mavlink_message_t* p_rxmsg) {
	switch (p_rxmsg->msgid) {
	case MAVLINK_MSG_ID_HEARTBEAT:
		REC_MAVLINK_MSG_ID_HEARTBEAT(p_rxmsg);
		break;
	case MAVLINK_MSG_ID_STATUSTEXT:
		REC_MAVLINK_MSG_ID_STATUSTEXT(p_rxmsg);
		AUDIO_WARNING1();
		break;
	case MAVLINK_MSG_ID_SYS_STATUS:
		REC_MAVLINK_MSG_ID_SYS_STATUS(p_rxmsg);
		break;
	case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
		REC_MAVLINK_MSG_ID_RC_CHANNELS_RAW(p_rxmsg);
		break;
	case MAVLINK_MSG_ID_RADIO:
		REC_MAVLINK_MSG_ID_RADIO(p_rxmsg);
		break;
	case MAVLINK_MSG_ID_RADIO_STATUS:
		REC_MAVLINK_MSG_ID_RADIO_STATUS(p_rxmsg);
		break;
	case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
		REC_MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT(p_rxmsg);
		break;
	case MAVLINK_MSG_ID_VFR_HUD:
		REC_MAVLINK_MSG_ID_VFR_HUD(p_rxmsg);
		break;
	case MAVLINK_MSG_ID_HIL_CONTROLS:
		REC_MAVLINK_MSG_ID_HIL_CONTROLS(p_rxmsg);
		break;
	case MAVLINK_MSG_ID_GPS_RAW_INT:
		REC_MAVLINK_MSG_ID_GPS_RAW_INT(p_rxmsg);
		break;
	case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
		REC_MAVLINK_MSG_ID_GLOBAL_POSITION_INT(p_rxmsg);
		break;
	case MAVLINK_MSG_ID_BATTERY_STATUS:
		telemetry_data.current_consumed = mavlink_msg_battery_status_get_current_consumed(p_rxmsg);
		_MAV_RETURN_uint16_t_array(p_rxmsg, telemetry_data.cells_v, 4,  10);

		break;
	case MAVLINK_MSG_ID_DATA16:
		mavlink_handle_custom_data(p_rxmsg);
		break;
#ifdef MAVLINK_PARAMS
	case MAVLINK_MSG_ID_PARAM_VALUE:
		REC_MAVLINK_MSG_ID_PARAM_VALUE(p_rxmsg);
		break;
#endif

	}

}


/*!	\brief Looks like some kind of task switcher on a timer
 *	\todo Figure out where this was used for and intergrate in current
 *	implemnetation. Function disabled without any side affects.
 */
#if 0
 void MAVLINK10mspoll(uint8_t count) {
	switch (count) {
	case 2: // MAVLINK_MSG_ID_ACTION
		if (watch_mav_req_id_action > 0) {
			watch_mav_req_id_action--;
			// Repeat  is not ack : 150ms*0x07
			if ((watch_mav_req_id_action & 0x07) == 0) {
				uint8_t action = MAVLINK_CtrlMode2Action(telemetry_data.req_mode);
				MAVLINK_msg_action_pack_send(action);
				char *ptr = mav_statustext;
				*ptr++ = 'R';
				*ptr++ = 'Q';
				*ptr++ = ' ';
				*ptr++ = action / 10 + '0';
				*ptr++ = action % 10 + '0';
				*ptr++ = 0;
			}
		}
		if (telemetry_data.ack_result < 5) {
			if (telemetry_data.ack_result > 0) {
				telemetry_data.ack_result++;
			}
		}
		break;
#ifdef MAVLINK_PARAMS
	case 4: // MAVLINK_MSG_ID_PARAM_REQUEST_LIST
		if (watch_mav_req_params_list > 0) {
			watch_mav_req_params_list--;
			if (watch_mav_req_params_list == 0) {
				mav_req_params_nb_recv = 0;
				MAVLINK_msg_param_request_list_send();
				watch_mav_req_params_list = 20;
			}
		}
		break;
#endif
	case 6: // MAVLINK_MSG_ID_REQUEST_DATA_STREAM
		if (watch_mav_req_start_data_stream > 0) {
			watch_mav_req_start_data_stream--;
			if (watch_mav_req_start_data_stream == 0) {
				uint8_t req_stream_id = 2;
				uint16_t req_message_rate = 1;

				MAVLINK_msg_request_data_stream_pack_send(req_stream_id, req_message_rate, data_stream_start_stop);
				watch_mav_req_start_data_stream = 20;
				data_stream_start_stop = 1; // maybe start next time
			}
		}
		break;
#ifdef MAVLINK_PARAMS
	case 8: // MAVLINK_MSG_ID_PARAM_SET
		if (watch_mav_req_params_set > 0) {
			watch_mav_req_params_set--;
			if (watch_mav_req_params_set == 0) {
				for (uint8_t idx = 0; idx < NB_PARAMS; idx++) {
					if (getParam(idx)->repeat) {
						getParam(idx)->repeat--;
						MAVLINK_msg_param_set(idx);
						watch_mav_req_params_set = 3; // 300ms
						return;
					}
				}
			}
		}
		break;
#endif
	default:
		return;
	}
}
#endif
