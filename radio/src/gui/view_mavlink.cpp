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
 
/*!	\file view_mavlink.cpp
 *	Mavlink menu
 *	Contains the menu specific code for Mavlink support.
 */

#include "gui/view_mavlink.h"

// Globals declaration
 
/*!	\brief Top Mavlink Menu definition
 *	\details Registers button events and handles that info. Buttons select menus,
 *	these are launched from the MAVLINK_menu switch statement. Setup menu is
 *	lanuched by the menu button. On exit (with exit button) the mavlink
 *	extension is reinitialized.
 */
void menuTelemetryMavlink(uint8_t event) {
	
	switch (event) // new event received, branch accordingly
	{
	case EVT_ENTRY:
		MAVLINK_menu = MENU_MODE;
		break;

	case EVT_KEY_FIRST(KEY_UP):
		if (MAVLINK_menu > 0)
		{
			MAVLINK_menu--;
			break;
		}
		else
		{
			chainMenu(menuMainView);
			return;
		}
	case EVT_KEY_FIRST(KEY_DOWN):
		if (MAVLINK_menu < MAX_MAVLINK_MENU - 1)
			MAVLINK_menu++;
		break;
	case EVT_KEY_FIRST(KEY_MENU):
		return;
	case EVT_KEY_FIRST(KEY_EXIT):
		chainMenu(menuMainView);
		return;
	}

	switch (MAVLINK_menu) {
	case MENU_INFO:
		menuTelemetryMavlinkInfos();
		break;
	case MENU_MODE:
		menuTelemetryMavlinkFlightMode();
		break;
	case MENU_BATT:
		menuTelemetryMavlinkBattery();
		break;
	case MENU_NAV:
		menuTelemetryMavlinkNavigation();
		break;
	case MENU_GPS:
		menuTelemetryMavlinkGPS();
		break;
	case MENU_RADIO:
		menuTelemetryMavlinkRadio();
		break;
#ifdef DUMP_RX_TX
	case MENU_DUMP_RX:
	case MENU_DUMP_DIAG:
		menuTelemetryMavlinkDump(event);
		break;
#endif
	default:
		break;
	}

	showStatusMessage(event);

}

void showStatusMessage(uint8_t event) {
	uint8_t x,y;
	x = 0;
	y = FH*2+3;

	if(*mav_statustext != 0) { // low severity message will show for 3s
		if(msg_severity < SEVERITY_MEDIUM) {
			if(get_tmr10ms() - msg_timestamp > 300) {
				*mav_statustext = 0;
			}
		} else {
			if(get_tmr10ms() - msg_timestamp > 2000) { //else show for 20s
				*mav_statustext = 0;
			}
		}
		//or dismiss manually
		if(event == EVT_KEY_FIRST(KEY_LEFT) || event == EVT_KEY_FIRST(KEY_RIGHT)) {
			*mav_statustext = 0;
		}
	}

	if(*mav_statustext != 0) {

		uint8_t phase = ((get_tmr10ms() % 128)<64);
		lcd_filled_rect(x, y-3, LCD_W, FH+6, 255, phase?ERASE:FORCE);
		lcd_rect(x, y-3, LCD_W, FH+6);
		x+=2;
		lcd_putsAtt(x, y, mav_statustext, BSS|(phase?0:INVERS));
	}
}

/*!	\brief Float variable display helper
 *	\param x x position on the screen
 *	\param y y position on the screen
 *	\param val value to display
 *	\param percis precision to display
 *	\param mode Use one of the defines in lcd.h line 81
 *	\details The maximum value is 9999. The position is the position of the ones, see below for explanation "*" marks
 *	the x position.
 *	\verbatim
          *
       9999.9999 \endverbatim
 */
void lcd_outdezFloat(uint8_t x, uint8_t y, float val, uint8_t precis, uint8_t mode) {
	char c;
	int16_t lnum = val;
	uint8_t x1 = x;
	val -= lnum;
	uint8_t xinc = FWNUM;
	if (mode & DBLSIZE)
		xinc *= 2;

	int8_t i = 0;
	lnum = abs(lnum);
	for (; i < 4; i++) {
		c = (lnum % 10) + '0';
		x1 -= xinc;
		lcd_putcAtt(x1, y, c, mode);
		lnum /= 10;
		if (lnum == 0) {
			break;
		}
	}
	if (lnum != 0) {
		// Error number too big
		x1 = x;
		for (i = 0; i < 4; i++) {
			x1 -= FW;
			lcd_putcAtt(x1, y, '?', mode);
		}
	} else {
		if (val < 0) {
			val = -val;
			x1 -= xinc;
			lcd_putcAtt(x1, y, '-', mode);
		}
		if (precis)
		{
			uint8_t y_temp = y;
			if (mode & DBLSIZE)
				y_temp += FH;
			lcd_putcAtt(x, y_temp, '.', (mode & (~DBLSIZE)));
			x -= (xinc / 2);
		}
		for (i = 0; i < precis; i++) {
			val *= 10;
			int a = val;
			c = a + '0';
			x += xinc;
			lcd_putcAtt(x, y, c, mode);
			val -= a;
		}
	}
}

/*!	\brief Fightmode string printer
 *	\details Decodes the flight mode from Mavlink custom mode enum to a string.
 *	This funtion can handle ArduPilot and ArduCoper code.
 *	To support new autopilot pilots add a STR_MAVLINK_... to the translations,
 *	and if requred a lut (see arduplane for examle) if there are unused modes
 *	in the sequence.
 */

void print_mav_mode(uint8_t x, uint8_t y, uint32_t custom_mode, uint8_t attr) //, const char * mode_text_p)
{
	uint8_t mode = (uint8_t) custom_mode;
	switch (telemetry_data.type_autopilot) {
	case MAVLINK_ARDUCOPTER:
		lcd_putsiAtt(x,y,STR_MAVLINK_AC_MODES,mode,attr);
		break;
	case MAVLINK_ARDUPLANE:
		lcd_putsiAtt(x,y,STR_MAVLINK_AP_MODES,ap_modes_lut[custom_mode],attr);
		break;
	default:
		lcd_putsAtt (FW, y, PSTR("INV. MAV TYPE"), attr);
		break;
	}
}

/*!	\brief Menu header
 *	\details Small helper function to print the standard header on the screen.
 */
void mav_title(const pm_char * s, uint8_t index)
{
  lcd_putsAtt(0, 0, PSTR("MAVLINK"), INVERS);
  lcd_puts(10 * FW, 0, s);
  displayScreenIndex(index, MAX_MAVLINK_MENU, INVERS);
  #if defined(PCBSKY9X)
  lcd_putc(7 * FW, 0, mav_heartbeat+'0');	/* ok til 9 :-) */
  #else

  lcd_putc(7 * FW, 0, ((get_tmr10ms() - mav_heartbeat) < 50) ? '*' : ' ');
  #endif
  lcd_putc(8 * FW, 0, telemetry_data.armed ? 'A' : 'N');
}



void menuTelemetryMavlinkRadio() {
	mav_title(PSTR("RADIO"), MAVLINK_menu);

	uint8_t x, ynum;
	x = 0;
	ynum = FH;

	char buf[24];
	snprintf_P(buf, sizeof(buf), PSTR("RSSI  %d/%d dBm"), radioStatus.rssi, radioStatus.remRssi);
	lcd_putsAtt(x, ynum + FH, buf, BSS);
//
	ynum += FH;
	snprintf_P(buf, sizeof(buf), PSTR("Noise %d/%d dBm"), radioStatus.noise, radioStatus.remNoise);
	lcd_putsAtt(x, ynum + FH, buf, BSS);

	ynum += 2*FH;
	snprintf_P(buf, sizeof(buf), PSTR("TX %u (%u)"), radioStatus.txGood, radioStatus.txBad);
	lcd_putsAtt(x, ynum + FH, buf, BSS);

	ynum += FH;
	snprintf_P(buf, sizeof(buf), PSTR("RX %u (%u)"), radioStatus.rxGood, radioStatus.rxBad);
	lcd_putsAtt(x, ynum + FH, buf, BSS);
}
/*!	\brief Global info menu
 *	\details Quick status overview menu. The menu should contain current mode, 
 *	armed | disarmed, battery status and RSSI info. Menu must be clean and
 *	readable with a quick glance.
 *	\todo Make menu as described as above.
 */
void menuTelemetryMavlinkInfos(void) {

	mav_title(STR_MAVLINK_INFOS, MAVLINK_menu);

	uint8_t x1, x2, xnum, y;
	x1 = FW;
	x2 = 7 * FW;
	xnum = x2 + 8 * FWNUM;
	y = FH;
/*
	char * ptr = mav_statustext;
	for (uint8_t j = 0; j < LEN_STATUSTEXT; j++) {
		if (*ptr == 0) {
			lcd_putc(x1, y, ' ');
		} else {
			lcd_putc(x1, y, *ptr++);
		}
		x1 += FW;
	}
	x1 = FW;
	y += FH;
*/
	mavlink_status_t* p_status = mavlink_get_channel_status(MAVLINK_COMM_0);

	if (telemetry_data.status) {

		lcd_putsnAtt(x1, y, STR_MAVLINK_MODE, 4, 0);
		if (telemetry_data.armed)
			lcd_putsnAtt(x2, y, PSTR("A"), 1, 0);
		lcd_outdezAtt(xnum, y, telemetry_data.mode, 0);

		y += FH;
		lcd_puts(x1, y, PSTR("BATT"));
		lcd_outdezNAtt(xnum, y, telemetry_data.vbat, PREC1, 5);
		y += FH;
		lcd_puts(x1, y, PSTR("PKT DROP"));
		lcd_outdezAtt(xnum, y, p_status->packet_rx_drop_count + p_status->parse_error, 0);
		y += FH;
		lcd_puts(x1, y, PSTR("PKT REC"));
		lcd_outdezAtt(xnum, y, p_status->packet_rx_success_count, 0);		/* TODO use correct var */
		y += FH;
		lcd_puts(x1, y, PSTR("MAV Comp"));
		lcd_outdezAtt(xnum, y, telemetry_data.mav_compid, 0);
		y += FH;
		lcd_puts(x1, y, PSTR("MAV Sys"));
		lcd_outdezAtt(xnum, y, telemetry_data.mav_sysid, 0);
		y += FH;
		lcd_puts(x1, y, PSTR("Rad Comp"));
		lcd_outdezAtt(xnum, y, telemetry_data.radio_compid, 0);
		y += FH;
		lcd_puts(x1, y, PSTR("Rad Sys"));
		lcd_outdezAtt(xnum, y, telemetry_data.radio_sysid, 0);
		
	}
}

/*!	\brief Flight mode menu
 *	\details Clear display of current flight mode.
 *	\todo Add functionality to change flight mode.
 */
void menuTelemetryMavlinkFlightMode() {
	
	//mav_title(STR_MAVLINK_MODE, MAVLINK_menu);
	displayScreenIndex(MAVLINK_menu, MAX_MAVLINK_MENU, INVERS);

	uint8_t x, y;
	x = 0;
	y = 0;


	lcd_putcAtt (FW*4, y, 'V', 0);
	lcd_outdezAtt(FW*4, y, telemetry_data.vbat, PREC1); //bat



	if(get_tmr10ms() % 256 < 128) {
		lcd_putcAtt (FW*10, y, 'A', 0);
		lcd_outdezAtt(FW*10, y, telemetry_data.ibat, PREC1); //bat

		lcd_putcAtt (FW*14, y, '%', 0);
		lcd_outdezAtt(FW*14, y, telemetry_data.rem_bat, 0); //bat
	} else {

		lcd_putsAtt (FW*10, y, PSTR("mAh"), 0);
		lcd_outdezAtt(FW*10, y, telemetry_data.current_consumed, 0); //bat

	}


	lcd_putc(FW * 16, 0, ((get_tmr10ms() - mav_heartbeat) < 50) ? '*' : ' ');


	y = FH;
	
	uint16_t hb_time = get_tmr10ms() - mav_heartbeat;
	if(mav_heartbeat != 0 && hb_time > 300) {

		if(!telemetry_data.link_lost && (get_tmr10ms() % 128) > 64) {
			telemetry_data.link_lost = 1;
			AUDIO_WARNING2();

		} else if(telemetry_data.link_lost && (get_tmr10ms() % 128) < 64) {
			telemetry_data.link_lost = 0;
		}

		lcd_putsAtt(FW, y, PSTR("LINK LOSS"), DBLSIZE);
		lcd_outdezAtt(lcdLastPos+5*FW, y, hb_time/10, DBLSIZE|PREC1);
	} else {
		print_mav_mode(FW, y, telemetry_data.custom_mode, DBLSIZE);
		telemetry_data.link_lost = 0;
	}
    y += 2 * FH;

	////// status text

	
    if (telemetry_data.armed) {
    	lcd_putsAtt (FW, y, PSTR(" Armed  "), INVERS);
    } else {
    	lcd_putsAtt (FW, y, PSTR("Disarmed"), 0);
    }
    y+= FH;
    lcd_putsAtt (FW, y, PSTR("GPS: "), 0);
    switch(telemetry_data.fix_type) {
    case 0:
    	lcd_putsAtt (lcdNextPos, y, PSTR("NoHw"), INVERS);
    	break;
    case 1:
    	lcd_putsAtt (lcdNextPos, y, PSTR("NoFix"), INVERS);
    	break;
    case 2:
    	lcd_putsAtt (lcdNextPos, y, PSTR("2DFix"), 0);
    	break;
    case 3:
    	lcd_putsAtt (lcdNextPos, y, PSTR("3DFix"), 0);
    	break;
    case 4:
    	lcd_putsAtt (lcdNextPos, y, PSTR("3D+DGPS"), 0);
    	break;
    }

    y+= FH;
    //snprintf_P(buf, sizeof(buf), PSTR("SATS:%02d HDOP:"), 2);

    lcd_putsAtt (FW, y, PSTR("SATS:"), 0);
    lcd_outdezAtt(8*FW, y, telemetry_data.satellites_visible, 0); //sats

    lcd_putsAtt (9*FW, y, PSTR("HDOP:"), 0);
    lcd_outdezAtt(13*FW, y, telemetry_data.hdop, PREC1|LEFT); //hdop

    y+= FH;
    lcd_putsAtt (FW, y, PSTR("THR%:"), 0);
    lcd_outdezAtt(8*FW, y, telemetry_data.throttle, 0);

    lcd_putsAtt (9*FW, y, PSTR("WPdist:"), 0);
    lcd_outdezAtt(lcdNextPos, y, telemetry_data.wp_dist, LEFT);
    lcd_putsAtt (lcdNextPos, y, PSTR("m"), 0);


    y+= FH;
    lcd_putsAtt (0, y, PSTR("RSSI"), 0);
    lcd_outdezAtt(FW*7, y, radioStatus.rssi, 0);
    lcd_putsAtt (FW*7, y, PSTR("dBm"), 0);

    if(get_tmr10ms() % 256 < 128) {
    	 x = LCD_W-3*FW;
    	lcd_putsAtt (x-7*FW, y, PSTR("Spd"), 0);
		lcd_outdezAtt(x, y, (telemetry_data.v/10)*36/10, PREC1);
		lcd_putsAtt (x, y, PSTR("kmh"), 0);
    } else {
    	x = LCD_W-3*FW;
		lcd_putsAtt (x-7*FW, y, PSTR("Alt"), 0);
		lcd_outdezAtt(x, y, telemetry_data.loc_current.rel_alt, PREC1);
		lcd_putsAtt (x, y, PSTR("m"), 0);
    }
    {
    uint8_t totalh = FH*4;
    lcd_filled_rect(0, FH*3-FH/2, 4, totalh, 255, 0);
    totalh-=2;

    uint8_t h = (uint16_t)totalh * telemetry_data.throttle / 100;
    lcd_vline(1, FH*3-FH/2+1, totalh - h);
    lcd_vline(2, FH*3-FH/2+1, totalh - h);
    }

    {
		Line line({0, 0}, {0, -FW*2});
		line.rotate(DEG8(telemetry_data.course));
		line.translate({LCD_W-FW*2, FW*6});
		line.draw();
    }
    {
		Triangle t({0, -12}, {-4,6}, {4,6});
		t.rotate(DEG8(telemetry_data.heading));
		t.translate({LCD_W-FW*2, FW*6});
		t.fill();
    }

    lcd_rect(LCD_W-FW*4, FW*4, FW*4, FW*4, 255, ROUND);
    lcd_putc(LCD_W-FW*2-FW/2, FW*4-3, 'N');



}

/*!	\brief Batterystatus dislplay
 *	\details Shows flight batery status.
 *	Also RC and PC RSSI are in this menu. 
 */
void menuTelemetryMavlinkBattery(void) {
	
	mav_title(STR_MAVLINK_BAT_MENU_TITLE, MAVLINK_menu);
	
	uint8_t x, y, ynum;
	x = 7 * FWNUM;
//	x = xnum + 0 * FW;
	ynum = 1 * FH;
	y = 2 * FH;
	
	lcd_outdezAtt(x, ynum, telemetry_data.vbat, (DBLSIZE | PREC1 | UNSIGN));
	lcd_puts(x, y, PSTR("V"));
	x += 4 * (2 * FWNUM);
	lcd_outdezAtt(x, ynum, telemetry_data.ibat, (DBLSIZE | PREC1 | UNSIGN));
	lcd_puts(x, y, PSTR("A"));
	x += 4 * (2 * FWNUM);
	lcd_outdezAtt(x, ynum, telemetry_data.pwrbat, (DBLSIZE | UNSIGN));
	lcd_puts(x, y, PSTR("W"));

	y += FH+ FH/2;
	x = LCD_W - 1*FWNUM;
	lcd_outdezAtt(x,y, telemetry_data.cells_v[0]/10, PREC2);
	lcd_puts(x, y, PSTR("V"));
	y += FH;
	lcd_outdezAtt(x,y, telemetry_data.cells_v[1]/10, PREC2);
	lcd_puts(x, y, PSTR("V"));
	y += FH;
	lcd_outdezAtt(x,y, telemetry_data.cells_v[2]/10, PREC2);
	lcd_puts(x, y, PSTR("V"));
	y += FH;
	lcd_outdezAtt(x,y, telemetry_data.cells_v[3]/10, PREC2);
	lcd_puts(x, y, PSTR("V"));


	
	//lcd_outdezAtt(x, ynum, telemetry_data.rem_bat, (DBLSIZE | UNSIGN));
	//lcd_puts(x, y, PSTR("%"));

   // lcd_puts  (x, y, PSTR(""));
	ynum += 2 * FH;
	x = 7 * FWNUM;
	lcd_outdezAtt(x, ynum, telemetry_data.rem_bat, (DBLSIZE | UNSIGN));
	lcd_puts(x, ynum + FH, PSTR("%"));

	//ynum += 2 * FH;
	x = 14 * FWNUM;
	//lcd_outdezAtt(x, ynum, telemetry_data.battery_status.current_consumed, (UNSIGN));
	lcd_outdezAtt(x, ynum + FH, telemetry_data.current_consumed, UNSIGN);
	lcd_puts(x, ynum + FH, PSTR("mAh"));

	char buf[24];
	ynum += 2*FH;
	x = 0;

	snprintf_P(buf, sizeof(buf), PSTR("RSSI %d/%ddBm"), radioStatus.rssi, radioStatus.remRssi);
	lcd_putsAtt(x, ynum + FH, buf, BSS);
//
	ynum += FH;
	snprintf_P(buf, sizeof(buf), PSTR("Noise %d/%ddBm"), radioStatus.noise, radioStatus.remNoise);
	lcd_putsAtt(x, ynum + FH, buf, BSS);

//	if (g_model.mavlink.pc_rssi_en)
//	{
//		x += 8 * (2 * FWNUM);
//		lcd_puts(x, y, STR_MAVLINK_PC_RSSI_LABEL);
//		lcd_outdezAtt(x + 7 * FWNUM, ynum, telemetry_data.pc_rssi, (DBLSIZE));
//		lcd_puts(x + 7 * FWNUM, ynum + FH,  PSTR("%"));
//	}
    
}

/*!	\brief Navigation display
 *	\details Shows Navigation telemetry.
 *	Altitude in this menu is the relative (to the home location) altitude. This
 *	is the same altitude used by the waypoints.
 *	\todo Add a similar menu to fly back to the home location.
 */
void menuTelemetryMavlinkNavigation(void) {
	
	mav_title(STR_MAVLINK_NAV_MENU_TITLE, MAVLINK_menu);
	
	uint8_t x, y, ynum;
	x = 7 * FWNUM;
//	x = xnum + 0 * FW;
	ynum = 2 * FH;
	y = FH;
	
    
	x = 0;	
    lcd_puts  (x, y, STR_MAVLINK_COURSE);
	lcd_outdezAtt(x + 7 * FWNUM, ynum, telemetry_data.course, (DBLSIZE | UNSIGN));
	lcd_puts(x + 7 * FWNUM, ynum, PSTR("o"));



	x = 8 * (2 * FWNUM) + FWNUM;
	lcd_puts(x, y, STR_MAVLINK_HEADING);
	lcd_outdezAtt(x + 7 * FWNUM, ynum, telemetry_data.heading, (DBLSIZE | UNSIGN));
	lcd_puts(x + 7 * FWNUM, ynum,  PSTR("o"));
	y += 3 * FH;
	ynum += 3 * FH;
	
	x = 0;	
    lcd_puts  (x, y, STR_MAVLINK_BEARING);
	lcd_outdezAtt(x + 7 * FWNUM, ynum, telemetry_data.bearing, (DBLSIZE | UNSIGN));
	lcd_puts(x + 7 * FWNUM, ynum, PSTR("o"));

	x = 5 * (2 * FWNUM) - FWNUM;
    lcd_puts  (x, y, PSTR("Speed"));
    if(abs(telemetry_data.v) > 100) {
    	lcd_outdezAtt(x + 5 * FWNUM, ynum, ((telemetry_data.v/10)*36)/100, (DBLSIZE)); //convert to kmh
    } else {
    	lcd_outdezAtt(x + 5 * FWNUM, ynum, (telemetry_data.v/10)*36/10, (PREC1|DBLSIZE)); //convert to kmh
    }
	lcd_puts(x + 5 * FWNUM, ynum + FH,  PSTR("kmh"));

	x = 8 * (2 * FWNUM) + FWNUM;
    lcd_puts(x, y, STR_MAVLINK_ALTITUDE);
    if(abs(telemetry_data.loc_current.rel_alt > 1000)) {
    	lcd_outdezAtt(x + 8 * FWNUM - 3, ynum, telemetry_data.loc_current.rel_alt/10, (DBLSIZE));
    } else {
    	lcd_outdezAtt(x + 8 * FWNUM - 3, ynum, telemetry_data.loc_current.rel_alt, (PREC1|DBLSIZE));
    }
	lcd_puts(x + 8 * FWNUM - 3, ynum + FH,  PSTR("m"));
 
}



/*!	\brief GPS information menu
 *	\details Menu gives a lot of info from the gps like fix type, position,
 *	attitude, heading and velocity. Text is small and the user must focus to
 *	read it.
 *	\todo Text is small. Should we do something about this or leaf it like this.
 *	I don't think will be used much when a user is concentrated on flying.
 */
void menuTelemetryMavlinkGPS(void) {
	mav_title(STR_MAVLINK_GPS, MAVLINK_menu);

	uint8_t x1, x2, xnum, xnum2, y;
	x1 = FW;
	x2 = x1 + 12 * FW;
	xnum = 7 * FW + 3 * FWNUM;
	xnum2 = xnum + 11 * FWNUM;
	y = FH;

	lcd_putsnAtt(x1, y, STR_MAVLINK_GPS, 3, 0);
	if (telemetry_data.fix_type < 2) {
		lcd_putsnAtt(xnum, y, STR_MAVLINK_NO_FIX, 6, 0);
	} else {
		lcd_outdezNAtt(xnum, y, telemetry_data.fix_type, 0, 3);
		lcd_puts(xnum, y, PSTR("D"));
	}
	lcd_puts(x2, y, STR_MAVLINK_SAT);
	lcd_outdezNAtt(xnum2, y, telemetry_data.satellites_visible, 0, 2);

//	if (telemetry_data.fix_type > 0) {
	y += FH;
	lcd_puts(x1, y, STR_MAVLINK_HDOP);
	lcd_outdezAtt(xnum, y, telemetry_data.hdop, PREC1);

	y += FH;
	lcd_puts(x1, y, STR_MAVLINK_LAT);
	lcd_outdezFloat(xnum, y, telemetry_data.loc_current.lat, 2);

	lcd_putsnAtt(x2, y, STR_MAVLINK_LON, 3, 0);
	lcd_outdezFloat(xnum2, y, telemetry_data.loc_current.lon, 2);

	y += FH;
	lcd_putsnAtt(x1, y, STR_MAVLINK_ALTITUDE, 3, 0);
	lcd_outdezAtt(xnum, y, telemetry_data.loc_current.gps_alt, 0);

	y += FH;
	lcd_putsnAtt(x1, y, STR_MAVLINK_COURSE, 6, 0);
	lcd_outdezFloat(xnum, y, telemetry_data.course, 2);

	y += FH;
	lcd_putsnAtt(x1, y, PSTR("V"), 1, 0);
	lcd_outdezAtt(xnum, y, telemetry_data.v, 0);
	//}
}

#ifdef DUMP_RX_TX
//! \brief Display one byte as hex.
void lcd_outhex2(uint8_t x, uint8_t y, uint8_t val) {
	x += FWNUM * 2;
	for (int i = 0; i < 2; i++) {
		x -= FWNUM;
		char c = val & 0xf;
		c = c > 9 ? c + 'A' - 10 : c + '0';
		lcd_putcAtt(x, y, c, c >= 'A' ? CONDENSED : 0);
		val >>= 4;
	}
}

//! \brief Hex dump of the current mavlink message.
void menuTelemetryMavlinkDump(uint8_t event) {
	uint8_t x = 0;
	uint8_t y = FH;
	uint16_t count = 0;
	uint16_t bufferLen = 0;
	uint8_t *ptr = NULL;
	switch (MAVLINK_menu) {
		case MENU_DUMP_RX:
			mav_dump_rx = 1;
			mav_title(PSTR("RX"), MAVLINK_menu);
			bufferLen = mavlinkRxBufferCount;
			ptr = mavlinkRxBuffer;
			break;

		case MENU_DUMP_DIAG:
			mav_title(PSTR("Diag"), MAVLINK_menu);
		//	bufferLen = serialTxBufferCount;
		//	ptr = ptrTxISR;
			break;
		
		default:
			break;
	}
	for (uint16_t var = 0; var < bufferLen; var++) {
		uint8_t byte = *ptr++;
		lcd_outhex2(x, y, byte);
		x += 2 * FW;
		count++;
		if (count > 8) {
			count = 0;
			x = 0;
			y += FH;
			if (y == (6 * FH))
			break;
		}
	}
}
#endif


/*!	\brief Mavlink General setup menu.
 *	\details Setup menu for generic mavlink settings.
 *	Current menu items
 *	- RC RSSI scale item. Used to adjust the scale of the RSSI indicator to match
 *	the actual rssi value
 *	- PC RSSI enable item. Can be used to dissable PC RSSI display if not used.
 *	This funcion is called from the model setup menus, not directly by the
 *	telemetry menus
 */


static const char modemStr[20][11] PROGMEM = {
		{},
		{},
		{"FSK 2k"},         ///< GFSK, No Manchester, Rb = 2kbs,    Fd = 5kHz
		{"FSK 2.4k"},      ///< GFSK, No Manchester, Rb = 2.4kbs,  Fd = 9.6kHz
		{"FSK 4.8k"},      ///< GFSK, No Manchester, Rb = 4.8kbs,  Fd = 9.6kHz
		{"FSK 9.6k"},      ///< GFSK, No Manchester, Rb = 9.6kbs,  Fd = 9.6kHz
		{"FSK 19.2k"},    ///< GFSK, No Manchester, Rb = 19.2kbs, Fd = 9.6kHz
		{"FSK 38.4k"},   ///< GFSK, No Manchester, Rb = 38.4kbs, Fd = 19.6kHz
		{"FSK 57.6k"},   ///< GFSK, No Manchester, Rb = 57.6kbs, Fd = 28.8kHz
		{"FSK 125k"},     ///< GFSK, No Manchester, Rb = 125kbs,  Fd = 125kHz
		{0},     ///< FSK, No Manchester, Rb = 512bs,  Fd = 2.5kHz, for POCSAG compatibility
		{0},     ///< FSK, No Manchester, Rb = 512bs,  Fd = 4.5kHz, for POCSAG compatibility

		{"GFSK 2k"},         ///< GFSK, No Manchester, Rb = 2kbs,    Fd = 5kHz
		{"GFSK 2.4k"},      ///< GFSK, No Manchester, Rb = 2.4kbs,  Fd = 9.6kHz
		{"GFSK 4.8k"},      ///< GFSK, No Manchester, Rb = 4.8kbs,  Fd = 9.6kHz
		{"GFSK 9.6k"},      ///< GFSK, No Manchester, Rb = 9.6kbs,  Fd = 9.6kHz
		{"GFSK 19.2k"},    ///< GFSK, No Manchester, Rb = 19.2kbs, Fd = 9.6kHz
		{"GFSK 38.4k"},   ///< GFSK, No Manchester, Rb = 38.4kbs, Fd = 19.6kHz
		{"GFSK 57.6k"},   ///< GFSK, No Manchester, Rb = 57.6kbs, Fd = 28.8kHz
		{"GFSK 125k"},     ///< GFSK, No Manchester, Rb = 125kbs,  Fd = 125kHz
};

void menuTelemetryMavlinkSetup(uint8_t event) {
	
	MENU(STR_MAVMENUSETUP_TITLE, menuTabModel, e_MavSetup, ITEM_MAVLINK_MAX + 1, {0, 0, 1/*to force edit mode*/});
	
	uint8_t sub = m_posVert - 1;

	for (uint8_t i=0; i<LCD_LINES-1; i++) {
		uint8_t y = 1 + 1*FH + i*FH;
		uint8_t k = i+s_pgOfs;
		uint8_t blink = ((s_editMode>0) ? BLINK|INVERS : INVERS);
		uint8_t attr = (sub == k ? blink : 0);
		switch(k) {	
		case ITEM_MAVLINK_RADIO_FREQUENCY:{
			static const pm_char F[] PROGMEM = "Frequency";
	        lcd_putsLeft(y, F);
	        uint16_t f = radioCfg.frequency/100000;
	        lcd_outdezAtt(RADIO_SETUP_2ND_COLUMN, y, f, attr|LEFT|PREC1);
	        lcd_puts(lcdLastPos, y, PSTR("MHz"));

	        if (attr) CHECK_INCDEC_GENVAR(event, f, 3840, 5000);
	        radioCfg.frequency = f*100000;
			break;
		}
		case ITEM_MAVLINK_RADIO_FREQHOPS:{
			static const pm_char F[] PROGMEM = "F.Hop Channels";
	        lcd_putsLeft(y, F);
	        lcd_outdezAtt(RADIO_SETUP_2ND_COLUMN, y, radioCfg.fhop_channels, attr|LEFT);

	        if (attr) CHECK_INCDEC_GENVAR(event, radioCfg.fhop_channels, 0, 32);

		break;
		}
		case ITEM_MAVLINK_RADIO_TXINTERVAL:{
			static const pm_char F[] PROGMEM = "TX Interval";
	        lcd_putsLeft(y, F);
	        lcd_outdezAtt(RADIO_SETUP_2ND_COLUMN, y, radioCfg.txInterval, attr|LEFT);
	        lcd_puts(lcdLastPos, y, PSTR("ms"));

	        if (attr) CHECK_INCDEC_GENVAR(event, radioCfg.txInterval, 10, 1000);

		break;
		}
		case ITEM_MAVLINK_RADIO_CFG:{
			static const pm_char F[] PROGMEM = "Modem Cfg";
			char str[12];
			strcpy_P(str, &(modemStr[radioCfg.modemCfg][0]));
	        lcd_putsLeft(y, F);
	       // lcd_outdezAtt(RADIO_SETUP_2ND_COLUMN, y, radioCfg.modemCfg, attr|LEFT);
	        lcd_putsAtt(RADIO_SETUP_2ND_COLUMN-2*FW, y, str, attr|BSS);

	        if (attr) CHECK_INCDEC_GENVAR(event, radioCfg.modemCfg, 2, 19);

	        if(radioCfg.modemCfg == 9) {
	        	radioCfg.modemCfg = 11;
	        } else
	        if(radioCfg.modemCfg == 10) {
	        	radioCfg.modemCfg = 8;
	        }

		break;
		}
		case ITEM_MAVLINK_RADIO_TXPOWER:{
			static const pm_char F[] PROGMEM = "TX Power";
	        lcd_putsLeft(y, F);
	        lcd_outdezAtt(RADIO_SETUP_2ND_COLUMN, y, abs(radioCfg.txPower*3-1), attr|LEFT);
	        lcd_puts(lcdLastPos, y, PSTR("dBm"));

	        if (attr) CHECK_INCDEC_GENVAR(event, radioCfg.txPower, 0, 7);
		break;
		}

		}
	}

	char buf[32];
	uint8_t ynum = LCD_H - FH;

	snprintf_P(buf, sizeof(buf), PSTR("RSSI%d/%d N%d/%d"), radioStatus.rssi, radioStatus.remRssi,
			radioStatus.noise, radioStatus.remNoise);
	lcd_putsAtt(0, ynum, buf, BSS|INVERS);


}
