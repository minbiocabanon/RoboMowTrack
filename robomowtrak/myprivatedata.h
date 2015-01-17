//--------------------------------------------------
//! \file		mygpscoord.h
//! \brief		header file for private data as GPS coordinate, phone number and wifi credentials
//! \date		2014-Nov
//! \author		minbiocabanon
//--------------------------------------------------

//----------------------------------------------------------------------
//!\brief	These are DEFAULTS parameters !
//----------------------------------------------------------------------

// Alarm allowed ?
#define FLAG_ALARM_ONOFF	1	// 1 = alarm allowed , 0 = alarme not allowed (will not send SMS)
#define FLAG_PERIODIC_STATUS_ONOFF	1	// 1 = periodic status allowed , 0 = periodic status not allowed (will not send SMS)
#define FLAG_ALARM_LOW_BAT			1	// 1 = check input voltage level, set TRIG_INPUT_LEVEL to define treshol 	; 0 = no check

// Lat/Lon station position (for geofencing)
#define BASE_LAT	43.12489		
#define BASE_LAT_DIR	'N'
#define BASE_LON	1.5407
#define BASE_LON_DIR	'E'

// Phone number to call or for SMS
#define MYPHONENUMBER	"+33666666666"

// SMS Menu
#define SMSSECRET	"1234"

// battery level trigger for alarm , in % 
#define BAT_LEVEL_TRIG	20

// WiFi
#define WIFI_AP "****"  // replace with your setting
#define WIFI_PWD "MINBIOCABANON"  // replace with your setting

//----------------------------------------------------------------------
//!\brief	Structure where user parameters are stored in EEPROM (tuned by SMS)
//----------------------------------------------------------------------
struct EEPROM_param {
	bool flag_data_written;				// when true, this structure contains data. Should be false only at the very first start
	bool flag_alarm_onoff;				// 1 = alarm on ; 0 = alarm off
	bool flag_periodic_status_onoff;	// 1 = periodic status on ; 0 = periodic status off	
	bool flag_alarm_low_bat;			// 1 = check input voltage (can be an external batt.) ; 0 = do not check input voltage
	char smssecret[5];
	char myphonenumber[13];	
	double base_lat;
	char base_lat_dir;
	double base_lon;
	char base_lon_dir;
	int bat_level_trig;			// battery level, when trigged, should send an alarm
}MyParam;