//--------------------------------------------------
//! \file		mygpscoord.h
//! \brief		header file for private data as GPS coordinate, phone number and wifi credentials
//! \date		2014-Nov
//! \author		minbiocabanon
//--------------------------------------------------

//----------------------------------------------------------------------
//!\brief	These are DEFAULTS parameters !
//----------------------------------------------------------------------
// Lat/Lon station position (for geofencing)
#define BASE_LAT	43.791489		
#define BASE_LAT_DIR	'N'
// #define BASE_LAT	43.091489		
#define BASE_LON	1.1077
#define BASE_LON_DIR	'E'

// Phone number to call or for SMS
#define MYPHONENUMBER	"0630213069"

// SMS Menu
#define SMSSECRET	"1234"

// battery level trigger for alarm , in % 
#define BAT_LEVEL_TRIG	20

// WiFi
#define WIFI_AP "VIRUS"  // replace with your setting
#define WIFI_PWD "MINBIOCABANON"  // replace with your setting

//----------------------------------------------------------------------
//!\brief	Structure where user parameters are stored in EEPROM (tuned by SMS)
//----------------------------------------------------------------------
struct EEPROM_param {
	bool flag_data_written;		// when true, this structure contains data. Should be false only at the very first start
	bool flag_alarm_onoff;		// 1 = alarm on ; 0 = alarm off
	char smssecret[4];
	char myphonenumber[10];	
	float base_lat;
	char base_lat_dir;
	float base_lon;
	char base_lon_dir;
	int bat_level_trig;			// battery level, when trigged, should send an alarm
}MyParam;