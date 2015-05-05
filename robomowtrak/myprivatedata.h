//--------------------------------------------------
//! \file		myprivatedata.h
//! \brief		header file for private data as GPS coordinate, phone number and wifi credentials
//! \date		2015-Jan
//! \author		minbiocabanon
//--------------------------------------------------

//----------------------------------------------------------------------
//!\brief	These are DEFAULTS parameters !
//----------------------------------------------------------------------

// Alarm allowed ?
#define FLAG_ALARM_ONOFF			1	// 1 = send SMS geofencing alarm 	; 0 = alarm not allowed (will not send SMS)
#define FLAG_PERIODIC_STATUS_ONOFF	1	// 1 = send periodic status  		; 0 = periodic status not allowed (will not send SMS)
#define FLAG_ALARM_LOW_BAT			1	// 1 = send alarm when low voltage, set TRIG_INPUT_LEVEL to define treshol 	; 0 = no check
#define FLAG_ALARM_FLOOD			0	// 1 = send alarm if flood sensor detects water ;  0 = don't care about flooding

#define TRIG_INPUT_LEVEL			23.2	// in volt, when input voltage is lower than this value, an SMS alarm will be sent
					// 11.6V is a good level trig for 12V lead acid battery. Set lower voltage at your own risk !
					// 23.2V is a good level trig for 24V lead acid battery. Set lower voltage at your own risk !

//Params for geofencing
#define RADIUS					150			// radius in meter for geofencing centered in BASE_LAT,BASE_LON. When GPS pos is outside this radius -> Alarm !

// Lat/Lon station position (for geofencing)
#define BASE_LAT	43.791489		
#define BASE_LAT_DIR	'N'	
#define BASE_LON	1.1077
#define BASE_LON_DIR	'E'

// Phone number to call or for SMS
#define MYPHONENUMBER	"+33630213069"		// Default phone number where to send messages

// SMS Menu
#define SMSSECRET	"1234"					// Default secret code to activate menu

// WiFi
#define WIFI_AP "VIRUS"  // replace with your setting
#define WIFI_PWD "MINBIOCABANON"  // replace with your setting