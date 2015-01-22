//--------------------------------------------------
//! \file		mygpscoord.h
//! \brief		header file for private data as GPS coordinate, phone number and wifi credentials
//! \date		2015-Jan
//! \author		minbiocabanon
//--------------------------------------------------

//----------------------------------------------------------------------
//!\brief	These are DEFAULTS parameters !
//----------------------------------------------------------------------

// Alarm allowed ?
#define FLAG_ALARM_ONOFF			1	// 1 = alarm allowed , 0 = alarme not allowed (will not send SMS)
#define FLAG_PERIODIC_STATUS_ONOFF	1	// 1 = periodic status allowed , 0 = periodic status not allowed (will not send SMS)
#define FLAG_ALARM_LOW_BAT			1	// 1 = check input voltage level, set TRIG_INPUT_LEVEL to define treshol 	; 0 = no check

#define TRIG_INPUT_LEVEL			11.6	// in volt, when input voltage is lower than this value, an SMS alarm will be sent
				// 11.6V is a good level trig for 12V lead acid battery. Set lower voltage at your own risk !

//Params for geofencing
#define RADIUS					150			// radius in meter for geofencing centered in BASE_LAT,BASE_LON. When GPS pos is outside this radius -> Alarm !

// Lat/Lon station position (for geofencing)
#define BASE_LAT	43.12489		
#define BASE_LAT_DIR	'N'
#define BASE_LON	1.5407
#define BASE_LON_DIR	'E'

// Phone number to call or for SMS
#define MYPHONENUMBER	"+33666666666"

// SMS Menu
#define SMSSECRET	"1234"

// battery level trigger for alarm , in % , WARNING, LIPO level are only 100,66 and 33%
// Do not use value < 33% because linkitone will not give you another value until 0% ...
#define LIPO_LEVEL_TRIG		33		// in % , not used yet !

// WiFi
#define WIFI_AP "****"  			// replace with your setting
#define WIFI_PWD "MINBIOCABANON"  	// replace with your setting