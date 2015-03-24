//--------------------------------------------------
//! \file		myperiod.h
//! \brief		header file for period tuning
//! \date		2015-Mar
//! \author		minbiocabanon
//--------------------------------------------------

//----------------------------------------------------------------------
//!\brief	These are DEFAULTS parameters !
//----------------------------------------------------------------------

//#define PERIOD_CHECK_FLOOD			36000000	// 60min. ,interval between 2 flood sensor check (can send an SMS alert if water is detected)
#define PERIOD_CHECK_FLOOD			30000	// 60min. ,interval between 2 flood sensor check (can send an SMS alert if water is detected)
#define PERIOD_CHECK_ANALOG_LEVEL 	36000000	// 60min. , interval between 2 analog level check (can send an SMS alert if level are low)
#define	PERIOD_TEST_GEOFENCING		36000000	// 60min. , interval between 2 geofencing check, in milliseconds (can send an SMS alert if we are outside area)
#define PERIODIC_STATUS_SMS_H		12			// Hour for time of periodic status
#define PERIODIC_STATUS_SMS_M		00			// Minute for time of periodic status

// !!! DO  NOT MODIFY !!!!
#define	PERIOD_GET_GPS			5000		// 5 sec. , interval between 2 GPS positions, in milliseconds
#define PERIOD_LIPO_INFO		120000		// 2 min. ,interval between 2 battery level measurement, in milliseconds
#define PERIOD_READ_ANALOG		120000		// 2 min. ,interval between 2 analog input read (external supply), in milliseconds
#define PERIOD_CHECK_SMS		1000		// 1 sec., interval between 2 SMS check, in milliseconds
#define TIMEOUT_SMS_MENU		300000		// 5 min., when timeout, SMS menu return to login (user should send password again to log), in milliseconds
#define PERIODIC_STATUS_SMS		60000		// 1 min. (DO NOT CHANGE) : interval between two Hour+Minute check of periodic time (see below)
