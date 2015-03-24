//--------------------------------------------------
//! \file		robomowtrak.ino
//! \brief		GPS TRACKER
//! \brief		Use GPS for localisation.
//! \brief		Full configurable by SMS.
//! \brief		SMS alert only.
//! \brief		Read voltage input and can set an alarm on low power.
//! \brief		Monitor LiPo cell voltage and can set an alarm on low power.
//! \brief		Flood sensor interface (GPIO) or other sensor.
//! \brief		Serial message are for debug purpose only.
//! \brief		NOT USED YET : Wifi for tracking/logging position while in my garden
//! \date		2015-Jan
//! \author		minbiocabanon
//--------------------------------------------------

//--------------------------------------------------
//! some notes here :
//! google url to send by sms : https://www.google.com/maps?q=<lat>,<lon>
//!
//! millis() overflow/reset over 50 days
//--------------------------------------------------
#include <LTask.h>
#include <vmthread.h>
#include <stddef.h>
#include <LGPS.h>
#include <LGSM.h>
#include <LBattery.h>
#include <math.h>
#include <LEEPROM.h>
#include <LDateTime.h>
#include "RunningMedian.h"

#include "EEPROMAnything.h"
#include "myprivatedata.h"
#include "robomowtrak.h"

#define	PERIOD_GET_GPS			5000		// 5 sec. , interval between 2 GPS positions, in milliseconds
#define	PERIOD_TEST_GEOFENCING	120000		// 2 min. , interval between 2 geofencing check, in milliseconds (can send an SMS alert if we are outside area)
#define PERIOD_LIPO_INFO		120000		// 2 min. ,interval between 2 battery level measurement, in milliseconds
#define PERIOD_READ_ANALOG		120000		// 2 min. ,interval between 2 analog input read (external supply), in milliseconds
#define PERIOD_CHECK_ANALOG_LEVEL 1200000	// 20 min. , interval between 2 analog level check (can send an SMS alert if level are low)
#define PERIOD_CHECK_FLOOD		600000		// 10 min. ,interval between 2 flood sensor check (can send an SMS alert if water is detected)
#define PERIOD_CHECK_SMS		1000		// 1 sec., interval between 2 SMS check, in milliseconds
#define TIMEOUT_SMS_MENU		300000		// 5 min., when timeout, SMS menu return to login (user should send password again to log), in milliseconds

#define PERIODIC_STATUS_SMS		60000		// 1 min. (DO NOT CHANGE) : interval between two Hour+Minute check of periodic time (see after)
#define PERIODIC_STATUS_SMS_H	12			// Hour for time of periodic status
#define PERIODIC_STATUS_SMS_M	00			// Minute for time of periodic status

// SMS menu architecture
#define TXT_MAIN_MENU	"Main Menu\r\n1 : Status\r\n2 : Alarm ON\r\n3 : Alarm OFF\r\n4 : Params\r\n0 : Exit"
#define TXT_PARAMS_MENU "Params Menu\r\n5 : Change default num.\r\n6 : Change coord.\r\n7 : Change radius\r\n8 : Change secret\r\n9 : Periodic status ON\r\n10 : Periodic status OFF\r\n11 : Low power alarm ON\r\n12 : Low power alarm OFF\r\n13 : Change low power trig.\r\n14 : Restore factory settings"

// Led gpio definition
#define LEDGPS  				13
#define LEDALARM  				12
// Other gpio
#define FLOODSENSOR  			8			// digital input where flood sensor is connected
#define FLOODSENSOR_ACTIVE		0			// 0 or 1 ,Set level when flood sensor is active (water detected)

// Analog input
#define NB_SAMPLE_ANALOG		16
#define VOLT_DIVIDER_INPUT		18.0 		// Voltage divider ratio for mesuring input voltage. 
#define MAX_DC_IN				36			// Max input voltage
#define MIN_DC_IN				9			// Minimum input voltage
// Lipo
// battery level trigger for alarm , in % , WARNING, LIPO level are only 100,66 and 33%
// Do not use value < 33% because linkitone will not give you another value until 0% ...
#define LIPO_LEVEL_TRIG	33	// in % , values available 33 - 66 (0 and exlucded logically)

// GPS
gpsSentenceInfoStruct info;

// Median computation
RunningMedian samples = RunningMedian(NB_SAMPLE_ANALOG);

// Miscalleneous 
char buff[256];
unsigned long taskGetGPS;
unsigned long taskTestGeof;
unsigned long taskGetLiPo;
unsigned long taskGetAnalog;
unsigned long taskCheckInputVoltage;
unsigned long taskCheckSMS;
unsigned long taskCheckFlood;
unsigned long taskStatusSMS;
unsigned long TimeOutSMSMenu;


//----------------------------------------------------------------------
//!\brief	returns distance in meters between two positions, both specified
//!\brief	as signed decimal-degrees latitude and longitude. Uses great-circle
//!\brief	distance computation for hypothetical sphere of radius 6372795 meters.
//!\brief	Because Earth is no exact sphere, rounding errors may be up to 0.5%.
//!\param	float lat1, float long1, float lat2, float long2
//!\return	meters (float)
//---------------------------------------------------------------------- 
float DistanceBetween(float lat1, float long1, float lat2, float long2){
	// Courtesy of Maarten Lamers
	float delta = radians(long1-long2);
	float sdlong = sin(delta);
	float cdlong = cos(delta);
	lat1 = radians(lat1);
	lat2 = radians(lat2);
	float slat1 = sin(lat1);
	float clat1 = cos(lat1);
	float slat2 = sin(lat2);
	float clat2 = cos(lat2);
	delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
	delta = sq(delta);
	delta += sq(clat2 * sdlong);
	delta = sqrt(delta);
	float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
	delta = atan2(delta, denom);
	return delta * 6372795;
}

//----------------------------------------------------------------------
//!\brief	return position of the comma number 'num' in the char array 'str'
//!\return  char
//----------------------------------------------------------------------
static unsigned char getComma(unsigned char num,const char *str){
	unsigned char i,j = 0;
	int len=strlen(str);
	for(i = 0;i < len;i ++){
		if(str[i] == ',')
			j++;
		if(j == num)
			return i + 1; 
		}
	return 0; 
}

//----------------------------------------------------------------------
//!\brief	convert char buffer to float
//!\return  float
//----------------------------------------------------------------------
static float getFloatNumber(const char *s){
	char buf[10];
	unsigned char i;
	float rev;

	i=getComma(1, s);
	i = i - 1;
	strncpy(buf, s, i);
	buf[i] = 0;
	rev=atof(buf);
	return rev; 
}

//----------------------------------------------------------------------
//!\brief	convert char buffer to int
//!\return  float
//----------------------------------------------------------------------
static float getIntNumber(const char *s){
	char buf[10];
	unsigned char i;
	float rev;

	i=getComma(1, s);
	i = i - 1;
	strncpy(buf, s, i);
	buf[i] = 0;
	rev=atoi(buf);
	return rev; 
}

//----------------------------------------------------------------------
//!\brief	Parse GPS NMEA buffer for extracting data
//!\return  -
//----------------------------------------------------------------------
void parseGPGGA(const char* GPGGAstr){
  /* Refer to http://www.gpsinformation.org/dale/nmea.htm#GGA
   * Sample data: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
   * Where:
   *  GGA          Global Positioning System Fix Data
   *  123519       Fix taken at 12:35:19 UTC
   *  4807.038,N   Latitude 48 deg 07.038' N
   *  01131.000,E  Longitude 11 deg 31.000' E
   *  1            Fix quality: 0 = invalid
   *                            1 = GPS fix (SPS)
   *                            2 = DGPS fix
   *                            3 = PPS fix
   *                            4 = Real Time Kinematic
   *                            5 = Float RTK
   *                            6 = estimated (dead reckoning) (2.3 feature)
   *                            7 = Manual input mode
   *                            8 = Simulation mode
   *  08           Number of satellites being tracked
   *  0.9          Horizontal dilution of position
   *  545.4,M      Altitude, Meters, above mean sea level
   *  46.9,M       Height of geoid (mean sea level) above WGS84
   *                   ellipsoid
   *  (empty field) time in seconds since last DGPS update
   *  (empty field) DGPS station ID number
   *  *47          the checksum data, always begins with *
   */

	if(GPGGAstr[0] == '$'){
		int tmp;
		tmp = getComma(1, GPGGAstr);
		MyGPSPos.hour     = (GPGGAstr[tmp + 0] - '0') * 10 + (GPGGAstr[tmp + 1] - '0');
		MyGPSPos.minute   = (GPGGAstr[tmp + 2] - '0') * 10 + (GPGGAstr[tmp + 3] - '0');
		MyGPSPos.second    = (GPGGAstr[tmp + 4] - '0') * 10 + (GPGGAstr[tmp + 5] - '0');

		//get time
		sprintf(buff, "UTC time %02d:%02d:%02d", MyGPSPos.hour, MyGPSPos.minute, MyGPSPos.second);
		Serial.print(buff);
		//get lat/lon coordinates
		float latitudetmp;
		float longitudetmp;
		tmp = getComma(2, GPGGAstr);
		latitudetmp = getFloatNumber(&GPGGAstr[tmp]);
		tmp = getComma(4, GPGGAstr);
		longitudetmp = getFloatNumber(&GPGGAstr[tmp]);
		// need to convert format
		convertCoords(latitudetmp, longitudetmp, MyGPSPos.latitude, MyGPSPos.longitude);
		//get lat/lon direction
		tmp = getComma(3, GPGGAstr);
		MyGPSPos.latitude_dir = (GPGGAstr[tmp]);
		tmp = getComma(5, GPGGAstr);
		MyGPSPos.longitude_dir = (GPGGAstr[tmp]);
		
		//sprintf(buff, "latitude = %10.4f-%c, longitude = %10.4f-%c", MyGPSPos.latitude, MyGPSPos.latitude_dir, MyGPSPos.longitude, MyGPSPos.longitude_dir);
		//Serial.println(buff); 
		
		//get GPS fix quality
		tmp = getComma(6, GPGGAstr);
		MyGPSPos.fix = getIntNumber(&GPGGAstr[tmp]);    
		sprintf(buff, "  -  GPS fix quality = %d", MyGPSPos.fix);
		Serial.print(buff);   
		//get satellites in view
		tmp = getComma(7, GPGGAstr);
		MyGPSPos.num = getIntNumber(&GPGGAstr[tmp]);    
		sprintf(buff, "  -  %d satellites", MyGPSPos.num);
		Serial.println(buff); 
	}
	else{
		Serial.println("No GPS data"); 
	}
}

//----------------------------------------------------------------------
//!\brief	Convert GPGGA coordinates (degrees-mins-secs) to true decimal-degrees
//!\return  -
//----------------------------------------------------------------------
void convertCoords(float latitude, float longitude, float &lat_return, float &lon_return){
	int lat_deg_int = int(latitude/100);		//extract the first 2 chars to get the latitudinal degrees
	int lon_deg_int = int(longitude/100);		//extract first 3 chars to get the longitudinal degrees
    // must now take remainder/60
    //this is to convert from degrees-mins-secs to decimal degrees
    // so the coordinates are "google mappable"
    float latitude_float = latitude - lat_deg_int * 100;		//remove the degrees part of the coordinates - so we are left with only minutes-seconds part of the coordinates
    float longitude_float = longitude - lon_deg_int * 100;     
    lat_return = lat_deg_int + latitude_float / 60 ;			//add back on the degrees part, so it is decimal degrees
    lon_return = lon_deg_int + longitude_float / 60 ;
}

//----------------------------------------------------------------------
//!\brief	Grab GPS position from serial
//!\return  -
//----------------------------------------------------------------------
void GetGPSPos(void){
	// For one second we parse GPS data and report some key values
	if(MyFlag.taskGetGPS){
		MyFlag.taskGetGPS = false;
		Serial.println("--- LGPS loop ---"); 
		LGPS.getData(&info);
		//Serial.print((char*)info.GPGGA); 
		parseGPGGA((const char*)info.GPGGA);
				
		//check fix 
		//if GPS fix is OK
		if ( MyGPSPos.fix == GPS || MyGPSPos.fix == DGPS || MyGPSPos.fix == PPS ){
			//set a flag
			MyFlag.fix3D = true;
		}
		else{
			//reset flag 
			MyFlag.fix3D = false;
		}
		sprintf(buff, "Current position is : https://www.google.com/maps?q=%2.6f%c,%3.6f%c", MyGPSPos.latitude, MyGPSPos.latitude_dir, MyGPSPos.longitude, MyGPSPos.longitude_dir);
		Serial.println(buff);
		Serial.println();
	}
}


//----------------------------------------------------------------------
//!\brief	Get analog voltage of DC input (can be an external battery)
//!\return  -
//----------------------------------------------------------------------
void GetAnalogRead(void){
	// if it's time to get analog input for monitoring external supply
	if(MyFlag.taskGetAnalog){
		Serial.println("-- Analog input read --");
		MyFlag.taskGetAnalog = false;
		// read 16 times and average
		unsigned int i = 0;
		//on fait plusieurs mesures
		for( i = 0; i < NB_SAMPLE_ANALOG; i++){
			//read analog input
			long x  = analogRead(A0);	//gives value between 0 to 1023
			samples.add(x);
			delay(10);
		}
		//ocompute median value
		MyExternalSupply.raw = samples.getMedian();
		sprintf(buff," Analog raw input = %d\r\n", MyExternalSupply.raw );
		Serial.print(buff);
		// convert raw data to voltage
		MyExternalSupply.analog_voltage = MyExternalSupply.raw * 5.0 / 1024.0;
		sprintf(buff," Analog voltage= %2.2fV\r\n", MyExternalSupply.analog_voltage );
		Serial.print(buff);
		// compute true input voltage
		MyExternalSupply.input_voltage = MyExternalSupply.analog_voltage * VOLT_DIVIDER_INPUT + 0.425; // +0.5V for forward voltage of protection diode
		sprintf(buff," Input voltage= %2.1fV\r\n", MyExternalSupply.input_voltage );
		Serial.println(buff);
	}	
}

//----------------------------------------------------------------------
//!\brief	Read Digital input : check if flood sensor detects some water
//!\return  -
//----------------------------------------------------------------------
void GetDigitalInput(void){
	// if it's time to get digital input from flood sensor
	if(MyFlag.taskCheckFlood){
		Serial.println("-- Digital input read --");
		MyFlag.taskCheckFlood = false;
		//read digital input
		MyGpio.FloodSensor = digitalRead(FLOODSENSOR);
		sprintf(buff," Digital input = %d\r\n", MyGpio.FloodSensor );
		Serial.print(buff);
		
		// if input is true, we are diviiiiiing !
		if ( MyGpio.FloodSensor == FLOODSENSOR_ACTIVE ){
			//prepare SMS to warn user
			sprintf(buff, "Alert! flood sensor has detect water.\r\n Input level is %d.", MyGpio.FloodSensor); 
			Serial.println(buff);
			//send SMS
			SendSMS(MySMS.incomingnumber, buff);
		}
	}
}

//----------------------------------------------------------------------
//!\brief	Grab LiPo battery level and status
//!\return  -
//----------------------------------------------------------------------
void GetLiPoInfo(void){
	// if it's time to get LiPo voltage and status
	if(MyFlag.taskGetLiPo){
		Serial.println("-- Battery Info --");
		MyFlag.taskGetLiPo = false;
		MyBattery.LiPo_level = LBattery.level();
		MyBattery.charging_status  = LBattery.isCharging();
		sprintf(buff," battery level = %d%%", MyBattery.LiPo_level );
		Serial.print(buff);
		//convert charging direction status
		char chargdir[24];
		sprintf(chargdir,"discharging");
		//convert bit to string
		if(MyBattery.charging_status)
			sprintf(chargdir,"charging");		
		sprintf(buff," is %s\r\n", chargdir );
		Serial.println(buff);
	}
}

//----------------------------------------------------------------------
//!\brief	Do geofencing detection. if we are outside autorized area -> alarm!
//!\return  -
//----------------------------------------------------------------------
void Geofencing(void){
	//check if GPS fix is good
	if (MyFlag.fix3D && MyFlag.taskTestGeof){
		MyFlag.taskTestGeof = false;
		Serial.println("-- Geofencing --"); 
		//compute distance between actual position and reference position
		float distance_base = DistanceBetween(MyParam.base_lat, MyParam.base_lon, MyGPSPos.latitude, MyGPSPos.longitude);
		sprintf(buff, "distance BASE->Robot: %.1f m", distance_base);
		Serial.println(buff);
		
		//check where we are
		if(distance_base <= RADIUS){
			Serial.println("Position is inside area");
			MyFlag.PosOutiseArea = false;
		}
		else{
			Serial.println("ALARM, outside AREA !!!!");
			MyFlag.PosOutiseArea = true;
		}
		Serial.println();
	}
}

//----------------------------------------------------------------------
//!\brief	Verify if SMS is incoming
//!\return  -
//----------------------------------------------------------------------
void CheckSMSrecept(void){
	// Check if there is new SMS
	if(MyFlag.taskCheckSMS && LSMS.available()){
		MyFlag.taskCheckSMS = false;
		char buf[20];
		int v, i = 0;
		//flush buffer before writing in it
		memset(&MySMS.message[0], 0, sizeof(MySMS.message));
		Serial.println("--- SMS received ---");
		// display Number part
		LSMS.remoteNumber(buf, 20);
		size_t destination_size = sizeof (MySMS.incomingnumber);
		snprintf(MySMS.incomingnumber, destination_size, "%s", buf);
		Serial.print("Number:");
		Serial.println(MySMS.incomingnumber);
		// display Content part
		Serial.print("Content:");
		//copy SMS to buffer
		while(true){
			v = LSMS.read();
			if(v < 0)
				break;
			MySMS.message[i] = (char)v;
			Serial.print(MySMS.message[i]);
			i++;
		}
		Serial.println();
		// delete message
		LSMS.flush();
		// set flag to analyse this SMS
		MyFlag.SMSReceived = true;
	}
}

//----------------------------------------------------------------------
//!\brief	Proceed to change number in EEPROM from sms command
//!\brief	MySMS.message should contain : +33---old---,+33---new---
//!\return  -
//----------------------------------------------------------------------
void ProcessChgNum(){

	//check lengh before split
	if( strlen(MySMS.message) == 25 ){
		// Read each command pair 
		char* command = strtok(MySMS.message, ",");
		sprintf(buff, "old num : %s\n",command);
		Serial.println(buff);
		
		//compare old number with the one stored in EEPROM
		if( strcmp(command, MyParam.myphonenumber) == 0){
			// old is OK , we can store new number in EEPROM
			// Find the next command in input string
			command = strtok (NULL, ",");
			sprintf(buff, "new num : %s\n", command);
			Serial.println(buff);
			size_t destination_size = sizeof (MyParam.myphonenumber);
			snprintf(MyParam.myphonenumber, destination_size, "%s", command);
			//Save change in EEPROM
			EEPROM_writeAnything(0, MyParam);
			Serial.println("New number saved in EEPROM");
			sprintf(buff, "New phone number saved : %s", MyParam.myphonenumber); 
			Serial.println(buff);
			//send SMS
			SendSMS(MySMS.incomingnumber, buff);	
			//change state machine to Main_menu
			MySMS.menupos = SM_MENU_MAIN;			
		}
		else{
			sprintf(buff, "Error in old phone number : %s.", command); 
			Serial.println(buff);
			//send SMS
			SendSMS(MySMS.incomingnumber, buff);	
			//change state machine to Main_menu
			MySMS.menupos = SM_MENU_MAIN;			
		}
	}
	else{
		sprintf(buff, "Error in size parameters (%d): %s.", strlen(MySMS.message),MySMS.message); 
		Serial.println(buff);
		//send SMS
		SendSMS(MySMS.incomingnumber, buff);	
		//change state machine to Main_menu
		MySMS.menupos = SM_MENU_MAIN;
	}
}

//----------------------------------------------------------------------
//!\brief	Proceed to change coordinates of main area in EEPROM from sms command
//!\brief	MySMS.message should contain : 49.791489,N,179.1077,E or 'Here'
//!\return  -
//----------------------------------------------------------------------
void ProcessChgCoord(){

	// check lengh before split
	if( strlen (MySMS.message) <= 22 ){
		double newlat, newlon;
		char newlatdir, newlondir;
		
		// Read first field : could be a lat or 'here' word 
		char* command = strtok(MySMS.message, ",");
		if( strcmp(command, "Here") == 0 || strcmp(command, "HERE") == 0 || strcmp(command, "here") == 0){
			// check if GPS fix is good
			if (MyFlag.fix3D) {
				// say it's ok
				Serial.println(" Save actual position as area position.");
				MyParam.base_lat = MyGPSPos.latitude;
				MyParam.base_lat_dir = MyGPSPos.latitude_dir;
				MyParam.base_lon = MyGPSPos.longitude;
				MyParam.base_lon_dir = MyGPSPos.longitude_dir;
				// prepare SMS to confirm data are OK
				sprintf(buff, " New coord. saved : %2.6f,%c,%3.6f,%c", MyParam.base_lat, MyParam.base_lat_dir, MyParam.base_lon, MyParam.base_lon_dir); 
				Serial.println(buff);
				// send SMS
				SendSMS(MySMS.incomingnumber, buff);	
				// change state machine to Main_menu
				MySMS.menupos = SM_MENU_MAIN;			
				// Save change in EEPROM
				EEPROM_writeAnything(0, MyParam);
				Serial.println(" New coord. saved in EEPROM");
			}
			else{
				// say it's not OK because GPS is not fixed
				// prepare SMS to confirm data are OK
				sprintf(buff, " GPS not fixed, can't save position. Retry later"); 
				Serial.println(buff);
				// send SMS
				SendSMS(MySMS.incomingnumber, buff);	
				// change state machine to Main_menu
				MySMS.menupos = SM_MENU_MAIN;
			}
		}
		else{
			// Command contain lat (string)
			//convert to lat float
			newlat = atof(command);
			sprintf(buff," lat : %2.6f",newlat);
			Serial.println(buff);
			
			// Read next field : lat direction
			command = strtok(NULL, ",");
			//copy only the dir to char
			newlatdir = command[0];
			sprintf(buff," lat_dir : %c",newlatdir);
			Serial.println(buff);
			
			// Find the next field : lon
			command = strtok (NULL, ",");
			//convert to lon float
			newlon = atof(command);
			sprintf(buff," lon : %3.6f",newlon);
			Serial.println(buff);		

			// Read next field : lon direction
			command = strtok(NULL, ",");
			//copy only the dir to char
			newlondir = command[0];
			sprintf(buff," lat_dir : %c",newlondir);
			Serial.println(buff);
			
			// proceed to a global check
			if( (newlatdir == 'N' || newlatdir == 'n' || newlatdir == 'S' || newlatdir == 's') && (newlondir == 'E' || newlondir == 'e' || newlondir == 'W' || newlondir == 'w') && ( newlat >= 0.0 && newlat < 90.0 ) && ( newlon >= 0.0 && newlat < 180.0) ){
				// say it's ok
				Serial.println(" Data checked !");
				// save all data in structure
				MyParam.base_lat = newlat;
				MyParam.base_lat_dir = newlatdir;
				MyParam.base_lon = newlon;
				MyParam.base_lon_dir = newlondir;
				//prepare SMS to confirm data are OK
				sprintf(buff, " New coord. saved : %2.6f,%c,%3.6f,%c", newlat, newlatdir, newlon, newlondir); 
				Serial.println(buff);
				//send SMS
				SendSMS(MySMS.incomingnumber, buff);	
				//change state machine to Main_menu
				MySMS.menupos = SM_MENU_MAIN;			
				//Save change in EEPROM
				EEPROM_writeAnything(0, MyParam);
				Serial.println(" New coord. saved in EEPROM");
			}
			else{
				sprintf(buff, " Data error : %f,%c,%f,%c", newlat, newlatdir, newlon, newlondir); 
				Serial.println(buff);
				//send SMS
				SendSMS(MySMS.incomingnumber, buff);	
				//change state machine to Main_menu
				MySMS.menupos = SM_MENU_MAIN;			
			}
		}
	}
	else{
		sprintf(buff, " Error in size parameters : %s.", MySMS.message); 
		Serial.println(buff);
		//send SMS
		SendSMS(MySMS.incomingnumber, buff);	
		//change state machine to Main_menu
		MySMS.menupos = SM_MENU_MAIN;
	}
}

//----------------------------------------------------------------------
//!\brief	Proceed to change secret code in EEPROM from sms command
//!\brief	MySMS.message should contain : oldcode,newcode
//!\return  -
//----------------------------------------------------------------------
void ProcessChgSecret(){
	//check lengh before split
	if( strlen(MySMS.message) == 9 ){
		// Read each command pair 
		char* command = strtok(MySMS.message, ",");
		sprintf(buff, "old code : %s\n",command);
		Serial.println(buff);
		
		//compare old number with the one stored in EEPROM
		if( strcmp(command, MyParam.smssecret) == 0){
			// old is OK , we can store new code in EEPROM
			// Find the next command in input string
			command = strtok (NULL, ",");
			sprintf(buff, "new code : %s\n",command);
			Serial.println(buff);
			size_t destination_size = sizeof (MyParam.smssecret);
			snprintf(MyParam.smssecret, destination_size, "%s", command);
			//Save change in EEPROM
			EEPROM_writeAnything(0, MyParam);
			Serial.println("New secret code saved in EEPROM");
			sprintf(buff, "New secret code saved : %s", MyParam.smssecret); 
			Serial.println(buff);
			//send SMS
			SendSMS(MySMS.incomingnumber, buff);	
			//change state machine to Main_menu
			MySMS.menupos = SM_MENU_MAIN;			
		}
		else{
			sprintf(buff, "Error in old secret code : %s.", command); 
			Serial.println(buff);
			//send SMS
			SendSMS(MySMS.incomingnumber, buff);	
			//change state machine to Main_menu
			MySMS.menupos = SM_MENU_MAIN;			
		}
	}
	else{
		sprintf(buff, "Error in size parameters (%d): %s.", strlen(MySMS.message), MySMS.message); 
		Serial.println(buff);
		//send SMS
		SendSMS(MySMS.incomingnumber, buff);	
		//change state machine to Main_menu
		MySMS.menupos = SM_MENU_MAIN;
	}
}

//----------------------------------------------------------------------
//!\brief	Proceed to radius of geofencing
//!\brief	MySMS.message should contain radius in meter
//!\return  -
//----------------------------------------------------------------------
void ProcessChgRadius(){
	//check lengh before split
	if( strlen(MySMS.message) <= 5 ){
		// convert SMS to integer
		unsigned int radius_sms = atoi(MySMS.message);
		sprintf(buff, "SMS content as meter : %d\n", radius_sms);
		Serial.println(buff);
		
		//compare old number with the one stored in EEPROM
		if( radius_sms > 1 and radius_sms <= 10000 ){
			// value is OK , we can store it in EEPROM
			MyParam.radius = radius_sms;
			//Save change in EEPROM
			EEPROM_writeAnything(0, MyParam);
			Serial.println("New value saved in EEPROM");
			sprintf(buff, "New radius saved : %d m", MyParam.radius); 
			Serial.println(buff);
			//send SMS
			SendSMS(MySMS.incomingnumber, buff);	
			//change state machine to Main_menu
			MySMS.menupos = SM_MENU_MAIN;			
		}
		else{
			sprintf(buff, "Error, value is outside rangee : %d m", radius_sms); 
			Serial.println(buff);
			//send SMS
			SendSMS(MySMS.incomingnumber, buff);	
			//change state machine to Main_menu
			MySMS.menupos = SM_MENU_MAIN;			
		}
	}
	else{
		sprintf(buff, "Error in size parameters (%d): %s.", strlen(MySMS.message), MySMS.message); 
		Serial.println(buff);
		//send SMS
		SendSMS(MySMS.incomingnumber, buff);	
		//change state machine to Main_menu
		MySMS.menupos = SM_MENU_MAIN;
	}
}

//----------------------------------------------------------------------
//!\brief	Proceed to change voltage of low power trigger alarm
//!\brief	MySMS.message should contain a tension like  11.6
//!\return  -
//----------------------------------------------------------------------
void ProcessLowPowTrig(){
	//check lengh before getting data
	if( strlen(MySMS.message) <= 5 ){
		// convert SMS to float
		float value_sms = atof(MySMS.message);
		sprintf(buff, "SMS content as volt : %2.1f\n",value_sms);
		Serial.println(buff);
		
		// check that it is a value inside the range
		if( value_sms > MIN_DC_IN and value_sms <= MAX_DC_IN ){
			// value is OK , we can store it in EEPROM
			MyParam.trig_input_level = value_sms;
			//Save change in EEPROM
			EEPROM_writeAnything(0, MyParam);
			Serial.println("New value saved in EEPROM");
			sprintf(buff, "New trigger value for low power voltage saved : %2.1fV", MyParam.trig_input_level); 
			Serial.println(buff);
			//send SMS
			SendSMS(MySMS.incomingnumber, buff);	
			//change state machine to Main_menu
			MySMS.menupos = SM_MENU_MAIN;			
		}
		else{
			sprintf(buff, "Error, value is outside rangee : %2.1fV", value_sms); 
			Serial.println(buff);
			//send SMS
			SendSMS(MySMS.incomingnumber, buff);	
			//change state machine to Main_menu
			MySMS.menupos = SM_MENU_MAIN;			
		}
	}
	else{
		sprintf(buff, "Error in size parameters (%d): %s.", strlen(MySMS.message), MySMS.message); 
		Serial.println(buff);
		//send SMS
		SendSMS(MySMS.incomingnumber, buff);	
		//change state machine to Main_menu
		MySMS.menupos = SM_MENU_MAIN;
	}
}

//----------------------------------------------------------------------
//!\brief	Proceed to restore all factory settings 
//!\brief	MySMS.message should contain : y or n (Y or N)
//!\return  -
//----------------------------------------------------------------------
void ProcessRestoreDefault(){
	//check lengh , message should contain y or n (Y or N) 
	if( strlen(MySMS.message) == 1 ){
		// Read sms content
		sprintf(buff, " response : %s\n", MySMS.message);
		Serial.println(buff);
		
		//If restoration id confirmed
		if( (strcmp("Y", MySMS.message) == 0) || (strcmp("y", MySMS.message) == 0) ){
			// then revert all parameters to default !
			
			//little trick : use LoadParamEEPROM function with the flat_data_written forced to false
			// this will act as the very first boot 
			MyParam.flag_data_written = false;
			//SAVE IN EEPROM !
			EEPROM_writeAnything(0, MyParam);
			// then load default param in EEPROM
			LoadParamEEPROM();
			//print structure
			PrintMyParam();	
			
			//prepare an sms to confirm and display default password (it may has been changed)
			sprintf(buff, "Parameters restored to factory settings!!\r\nSecret code: %s", MyParam.smssecret); 
			Serial.println(buff);			
		}
		else{ 
			//prepare an sms to confirm
			sprintf(buff, "Restoration aborted, message received : %s.\r\n", MySMS.message); 
			Serial.println(buff);			
		}
	}
	else{
		sprintf(buff, "Error in message received : %s , size(%d).\r\n", MySMS.message, strlen(MySMS.message)); 
		Serial.println(buff);		
	}
	
	//send SMS
	SendSMS(MySMS.incomingnumber, buff);			
	//change state machine to Main_menu
	MySMS.menupos = SM_MENU_MAIN;	
}

//----------------------------------------------------------------------
//!\brief	Does action selected by user in the main menu
//!\return  -
//----------------------------------------------------------------------
void ProcessMenuMain(void){
	int val = atoi(MySMS.message);
	char flagalarm[4];
	switch(val){
		case CMD_EXIT:
			Serial.println(" Exit !");
			// Force to return to SM_LOGIN state -> need to receive secret code 
			MySMS.menupos = SM_LOGIN;
			break;	
		case CMD_STATUS:		//status
			Serial.println("Status required ");
			
			//convert flag_alarm_onoff
			sprintf(flagalarm,"OFF");
			//convert bit to string
			if(MyParam.flag_alarm_onoff)
				sprintf(flagalarm,"ON");

			//convert flag_periodic_status_onoff
			char flagalarm_period[4];
			sprintf(flagalarm_period,"OFF");
			//convert bit to string
			if(MyParam.flag_periodic_status_onoff)
				sprintf(flagalarm_period,"ON");

			//convert flag_periodic_status_onoff
			char flagalarm_lowbat[4];
			sprintf(flagalarm_lowbat,"OFF");
			//convert bit to string
			if(MyParam.flag_alarm_low_bat)
				sprintf(flagalarm_lowbat,"ON");

			//convert flag_periodic_status_onoff
			char flagalarm_flood[4];
			sprintf(flagalarm_flood,"OFF");
			//convert bit to string
			if(MyParam.flag_alarm_flood)
				sprintf(flagalarm_flood,"ON");
				
			//convert charging direction status
			char chargdir[24];
			sprintf(chargdir,"discharging");
			//convert bit to string
			if(MyBattery.charging_status)
				sprintf(chargdir,"charging");				
			//if GPS is fixed , prepare a complete message
			if(MyFlag.fix3D == true){
				sprintf(buff, "Status : \r\nCurrent position is : https://www.google.com/maps?q=%2.6f%c,%3.6f%c \r\nLiPo = %d%%, %s\r\nExternal supply : %2.1fV\r\nGeofencing alarm is %s.\r\nPeriodic SMS is %s.\r\nLow input voltage alarm is %s.\r\nFlood alarm is %s.", MyGPSPos.latitude, MyGPSPos.latitude_dir, MyGPSPos.longitude, MyGPSPos.longitude_dir, MyBattery.LiPo_level, chargdir, MyExternalSupply.input_voltage, flagalarm, flagalarm_period, flagalarm_lowbat, flagalarm_flood); 
			}
			// else, use short form message
			else{
				sprintf(buff, "Status : \r\nNO position fix.\r\nLiPo = %d%%, %s\r\nExternal supply : %2.1fV\r\nGeofencing alarm is %s.\r\nPeriodic SMS is %s.\r\nLow input voltage alarm is %s.\r\nFlood alarm is %s.", MyBattery.LiPo_level, chargdir, MyExternalSupply.input_voltage, flagalarm, flagalarm_period, flagalarm_lowbat, flagalarm_flood); 
			}
			Serial.println(buff);
			SendSMS(MySMS.incomingnumber, buff);
			break;
			
		case CMD_ALM_ON:		// alarm ON
			Serial.println("Alarm ON required");
			MyParam.flag_alarm_onoff = true;
			//convert flag_alarm_onoff
			snprintf(flagalarm,3,"ON");	
			//prepare SMS content
			sprintf(buff, "Alarm switch to %s state", flagalarm); 
			Serial.println(buff);
			//send SMS
			SendSMS(MySMS.incomingnumber, buff);
			//Save change in EEPROM
			EEPROM_writeAnything(0, MyParam);
			Serial.println("Data saved in EEPROM");
			break;
			
		case CMD_ALM_OFF:		// alarm OFF
			Serial.println("Alarm OFF required");
			MyParam.flag_alarm_onoff = false;
			//convert flag_alarm_onoff
			snprintf(flagalarm,4,"OFF");
			//prepare SMS content
			sprintf(buff, "Alarm switch to %s state", flagalarm); 
			Serial.println(buff);
			//send SMS
			SendSMS(MySMS.incomingnumber, buff);
			//Save change in EEPROM
			EEPROM_writeAnything(0, MyParam);
			Serial.println("Data saved in EEPROM");	
			break;
			
		case CMD_PARAMS:	//go to sub menu params
			sprintf(buff, TXT_PARAMS_MENU); 
			Serial.println(buff);
			SendSMS(MySMS.incomingnumber, buff);
			break;
			
		case CMD_CHG_NUM:
			Serial.println("Change number");
			//prepare SMS content
			sprintf(buff, "Send : +336--old---,+336--new---"); 
			Serial.println(buff);
			//send SMS
			SendSMS(MySMS.incomingnumber, buff);
			MySMS.menupos = SM_CHG_NUM;
			break;
			
		case CMD_CHG_COORD:
			Serial.println("Change coordinates");
			//prepare SMS content
			sprintf(buff, "Send : 49.791489,N,179.1077,E\r\nor 'Here', to set current position as new coord."); 
			Serial.println(buff);
			//send SMS
			SendSMS(MySMS.incomingnumber, buff);
			MySMS.menupos = SM_CHG_COORD;		
			break;
			
		case CMD_CHG_RADIUS:
			// TO DO !!!
			Serial.println("Change radius for geofencing");
			//prepare SMS content
			sprintf(buff, "Send radius in meter (1-10000).\r\nActual radius is %d m", MyParam.radius); 
			Serial.println(buff);
			//send SMS
			SendSMS(MySMS.incomingnumber, buff);
			MySMS.menupos = SM_CHG_RADIUS;				
			break;
			
		case CMD_CHG_SECRET:
			Serial.println("Change secret code");
			//prepare SMS content
			sprintf(buff, "Send : oldcode,newcode\r\nLimit to 4 caracters."); 
			Serial.println(buff);
			//send SMS
			SendSMS(MySMS.incomingnumber, buff);
			MySMS.menupos = SM_CHG_SECRET;		
			break;
			
		case CMD_PERIODIC_STATUS_ON:
			Serial.println("Periodic status ON required");
			MyParam.flag_periodic_status_onoff = true;
			//convert flag_periodic_status_onoff
			snprintf(flagalarm,4,"ON");
			//prepare SMS content
			sprintf(buff, "Periodic status switched to %s state", flagalarm); 
			Serial.println(buff);
			//send SMS
			SendSMS(MySMS.incomingnumber, buff);
			//Save change in EEPROM
			EEPROM_writeAnything(0, MyParam);
			Serial.println("Data saved in EEPROM");	
			break;

		case CMD_PERIODIC_STATUS_OFF:
			Serial.println("Periodic status OFF required");
			MyParam.flag_periodic_status_onoff = false;
			//convert flag_periodic_status_onoff
			snprintf(flagalarm,4,"OFF");
			//prepare SMS content
			sprintf(buff, "Periodic status switched to %s state", flagalarm); 
			Serial.println(buff);
			//send SMS
			SendSMS(MySMS.incomingnumber, buff);
			//Save change in EEPROM
			EEPROM_writeAnything(0, MyParam);
			Serial.println("Data saved in EEPROM");	
			break;	

		case CMD_LOWPOWER_ON:
			Serial.println("Low power alarm ON required");
			MyParam.flag_alarm_low_bat = true;
			//convert flag_alarm_low_bat
			snprintf(flagalarm,4,"ON");
			//prepare SMS content
			sprintf(buff, "Low power alarm switched to %s state", flagalarm); 
			Serial.println(buff);
			//send SMS
			SendSMS(MySMS.incomingnumber, buff);
			//Save change in EEPROM
			EEPROM_writeAnything(0, MyParam);
			Serial.println("Data saved in EEPROM");	
			break;

		case CMD_LOWPOWER_OFF:
			Serial.println("Low power alarm OFF required");
			MyParam.flag_alarm_low_bat = false;
			//convert flag_alarm_low_bat
			snprintf(flagalarm,4,"OFF");
			//prepare SMS content
			sprintf(buff, "Low power alarm switched to %s state", flagalarm); 
			Serial.println(buff);
			//send SMS
			SendSMS(MySMS.incomingnumber, buff);
			//Save change in EEPROM
			EEPROM_writeAnything(0, MyParam);
			Serial.println("Data saved in EEPROM");	
			break;			
		
		case CMD_CHG_LOWPOW_TRIG:
			Serial.println("Change low power trigger level");
			//prepare SMS content
			sprintf(buff, "Send tension in volt, ex. :  11.6\r\nActual trig. is %2.1fV", MyParam.trig_input_level); 
			Serial.println(buff);
			//send SMS
			SendSMS(MySMS.incomingnumber, buff);
			MySMS.menupos = SM_CHG_LOWPOW_TRIG;		
			break;

		case CMD_RESTORE_DFLT:
			//prepare SMS content
			sprintf(buff, "CONFIRM RESTORE DEFAULT SETTINGS Y/N ?"); 
			Serial.println(buff);
			//send SMS
			SendSMS(MySMS.incomingnumber, buff);
			MySMS.menupos = SM_RESTORE_DFLT;		
			break;
			
		default:
			//prepare SMS content
			sprintf(buff, "Error"); 
			Serial.println(buff);
			//send SMS
			SendSMS(MySMS.incomingnumber, buff);		
			break;
	}
}

//----------------------------------------------------------------------
//!\brief	Manage SMS menu
//!\return  -
//----------------------------------------------------------------------
void MenuSMS(void){
	// if a new message is received
	if( MyFlag.SMSReceived == true ){
		Serial.println("--- SMS Menu manager ---");
		switch(MySMS.menupos){
			default:			
			case SM_LOGIN:
				//compare secret code with received sms code
				if( strcmp(MySMS.message, MyParam.smssecret) == 0 ){
					// password is OK
					Serial.println("Password is OK.");
					// password is OK, we can send main menu
					MySMS.menupos = SM_MENU_MAIN;
					sprintf(buff, TXT_MAIN_MENU); 
					Serial.println(buff);
					SendSMS(MySMS.incomingnumber, buff);
					// start timer to auto-logout when no action occurs
					TimeOutSMSMenu = millis();
				}
				else{
					Serial.println("Wrong Password. Do nothing !");
				}
				break;
			
			case SM_MENU_MAIN:
				// reload timer to avoid auto-logout
				TimeOutSMSMenu = millis();
				Serial.println("Menu Main ");
				ProcessMenuMain();
				break;
			
			case SM_CHG_NUM:
				// reload timer to avoid auto-logout
				TimeOutSMSMenu = millis();
				Serial.println("Proceed to change number");
				ProcessChgNum();
				break;
				
			case SM_CHG_COORD:
				// reload timer to avoid auto-logout
				TimeOutSMSMenu = millis();
				Serial.println("Proceed to change coordinates");
				ProcessChgCoord();
				break;	
				
			case SM_CHG_RADIUS:
				// reload timer to avoid auto-logout
				TimeOutSMSMenu = millis();
				Serial.println("Proceed to change geofencing radius");
				ProcessChgRadius();
				break;
				
			case SM_CHG_SECRET:
				// reload timer to avoid auto-logout
				TimeOutSMSMenu = millis();
				Serial.println("Proceed to change secret code");
				ProcessChgSecret();
				break;
			
			case SM_RESTORE_DFLT:
				// reload timer to avoid auto-logout
				TimeOutSMSMenu = millis();
				Serial.println("Proceed to restore default settings");
				ProcessRestoreDefault();				
				break;
			
			case SM_CHG_LOWPOW_TRIG:
				// reload timer to avoid auto-logout
				TimeOutSMSMenu = millis();
				Serial.println("Proceed to change low power level");
				ProcessLowPowTrig();
				break;
		}
	
		// SMS read reset flag
		MyFlag.SMSReceived = false;
	}
}

//----------------------------------------------------------------------
//!\brief	Send an SMS with specific number and message
//!\param	phonenumber (char[]) 
//!\param  	message (char[120])
//!\return  true or false
//----------------------------------------------------------------------
bool SendSMS( const char *phonenumber, const char *message ){
	Serial.println("  Sending SMS ...");
	LSMS.beginSMS(phonenumber);
	LSMS.print(message);
	bool ret;
	if(LSMS.endSMS()){
		Serial.println("  SMS sent");
		ret = true;
	}
	else{
		Serial.println("  SMS not sent");
		ret = false;
	}
	return ret;
}
//----------------------------------------------------------------------
//!\brief	Manage alert when occurs
//!\return  -
//----------------------------------------------------------------------
void AlertMng(void){
	// if alarm is allowed AND position is outside autorized area
	if ( MyParam.flag_alarm_onoff && MyFlag.PosOutiseArea){
		MyFlag.PosOutiseArea = false;
		Serial.println("--- AlertMng : start sending SMS"); 		
		//convert charging direction status
		char chargdir[24];
		sprintf(chargdir,"discharging");
		//convert bit to string
		if(MyBattery.charging_status)
			sprintf(chargdir,"charging");		
		sprintf(buff, "Robomow Alert !! Current position is : https://www.google.com/maps?q=%2.6f%c,%3.6f%c \r\nLiPo = %d%%, %s\r\nExternal supply : %2.1fV", MyGPSPos.latitude, MyGPSPos.latitude_dir, MyGPSPos.longitude, MyGPSPos.longitude_dir, MyBattery.LiPo_level, chargdir, MyExternalSupply.input_voltage); 
		Serial.println(buff);
		SendSMS(MyParam.myphonenumber, buff);
	}
	
	// each minute : if periodic status sms is required
	if ( MyParam.flag_periodic_status_onoff && MyFlag.taskStatusSMS){
		// reset flag
		MyFlag.taskStatusSMS = false;
		// check if hour + minute is reach 
		if ( MyGPSPos.hour == PERIODIC_STATUS_SMS_H && MyGPSPos.minute == PERIODIC_STATUS_SMS_M){
			// It's time to send a status SMS !!
			Serial.println("--- AlertMng : periodic status SMS"); 
			sprintf(buff, "  Ring ! it's %d:%d , time to send a periodic status SMS", MyGPSPos.hour, MyGPSPos.minute ); 
			Serial.println(buff);			
			//convert flag_alarm_onoff
			char flagalarm[4];
			sprintf(flagalarm,"OFF");
			//convert bit to string
			if(MyParam.flag_alarm_onoff)
				sprintf(flagalarm,"ON");

			//convert flag_periodic_status_onoff
			char flagalarm_period[4];
			sprintf(flagalarm_period,"OFF");
			//convert bit to string
			if(MyParam.flag_periodic_status_onoff)
				sprintf(flagalarm_period,"ON");

			//convert flag_periodic_status_onoff
			char flagalarm_lowbat[4];
			sprintf(flagalarm_lowbat,"OFF");
			//convert bit to string
			if(MyParam.flag_alarm_low_bat)
				sprintf(flagalarm_lowbat,"ON");

			//convert flag_periodic_status_onoff
			char flagalarm_flood[4];
			sprintf(flagalarm_flood,"OFF");
			//convert bit to string
			if(MyParam.flag_alarm_flood)
				sprintf(flagalarm_flood,"ON");				
				
			//convert charging direction status
			char chargdir[24];
			sprintf(chargdir,"discharging");
			//convert bit to string
			if(MyBattery.charging_status)
				sprintf(chargdir,"charging");			
			sprintf(buff, "Periodic status : \r\nCurrent position is : https://www.google.com/maps?q=%2.6f%c,%3.6f%c \r\nLiPo = %d%%, %s\r\nExternal supply : %2.1fV\r\nGeofencing alarm is %s.\r\nPeriodic SMS is %s.\r\nLow input voltage alarm is %s.\r\nFlood alarm is %s.", MyGPSPos.latitude, MyGPSPos.latitude_dir, MyGPSPos.longitude, MyGPSPos.longitude_dir, MyBattery.LiPo_level, chargdir, MyExternalSupply.input_voltage, flagalarm, flagalarm_period, flagalarm_lowbat, flagalarm_flood); 
			Serial.println(buff);
			SendSMS(MyParam.myphonenumber, buff);
		}
	}

	// Check input supply level (can be an external battery) and LiPo level
	if (  MyFlag.taskCheckInputVoltage ){
		Serial.println("--- AlertMng : Check input voltage"); 
		// If input voltage is lower than alarm treshold
		if( (MyExternalSupply.input_voltage <= MyParam.trig_input_level) && MyParam.flag_alarm_low_bat ){
			// add some debug and send an alarm SMS
			sprintf(buff, "  LOW BATTERY ALARM\r\n  Input voltage is lower than TRIG_INPUT_LEVEL :\r\n  %2.1fV <= %2.1fV", MyExternalSupply.input_voltage, MyParam.trig_input_level ); 
			Serial.println(buff);
			SendSMS(MyParam.myphonenumber, buff);
		}
		
		Serial.println("--- AlertMng : Check LiPo voltage"); 
		// If LiPo voltage is lower than alarm treshold
		if( MyBattery.LiPo_level <= MyParam.lipo_level_trig ){
			// add some debug and send an alarm SMS
			sprintf(buff, "  LOW VOLTAGE LiPo ALARM\r\n  LiPo voltage is lower than LIPO_LEVEL_TRIG :\r\n  %d%% <= %d%%", MyBattery.LiPo_level, MyParam.lipo_level_trig ); 
			Serial.println(buff);
			SendSMS(MyParam.myphonenumber, buff);
		}
		
		// reset flags
		MyParam.flag_alarm_low_bat = false;
		MyFlag.taskCheckInputVoltage = false;
	}
}
//----------------------------------------------------------------------
//!\brief	Load params from EEPROM
//----------------------------------------------------------------------
void LoadParamEEPROM() {
	
	EEPROM_readAnything(0, MyParam);
	
	//uncomment this line to erase EEPROM parameters with DEFAULT parameters
	// MyParam.flag_data_written = false;
	
	//check if parameters were already written
	if( MyParam.flag_data_written == false ){
		Serial.println("--- !!! Loading DEFAULT parameters from EEPROM ...  --- ");
		//EEPROM is empty , so load default parameters (see myprivatedata.h)
		MyParam.flag_alarm_onoff = FLAG_ALARM_ONOFF;
		MyParam.flag_periodic_status_onoff = FLAG_PERIODIC_STATUS_ONOFF;	
		MyParam.flag_alarm_low_bat = FLAG_ALARM_LOW_BAT;
		MyParam.flag_alarm_flood = FLAG_ALARM_FLOOD;
		size_t destination_size = sizeof (MyParam.smssecret);
		snprintf(MyParam.smssecret, destination_size, "%s", SMSSECRET);
		destination_size = sizeof (MyParam.myphonenumber);
		snprintf(MyParam.myphonenumber, destination_size, "%s", MYPHONENUMBER);
		MyParam.radius = RADIUS;
		MyParam.base_lat = BASE_LAT;
		MyParam.base_lat_dir = BASE_LAT_DIR;
		MyParam.base_lon = BASE_LON;
		MyParam.base_lon_dir = BASE_LON_DIR;
		MyParam.lipo_level_trig = LIPO_LEVEL_TRIG;
		MyParam.trig_input_level = TRIG_INPUT_LEVEL;
		//set flag that default data are stored
		MyParam.flag_data_written = true;
		
		//SAVE IN EEPROM !
		EEPROM_writeAnything(0, MyParam);
		Serial.println("--- !!! DEFAULT parameters stored in EEPROM !!! --- ");
	}
	else{
		Serial.println("--- Parameters loaded from EEPROM --- ");
	}
}

//----------------------------------------------------------------------
//!\brief	Print params of MyParam structure
//----------------------------------------------------------------------
void PrintMyParam() {
	char flag[4];
	Serial.println("--- MyParam contents --- ");
	
	sprintf(flag,"OFF");
	//convert bit to string
	if(MyParam.flag_data_written)
		sprintf(flag,"ON");
	sprintf(buff, "  flag_data_written = %s", flag);
	Serial.println(buff);
	
	sprintf(flag,"OFF");
	//convert bit to string
	if(MyParam.flag_alarm_onoff)
		sprintf(flag,"ON");	
	sprintf(buff, "  flag_alarm_onoff = %s", flag);
	Serial.println(buff);
	
	sprintf(flag,"OFF");
	//convert bit to string
	if(MyParam.flag_periodic_status_onoff)
		sprintf(flag,"ON");	
	sprintf(buff, "  flag_periodic_status_onoff = %s", flag);
	Serial.println(buff);	

	sprintf(flag,"OFF");
	//convert bit to string
	if(MyParam.flag_alarm_low_bat)
		sprintf(flag,"ON");	
	sprintf(buff, "  flag_alarm_low_bat = %s", flag);
	Serial.println(buff);

	sprintf(flag,"OFF");
	//convert bit to string
	if(MyParam.flag_alarm_flood)
		sprintf(flag,"ON");	
	sprintf(buff, "  flag_alarm_flood = %s", flag);
	Serial.println(buff);
	
	sprintf(buff, "  smssecret = %s", MyParam.smssecret);
	Serial.println(buff);
	sprintf(buff, "  myphonenumber = %s", MyParam.myphonenumber);
	Serial.println(buff);
	sprintf(buff, "  radius = %d", MyParam.radius);
	Serial.println(buff);	
	sprintf(buff, "  base_lat = %2.6f", MyParam.base_lat);
	Serial.println(buff);
	sprintf(buff, "  base_lat_dir = %c", MyParam.base_lat_dir);
	Serial.println(buff);
	sprintf(buff, "  base_lon = %3.6f", MyParam.base_lon);
	Serial.println(buff);
	sprintf(buff, "  base_lon_dir = %c", MyParam.base_lon_dir);
	Serial.println(buff);
	sprintf(buff, "  lipo_level_trig = %d%%", MyParam.lipo_level_trig);
	Serial.println(buff);
	sprintf(buff, "  trig_input_level = %2.1fV", MyParam.trig_input_level);
	Serial.println(buff);	
}

//----------------------------------------------------------------------
//!\brief           scheduler()
//----------------------------------------------------------------------
void Scheduler() {

	if( (millis() - taskGetGPS) > PERIOD_GET_GPS){
		taskGetGPS = millis();
		MyFlag.taskGetGPS = true;	
	}
	
	if( (millis() - taskTestGeof) > PERIOD_TEST_GEOFENCING){
		taskTestGeof = millis();
		MyFlag.taskTestGeof = true;
	}
	
	if( (millis() - taskGetLiPo) > PERIOD_LIPO_INFO){
		taskGetLiPo = millis();
		MyFlag.taskGetLiPo = true;
	}	
	
	if( (millis() - taskCheckSMS) > PERIOD_CHECK_SMS){
		taskCheckSMS = millis();
		MyFlag.taskCheckSMS = true;
	}
	
	if( (millis() - taskCheckFlood) > PERIOD_CHECK_FLOOD){
		taskCheckFlood = millis();
		MyFlag.taskCheckFlood = true;
	}	
	
	if( (millis() - taskStatusSMS) > PERIODIC_STATUS_SMS){
		taskStatusSMS = millis();
		MyFlag.taskStatusSMS = true;
	}

	if( (millis() - taskGetAnalog) > PERIOD_READ_ANALOG){
		taskGetAnalog = millis();
		MyFlag.taskGetAnalog = true;
	}	

	if( (millis() - taskCheckInputVoltage) > PERIOD_CHECK_ANALOG_LEVEL){
		taskCheckInputVoltage = millis();
		MyFlag.taskCheckInputVoltage = true;
	}
	
	
	
	if( ((millis() - TimeOutSMSMenu) > TIMEOUT_SMS_MENU) && MySMS.menupos != SM_LOGIN){
		MySMS.menupos = SM_LOGIN;
		Serial.println("--- SMS Menu manager : Timeout ---");
	}
}

//----------------------------------------------------------------------
//!\brief           SETUP()
//----------------------------------------------------------------------
void setup() {
	// put your setup code here, to run once:
	Serial.begin(115200);
	
	// set I/O direction
	pinMode(LEDALARM, OUTPUT);
	pinMode(LEDGPS, OUTPUT);
	pinMode(FLOODSENSOR, INPUT);
	
	delay(5000);
	Serial.println("RoboMowTrak "); 
	// GPS power on
	LGPS.powerOn();
	Serial.println("GPS Powered on.");
	// set default value for GPS (needed for default led status)
	MyGPSPos.fix = Error;
	
	// LTask will help you out with locking the mutex so you can access the global data
    LTask.remoteCall(createThread1, NULL);
	//LTask.remoteCall(createThread2, NULL);
	Serial.println("Launch threads.");
	
	// GSM setup
	while(!LSMS.ready()){
		delay(1000);
		Serial.println("Please insert SIM");
	}
	Serial.println("SIM ready.");	
	
	Serial.println("Deleting SMS received ...");
	//delete ALL sms received while powered off
	while(LSMS.available()){
		LSMS.flush(); // delete message
	}
	
	// load params from EEPROM
	LoadParamEEPROM();
	//print structure
	PrintMyParam();
	
	// init default position in sms menu
	MySMS.menupos = SM_LOGIN;
	
	// for scheduler
	taskGetGPS = millis();
	taskTestGeof = millis();
	taskGetLiPo = millis();
	taskGetAnalog = millis();
	taskCheckSMS = millis();
	taskStatusSMS = millis();
	
	
	// set this flag to proceed a first LiPO level read (if an SMS is received before timer occurs)
	MyFlag.taskGetLiPo = true;
	// set this flag to proceed a first analog read (external supply)
	MyFlag.taskGetAnalog = true;
	
	//GPIO setup
	pinMode(LEDGPS, OUTPUT);
	pinMode(LEDALARM, OUTPUT);
	
	Serial.println("Setup done.");	
}

//----------------------------------------------------------------------
//!\brief           LOOP()
//----------------------------------------------------------------------
void loop() {
	Scheduler();
	GetGPSPos();
	GetLiPoInfo();
	GetAnalogRead();
	GetDigitalInput();
	CheckSMSrecept();
	MenuSMS();
	// SendGPS2Wifi();
	Geofencing();
	AlertMng();
}

//----------------------------------------------------------------------
//!\brief           THREAD DECLARATION
//----------------------------------------------------------------------
boolean createThread1(void* userdata) {
        // The priority can be 1 - 255 and default priority are 0
        // the arduino priority are 245
        vm_thread_create(thread_ledgps, NULL, 255);
    return true;
}

boolean createThread2(void* userdata) {
        // The priority can be 1 - 255 and default priority are 0
        // the arduino priority are 245
		vm_thread_create(thread_ledalarm, NULL, 255);
    return true;
}

//----------------------------------------------------------------------
//!\brief           THREAD LED GPS
//---------------------------------------------------------------------- 
VMINT32 thread_ledgps(VM_THREAD_HANDLE thread_handle, void* user_data){
    for (;;){
		switch(MyGPSPos.fix){
			case Invalid:
				// blink led as pulse
				digitalWrite(LEDGPS, HIGH);
				delay(500);
				digitalWrite(LEDGPS, LOW);
				delay(500);
				break;
			case GPS:
			case DGPS:
			case PPS:
			case RTK:
			case FloatRTK:
			case DR:
			case Manual:
			case Simulation:
				// blink led as slow pulse
				digitalWrite(LEDGPS, HIGH);
				delay(150);
				digitalWrite(LEDGPS, LOW);
				delay(2850);
				break;
			case Error:
				// Fast blinking led
				digitalWrite(LEDGPS, HIGH);
				delay(100);
				digitalWrite(LEDGPS, LOW);
				delay(100);
				break;
		}
		//DEBUG
		// sprintf(buff, "MyGPSPos.fix = %d", MyGPSPos.fix);
		// Serial.println(buff);
		// delay(1000);
	}
    return 0;
}

//----------------------------------------------------------------------
//!\brief           THREAD LED ALARM
//---------------------------------------------------------------------- 
VMINT32 thread_ledalarm(VM_THREAD_HANDLE thread_handle, void* user_data){
    for (;;){
        Serial.println("test thread2");
        delay(2000);
    }
    return 0;
}