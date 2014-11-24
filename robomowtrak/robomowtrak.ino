//--------------------------------------------------
//! \file		robomowtrak.ino
//! \brief		GPS tracker for a robomow
//! \brief		User GPS for localisation. Wifi for tracking/logging position while in my garden. When outsite my garden, use SMS to send alert.
//! \date		2014-Nov
//! \author		minbiocabanon
//--------------------------------------------------

//--------------------------------------------------
//! some notes here :
//! google url to send by sms : https://www.google.com/maps?q=<lat>,<lon>
//!
//--------------------------------------------------

#include <LGPS.h>
#include <LGSM.h>
#include <math.h>
#include "myprivatedata.h"

//Params for geofencing
#define RADIUS_MINI		20.0		// radius in meter where we consider that we are exactly parked in the area
#define RADIUS_MAXI		80.0		// radius in meter for geofencing centered in BASE_LAT,BASE_LON. When GPS pos is outside this radius -> Alarm !

#define	PERIOD_GET_GPS	5000			// interval between 2 GPS positions in milliseconds
#define	PERIOD_TEST_GEOFENCING	60000	// interval between 2 geofencing check

gpsSentenceInfoStruct info;
char buff[256];
unsigned long taskGetGPS;
unsigned long taskTestGeof;



struct GPSPos {
	float latitude;
	char latitude_dir;
	float longitude;
	char longitude_dir;
	int hour;
	int minute;
	int second;
	int	num;
	int fix;
};
GPSPos MyGPSPos;

struct FlagReg {
	bool taskGetGPS;	// flat to indicate when process to get GPS possition
	bool taskTestGeof;	// flat to indicate when process geofencing
	bool fix3D;			// flag to indicate if fix is 3D (at least) or not
	bool PosOutiseArea;	// flag to indicate if fix is 3D (at least) or not
};
FlagReg MyFlag;


enum FixQuality {
	Invalid,	// 0
	GPS,		// 1
	DGPS,		// 2
	PPS,		// 3
	RTK,		// 4 Real Time Kinematic
	FloatRTK,	// 5
	DR,			// 6 Dead Reckoning
	Manual,		// 7
	Simulation	// 8
};
FixQuality GPSfix;

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
		sprintf(buff, "UTC timer %2d-%2d-%2d", MyGPSPos.hour, MyGPSPos.minute, MyGPSPos.second);
		Serial.println(buff);
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
		
		sprintf(buff, "latitude = %10.4f-%c, longitude = %10.4f-%c", MyGPSPos.latitude, MyGPSPos.latitude_dir, MyGPSPos.longitude, MyGPSPos.longitude_dir);
		Serial.println(buff); 
		
		//get GPS fix quality
		tmp = getComma(6, GPGGAstr);
		MyGPSPos.fix = getIntNumber(&GPGGAstr[tmp]);    
		sprintf(buff, "GPS fix quality = %d", MyGPSPos.fix);
		Serial.println(buff);   
		//get satellites in view
		tmp = getComma(7, GPGGAstr);
		MyGPSPos.num = getIntNumber(&GPGGAstr[tmp]);    
		sprintf(buff, "satellites number = %d", MyGPSPos.num);
		Serial.println(buff); 
	}
	else{
		Serial.println("Not get data"); 
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
		Serial.println((char*)info.GPGGA); 
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
		
		char buff[256];
		sprintf(buff, "Current position is : https://www.google.com/maps?q=%2.6f%c,%3.6f%c", MyGPSPos.latitude, MyGPSPos.latitude_dir, MyGPSPos.longitude, MyGPSPos.longitude_dir);
		Serial.println(buff);
		Serial.println();
	
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
		//compute distance between actual position and reference position
		float distance_base = DistanceBetween(BASE_LAT, BASE_LON, MyGPSPos.latitude, MyGPSPos.longitude);
		Serial.print("distance BASE->Robot: ");
		Serial.println(distance_base,1);
		
		//check where we are
		if(distance_base <= RADIUS_MINI){
			Serial.println("Position is inside area");
			MyFlag.PosOutiseArea = false;
		}
		else if((distance_base > RADIUS_MINI) && (distance_base <= RADIUS_MAXI) ){
			Serial.println("We are inside area, keep cool !!");
			MyFlag.PosOutiseArea = false;
		}
		else{
			Serial.println("ALARM, outside AREA !!!!");
			MyFlag.PosOutiseArea = true;
		}
	}
}

//----------------------------------------------------------------------
//!\brief	Manage alert when occurs
//!\return  -
//----------------------------------------------------------------------
void AlertMng(void){

	if (MyFlag.PosOutiseArea){
		MyFlag.PosOutiseArea = false;
		Serial.println("AlertMng : start sending SMS"); 
		
		LSMS.beginSMS(MYPHONENUMBER);
		char SMSbuff[256];
		sprintf(SMSbuff, "Robomow Alert !! current position is : https://www.google.com/maps?q=%2.6f%c,%3.6f%c", MyGPSPos.latitude, MyGPSPos.latitude_dir, MyGPSPos.longitude, MyGPSPos.longitude_dir);
		Serial.println(SMSbuff);
		LSMS.print(SMSbuff);
		if(LSMS.endSMS()){
			Serial.println("SMS is sent"); 
		}
		else{
			Serial.println("SMS is not sent");
		}
	}
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
}

//----------------------------------------------------------------------
//!\brief           SETUP()
//----------------------------------------------------------------------
void setup() {
	// put your setup code here, to run once:
	Serial.begin(115200);
	Serial.println("RoboMowTrak "); 
	// GPS power on
	LGPS.powerOn();
	Serial.println("LGPS Power on, and waiting ..."); 
	
	// GSM setup
	while(!LSMS.ready())
		delay(1000);
	Serial.println("SIM ready for work!");	
	
	// for scheduler
	taskGetGPS = millis();
	taskTestGeof = millis();
	
}

//----------------------------------------------------------------------
//!\brief           LOOP()
//----------------------------------------------------------------------
void loop() {
	Scheduler();
	GetGPSPos();
	// SendGPS2Wifi();
	Geofencing();
	AlertMng();
}
