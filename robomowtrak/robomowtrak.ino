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
//! millis() overflow/reset over 50 days
//--------------------------------------------------

#include <LGPS.h>
#include <LGSM.h>
#include <LBattery.h>
#include <math.h>
#include <LEEPROM.h>
#include "EEPROMAnything.h"
#include "myprivatedata.h"

//Params for geofencing
#define RADIUS_MINI		20.0		// radius in meter where we consider that we are exactly parked in the area
#define RADIUS_MAXI		80.0		// radius in meter for geofencing centered in BASE_LAT,BASE_LON. When GPS pos is outside this radius -> Alarm !

#define	PERIOD_GET_GPS			5000	// interval between 2 GPS positions in milliseconds
#define	PERIOD_TEST_GEOFENCING	120000	// interval between 2 geofencing check
#define PERIOD_BAT_INFO			120000	// interval between 2 battery level measurement
#define PERIOD_CHECK_SMS		2000	// interval between 2 SMS check
#define TIMEOUT_SMS_MENU		300000	// When timeout, SMS menu return to login (user should send password again to log)

// SMS menu architecture
#define TXT_MAIN_MENU	"Main Menu \r\n1 : Status\r\n2 : Alarm ON\r\n3 : Alarm OFF"

gpsSentenceInfoStruct info;
char buff[256];
unsigned long taskGetGPS;
unsigned long taskTestGeof;
unsigned long taskGetBat;
unsigned long taskCheckSMS;
unsigned long TimeOutSMSMenu;



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
	}MyGPSPos;


struct Battery {
	unsigned int bat_level;
	unsigned int charging_status;
	}MyBattery;

struct SMS {
	char message[256];
	char incomingnumber[13];
	int menupos;
	int menulevel;
	}MySMS;
	
struct FlagReg {
	bool taskGetGPS;	// flag to indicate when process to get GPS possition
	bool taskGetBat;		// flag to indicate that we have to get battery level and charging status
	bool taskTestGeof;	// flag to indicate when process geofencing
	bool taskCheckSMS;	// flag to indicate when check SMS
	bool SMSReceived;	// flag to indicate that an SMS has been received
	bool fix3D;			// flag to indicate if fix is 3D (at least) or not
	bool PosOutiseArea;	// flag to indicate if fix is 3D (at least) or not
	}MyFlag;



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
	}GPSfix;


enum SMSMENU{
	SM_NOSTATE,		//0
	SM_LOGIN,		//1
	SM_MENU_MAIN,	//2
	SM_ACTION1,		//3
	SM_ACTION2,		//4
	SM_ACTION3,		//5
	SM_ACTION4,		//6
	SM_ACTION5,		//7
	SM_ACTION6,		//8
	SM_ACTION7		//9
	};

enum CMDSMS{
	CMD_NOPE,			//0
	CMD_STATUS,			//1
	CMD_ALM_ON,			//2
	CMD_ALM_OFF,		//3
	CMD_ACTION2,		//4
	CMD_ACTION3,		//5
	CMD_ACTION4,		//6
	CMD_ACTION5,		//7
	CMD_ACTION6,		//8
	CMD_ACTION7			//9
	};	

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
		Serial.print((char*)info.GPGGA); 
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
//!\brief	Grab battery level and status
//!\return  -
//----------------------------------------------------------------------
void GetBatInfo(void){
	// For one second we parse GPS data and report some key values
	if(MyFlag.taskGetBat){
		Serial.println("-- Battery Info --");
		MyFlag.taskGetBat = false;
		MyBattery.bat_level = LBattery.level();
		MyBattery.charging_status  = LBattery.isCharging();
		sprintf(buff,"battery level = %d%", MyBattery.bat_level );
		Serial.print(buff);
		sprintf(buff," is charging = %d \n", MyBattery.charging_status );
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
//!\brief	Verify SMS password received
//!\return  boolean true ou false
//----------------------------------------------------------------------
bool CheckSMSPassword(){
	// size_t destination_size = sizeof (MyParam.smssecret);
	// Serial.print("MyParam.smssecret=");
	// for(int i = 0; i <= destination_size; i++)
		// Serial.print(MyParam.smssecret[i]);

	if( strcmp(MySMS.message, MyParam.smssecret) == 0)
		return true;
	else
		return  false;
}

//----------------------------------------------------------------------
//!\brief	Does action selected by user in the main menu
//!\return  -
//----------------------------------------------------------------------
void ProcessMenuMain(void){
	int val = atoi(MySMS.message);
	char flagalarm[4];
	switch(val){
		case CMD_STATUS:		//status
			Serial.println("Status required ");
			//convert flag_alarm_onoff

			sprintf(flagalarm,"OFF");
			if(MyParam.flag_alarm_onoff)
				sprintf(flagalarm,"ON");
			sprintf(buff, "Status : \r\nCurrent position is : https://www.google.com/maps?q=%2.6f%c,%3.6f%c \r\nBat = %d, status = %d\r\nAlarm is %s.", MyGPSPos.latitude, MyGPSPos.latitude_dir, MyGPSPos.longitude, MyGPSPos.longitude_dir, MyBattery.bat_level, MyBattery.charging_status, flagalarm); 
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
		case CMD_ACTION2:
			break;
		default:
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
			case SM_NOSTATE:
				break;
			
			case SM_LOGIN:
				if( CheckSMSPassword() == true){
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
	LSMS.beginSMS(phonenumber);
	LSMS.print(message);
	bool ret;
	if(LSMS.endSMS()){
		Serial.println("SMS is sent");
		ret = true;
	}
	else{
		Serial.println("SMS is not sent");
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
		Serial.println("AlertMng : start sending SMS"); 
		sprintf(buff, "Robomow Alert !! current position is : https://www.google.com/maps?q=%2.6f%c,%3.6f%c \n Bat = %d, status = %d", MyGPSPos.latitude, MyGPSPos.latitude_dir, MyGPSPos.longitude, MyGPSPos.longitude_dir, MyBattery.bat_level, MyBattery.charging_status); 
		Serial.println(buff);
		SendSMS(MyParam.myphonenumber, buff);
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
	
	if( (millis() - taskGetBat) > PERIOD_BAT_INFO){
		taskGetBat = millis();
		MyFlag.taskGetBat = true;		
	}	
	
	if( (millis() - taskCheckSMS) > PERIOD_CHECK_SMS){
		taskCheckSMS = millis();
		MyFlag.taskCheckSMS = true;		
	}
	
	if( ((millis() - TimeOutSMSMenu) > TIMEOUT_SMS_MENU) && MySMS.menupos != SM_LOGIN){
		MySMS.menupos = SM_LOGIN;	
		Serial.println("--- SMS Menu manger : Timeout ---");
	}
}

//----------------------------------------------------------------------
//!\brief	Load params from EEPROM
//----------------------------------------------------------------------
void LoadParamEEPROM() {
	
	EEPROM_readAnything(0, MyParam);
	
	//uncomment this line to erase EEPROM parameters with DEFAULT parameters
	//MyParam.flag_data_written = false;
	
	//check if parameters were already written
	if( MyParam.flag_data_written == false ){
		Serial.println("--- !!! Loading DEFAULT parameters from EEPROM ...  --- ");
		//EEPROM is empty , so load default parameters (see myprivatedata.h)
		MyParam.flag_alarm_onoff = FLAG_ALARM_ONOFF;
		
		size_t destination_size = sizeof (MyParam.smssecret);
		snprintf(MyParam.smssecret, destination_size, "%s", SMSSECRET);
		destination_size = sizeof (MyParam.myphonenumber);
		snprintf(MyParam.myphonenumber, destination_size, "%s", MYPHONENUMBER);		
		MyParam.base_lat = BASE_LAT;
		MyParam.base_lat_dir = BASE_LAT_DIR;
		MyParam.base_lon = BASE_LON;
		MyParam.base_lon_dir = BASE_LON_DIR;
		MyParam.bat_level_trig = BAT_LEVEL_TRIG;
	
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
	
	// load params from EEPROM
	LoadParamEEPROM();
	
	// init default position in sms menu
	MySMS.menupos = SM_LOGIN;
	
	// for scheduler
	taskGetGPS = millis();
	taskTestGeof = millis();
	taskGetBat = millis();
	taskCheckSMS = millis();
	
}

//----------------------------------------------------------------------
//!\brief           LOOP()
//----------------------------------------------------------------------
void loop() {
	Scheduler();
	GetGPSPos();
	GetBatInfo();
	CheckSMSrecept();
	MenuSMS();
	// SendGPS2Wifi();
	Geofencing();
	AlertMng();
}
