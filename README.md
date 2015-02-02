RoboMowTrack
============

GPS tracker on a linkitone platform

More info about linkitone : 

- http://labs.mediatek.com/
- http://www.seeedstudio.com/depot/LinkIt-ONE-p-2017.html

![Linkitone pic](/docs/Linkitone.jpg)

How it works ?
============

![RoboMowTrack architecture](/docs/diagram.png)


 * Use GPS for geolocalisation.
 * Fully configurable by SMS.
 * SMS alert only.
 * Read voltage input and can set an alarm on low power.
 * Monitor LiPo cell voltage and can set an alarm on low power.
 * Flood sensor interface (GPIO).
 * Serial messages are for debug purpose only.
 
 Other possibilities :
 
 * Monitor temperature(s), pressure, temperature, shocks ... (choose your sensor !)
 * Wifi for tracking/logging position while local wifi available

Applications
============ 

 * My application : GPS tracker / Geofencing for my robomow (anti theft)
 * Other application without modifying source code : Geofencing, monitoring for yatch / boat parked at the harbor, ...
 * Other applications which some modification : GPS tracker for car / truck / caravan , freezer monitoring in second home when power outage, ...
 

Principles
============

##Geofencing

The device check if GPS position is inside an round area centered on an custom coordinates.
When device is outside this area, an SMS alert is sent to your phone.
SMS will contain an url to googlemap with the actual GPS position.
Area coordinates, radius and phone number is configurable by SMS.

##Analog inputs

The device can monitor some analog inputs :

-  LiPo battery : linkitone API only return these levels : 0 - 33 - 66 - 100% . So, by default, an SMS alert is sent to your phone when LiPo voltage is lower than 33%. This trigger is not customizable by SMS.

- Power supply voltage : device read voltage of this input and can send an SMS when is lower than a trigger. This value can be set by SMS. Default value is 11.6V (for a 12V lead battery).

##Digital inputs

The device can monitor digital input. By default, a flood sensor is connected. Device can send an SMS if the sensor detect water.

##Settings

A lot of parameters can bu set by SMS. You need to send your secret code to the device to receive the menu. Default secret code is *1234*.

Available settings are :

- Set geofencing alarm ON/OFF
- Change phone number where are sent alarms
- Change coordinates of the area. You can send latitude/longitude coordinates if you know them, or you can set coordinates with actual GPS position of the device (need a good GPS fix!).
- Change radius of the area. A large radius is more tolerant, a small radius is more restrictive. Do not set radius that is less than the GPS precision (~10m).
- Set periodic SMS ON/OFF. When true, the device will send you a daily status SMS.
- Set low power alarm ON/OFF. When true, the device will send you an SMS when it detects low voltage on analog inputs.
-  Change low power voltage trigger : you can set voltage for analog input, when voltage is lower than this value, device will send you SMS.
- Restore factory settings.

## Serial port

Serial port is available on micro USB connector. It is only for debugging purpose. No maintenance or configuration messages are available through this serial port.

Screenshots
============

#SMS menu

![Linkitone pic](/docs/SMS_menu.jpg)

#SMS alarms

![Linkitone pic](/docs/SMS_alarms.jpg)

 
Instructions
============

Don't forget to set your settings in this file :

	~/robomowtrak/myprivatedata.h
	

Compile files and uploard to linkitone with arduino IDE (follow instructions in Developer's guide)

http://labs.mediatek.com/site/global/developer_tools/mediatek_linkit/sdk_intro/index.gsp

User 'modem port' to check debug message (SIM detected, GPS Fix, ...)
	
	

Troublshooting
============

If data in EEPROM are corrupted, in the function LoadParamEEPROM() please uncomment 

	MyParam.flag_data_written = false;
	
Compile the code, run it once (at least) and recompile the code this line commented.
Corrupted data in EEPROM can do impossible connexion to linkitone by SMS (secret code corrupted!).

