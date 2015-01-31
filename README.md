RoboMowTrack
============

GPS tracker on a linkitone platform

More info about linkitone : http://labs.mediatek.com/

How it works ?
============

![Domini architecture](/docs/diagram.png)


 * Use GPS for geolocalisation.
 * Fully configurable by SMS.
 * SMS alert only.
 * Read voltage input and can set an alarm on low power.
 * Monitor LiPo cell voltage and can set an alarm on low power.
 * Flood sensor interface (GPIO).
 * Serial messages are for debug purpose only.
 
 Other possibilities
 * Monitor temperature(s), pressure, temperature, shocks ... (choose your sensor !)
 * Wifi for tracking/logging position while local wifi available

Instuctions
============

Don't forget to set your settings in this file :
	~/robomowtrak/myprivatedata.h
	
If data in EEPROM are corrupted, in the function LoadParamEEPROM() please uncomment 
	MyParam.flag_data_written = false;
	
Compile the code, run it once (at least) and recompile the code this line commented.
Corrupted data in EEPROM can do impossible connexion to linkitone by SMS (secret code corrupted!).

