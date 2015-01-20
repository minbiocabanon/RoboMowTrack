RoboMowTrack
============

GPS tracker on a linkitone platform

More info about linkitone : http://labs.mediatek.com/

Instuctions
============

Don't forget to set your settings in this file :
	~/robomowtrak/myprivatedata.h
	
If data in EEPROM are corrupted, in the function LoadParamEEPROM() please uncomment 
	MyParam.flag_data_written = false;
	
Compile the code, run it once (at least) and recompile the code this line commented.
Corrupted data in EEPROM can do impossible connexion to linkitone by SMS (secret code corrupted!).

