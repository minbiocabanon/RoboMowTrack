#include <LGSM.h>

void setup() {
	Serial.begin(9600);
	while(!LSMS.ready())
		delay(1000);
	Serial.println("SIM ready for work!");
	LSMS.beginSMS("0102030405");
	LSMS.print("Hello from LinkIt");
	if(LSMS.endSMS()){
		Serial.println("SMS is sent"); 
	}
	else{
		Serial.println("SMS is not sent");
	}
}
void loop()
{
	// do nothing
}