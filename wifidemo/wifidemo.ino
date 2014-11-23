#include <LWiFi.h>
#include <LWiFiClient.h>
#define SITE_URL "www.mediatek.com" 
#define WIFI_AP "VIRUS"  // replace with your setting
#define WIFI_PWD "MINBIOCABANON"  // replace with your setting

LWiFiClient c;

void setup() { 
	Serial.begin(9600);
	LWiFi.begin();
	Serial.println();
	Serial.print("Connecting to AP...");
	if(LWiFi.connectWEP(WIFI_AP, WIFI_PWD) < 0){
		Serial.println("FAIL!");
		return;
	}
	Serial.println("ok");
	Serial.print("Connecting to site...");
	if(!c.connect(SITE_URL, 80)){
		Serial.println("FAIL!");
		return;
	}
	Serial.println("ok");
	Serial.println("send HTTP GET request");
	c.println("GET / HTTP/1.1");
	c.println("Host: " SITE_URL);
	c.println("Connection: close");
	c.println();
}

void loop() {
	int v;
	while(c.available()){
		v = c.read();
		if(v < 0)
			break;
		Serial.print((char)v);
	}
	delay(100);
}