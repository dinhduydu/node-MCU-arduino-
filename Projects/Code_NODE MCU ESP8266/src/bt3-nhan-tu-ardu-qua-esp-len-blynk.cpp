#define BLYNK_TEMPLATE_ID "TMPLjw14UP6r"
#define BLYNK_DEVICE_NAME "autonomous vehicle monitoring on blynk"
#define BLYNK_AUTH_TOKEN "YNKiHmPxWv2UIMysAf898THdUaBiJog8"

#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

#include <SoftwareSerial.h>
SoftwareSerial s(D3, D4);

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Xom Tro 2";
char pass[] = "Thao270296";

int  point,zone = 0;

BlynkTimer timer;
BLYNK_WRITE(V0)
{
    int pinValueA = param.asInt(); 
    
    if (pinValueA == 1)
    {
        s.write(pinValueA);
        Serial.print(pinValueA);
    }
    else
    {
        s.write(pinValueA);
        Serial.print(pinValueA);
    }
}


BLYNK_WRITE(V1)
{
    int pinValueB = param.asInt(); 
    if (pinValueB == 1)
    {
        s.write(pinValueB+1);
        Serial.print(pinValueB+1);
    }
    else
    {
        s.write(pinValueB);
        Serial.print(pinValueB);
    }  
}


BLYNK_CONNECTED()
{
    Blynk.syncVirtual(V0);
    Blynk.syncVirtual(V1);
    Blynk.syncVirtual(V2);
    Blynk.syncVirtual(V3);
}

/*send zone to cloud*/
void myTimerEvent()
{
    Blynk.virtualWrite(V2, zone);
    Blynk.virtualWrite(V3, point);
    // Blynk.virtualWrite(V4, )
}



void setup()
{
    // pinMode(D5, INPUT);
    // pinMode(D6, OUTPUT);
    s.begin(115200);
    Serial.begin(9600);

    Blynk.begin(auth, ssid, pass);
    timer.setInterval(10L, myTimerEvent);
}

void loop()
{
    while(s.available()==0)
    {
        Blynk.run();
        timer.run();
    }

    zone = s.read();
    point = zone*20;
    Serial.println(zone);
}
