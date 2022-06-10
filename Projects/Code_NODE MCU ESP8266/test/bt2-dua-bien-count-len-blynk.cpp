/*************************************************************

  You can use this sketch as a debug tool that prints all incoming values
  sent by a widget connected to a Virtual Pin 1 in the Blynk App.

  App project setup:
    Slider widget (0...100) on V1
 *************************************************************/

// Template ID, Device Name and Auth Token are provided by the Blynk.Cloud
// See the Device Info tab, or Template settings
#define BLYNK_TEMPLATE_ID "TMPLjw14UP6r"
#define BLYNK_DEVICE_NAME "autonomous vehicle monitoring on blynk"
#define BLYNK_AUTH_TOKEN "UveQFGeEb0wVWRjKMqzCnabBnOf4nhrk"

// Comment this out to disable prints and save space
#define BLYNK_PRINT Serial


#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

char auth[] = BLYNK_AUTH_TOKEN;

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Xom Tro 2";
char pass[] = "Thao270296";

// This function will be called every time Slider Widget
// in Blynk app writes values to the Virtual Pin 1
int  count =0;
BlynkTimer timer;
BLYNK_WRITE(V0)
{
    int pinValueA = param.asInt(); // assigning incoming value from pin V0 to a variable
    // You can also use:
    // String i = param.asStr();
    // double d = param.asDouble();
    if (pinValueA == 1)
    {
        digitalWrite(D2,HIGH);
        Serial.println(1);
        delay(500);
    }
    else
    {
        digitalWrite(D2,LOW);
        Serial.println(0);
        delay(500);
        
    }
}


BLYNK_WRITE(V1)
{
    int pinValueB = param.asInt(); // assigning incoming value from pin V1 to a variable
    // You can also use:
    // String i = param.asStr();
    // double d = param.asDouble();
    if (pinValueB == 1)
    {
        digitalWrite(D3,HIGH);
        Serial.println(2);
        delay(500);
    }
    else
    {
        digitalWrite(D3,LOW);
        Serial.println(0);
        delay(500);
    }  
}

BLYNK_CONNECTED()
{
    Blynk.syncVirtual(V0);
    Blynk.syncVirtual(V1);
    Blynk.syncVirtual(V2);
}

void myTimerEvent()
{
     
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V2, count);
}



void setup()
{
    pinMode(D2,OUTPUT);
    pinMode(D3,OUTPUT);
  // Debug console
  Serial.begin(9600);

  Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);

  timer.setInterval(1000L, myTimerEvent);
}

void loop()
{
    count++;
    if (count == 100)
    {
        count =0;
    }
    Serial.print(count);


  Blynk.run();
  timer.run();
}
