#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
// Include queue support
#include <queue.h>
//define pin
#define ena1 5
#define ena2 6
#define in1 10
#define in2 4
#define in3 11
#define in4 7
#define  trig 8
#define  echo 9
MPU6050 mpu6050(Wire);
QueueHandle_t integerQueue1;


// define three Tasks


void Taskcontrolmotor( void *pvParameters );
void Taskavoidobstacle( void *pvParameters);


// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  // set up queue
integerQueue1 = xQueueCreate(1, // Queue length
                              sizeof(int) // Queue item size
                              );;                          

  // Now set up three Tasks to run independently.
  
  xTaskCreate(
    Taskcontrolmotor
    ,  "dieu khien dong co"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );
  xTaskCreate(
    Taskavoidobstacle
    ,  "tranh vat can"
    ,  128  // Stack size
    ,  NULL
    ,  0  // Priority
    ,  NULL );
//  xTaskCreate(
//    Taskdetectzone
//    ,  "phat hien zone"
//    ,  128  // Stack size
//    ,  NULL
//    ,  2  // Priority
//    ,  NULL );  
  // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started.
}

// khai bao bien toan cuc

int AngleZ ;
void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void Taskavoidobstacle( void *pvParameters)
{ 
  pinMode(echo,INPUT);
  pinMode(trig,OUTPUT);
  unsigned long thoigian;
  int khoangcach;
  int OB=0;
 for(;;)
    { 
      digitalWrite(trig,0);
      delayMicroseconds(2);
      digitalWrite(trig,1);
      delayMicroseconds(10);
      digitalWrite(trig,0);
      //chan echo se nhan xung phan xa lai va do do rong xung
      thoigian=pulseIn(echo,HIGH);
      //khoang cach= thoi gian x van toc
      //van toc am thanh trong khong khi =340m/s tuong duong 29,412 cm/s
      //chia cho 2 vi khi gui xung tinh tg di va ve
     xQueueSend(integerQueue1, &OB, portMAX_DELAY);
      khoangcach= thoigian /2 / 29.412;
      if(khoangcach <= 15) //neu khoang cach nho hon hoac bang 10cm
        { OB=1;
//        
          xQueueSend(integerQueue1, &OB, portMAX_DELAY);
           // One tick delay (15ms) in between reads for stability
           vTaskDelay (2 );
           
          }
      }
  
  }
 void Taskcontrolmotor( void *pvParameters)
{
  pinMode(4, OUTPUT);//in2 left Motor
  pinMode(10,OUTPUT);//in1 
  pinMode(5, OUTPUT);//en1
  pinMode(7, OUTPUT);//in4  right Motor
  pinMode(6, OUTPUT);//en2
  pinMode(11,OUTPUT);//in3
  int avoided=0;
  int OB=0;
  for(;;)    
  {
     mpu6050.update();
    AngleZ = (mpu6050.getAngleZ());
    Serial.print("goc z la");
     Serial.println(AngleZ);
  if (xQueueReceive(integerQueue1, &OB, portMAX_DELAY) == pdPASS) 
        {
    if(OB==0)
  {
    mpu6050.update();
    AngleZ = (mpu6050.getAngleZ());
    
     Serial.println(AngleZ);
     
        if (AngleZ >= -2 && AngleZ <= 2)
      {
      digitalWrite(in1, 1);
      digitalWrite(in3, 1);
      analogWrite(ena1, 77);
      analogWrite(ena2, 83);
      digitalWrite(in2, 0);
      digitalWrite(in4, 0);
        Serial.println("di thang");
      }
       if (AngleZ > 2)
      {
      digitalWrite(in1, 1);
      digitalWrite(in3, 1);
      analogWrite(ena1,66);
      analogWrite(ena2, 55);
      digitalWrite(in4, 0);
      digitalWrite(in2, 0);
        Serial.println("re trai");
      }
      if (AngleZ < -2)
      {
      digitalWrite(in1, 1);
      digitalWrite(in3, 1);
      analogWrite(ena1, 58);
      analogWrite(ena2, 68);
      digitalWrite(in2, 0);
      digitalWrite(in4, 0);
        Serial.println("re phai");
      }
  }
      

          if(OB==1&& avoided==0 )
           {mpu6050.update();
       AngleZ = (mpu6050.getAngleZ()); 
            Serial.println("co vat can");
            digitalWrite(in1, 0);
            digitalWrite(in3, 0);
           analogWrite(ena1,0);
           analogWrite(ena2, 0);
           digitalWrite(in4, 0);
           digitalWrite(in2, 0);
            

            }
       
        
      
  }
  }}
