/*Mini Task of Huge Project*/
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include "task.h"
#include "queue.h"
/*Thư viện MPU6050*/
#include <MPU6050_tockn.h>
#include <Wire.h>
MPU6050 mpu6050(Wire);
long timer = 0;
float z0,z =0 ;

/*Sử dụng bộ P*/
float Kp = 1;

/*Sử dụng thư viện Software Serial*/
#include <SoftwareSerial.h>
SoftwareSerial mySerial (6,7);

#define ULONG_MAX 0xFFFFFFFF

#define ENA 11
#define IN1 12
#define IN2 13
#define IN3 10
#define IN4 8
#define ENB 9

void TaskZone(void *pvParameters);
void TaskDrive(void *pvParameters);
void TaskMPU(void *pvParameters);

static TaskHandle_t TaskCheckHandle = NULL;// su dung bien cuc bo cho ca chuong trinh
static TaskHandle_t TaskDriveHandle = NULL;
static TaskHandle_t TaskMPUHandle = NULL;
QueueHandle_t  Queue,Queue1;

int count =0;
int read_count = 0;

static int sensorValue = 0;
void _start()
{
  // Cho phép xe chạy
  digitalWrite(ENB,HIGH);
  digitalWrite(ENA,HIGH);
}

void _stop()
{
  // Tắt xe 
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
}

void forward(){
  _start();// Cho phép xe chạy

  // set up chiều của xe
  digitalWrite(IN2,HIGH);
  digitalWrite(IN1,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
}

void reverse(){
  _start();// Cho phép xe chạy

  // set up chiều của xe
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
}

void turnRight(){
  _start();// Cho phép xe chạy

  // set up chiều của xe
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
}

void turnLeft(){
  _start();// Cho phép xe chạy

  // set up chiều của xe
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
}

void setSpeed(int motorspeed2){
  xQueueReceive(Queue1,&motorspeed2,portMAX_DELAY);
  analogWrite(ENA, motorspeed2);
  analogWrite(ENB, motorspeed2);
}

void setup() 
{
  pinMode(ENA,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENB,OUTPUT);;
    
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  Queue = xQueueCreate(1, sizeof(int));
  Queue1 = xQueueCreate(1, sizeof(int));
  if(Queue!=NULL||Queue1!=NULL )
  {
    
  xTaskCreate(
        TaskZone
        ,  "Zone"   
        ,  128 
        ,  NULL
        ,  0 
        ,  &TaskCheckHandle);   

  xTaskCreate(
        TaskDrive
        ,  "Drive" 
        ,  128  
        ,  NULL
        ,  0  
        ,  &TaskDriveHandle);  

  xTaskCreate(
        TaskMPU
        ,  "MPU"   
        ,  128 
        ,  NULL
        ,  0 
        ,  &TaskMPUHandle);    
  }
  vTaskStartScheduler(); 
}

void TaskZone(void *pvParameters ) 
{
  for(;;) 
  {
    sensorValue = analogRead(A0);
    if (sensorValue > 500)
    {
      count ++;
      Serial.print("count: ");Serial.println(count);
      Serial.print("sensorValue: ");Serial.println(sensorValue);

      vTaskDelay(20);// chờ cho xe đi qua zone thì mới kiểm tra tiếp       
    }
    if (sensorValue < 200)
    {
      digitalWrite(3,HIGH);
      Serial.println("nothing happened");
      vTaskDelay(20);
    }
    if (count ==3)
    {
      xQueueSend (Queue,&count,portMAX_DELAY);
      xTaskNotify(TaskDriveHandle,0,eNoAction);
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (count ==5)
    {
      xQueueSend (Queue,&count,portMAX_DELAY);
      xTaskNotify(TaskDriveHandle,0,eNoAction);
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

void TaskMPU(void *pvParameters ) 
{
  int state = 1;
  int pwm2, pwm;
  for(;;)
  {
    mpu6050.update();

    if(millis() - timer > 1000)
    {
      z = mpu6050.getAngleZ(); //state 2 , vd : z=10;
      if(state == 1)
      {
        z0 = z;
        state ++;
      }
    //Serial.print("z=");
    //Serial.println(z);
    //Serial.print("\pwm2=");
    //Serial.println(pwm2);
    if ((z - z0) > 0.1 || (z - z0) < 0.1)
    {
      /*Bộ điều khiển P*/
      pwm2 = pwm2 + Kp*(float)(z-z0);

      if(pwm2 > (pwm+4)) pwm2 = (pwm+4);
      
      if(pwm2 <(pwm-4) ) pwm2 = (pwm-4);
    }
    else
    {
      pwm2 = pwm;
    }
    xQueueSend (Queue1,&pwm2,portMAX_DELAY);
    timer = millis();
    }
  }
}

void TaskDrive  (void *pvParameters ) 
{
  uint32_t ulNotifiedValue;
  enum States{FORWARD,REVERSE,TURN,STOP};
  States state=FORWARD;
  for(;;) 
  {  
    switch(state) 
    {
      case FORWARD:  
       while(mySerial.available () == 0)
      {
        Serial.println("Chờ cho đến khi được nhấn");
      }      
      forward();
      setSpeed(70);
        
      xTaskNotifyWaitIndexed( 0,
                        ULONG_MAX, /* Clear all notification bits on entry. */
                        ULONG_MAX, /* Reset the notification value to 0 on exit. */
                        &ulNotifiedValue,
                        portMAX_DELAY);

      xQueueReceive(Queue,&read_count,portMAX_DELAY);
      Serial.print("the varialable named count was sent by TaskZone (3 or 5) is: ");Serial.println(read_count);
      if (read_count==3)
      { 
        Serial.println("count = 3, then this task is unblocked");
        state=REVERSE;
      }
      if (read_count ==5)
      {
        Serial.println("count = 5, then this task is unblocked");
        state = STOP;
      }
      break;
        
                
      case REVERSE:
      reverse();
      setSpeed(70);
      vTaskDelay(pdMS_TO_TICKS(1000));
      state=TURN;      
      break;
  
        
      case TURN:     
      turnRight();
      setSpeed(70);
      vTaskDelay(pdMS_TO_TICKS(1000));
      state=FORWARD;      
      break; 

        
      case STOP:
      _stop();
      setSpeed(0);  
    }
  }
}






void loop() 
{
  // empty
}