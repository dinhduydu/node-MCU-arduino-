/*Mini Task of Huge Project*/
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include "task.h"
#include "queue.h"
#define ULONG_MAX 0xFFFFFFFFUL

#define ENA 11
#define IN1 12
#define IN2 13
#define IN3 8
#define IN4 9
#define ENB 10

void Task1(void *pvParameters);
void Task2(void *pvParameters);

static TaskHandle_t Task1Handle = NULL;// su dung bien cuc bo cho ca chuong trinh
static TaskHandle_t Task2Handle = NULL;
QueueHandle_t  Queue;

int count =0;
int read_count = 0;

static int sensorValue = 0;
void _start()
{
  digitalWrite(ENA,HIGH);
  digitalWrite(ENB,HIGH);
}

void _stop()
{
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
}

void forward(){
  _start();
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
}

void reverse(){
  _start();
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
}

void turnRight(){
  _start();
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
}

void turnLeft(){
  _start();
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
}

void setSpeed(int motorspeed2){
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

  Queue = xQueueCreate(1, sizeof(int));
  if(Queue!=NULL)
  {
  
   xTaskCreate(
    Task1
    ,  "Dist"   
    ,  128 
    ,  NULL
    ,  0 
    ,  &Task1Handle);   

   xTaskCreate(
    Task2
    ,  "Drive" 
    ,  128  
    ,  NULL
    ,  0  
    ,  &Task2Handle);     
  }
     vTaskStartScheduler(); 
}

void Task1(void *pvParameters ) 
{
  
   int notificationValue_task1;
    for(;;) 
    {
        sensorValue = analogRead(A0);
        if (sensorValue > 500)
        {
          digitalWrite(3,LOW);
          count ++;
          Serial.print("count: ");Serial.println(count);
          Serial.print("sensorValue: ");Serial.println(sensorValue);

          vTaskDelay(200);       
        }
        if (sensorValue < 200)
        {
          digitalWrite(3,HIGH);
          Serial.println("nothing happened");
          vTaskDelay(200);
        }
        if (count ==3)
        {
          xQueueSend (Queue,&count,portMAX_DELAY);
          xTaskNotify(Task2Handle,0,eNoAction);
          vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (count ==5)
        {
          xQueueSend (Queue,&count,portMAX_DELAY);
          xTaskNotify(Task2Handle,0,eNoAction);
          vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}


void Task2  (void *pvParameters ) 
{
    uint32_t ulNotifiedValue;
    enum States{FORWARD,REVERSE,TURN,STOP};
    States state=FORWARD;
    for(;;) 
    {  
      switch(state) 
      {
        case FORWARD:        
        forward();
        setSpeed(60);
        
        xTaskNotifyWaitIndexed( 0,
                        ULONG_MAX, /* Clear all notification bits on entry. */
                        ULONG_MAX, /* Reset the notification value to 0 on exit. */
                        &ulNotifiedValue,
                        portMAX_DELAY);

        xQueueReceive(Queue,&read_count,portMAX_DELAY);
        Serial.print("the varialable named count was sent by Task1 (3 or 5) is: ");Serial.println(read_count);
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
        setSpeed(255);
        vTaskDelay(pdMS_TO_TICKS((1000+random(-500, 50))));
        state=TURN;      
        break;
  
        case TURN:     
        turnRight();
        setSpeed(190);
        vTaskDelay(pdMS_TO_TICKS((1000+random(-500, 50))));
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



    
