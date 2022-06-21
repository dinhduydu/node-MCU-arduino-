/*Khai báo thư viện RTOS*/
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include "task.h"
#include "queue.h"
#define ULONG_MAX 0xFFFFFFFF

/*Thư viện MPU6050*/
#include <MPU6050_tockn.h>
#include <Wire.h>
MPU6050 mpu6050(Wire);
float z0,z =0 ;

/*Sử dụng bộ P*/
float Kp = 1;

/*Sử dụng thư viện Software Serial*/
#include <SoftwareSerial.h>
SoftwareSerial mySerial (6,7);

/*Khai báo chân điều khiển L298 với ENA và ENB để băm xung*/
#define ENA 11
#define IN1 12
#define IN2 13
#define IN3 10
#define IN4 8
#define ENB 9

/*Gồm 3 Task: check Zone, Điều khiển xe và chạy thẳng dùng MPU*/
void TaskCheckZone(void *pvParameters);
void TaskDrive(void *pvParameters);
void TaskMPU(void *pvParameters);

static TaskHandle_t TaskCheckZoneHandle = NULL;// su dung bien cuc bo cho ca chuong trinh
static TaskHandle_t TaskDriveHandle = NULL;
static TaskHandle_t TaskMPUHandle = NULL;
QueueHandle_t  Queue,Queue1,Queue2;

/*Sử dụng Queue để chứa giá trị đếm Zone*/
int count =0;
int read_count = 0;
/*Giá trị analog của cảm biến hồng ngoại*/
int sensorValue = 0;

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

// void reverse(){
//   _start();// Cho phép xe chạy

//   // set up chiều của xe
//   digitalWrite(IN1,HIGH);
//   digitalWrite(IN2,LOW);
//   digitalWrite(IN3,LOW);
//   digitalWrite(IN4,HIGH);
// }

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

void setSpeed(int motorspeed2)
{
  /*Nhận giá trị từ TaskMPU gửi tới để set giá trị cho hàm setSpeed()*/
  xQueueReceive(Queue1,&motorspeed2,portMAX_DELAY);
  Serial.println(motorspeed2);
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
  mySerial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  /*Đo offset ban đầu*/
  mpu6050.calcGyroOffsets(true);

  Queue = xQueueCreate(1, sizeof(int));
  Queue1 = xQueueCreate(1, sizeof(int));
  Queue2 = xQueueCreate(1, sizeof(int));
  if(Queue!=NULL||Queue1!=NULL||Queue2!=NULL||Queue3!=NULL )
  {
  xTaskCreate(
        TaskCheckZone
        ,  "Zone"   
        ,  128 
        ,  NULL
        ,  1 
        ,  &TaskCheckZoneHandle);   

  xTaskCreate(
        TaskDrive
        ,  "Drive" 
        ,  128  
        ,  NULL
        ,  1  
        ,  &TaskDriveHandle);  

  xTaskCreate(
        TaskMPU
        ,  "MPU"   
        ,  128 
        ,  NULL
        ,  1 
        ,  &TaskMPUHandle);    
  }
  vTaskStartScheduler(); 
}

void TaskCheckZone(void *pvParameters ) 
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
      Serial.println("chưa đi qua zone");
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
  int turn =0;
  int Zone3 =0;
  int State = 1;
  int pwm2, pwm = 40;
  for(;;)
  {
    mpu6050.update();

    z = mpu6050.getAngleZ(); //state 2 , vd : z=10;
    if(State == 1)
    {
        /*Lấy giá trị z0 làm giá trị gốc*/
        z0 = z;
        State ++;
    }

    if ((z - z0) > 0.1 || (z - z0) < 0.1)
    {
      /*Bộ điều khiển P lấy giá trị đầu vào là sai lệch giữa z0 và z*/
      pwm2 = pwm2 + Kp*(float)(z-z0);

      if(pwm2 > (pwm+4)) pwm2 = (pwm+4);
      
      if(pwm2 <(pwm-4) ) pwm2 = (pwm-4);
    }
    else
    {
      pwm2 = pwm;
    }
    xQueueSend (Queue1,&pwm2,portMAX_DELAY);
    Serial.print("z=");
    Serial.println(z);
    Serial.print("pwm2=");
    Serial.println(pwm2);
    
    xQueueReceive(Queue,&Zone3,portMAX_DELAY);
    xQueueReceive(Queue2,&turn,portMAX_DELAY);
    if ((State ==2) & (Zone3 ==3))
    {
      if(turn == 1)
      {
        z0 = z;
        while(z + 83 > z0)
        { 
          mpu6050.update();
          z = mpu6050.getAngleZ(); //state 2 , vd : z=10;
          Serial.println(z);
        }
        xTaskNotify(TaskDriveHandle,0,eNoAction);
       }
      else if(turn == 2)
      {
        z0 = z;
        while(z - 83 < z0)
        {
          mpu6050.update();
          z = mpu6050.getAngleZ();
          Serial.println(z);
        }
        xTaskNotify(TaskDriveHandle,0,eNoAction);
      }
  }
}

/*Vừa điều khiển lái xe, vừa nhận tín hiệu từ ESP*/
void TaskDrive  (void *pvParameters ) 
{
  int turn_right = 1, turn_left = 2;
  String bienluu = ""; 
  uint32_t ulNotifiedValue;
  enum States{FORWARD,REVERSE,TURN,STOP};
  States state=FORWARD;
  for(;;) 
  {  
    switch(state) 
    {
      case FORWARD:  
      /*Không có tín hiệu từ esp truyền sang thì bị lặp ở trong vòng lặp while này*/
      while(mySerial.available () == 0)
      {
        Serial.println("Chờ cho đến khi được nhấn");
      }      
      forward();
      setSpeed(40);
      /*Chỉ khi Zone = 3 hoặc bằng 5 thì mới thoát lệnh đằng sau này và xe mới đi tiếp, ko thì cứ mãi đi thẳng*/
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
        state=TURN;
      }
      if (read_count >=5)
      {
        Serial.println("count = 5, then this task is unblocked");
        state = STOP;
      }
      break;
        
                
      // case REVERSE:
      // reverse();
      // setSpeed(70);
      // vTaskDelay(pdMS_TO_TICKS(1000));
      // state=TURN;      
      // break;
  
      /*Tương ứng với Zone = 3 thì rẽ trái hoặc phải*/  
      case TURN:    
      bienluu = mySerial.readString();

      if (bienluu.startsWith("1"))
      {
        turnLeft();
        xQueueSend (Queue2,&turn_left,portMAX_DELAY);
      }
      if (bienluu.startsWith("2"))
      {
        turnRight();
        xQueueSend (Queue2,&turn_right,portMAX_DELAY);
      }
      setSpeed(40);
      xTaskNotifyWaitIndexed( 0,
                        ULONG_MAX, /* Clear all notification bits on entry. */
                        ULONG_MAX, /* Reset the notification value to 0 on exit. */
                        &ulNotifiedValue,
                        portMAX_DELAY);     
      state=FORWARD;      
      break; 

      /*Tương ứng với Zone = 5 thì dừng*/
      case STOP:
      _stop();
      setSpeed(0);  
//    exit();
    }
  }
}






void loop() 
{
  // empty
}
