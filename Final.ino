#include <ESP32Servo.h>
#include "ESC.h"
#include <PID_v1.h>
// กำหนดขา GPIO
#define SENSOR_PINR 35   // ขา ADC สำหรับเซนเซอร์ขวา
#define SENSOR_PINL 34   // ขา ADC สำหรับเซนเซอร์ซ้าย
#define SENSOR_PINFrontR 32  // ขา ADC สำหรับเซนเซอร์ขวา
#define SENSOR_PINFrontL 33  // ขา ADC สำหรับเซนเซอร์ซ้าย
#define SERVO_PINL 12    // ขาสัญญาณเซอร์โวซ้าย
#define SERVO_PINR 2     // ขาสัญญาณเซอร์โวขวา
#define SPEED_MIN (1000)                                  // Set the Minimum Speed in microseconds
#define SPEED_MAX (2000)
#define SPEED_Normal (1150)
#define SPEED_safety (1075)
#define STOP_safety (60)                                   
#define MOTOR_PIN_RIGHT 15
#define MOTOR_PIN_LEFT 13
#define ServoBrakeL (130)
#define ServoBrakeL (40)
double setpoint = STOP_safety;    // ระยะห่างเป้าหมาย
double input;             // ระยะห่างปัจจุบัน
double output;
double Kp = 1, Ki = 0.1, Kd = 0.7;

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

int oESC;                                                 // Variable for the speed sent to the ESC
#define THRESHOLD 2000  // ค่าระยะห่างที่กำหนด (ขึ้นอยู่กับเซนเซอร์)
ESC myESCRIGHT (MOTOR_PIN_RIGHT, SPEED_MIN, SPEED_MAX, 500);         
ESC myESCLEFT (MOTOR_PIN_LEFT, SPEED_MIN, SPEED_MAX, 500);          // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
Servo myServoL;  // เซอร์โวซ้าย
Servo myServoR;  // เซอร์โวขวา
unsigned long previousMillis = 0;  // เก็บเวลาครั้งสุดท้าย
const long interval = 10; 

void setup() {
  myServoL.attach(SERVO_PINL);  // ผูกเซอร์โวซ้ายเข้ากับขาที่กำหนด
  myServoL.write(90);           // ตั้งเซอร์โวซ้ายให้อยู่ที่ 90 องศา
  myServoR.attach(SERVO_PINR);  // ผูกเซอร์โวขวาเข้ากับขาที่กำหนด
  myServoR.write(90);           // ตั้งเซอร์โวขวาให้อยู่ที่ 90 องศา
  setCpuFrequencyMhz(240);  
  myESCRIGHT.arm(); 
  delay(200);    
  myESCLEFT.arm(); 

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(SPEED_MIN, SPEED_safety);

  delay(2000);
  Serial.begin(115200);         // สำหรับ debug
}

void loop() {
  // อ่านค่าเซ็นเซอร์
  int sensorValueR = analogRead(SENSOR_PINR);
  int sensorValueL = analogRead(SENSOR_PINL);
  int count = 0 ;
  int sensorFrontValueRight = analogRead(SENSOR_PINFrontR);
  int sensorFrontValueleft = analogRead(SENSOR_PINFrontL);
 unsigned long currentMillis = millis();  // อ่านเวลาปัจจุบัน

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // อัปเดตเวลาครั้งสุดท้าย

  if (sensorValueR < 600) {
    sensorValueR = 0;
  }
  if (sensorValueL < 600) {
    sensorValueL=0;
  }

  // แสดงค่าที่อ่านได้ใน Serial Monitor
  Serial.print("sensorValueR :\t");
  Serial.print(sensorValueR);
  Serial.print(" sensorValueL :\t");
  Serial.print(sensorValueL);
  Serial.print(" |sensorFrontValueR :\t");
  Serial.print(sensorFrontValueRight);
  Serial.print(" sensorFrontValueL :\t");
  Serial.print(sensorFrontValueleft);

  float disLeft = 26441*(pow(sensorValueL,-0.995));
  float disRight = 26441*(pow(sensorValueR,-0.995));
  float disFrontLeft = 26441*(pow(sensorFrontValueleft,-0.995));
  float disFrontRight = 26441*(pow(sensorFrontValueRight,-0.995));
  float AVGdis = (disFrontLeft+disFrontRight)/2;
  float input = (disFrontLeft+disFrontRight)/2;

  myPID.Compute();

  Serial.print(" disLeft :\t");
  Serial.print(disLeft);
  Serial.print(" disRight :\t");
  Serial.print(disRight);
  Serial.print("| \t");
  Serial.print(" disForntRight :\t");
  Serial.print(disFrontRight);
  Serial.print(" disForntLeft :\t");
  Serial.println(disFrontLeft);
  // เงื่อนไขควบคุมเซอร์โว
  if (sensorValueR < THRESHOLD && sensorValueL > THRESHOLD) {
    // วัตถุอยู่ใกล้เซ็นเซอร์ซ้าย
    myServoL.write(15);  // เซอร์โวซ้ายตั้งเป็นกลาง
    myServoR.write(15);  // เซอร์โวขวาเบี่ยงไป
  } else if (sensorValueR > THRESHOLD && sensorValueL < THRESHOLD) {
    // วัตถุอยู่ใกล้เซ็นเซอร์ขวา
    myServoL.write(135);  // เซอร์โวซ้ายเบี่ยงไป
    myServoR.write(135);  // เซอร์โวขวาตั้งเป็นกลาง
  } else if (sensorValueR < THRESHOLD && sensorValueL < THRESHOLD) {
    // วัตถุอยู่ใกล้ทั้งสองด้าน
    myServoL.write(87);  // เซอร์โวซ้ายตั้งเป็นกลาง
    myServoR.write(90);  // เซอร์โวขวาตั้งเป็นกลาง
  }
  else{
    myServoL.write(87);  // เซอร์โวซ้ายตั้งเป็นกลาง
    myServoR.write(90);  // เซอร์โวขวาตั้งเป็นกลาง
  }
  if( AVGdis <= STOP_safety){

    myESCRIGHT.speed(SPEED_MIN);  
    myESCLEFT.speed(SPEED_MIN);
    while( count <= 2  ){
      Serial.print("========");
      Serial.print(count);
      if(count == 0){
        myServoL.write(130);  
        myServoR.write(40);
        delay(500);
        count++;
        
      }
      else{
        while(count <= 2){
          myServoL.write(87);  
          myServoR.write(90);
          delay(300);
          count++;
        }
        count++;

      }
    }
  }
  else if(AVGdis > STOP_safety)
  {
    myESCRIGHT.speed(output);  
    myESCLEFT.speed(output);
    count = 0;
  }
  Serial.print("| output :");
  Serial.println(output);
  }
}
