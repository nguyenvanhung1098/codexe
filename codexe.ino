#include <stdio.h>
#include <math.h>
#include <ESP8266WiFi.h>
#include "FirebaseESP8266.h"

#define FIREBASE_HOST "henhungxedoduong.firebaseio.com"
#define FIREBASE_AUTH "u2BqTufkYZAlMT6HQ5NP34X5htL1WwjPZMYfThwK"
#define WIFI_SSID "cslab-505"
#define WIFI_PASSWORD "ccslab-505"

//#define WIFI_SSID "FPT Telecom-CDA6"
//#define WIFI_PASSWORD "67896789"

FirebaseData firebaseData;


#define left_motor_low 15
#define left_motor_PWM 16
#define right_motor_low 0 
#define right_motor_PWM 2

#define IR1 14
#define IR2 12
#define IR3 13
#define IR4 5
#define IR5 4


int sensor[5];
int error;

int Kp = 50;
int Ki = 0;
int Kd  = 120;
int initial_motor_speed= 550;
int P , I, D;
int previous_error;
int PID_value;
int Start = 1;
int begin_error;
void connection()
{ 
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);  
}

void get_data()
{
  Firebase.getInt(firebaseData, "/Kd");
  Kd = firebaseData.floatData();
  Serial.println("Kd:");
  Serial.println(Kd);

  Firebase.getInt(firebaseData, "/Ki");
  Ki = firebaseData.floatData();
  Serial.println(Ki);

  Firebase.getInt(firebaseData, "/Kp");
  Kp = firebaseData.floatData();
  Serial.println(Kp);

  Firebase.getInt(firebaseData, "/initial_motor_speed");
  initial_motor_speed = firebaseData.floatData();
  Serial.println(initial_motor_speed);
  
  while(!Start){
    Firebase.getInt(firebaseData, "/Start");
    Start = firebaseData.intData();
  }
}

void setdata()
{
  Firebase.setInt(firebaseData, "/henhungxedoduong/PID/P",P);
  Firebase.setInt(firebaseData, "/henhungxedoduong/PID/I",I);
  Firebase.setInt(firebaseData, "/henhungxedoduong/PID/D",D);
  Firebase.setInt(firebaseData, "/henhungxedoduong/PID/PID_value",PID_value);
}



int get_error(){
  
    sensor[0] = digitalRead(IR1); // IR1 ben trai xe
    sensor[1] = digitalRead(IR2); // IR2
    sensor[2] = digitalRead(IR3); // IR3 chinh giua xe
    sensor[3] = digitalRead(IR4); // IR4
    sensor[4] = digitalRead(IR5); // IR5 ben phai xe
    int value = sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4];
    if (value <= 2  ){
      sensor[0] = sensor[0] ^ 1;
      sensor[1] = sensor[1] ^ 1;
      sensor[2] = sensor[2] ^ 1;
      sensor[3] = sensor[3] ^ 1;
      sensor[4] = sensor[4] ^ 1;
    }

    if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==0))
    error=4;
    else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==0))
    error=3;
    else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==1))
    error=2;
    else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==1))
    error=1;
    else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==1))
    error=0;
    else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==1))
    error=-1;
    else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1))
    error=-2;
    else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1))
    error=-3;
    else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1))
    error=-4;
    else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1))
      {
        if(error <= 0) error=-5;
        else error=5;
      }
}
void calculate_pid()
{  
    
    P = error; // loi
    I = I + error;
    if(P * previous_error > 8){
      Kp = 150;
    }
    else {
      Kp = 55;
    }
    D = error - previous_error ;
    PID_value = Kp * P + Ki * I  + Kd *  D; 
    begin_error = previous_error; // loi cu
    previous_error=error; // loi moi
}
void motor_control()
{     
      int left_motor_speed = initial_motor_speed + PID_value ;
      int right_motor_speed = initial_motor_speed - PID_value;

      constrain(left_motor_speed, 64, 1023);
      constrain(right_motor_speed,64,1023);
      //Firebase.setInt(firebaseData, "/henhungxedoduong/PID/left_motor_speed",left_motor_speed);
      //Firebase.setInt(firebaseData, "/henhungxedoduong/PID/right_motor_speed",right_motor_speed);
      digitalWrite(left_motor_low,LOW);   //Left Motor Speed
      digitalWrite(right_motor_low,LOW);  //Right Motor Speed
      analogWrite(left_motor_PWM,left_motor_speed);
      analogWrite(right_motor_PWM,right_motor_speed);
}

void setup()
{
  Serial.begin(115200);
  //set chan in_out put
  pinMode(IR1,INPUT);
  pinMode(IR2,INPUT);
  pinMode(IR3,INPUT);
  pinMode(IR4,INPUT);               
  pinMode(IR5,INPUT);
  pinMode(left_motor_PWM,OUTPUT);
  pinMode(left_motor_low,OUTPUT);
  pinMode(right_motor_PWM,OUTPUT);
  pinMode(right_motor_low,OUTPUT);
  // connection();
  // get_data();
}
void loop() {
   get_error();
   calculate_pid();
   motor_control();                                                                                                                                                                           
}
