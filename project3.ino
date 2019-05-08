#include <WiFi101.h>
#include <WiFiUdp.h>
#include <math.h>
#include <LSM303.h>
#include <Wire.h>
#include <SPI.h>
#include "arduino_secrets.h"

// Pin Definitions //
#define MOT_L_A 5
#define MOT_L_B 6
#define MOT_R_A 9
#define MOT_R_B 10
#define IRPIN_L 11
#define IRPIN_R 12

// Direction Definitions //
#define NORTH 0
#define SOUTH 180
#define EAST 90
#define WEST 270
#define angle_tolerance 15

// Motor Variables //
int pwm_motR = 0;
int pwm_motL = 0;
bool pickup = false;

// WiFi Variables // 
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
IPAddress ip(192,168,1,102); // static ip address
IPAddress gate(192,168,0,1);
IPAddress sub(255,255,0,0);
IPAddress ip2;
IPAddress server(192,168,1,102);

// UDP Variables //
unsigned int localPort = 5005;
WiFiUDP udp;

// IMU Variables //
LSM303 compass;
float cur_angle;

void setup() {
  APsetup();
  udp.begin(localPort);
  Serial.println("UDP setup complete");
  pinSetup();
  IMU_setup();
}

void loop() {
  checkIMU();
  updateMotor();
}

// Setup Functions //

void APsetup(){
  // Set WiFi pins on Feather M0
  WiFi.setPins(8,7,4,2);
  //Initialize serial and wait for port to open:
  Serial.begin(9600);

  WiFi.config(ip,gate,sub);
  WiFi.begin(ssid,pass);
  ip2 = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip2);
  Serial.println("Access point setup complete");
}

void pinSetup(){
  pinMode(MOT_L_A, OUTPUT);
  pinMode(MOT_L_B, OUTPUT);
  pinMode(MOT_R_A, OUTPUT);
  pinMode(MOT_R_B, OUTPUT);
  pinMode(IRPIN_L, INPUT);
  pinMode(IRPIN_R, INPUT);
  //attachInterrupt(digitalPinToInterrupt(IRPIN_L), ISR_L, RISING);
  //attachInterrupt((digitalPinToInterrupt(IRPIN_R), ISR_R, RISING);
}

void IMU_setup(){
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){+1254,  +465,   -5355};
  compass.m_max = (LSM303::vector<int16_t>){+1628,  +627,  -5099};
}

// Loop Functions //

void checkIMU(){
  compass.read();
  float az = (float)compass.a.z * 0.061;
  float g = az/1000.0;
  Serial.println(g);
  if (g > 1.10){
    Serial.println("Picked up!");
    pickup = true;
  }
//  else {
//    pickup = false;
//  }
}

void updateMotor(){
  if (pickup){
    pwm_motL = 0;
    pwm_motR = 0;
  }
  else {
    pwm_motL = 100;
    pwm_motR = 100;
  }
  
  analogWrite(MOT_L_A, pwm_motL);
  analogWrite(MOT_L_B, 0);
  analogWrite(MOT_R_A, pwm_motR);
  analogWrite(MOT_R_B, 0);
}

void rotate(float des_angle){
  int error = cur_angle - des_angle;
  Serial.print("Initial error: ");
  Serial.println(error);
  while(abs(error) > angle_tolerance){

    //if (error > 0){ // rotate counter-clockwise
  analogWrite(MOT_R_A, 100);
  analogWrite(MOT_R_B, 0);
  analogWrite(MOT_L_A, 0);
  analogWrite(MOT_L_B, 100);
    //}
//    else { // rotate clockwise
//      analogWrite(MOT_R_A, 0);
//      analogWrite(MOT_R_B, 100);
//      analogWrite(MOT_L_A, 100);
//      analogWrite(MOT_L_B, 0);
//    }
    compass.read();
    cur_angle = compass.heading();
    error = cur_angle - des_angle;
    Serial.print("Current Angle: ");
    Serial.println(cur_angle);
    Serial.print("Current Error: ");
    Serial.println(error);
  }
  Serial.println("Rotation complete");
  analogWrite(MOT_R_A, 0);
  analogWrite(MOT_R_B, 0);
  analogWrite(MOT_L_A, 0);
  analogWrite(MOT_L_B, 0);
}
