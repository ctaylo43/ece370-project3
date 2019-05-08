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


// Access Point Variables // 
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;
int status = WL_IDLE_STATUS

// UDP Variables //
unsigned int recPort = 5005;
unsigned int sendPort = 8004;
WiFiUDP recUdp;
WiFiUDP sendUdp;

// UDP Structs //
struct Velocity { // struct to send speed and angle of the robot
  int v;
  int theta;
};

struct sendData {
  double odom[3];
  double imu[6];
  double heading;
};

struct Velocity a;
struct sendData data;

// IMU Variables //
#define gravity 9.81
LSM303 compass;
float cur_angle;
bool pickup = false;

// Odometry //
#define L 80 // baseline in mm
#define W_RADIUS 20 // wheel radius in mm
#define GEAR_RATIO  75.81 // Motor gear ratio
#define TICKS 2 // 2 ticks per rotation of wheels
#define DELTA_THETA_R ((2*PI/TICKS)/GEAR_RATIO)   // wheel rotation per tick in rads
#define DIS_TICK (W_RADIUS*DELTA_THETA_R)    // distance wheel rotates per tick
#define PHI atan2(DIS_TICK, L)   // change in angle of the robot per tick in rads
float delta_x = 0, delta_y = 0

// Matrices //
Matrix<4,4> right_wheel = {
   cos(phi),  -sin(phi),  0,  0,
   sin(phi),  cos(phi), 0,  L / 2,
   0, 0,  1,  0,
   0, 0,  0,  1};

Matrix<4,4> right_translate = {
  1,  0,  0,  0,
  0,  1,  0,  -L,
  0,  0,  1,  0,
  0,  0,  0,  1};

Matrix<4,4> left_wheel = {
   cos(-phi),  -sin(-phi),  0,  0,
   sin(-phi),  cos(-phi), 0,  -L / 2,
   0, 0,  1,  0,
   0, 0,  0,  1};

Matrix<4,4> left_translate = {
  1,  0,  0,  0,
  0,  1,  0,  L,
  0,  0,  1,  0,
  0,  0,  0,  1};

Matrix<4,4> global_matrix = {
  1,  0,  0,  0,
  0,  1,  0,  0,
  0,  0,  1,  0,
  0,  0,  0,  1};

Matrix<4,4> right_transform = right_wheel * right_translate;
Matrix<4,4> left_transform = left_wheel * left_translate;

void setup() {
  APsetup();
  openUDP();
  pinSetup();
  IMU_setup();
  initStructs();
}

void loop() {
  checkIMU();
  checkUDP();
  updateMotors();
}

// Setup Functions //

void APsetup()
{
  // Set WiFi pins on Feather M0
  WiFi.setPins(8,7,4,2);
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  Serial.println("Setting up access point");
  
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    while (true);       // don't continue
  }

  Serial.print("Starting access point named: ");
  Serial.println(ssid);
  // Start network
  status = WiFi.beginAP(ssid);
  if (status != WL_AP_LISTENING) {
    Serial.println("Failed to create access point");
    // don't continue
    while (true);
  }

  // wait 10 seconds for connection:
    delay(10000);
    
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}

void openUDP(){
  recUdp.begin(recPort);
  sendUdp.begin(sendPort);
}

void pinSetup(){
  Serial.println("Initializing pins...");
  pinMode(MOT_L_A, OUTPUT);
  pinMode(MOT_L_B, OUTPUT);
  pinMode(MOT_R_A, OUTPUT);
  pinMode(MOT_R_B, OUTPUT);
  pinMode(IRPIN_L, INPUT);
  pinMode(IRPIN_R, INPUT);
  attachInterrupt(digitalPinToInterrupt(IRPIN_L), ISR_L, RISING);
  attachInterrupt((digitalPinToInterrupt(IRPIN_R), ISR_R, RISING);
  Serial.println("...Pin initialization complete");
}

void IMU_setup(){
  Serial.println("Setting up IMU...");
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){+1254,  +465,   -5355};
  compass.m_max = (LSM303::vector<int16_t>){+1628,  +627,  -5099};
  Serial.println("...IMU setup complete");
}

void initStructs(){
  Serial.println("Initializing velocity and sendData structs...");
  a.v = 0;
  a.theta = 0;
  compass.read();
  init_theta = compass.heading();
  Serial.print("Initial Angle: ");
  Serial.println(init_theta);
  cur_angle = init_theta;
  init_rad = init_theta*(PI/180);
  global_matrix(0,0) = cos(init_rad);
  global_matrix(0,1) = -sin(init_rad);
  global_matrix(1,0) = sin(init_rad);
  global_matrix(1,1) = cos(init_rad);
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
  cur_angle = compass.heading();
}

void checkUDP(){
  
}

void updateMotors(){
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

void ISR_L(){
  
}

void ISR_R(){
  
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
