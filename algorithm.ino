// Import_libraries
#include <Adafruit_TCS34725.h>    // Color_Sensor_Libraries
#include <AFMotor.h>              // Driver_Motor_Libraries
#include <Wire.h>                 // I2C

// Calling_method
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Initialize_motor_pinout
AF_DCMotor motorRB(1); // Right_Back_Motor
AF_DCMotor motorLB(2); // Left_Back_Motor
AF_DCMotor motorRF(3); // Right_Front_Motor
AF_DCMotor motorLF(4); // Left_Front_Motor

// Ultrasonic_(HC-SR04)_sensor_pinout_declaration
const int trig_left  = 2; 
const int echo_left  = 13;   
const int trig_front = A2; 
const int echo_front = A3;   
const int trig_right = A0;  
const int echo_right = A1;   

long duration_left;
long duration_front;
long duration_right;
int distance_left;
int distance_front;
int distance_right;

const int speed_point = 65;     // note : change this value depent on your project
int speed_left;
int speed_right;

const double set_point = 17;    // note : change this value depent on your project
const double Kp = 3.4;          // note : tuning this Proportional PID parameter
const double Kd = 7.1;          // note : tuning this Derivatif PID parameter

double error;
double error_sum;
double error_prev;

int proportional;
int derivatif;
int pd_controller;

uint16_t clear, red, green, blue;
float r;
float g;
float b;

void setup() {
  Serial.begin(9600);
  // Initialize_motor
  motorRB.run(RELEASE);
  motorLB.run(RELEASE);
  motorRF.run(RELEASE);
  motorLF.run(RELEASE);
  // Initialize_PWM_motor
  speed_right  = 0;
  speed_left   = 0;
  // Initialize_PINOUT
  pinMode(trig_left,  OUTPUT);
  pinMode(echo_left,  INPUT);
  pinMode(trig_front, OUTPUT);
  pinMode(echo_front, INPUT);
  pinMode(trig_right, OUTPUT);
  pinMode(echo_right, INPUT);
  // Initialize_error_value
  error     = 0;
  error_prev = 0;
  error_sum  = 0;
  // Initialize_parameters_value
  proportional  = 0;
  derivatif     = 0;
  pd_controller  = 0;
  // Initialize_color_sensor
  if(tcs.begin()) {
    Serial.println("TCS34725 Found!");
  }else{
    Serial.println("TCS34725 Not Found! Check the wired.");
    while (1);
  }
}

void loop() {
  // First_read_the_color
  readSensor();
  // While_the_robots_not_on_the_finish_color, so_the_robot_will_going_to_move
  while(r < 0.6){
    readSensor();
    error =  distance_left - set_point;
    // P_controller
    proportional = Kp * error;
    // D_controller
    derivatif = Kd * (error - error_prev);
    error_prev = error;
    // PD_controller_Calculation
    pd_controller = proportional + derivatif;
    // Serial.print("Error : ");
    // Serial.println(error);

    if(distance_left < 40 && distance_front > 20){
      isPD();
    }else if(distance_left < 40 && distance_front <= 20){
      turnRight(85);
    }else if(distance_left >= 40 && distance_front > 20){
      isPD();
    }else if(distance_left >= 40 && distance_front <= 20){
      if(distance_left > distance_right){
        turnLeft(85);
      }else if(distance_left < distance_right){
        turnRight(85);
      }else{
        goBackward(85);
      }
    }else{
      goBackward(85);
    }
  }
  // Serial.println("STOP");
  stop();
}

void isPD(){
  if(error > 0){
    speed_right  = speed_point + pd_controller;
    speed_left   = speed_point - pd_controller;
    speed_right  = constrain(speed_right, 5, 235);  // constrain_the_minimum_and_maximum_speed_value
    speed_left   = constrain(speed_left, 5, 235);   // constrain_the_minimum_and_maximum_speed_value
    goForward(speed_right, speed_left);
  }else if(error <= 0){
    speed_left   = speed_point - pd_controller;
    speed_right  = speed_point + pd_controller;
    speed_left   = constrain(speed_left, 5, 235);   // constrain_the_minimum_and_maximum_speed_value
    speed_right  = constrain(speed_right, 5, 235);  // constrain_the_minimum_and_maximum_speed_value 
    goForward(speed_right, speed_left);
  }
}

void goForward(int right_speed, int left_speed){
  motorRB.setSpeed(right_speed);
  motorRF.setSpeed(right_speed);
  motorLB.setSpeed(left_speed);
  motorLF.setSpeed(left_speed);
  motorRB.run(FORWARD);
  motorLB.run(FORWARD);
  motorRF.run(FORWARD);
  motorLF.run(FORWARD);
}

void goBackward(int pwm){
  motorRB.setSpeed(pwm);
  motorLB.setSpeed(pwm);
  motorRF.setSpeed(pwm);
  motorLF.setSpeed(pwm);
  motorRB.run(BACKWARD);
  motorLB.run(BACKWARD);
  motorRF.run(BACKWARD);
  motorLF.run(BACKWARD);
}

void turnAround(int pwm){
  motorRB.setSpeed(pwm);
  motorRF.setSpeed(pwm);
  motorLB.setSpeed(pwm);
  motorLF.setSpeed(pwm);
  motorLB.run(BACKWARD);
  motorLF.run(FORWARD);
  motorRB.run(FORWARD);
  motorRF.run(FORWARD);
}

void turnRight(int pwm){
  motorLB.setSpeed(pwm);
  motorLF.setSpeed(pwm);
  motorRB.setSpeed(pwm);
  motorRF.setSpeed(pwm);
  motorLB.run(FORWARD);
  motorLF.run(FORWARD);
  motorRB.run(BACKWARD);
  motorRF.run(BACKWARD);
}

void turnLeft(int pwm){
  motorRB.setSpeed(pwm);
  motorRF.setSpeed(pwm);
  motorLB.setSpeed(pwm);
  motorLF.setSpeed(pwm);
  motorRB.run(FORWARD);
  motorRF.run(FORWARD);
  motorLB.run(BACKWARD);
  motorLF.run(BACKWARD);
}

void stop(){
  motorRB.run(RELEASE);
  motorRF.run(RELEASE);
  motorLB.run(RELEASE);
  motorLF.run(RELEASE);
}

// Read_all_sensor_value
void readSensor(){
  sensorU_left();
  delay(1);
  sensorU_front();
  delay(1);
  sensorU_right();
  delay(1);
  sensor_color();
  delay(1);
}

// HC-SR04
void sensorU_left(){
  digitalWrite(trig_left, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_left, LOW);
  delayMicroseconds(2);
  duration_left = pulseIn(echo_left, HIGH);
  distance_left = (duration_left*0.034)/2;
  // Serial.print("Left Distance : ");
  // Serial.print(distance_left);
  // Serial.print(" cm\t");
}

void sensorU_front(){
  digitalWrite(trig_front, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_front, LOW);
  delayMicroseconds(2);
  duration_front = pulseIn(echo_front, HIGH);
  distance_front = (duration_front*0.034)/2;
  // Serial.print("Front Distance : ");
  // Serial.print(distance_front);
  // Serial.println(" cm\t");
}

void sensorU_right(){
  digitalWrite(trig_right, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_right, LOW);
  delayMicroseconds(2);
  duration_right = pulseIn(echo_right, HIGH);
  distance_right = (duration_right*0.034)/2;
  // Serial.print("Right Distance : ");
  // Serial.print(distance_right);
  // Serial.print(" cm\n");
}

// TCS34725
void sensor_color(){
  tcs.getRawData(&red, &green, &blue, &clear); 
  r = red;
  g = green;
  b = blue;
  // Normalize_color_value
  r /= clear;
  g /= clear;
  b /= clear;
  // Serial.print("\nR : "); Serial.print(r);
  // Serial.print("\tG : "); Serial.print(g);
  // Serial.print("\tB : "); Serial.println(b);
}
