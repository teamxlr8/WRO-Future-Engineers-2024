#include <NewPing.h>         // Ultrasonic sensor library
#include <Wire.h>
#include "AHRSProtocol.h"             // navX-Sensor Register Definition header file
#include <Servo.h>



Servo myservo;  // create servo object to control a servo
// Constants for straight position and max steering angles
const int STRAIGHT = 90;  // Assuming 90 is straight
const int RIGHT_LIMIT = 120;
const int LEFT_LIMIT = 60;
 

#define motorA 2
#define motorB 3


// Ultrasonic sensor pins
#define TRIG_PIN_LEFT 4
#define ECHO_PIN_LEFT 5
#define TRIG_PIN_RIGHT 6
#define ECHO_PIN_RIGHT 7
#define TRIG_PIN_FRONT 14
#define ECHO_PIN_FRONT 15


// Initialize ultrasonic sensors
NewPing sonarLeft(TRIG_PIN_LEFT, ECHO_PIN_LEFT, 500);    // Left sensor
NewPing sonarRight(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, 500); // Right sensor
NewPing sonarFront(TRIG_PIN_FRONT, ECHO_PIN_FRONT, 500); // Front sensor
int leftDistance, rightDistance,frontDistance;

//Initialize NavX parameters
byte data[512];
#define ITERATION_DELAY_MS                   10
#define NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT  0x32
#define NUM_BYTES_TO_READ                    8
int register_address = NAVX_REG_YAW_L;

int direction = 0;

//PD variables
const int desiredDistance = 25;  // Ideal distance from the wall (in cm)
const float Kp = 1.1;            // Proportional control constant, tune this for smoothness
const float Kd = 10; // Derivative gain
float previousError = 0;



// Variables for lap counting using NavX yaw
float currentyaw, previousYaw;
int lapCount = 0;
bool lapStartDetected = false;

// Yaw threshold for detecting 90-degree turns (adjust as necessary)
const int YAW_THRESHOLD = 30;
const int MAX_LAPS = 3;



void setup() {
  Serial.begin(115200);

  //motor pins
  pinMode(motorA, OUTPUT);  
  pinMode(motorB, OUTPUT);
  
  //navX
   Wire.begin(); // join i2c bus (address optional for master)

  for ( int i = 0; i < sizeof(data); i++ ) {
      data[i] = 0;
  }
  
  myservo.attach(9);  //Servo pin
  steerStraight();

  
 Serial.println("Go XLR8.");
 
}

void loop() {
//  readultrasonics();
//  delay(40);
//  while(frontDistance>80)
//  {
//      readultrasonics();
//    forward(150);
//    delay(40);
//    if(leftDistance>200) 
//      {
//       direction = 1;
//       }
//
//    if(rightDistance>200) 
//      {
//       direction = 2;
//       }
//  }
//
//  
//  stop();
//
//  while (direction == 1)
//  {
//    readultrasonics();
//    innerwalllogicleft();  
//    forward(150);
//    delay(40); 
//  }

  readultrasonics();
  
  innerwalllogicright();  
  
  // 2. Read NavX yaw for lap counting
  currentyaw = getYaw();
  lapCount = checkForLapCompletion(currentyaw, previousYaw, lapCount);
  previousYaw = currentyaw;

  // Stop the robot after 3 laps
  if (lapCount >= MAX_LAPS) {
      Serial.println("3 Laps Completed. Stopping Robot.");
    forward(150);
    delay(500); 
    stop();
    delay(1000);
    while(1);
  }

  delay(40);
}


// Function to check for lap completion using NavX yaw
int checkForLapCompletion(float yaw, float prevYaw, int laps) {

  Serial.println("In lapcount");
  Serial.println(abs(yaw - prevYaw));
  if (!lapStartDetected && abs(yaw - prevYaw) > YAW_THRESHOLD) {
    lapStartDetected = true;
    Serial.println("LAP STARTED");
    //delay(2000);
  }
  
  // Detect when the yaw has crossed a full 360-degree lap
  if (lapStartDetected && abs(yaw) < YAW_THRESHOLD) {
    laps++;
    lapStartDetected = false;
    Serial.println("Lap Completed: " + String(laps));
  }
  
  return laps;
}










void innerwalllogicleft()
{
  forward(255);
  // Calculate the error (difference from the ideal distance)
  int error =   leftDistance - desiredDistance;
  
  // Apply proportional control to adjust the steering
  int steeringAdjustment = Kp * error + Kd * (error - previousError);;
  
  // Ensure steeringAdjustment is within the valid range for your steering mechanism
  steeringAdjustment = constrain(steeringAdjustment, -30, 30);  // Limiting the adjustment to -30 (left) to 30 (right)

  // Adjust steering based on error
  if (steeringAdjustment > 0) {
    steerRight(steeringAdjustment);   // Steer left if too close to the wall
  } else if (steeringAdjustment < 0) {
    steerLeft(abs(steeringAdjustment));  // Steer right if too far from the wall
  } else {
    steerStraight();  // Move straight if at the desired distance
  }

  previousError = error;
  // Short delay to stabilize readings
  delay(40);
}


void innerwalllogicright()
{
  forward(255);
  // Calculate the error (difference from the ideal distance)
  int error =    desiredDistance - rightDistance;
  
  // Apply proportional control to adjust the steering
  int steeringAdjustment = Kp * error + Kd * (error - previousError);;
  
  // Ensure steeringAdjustment is within the valid range for your steering mechanism
  steeringAdjustment = constrain(steeringAdjustment, -30, 30);  // Limiting the adjustment to -30 (left) to 30 (right)

  // Adjust steering based on error
  if (steeringAdjustment > 0) {
    steerRight(steeringAdjustment);   // Steer left if too close to the wall
  } else if (steeringAdjustment < 0) {
    steerLeft(abs(steeringAdjustment));  // Steer right if too far from the wall
  } else {
    steerStraight();  // Move straight if at the desired distance
  }

  previousError = error;
  // Short delay to stabilize readings
  delay(40);
}












void readultrasonics()
{
  // Measure distances from the walls
  leftDistance = sonarLeft.ping_cm();
  rightDistance = sonarRight.ping_cm();
  frontDistance = sonarFront.ping_cm();
  // Debug print the distances
  Serial.print("Left: " + String(leftDistance) + " cm, ");
  Serial.print("Right: " + String(rightDistance) + " cm");
  Serial.print("Front: " + String(frontDistance) + " cm");
  Serial.println();
  
  
}


int getYaw()
{
 int i = 0;
  /* Transmit I2C data request */
  Wire.beginTransmission(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT); // Begin transmitting to navX-Sensor
  Wire.write(register_address);                                // Sends starting register address
  Wire.write(NUM_BYTES_TO_READ);                               // Send number of bytes to read
  Wire.endTransmission();                                      // Stop transmitting
  
  /* Receive the echoed value back */
  Wire.beginTransmission(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT); // Begin transmitting to navX-Sensor
  Wire.requestFrom(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT, NUM_BYTES_TO_READ);    // Send number of bytes to read
  delay(1);
  while(Wire.available()) {                                    // Read data (slave may send less than requested)
     data[i++] = Wire.read();
  }
  Wire.endTransmission();                                      // Stop transmitting

  /* Decode received data to floating-point orientation values */
  float yaw =     IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&data[0]);   // The cast is needed on arduino
  /* Display orientation values */
  Serial.print("yaw:  ");
  Serial.print(yaw,2);

  Serial.println("");

  delay(ITERATION_DELAY_MS);  

  return yaw;
  
}


void steerStraight() {
    myservo.write(STRAIGHT);   // Set servo to center/straight position
}

void steerRight(int angle) {
    int target = constrain(STRAIGHT + angle, STRAIGHT, RIGHT_LIMIT); // Constrain the angle to prevent exceeding limits
    myservo.write(target);    // Set servo to the calculated right angle
}

void steerLeft(int angle) {
    int target = constrain(STRAIGHT - angle, LEFT_LIMIT, STRAIGHT);  // Constrain the angle to prevent exceeding limits
    myservo.write(target);    // Set servo to the calculated left angle
}


void forward(int sp)
{
digitalWrite (motorA, LOW);
  analogWrite (motorB, sp);

}

void backward(int sp)
{
digitalWrite (motorB, HIGH);
  analogWrite (motorB, sp);

}

void stop()
{
  digitalWrite (motorA, LOW);
  analogWrite (motorB, 0);
}


void motortest()
{
  forward(200);
  delay(2000);
  stop();
  delay(2000);
  backward(200);
  delay(2000);
  stop();
  delay(2000);
}


void steertest()
{
  
  steerStraight();
  delay(2000);
  steerRight(30);
  delay(2000);
  steerLeft(30);
  delay(2000);
}