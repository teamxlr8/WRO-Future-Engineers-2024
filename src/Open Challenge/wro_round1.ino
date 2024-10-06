#include <NewPing.h>         // Ultrasonic sensor library
#include <Wire.h>
#include "AHRSProtocol.h"             // navX-Sensor Register Definition header file
#include <Servo.h>

#include <Pixy2.h>

// This is the main Pixy object 
Pixy2 pixy;


Servo myservo;  // create servo object to control a servo
// Constants for straight position and max steering angles
const int STRAIGHT = 90;  // Assuming 90 is straight
const int RIGHT_LIMIT = 120;
const int LEFT_LIMIT = 60;

#define motorA 8
#define motorB 3

#define green 17
#define yellow 16
#define start 2

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

unsigned long previousUltrasonicMillis = 0;
unsigned long previousNavxMillis = 0;
unsigned long currentMillis = 0;
const unsigned long ultrasonicInterval = 50;  // Poll ultrasonic every 50 ms
const unsigned long navxInterval = 100;       // Poll NavX every 100 ms

//Initialize NavX parameters
byte data[512];
#define ITERATION_DELAY_MS                   10
#define NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT  0x32
#define NUM_BYTES_TO_READ                    8
int register_address = NAVX_REG_YAW_L;

int direction = 0;

//PD variables
const int desiredDistance = 30;  // Ideal distance from the wall (in cm)                              //25
const float Kp = 1;            // Proportional control constant, tune this for smoothness           //1.0
const float Kd = 6; // Derivative gain                                                             //6
float previousError = 0;

// Variables for lap counting using NavX yaw
float currentYaw, previousYaw;
int lapCount = 0;
bool lapStartDetected = false;
bool turnCompleted = false; // New flag to detect full turns

// Yaw threshold for detecting 90-degree turns (adjust as necessary)
const int YAW_THRESHOLD = 40;
const int MAX_LAPS = 3;

bool passedLane1 = false;
bool passedLane2 = false;
bool passedLane3 = false;
bool passedLane4 = false;
bool reverse = false;  // Flag to track if the robot is moving in reverse

int initialYaw = 0;
float yawOffset = 0;  // This will hold the offset to set initial yaw to 90

bool passed90 = false;

// Variables for lap counting using NavX yaw
//float currentYaw, previousYaw;
//int lapCount = 0;
//bool lapStartDetected = false;



int x=0;
int y=0;

void setup() {
  Serial.begin(115200);

 pixy.init();
  //motor pins
  pinMode(motorA, OUTPUT);  
  pinMode(motorB, OUTPUT);

  pinMode(green,OUTPUT);
  pinMode(yellow,OUTPUT);
  pinMode(start,INPUT);
  digitalWrite(start,HIGH);
  //navX
   Wire.begin(); // join i2c bus (address optional for master)

  for ( int i = 0; i < sizeof(data); i++ ) {
      data[i] = 0;
  }
  
  myservo.attach(9);  //Servo pin
  steerStraight();

pixy.setLamp(1,1);
  delay(10000);
 Serial.println("Go XLR8.");
 digitalWrite(green,HIGH);
 initialYaw = getYaw();
 Serial.println(initialYaw);
 // Calculate the offset needed to make the initialYaw appear as 90 degrees
  yawOffset = 90 - initialYaw;
   Serial.print("Yaw Offset: ");
  Serial.println(yawOffset);
}

int mode = 0;
void loop() {

while(mode == 0)
  {
    Serial.println(digitalRead(start));
    if(digitalRead(start) == LOW)
    {
      mode = 1;break;  
    }
      
  }

while(mode == 1)
{
readultrasonics();
forward(130);
int DirCheck = leftDistance - rightDistance;
  if (DirCheck > -10 && DirCheck <10) {
    steerStraight();
    
  }
  else if (DirCheck <-10) {
    steerLeft(10);
  }

  else if (DirCheck >10) {
    steerRight(10);
  }


  
 // delay(1250);

 Serial.print("x");
 Serial.println(leftDistance);
 
 Serial.print("y");
 Serial.println(rightDistance);

if (rightDistance>=200) {
  Serial.println("GOing right");
  mode=2; break;
  
}

if (leftDistance>=200) {
  Serial.println("GOing left");
  mode=3; break;
}
}

while(mode == 2)  //rightwall
{
   currentMillis = millis();
  forward(150);
  
  // Poll Ultrasonic Sensors at regular intervals
  if (currentMillis - previousUltrasonicMillis >= ultrasonicInterval) {
    previousUltrasonicMillis = currentMillis;
    readultrasonics();  // Poll ultrasonic sensors
    innerwalllogicright();  // Update wall following logic
    rightlapcount();
   // pixyscan();
  }

  // Poll NavX Sensor at regular intervals
  if (currentMillis - previousNavxMillis >= navxInterval) {
    previousNavxMillis = currentMillis;
    // Stop the robot after completing 3 laps
    if (lapCount >= MAX_LAPS) {
      Serial.println("3 Laps Completed. Stopping Robot.");
      forward(150);
      delay(500);
      stop();
      delay(1000);
      while (1);  // Infinite loop to stop the robot
    }
  }
  
}



while(mode == 3)  //leftwall
{
  currentMillis = millis();
  forward(150);
  // Poll Ultrasonic Sensors at regular intervals
  if (currentMillis - previousUltrasonicMillis >= ultrasonicInterval) {
    previousUltrasonicMillis = currentMillis;
    readultrasonics();  // Poll ultrasonic sensors
    innerwalllogicleft();  // Update wall following logic
    leftlapcount();
   // pixyscan();
  }

  // Poll NavX Sensor at regular intervals
  if (currentMillis - previousNavxMillis >= navxInterval) {
    previousNavxMillis = currentMillis;
    // Stop the robot after completing 3 laps
    if (lapCount >= MAX_LAPS) {
      Serial.println("3 Laps Completed. Stopping Robot.");
      forward(150);
      delay(500);
      stop();
      delay(1000);
      while (1);  // Infinite loop to stop the robot
    }
  }
}
}
  



  




// Function to normalize yaw to the range -180 to 180 degrees
float normalizeYaw(float yaw) {
  if (yaw > 180) {
    yaw -= 360;
  } else if (yaw < -180) {
    yaw += 360;
  }
  return yaw;
}


void pixyscan()
{
int i; 
  // grab blocks!
  pixy.ccc.getBlocks();
  
  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks)
  {
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);
    for (i=0; i<pixy.ccc.numBlocks; i++)
    {
      Serial.print("  block ");
      Serial.print(i);
      Serial.print(": ");
      pixy.ccc.blocks[i].print();
    }

    if(pixy.ccc.blocks[0].m_signature == 2  && pixy.ccc.blocks[0].m_height > 40)
    {
      stop();
      digitalWrite(green,HIGH); 
      
    //  steerLeft(20);
      delay(10000);
      digitalWrite(green,LOW); 
     // steerStraight();
           
    }

    if(pixy.ccc.blocks[0].m_signature == 1  && pixy.ccc.blocks[0].m_height > 40)
    {
      
      digitalWrite(yellow,HIGH);      
      steerRight(20);
      delay(1000);
      digitalWrite(yellow,LOW); 
    //  steerStraight();
    }


    
  }    
  
}


void leftlapcount()
{

  // Get the current yaw reading and adjust it using the offset
  currentYaw = getYaw() + yawOffset;
  
  // Normalize the adjusted yaw to ensure it stays within the range -180 to 180
  currentYaw = normalizeYaw(currentYaw);

  Serial.println(currentYaw);
    // Check for passing through each lane's yaw range
  if (!passedLane1 && currentYaw >= 70 && currentYaw <= 110) {
    passedLane1 = true;
    Serial.println("Entered Lane 1 (Yaw: 80-100)");
  }

  if (passedLane1 && !passedLane4 && (currentYaw >= 160 && currentYaw <= 180) || (currentYaw >= -180 && currentYaw <= -160)) {
    passedLane4 = true;
    Serial.println("Entered Lane 4 (Yaw: 160 to 180)");
  }
  
 
  if (passedLane1 && passedLane4 && !passedLane3 && currentYaw >= -100 && currentYaw <= -70) {
    passedLane3 = true;
    Serial.println("Entered Lane 3 (Yaw: -100 to -70)");
  }
  
  if (passedLane1 && passedLane4 && passedLane3 && !passedLane2 && 
      ((currentYaw >= -20 && currentYaw <= 20) )) {
    passedLane2 = true;
    Serial.println("Entered Lane 2 (Yaw: -20 to 20)");
  }

  // If all lanes have been passed, count a lap and reset
  if ( passedLane2 && passedLane3 && passedLane4 && currentYaw >= 70 && currentYaw <= 100) {
    lapCount++;
    Serial.println("Lap Completed: " + String(lapCount));

    // Reset flags for next lap
    passedLane1 = false;
    passedLane2 = false;
    passedLane3 = false;
    passedLane4 = false;
  }  
  
  
}



void rightlapcount()
{
  // Get the current yaw reading and adjust it using the offset
  currentYaw = getYaw() + yawOffset;
  
  // Normalize the adjusted yaw to ensure it stays within the range -180 to 180
  currentYaw = normalizeYaw(currentYaw);

  Serial.println(currentYaw);
  // Check for passing through each lane's yaw range
  if (!passedLane1 && currentYaw >= 80 && currentYaw <= 100) {
    passedLane1 = true;
    Serial.println("Entered Lane 1 (Yaw: 80-100)");
  }
  if (passedLane1 && !passedLane2 && currentYaw >= -20 && currentYaw <= 20) {
    passedLane2 = true;
    Serial.println("Entered Lane 2 (Yaw: -20 to 20)");
  }
  if (passedLane1 && passedLane2 && !passedLane3 && currentYaw >= -100 && currentYaw <= -70) {
    passedLane3 = true;
    Serial.println("Entered Lane 3 (Yaw: -100 to -70)");
  }
  if (passedLane1 && passedLane2 && passedLane3 && !passedLane4 && 
      ((currentYaw >= 160 && currentYaw <= 180) || (currentYaw >= -180 && currentYaw <= -160))) {
    passedLane4 = true;
    Serial.println("Entered Lane 4 (Yaw: 160 to 180 or -160 to -180)");
  }

  // If all lanes have been passed, count a lap and reset
  if ( passedLane2 && passedLane3 && passedLane4 && currentYaw >= 80 && currentYaw <= 100) {
    lapCount++;
    Serial.println("Lap Completed: " + String(lapCount));

    // Reset flags for next lap
    passedLane1 = false;
    passedLane2 = false;
    passedLane3 = false;
    passedLane4 = false;
  }
  
  
}





void innerwalllogicleft()
{
  forward(150);
  // Calculate the error (difference from the ideal distance)
  int error =   leftDistance - desiredDistance;
 // int error =   leftDistance - rightDistance;
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
 // delay(40);
}


void innerwalllogicright()
{
  forward(150);
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
 // delay(40);
}

void readultrasonics()
{
  // Measure distances from the walls
  leftDistance = sonarLeft.ping_cm();
  rightDistance = sonarRight.ping_cm();
  frontDistance = sonarFront.ping_cm();
  // Debug print the distances
//  Serial.print("Left: " + String(leftDistance) + " cm, ");
//  Serial.print("Right: " + String(rightDistance) + " cm");
//  Serial.print("Front: " + String(frontDistance) + " cm");
//  Serial.println();
//  
  
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
//  Serial.print("yaw:  ");
//  Serial.print(yaw,2);
//
//  Serial.println("");

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
