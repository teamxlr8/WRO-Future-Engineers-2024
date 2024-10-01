#include <Wire.h>
#include "AHRSProtocol.h"             // navX-Sensor Register Definition header file
#include <NewPing.h>
#include <Servo.h>

#define TRIGGER_PIN 14

#define ECHO_PIN 15

#define MAX_DISTANCE 500// Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar1(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
#define TRIGGER_PIN 12

#define ECHO_PIN 11
int z = 0;
NewPing sonar2(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
Servo servo_pin_9;
int L = 0;
int F = 0;
String prefixstr = "";

byte data[512];

#define ITERATION_DELAY_MS                   10
#define NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT  0x32
#define NUM_BYTES_TO_READ                    8
float yaw;

int yawint ;
int lapcount = 0;
int oldyaw = 0;
int newyaw = 0;



void setup()
{
  servo_pin_9.attach(9);
  pinMode(A3, INPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);


  servo_pin_9.write( 90 );
  Serial.begin(115200);
  Wire.begin(); // join i2c bus (address optional for master)

  for ( int i = 0; i < sizeof(data); i++ ) {
    data[i] = 0;
  }
  digitalWrite( 2 , LOW );
  digitalWrite( 3 , HIGH );
}
int x = 0;
int y = 0;

int register_address = NAVX_REG_YAW_L;

void loop()
{

  //Serial.println(digitalRead(A3));
  L = sonar2.ping_cm();
  F = sonar1.ping_cm();
  //Serial.print(prefixstr + "Left:"+sonar2.ping_cm() );
  //Serial.println();
  //Serial.print(prefixstr + "Front:"+sonar1.ping_cm() );
  //Serial.println();
  if (( ( L < 30 ) && ( L > 20 ) )) {
    servo_pin_9.write( 90 );
  }
  if (( L > 30 )) {
    servo_pin_9.write( 105 );
  }
  if (( ( L < 20 ) && ( L > 1 ) )) {
    servo_pin_9.write( 75 );
  }
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
  while (Wire.available()) {                                   // Read data (slave may send less than requested)
    data[i++] = Wire.read();
  }
  Wire.endTransmission();                                      // Stop transmitting

  /* Decode received data to floating-point orientation values */
  yaw =     IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&data[0]);   // The cast is needed on arduino

  /* Display orientation values */
  Serial.print("yaw:  ");
  Serial.print(yaw, 2);
  Serial.print("yawint:");
  Serial.print(yawint);
  Serial.println("");

  delay(ITERATION_DELAY_MS);

  mapsides();
  /*x= yaw;

    z=x-z;
    Serial.println(y);
    if( x>=80)
    {
    y=y+1;}
    if ( y >= 12)
    {
        digitalWrite( 2 , LOW );
    digitalWrite( 3 , LOW );
    }*/

}


void mapsides()
{
  if (yaw > -20 && yaw < 20)
  {
    yawint = 0;
    newyaw = 0;
  }


  if (yaw > 60 && yaw < 99)
  {
    yawint = 1;
    newyaw = 1;
  }


  if (yaw > 150 && yaw < 185)
  {
    yawint = 2;
    newyaw = 2;
  }

  if (yaw > -180 && yaw < -155)
  {
    yawint = 3;
    newyaw = 3;
  }

  if (oldyaw == 1 and newyaw == 0)
  {
    lapcount++;
    Serial.println("LAP COMPLETED");
  }

  if (lapcount == 3)
  {
    Serial.println("STOPPING ROBOT");
    delay(1000);
    digitalWrite( 2 , LOW );
    digitalWrite( 3 , LOW );
    delay(20000);
  }


  oldyaw = yawint;

}
