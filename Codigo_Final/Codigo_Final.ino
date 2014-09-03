#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>
#include "Kalman.h"

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 700
#define MXPR 30 
#define MXN 32
#define MYP 31
#define MYN 33

Kalman kalmanX;
Kalman kalmanY;
float inverOutput;
boolean e=false;
 
Servo motorXP;
Servo motorXN;
Servo motorYP;
Servo motorYN;

int velX=1400;
int velY=1400;

/* PID Data */
float kp=0.6;
float ki=1.60;
float kd=0;

double Setpoint, Input, Output;    //Define Variables we'll be connecting to
PID myPID(&Input, &Output, &Setpoint,kp,ki,kd, DIRECT);   //Specify the links and initial tuning parameters


/* IMU Data */
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

double accXangle, accYangle; // Angle calculate using the accelerometer
double temp; // Temperature
double gyroXangle, gyroYangle; // Angle calculate using the gyro

double kalAngleX, kalAngleY; // Calculate the angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

void setup() {

  Serial.begin(115200);
  
  Setpoint = 180;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(200);
  
  Wire.begin();

  //Se inicializa el sensor
  iniciarSENSORIMU();
  iniciarMOTORES();
}

void loop() {

 motorXN.writeMicroseconds(1400);
 /*motorYP.writeMicroseconds(1400);


  if(lecturaAngulo(0) < 178 ){
    motorYN.writeMicroseconds(velY);
    if(velY<1999){
    velY=velY+5;
    }
  }

  if(lecturaAngulo(0) > 182){
    motorYN.writeMicroseconds(velY);
    if(velY > 1000){
    velY=velY-5;
    }
  }*/
motorXP.writeMicroseconds(1400);
  /*if(lecturaAngulo(1) < 178  &&  lecturaAngulo(1) > 182){
    motorXP.writeMicroseconds(velX);
  
  }
  else{

  if(lecturaAngulo(1) < 178){
    motorXP.writeMicroseconds(velX);
   if(velX > 1000){
    velX=velX-5;
    }
  }
  if(lecturaAngulo(1) > 182){
    motorXP.writeMicroseconds(velX);
    if(velX<1999){
    velX=velX+5;
    }  
  }
}

  delay(150);
*/

}





/*
***************************************************
 BLOQUES DE FUNCIONES 
 ***************************************************
 */

void iniciarSENSORIMU(){
  
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
  
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

    while (i2cRead(0x75, i2cData, 1));
 

  delay(1000); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;

  kalmanX.setAngle(accXangle); // Set starting angle
  kalmanY.setAngle(accYangle);
  gyroXangle = accXangle;
  gyroYangle = accYangle;


  timer = micros();
}

float lecturaAngulo(int val){
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  gyroX = ((i2cData[8] << 8) | i2cData[9]);
  gyroY = ((i2cData[10] << 8) | i2cData[11]);
  gyroZ = ((i2cData[12] << 8) | i2cData[13]);

  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;

  double gyroXrate = (double)gyroX / 131.0;
  double gyroYrate = -((double)gyroY / 131.0);


  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - timer) / 1000000); // Calculate the angle using a Kalman filter
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros() - timer) / 1000000);
  timer = micros();

  // PID
    Input = kalAngleX; 
    myPID.Compute();    
 
  

  float valEnX = kalAngleX;
  float valEnY = kalAngleY;

  //Serial.println(kalAngleY);
  if(val==1)
    return valEnX; 
  else
    return valEnY;

}


void iniciarMOTORES(){

  motorXP.attach(MXPR);
  motorXN.attach(MXN);
  motorYP.attach(MYP);
  motorYN.attach(MYN);

  motorXP.writeMicroseconds(MAX_SIGNAL);
  motorXN.writeMicroseconds(MAX_SIGNAL);
  motorYP.writeMicroseconds(MAX_SIGNAL);
  motorYN.writeMicroseconds(MAX_SIGNAL);
  delay(2000);

  motorXP.writeMicroseconds(MIN_SIGNAL);
  motorXN.writeMicroseconds(MIN_SIGNAL);
  motorYP.writeMicroseconds(MIN_SIGNAL);
  motorYN.writeMicroseconds(MIN_SIGNAL);
  delay(4000);
}




