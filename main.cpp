#include <Arduino.h>
#include<Wire.h>

// Define 
#define MPU_ADDR 0x68 // I2C address of the MPU-6050
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define TIMESTEP 10

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int16_t prevAcZ, prevAcX, prevAcY, prevGyZ, prevGyX;
int16_t offsetAcZ, offsetGyX;

int i;

double CutOffFreq = 1;

const double Alpha = (2 * PI * CutOffFreq * 0.001 * TIMESTEP) / ((2 * PI * CutOffFreq * 0.001 * TIMESTEP) + 1); //2 * pi * cutoff Frequency
const int NormAcZ = 16763;

unsigned long long currMillis;
unsigned long long prevMillis = 0;
unsigned long long millisBetweenSteps = 25;

double arx, ary, arz, grx, gry, grz, gsx, gsy, gsz, rx, ry, rz;
double gyroScale = 131;

float previousError = 0;
float integral = 0;
float SetPoint = 0; // in degrees (angle of the body should be 90)
float outputSpeed;
float error;
float derivative;
float measuredValue;

float Kp = 100, Ki = 20, Kd = 0;
bool PositionSafe; //Flag to indicate PID needs to be turned on

const int FREEFALLTIME = 10; // iterations in freefall
int freefallCount; //Freefall condition where we 
double GyXSum;

bool dirMotor1;
double reqVel;
bool reqDir;
double ControlCounter;
bool MotorsHigh;

int STEP_PIN1;
int STEP_PIN2;

int RobotMode; // 0 = safe, 1 = freefall, 2 = PID Controlled

void PidControlLoop(double measuredValue){
  
  error = SetPoint - measuredValue;
  integral += error * 0.001 * TIMESTEP;
  derivative = (error - previousError)/(0.001 * TIMESTEP);
  outputSpeed = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;
  delay(TIMESTEP);
  
}


void Motor1Setup(){
  
  digitalWrite(4, LOW);// Step
  digitalWrite(5, LOW);// Direction
  digitalWrite(6, HIGH);// MS1
  digitalWrite(7, HIGH);// MS2
  digitalWrite(8, HIGH);// MS3
  
}

void Motor2Setup(){
  
  digitalWrite(9, LOW); // Step
  digitalWrite(10, LOW); // Direction
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);     
  
}



void setup() {

  // Accel Comm Setup
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);  
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  i = 1;

  /*
      Digital Pin 2 and 3 is used by the Accelerometer
  */

//  // setting up digital pins to ouput.
//  for( int i = 4;  i < 12; i++){
//    pinMode(i, OUTPUT); 
//  }

  /*Motor 1 Setup*/
//  Motor1Setup();

  /*Motor 2 Setup*/
//  Motor2Setup();

  //optional for serial debug
  Serial.begin(9600);
}

void loop() 
{

  // Data Acquisition Step
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  gsx = GyX/gyroScale;   gsy = GyY/gyroScale;   gsz = GyZ/gyroScale;


  // Data Filtering Step

  Serial.print(""); Serial.print(AcZ);
  if(NormAcZ + 100 > AcZ && AcZ > NormAcZ - 100)
  {
    offsetAcZ = NormAcZ - AcZ;
    PositionSafe = true;
  }

  // Accel low pass filter.
  AcX = ((1-Alpha) * prevAcX ) + ((Alpha) * (AcX) );
  AcY = ((1-Alpha) * prevAcY ) + ((Alpha) * (AcY) );
  AcZ = ((1-Alpha) * prevAcZ ) + ((Alpha) * (AcZ) );

  //
  GyX = ((1-Alpha) * prevGyX) + ((Alpha) * (GyZ) );

  prevAcX = AcX;
  prevAcY = AcY;
  prevAcZ = AcZ;
  prevGyX = GyX;


  AcZ = AcZ + offsetAcZ;
  

  //Conversion

  arz = (180/PI)*atan(sqrt(square(AcY)+square(AcX))/AcZ); //calculate Z angle

  if (i == 1) {
    grx = arx;
    gry = ary;
    grz = arz;
  }
  // integrate to find the gyro angle
  else{
    grx = grx + (0.001 * TIMESTEP * gsx);
    gry = gry + (0.001 * TIMESTEP * gsy);
    grz = grz + (0.001 * TIMESTEP * gsz);
  }  


  rz = (0.96 * arz) + (0.04 * grz); // Z fusion angles (filter high pass + low pass)
  

  Serial.print("      "); Serial.print(AcZ);
  Serial.print("      "); Serial.print(rz);
  Serial.print("      "); Serial.print(outputSpeed);
  Serial.println();

  //Logic to determine RobotMode
  if(NormAcZ + 100 > AcZ && AcZ > NormAcZ - 100)
  {
    RobotMode = 0;

    freefallCount = 0;
  }
  else if(AcZ < NormAcZ - 100 && freefallCount < FREEFALLTIME)
  {
    RobotMode = 1;

    ++freefallCount;
    
    if(freefallCount > FREEFALLTIME)
      RobotMode = 2;
  }
  else
  {
    RobotMode = 2;
  }

  if(RobotMode == 0)
  {
    //Do some resets & calculate the offsets
  
    offsetAcZ = NormAcZ - AcZ;
    offsetGyX = -GyX;
  }
  else if (RobotMode == 1)
  {
    GyXSum += GyX + offsetGyX;

    if(GyXSum > 0)
      reqDir = true; //cw
    else
      reqDir = false;
  }
  if(RobotMode == 2)
  {
    //CalculateDesiredVelocityandsteps(); <- On you, buddy
    
    if(ControlCounter >= 1000 / (2 * reqVel))
    {
        ControlCounter = 0;

        if(MotorsHigh)
          MotorsHigh = false;
        else
          MotorsHigh = true; 
    }

    if(MotorsHigh)
    {
      digitalWrite(STEP_PIN1, HIGH);
      digitalWrite(STEP_PIN2, HIGH);    
    }  
    else
    {
      digitalWrite(STEP_PIN1, LOW);
      digitalWrite(STEP_PIN2, LOW);
    }
    
    ControlCounter += TIMESTEP;
  }
  else
  {
    ControlCounter = 0;
    MotorsHigh = false;
  }


  // Pid Control Step
  // 
  PidControlLoop(rz);

  // non blocking speed control.
//  currMillis = millis();
//
//  if(currMillis - prevMillis >= millisBetweenSteps){
//     prevMillis = currMillis;
//     digitalWrite(4,HIGH);
//     digitalWrite(4,LOW);
//  }

}