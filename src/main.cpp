#include <Arduino.h>
#include<Wire.h>
#include<PID_v1.h>


// Define 
#define MPU_ADDR 0x68 // I2C address of the MPU-6050
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define TIMESTEP 10	//step between iterations, in ms.
#define MAXOUTPUTSPEED 100 //Maximum speed for PID output

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int16_t prevAcZ, prevAcX, prevAcY, prevGyZ, prevGyX;
int16_t offsetAcZ, offsetGyX;

int i, numSafeCycles;

double CutOffFreq = 1;

const double Alpha = (2 * PI * CutOffFreq * 0.001 * TIMESTEP) / ((2 * PI * CutOffFreq * 0.001 * TIMESTEP) + 1); //2 * pi * cutoff Frequency
const int NormAcZ = 16763;

unsigned long long windowStartTime;

double arx, ary, arz, grx, gry, grz, gsx, gsy, gsz, rx, ry, rz;
double gyroScale = 131;


double setPoint, input, output;
double Kp = 1, Ki= 2, Kd = 0.5;

PID myPid(&setPoint, &input, &output, Kp, Ki, Kd, DIRECT);

int windowSize = 5000;


const int FREEFALLTIME = 10; // iterations in freefall
int freefallCount; 

double GyXSum, offsetGyXSum, offsetAcZSum;
double ControlCounter;
double reqVel;

bool reqDir;
bool MotorsHigh;
bool PositionSafe; //Flag to indicate PID needs to be turned on

uint8_t STEP_PIN1 = 4;
uint8_t STEP_PIN2 = 9;
uint8_t DIR_PIN1 = 5;
uint8_t DIR_PIN2 = 10;


int RobotMode; // 0 = safe, 1 = freefall, 2 = PID Controlled

// 	Changes		//
//Decides which mode the robot is in
void RobotModeCalc()
{
	//Logic to determine RobotMode
	if (NormAcZ + 100 > AcZ && AcZ > NormAcZ - 100)
	{
		RobotMode = 0;
		freefallCount = 0;
	}
	else
	{
		RobotMode = 1;
	}
}
//			//

void Motor1Setup(){
  
  digitalWrite(STEP_PIN1, LOW);// Step
  digitalWrite(DIR_PIN1, LOW);// Direction
  digitalWrite(6, HIGH);// MS1
  digitalWrite(7, HIGH);// MS2
  digitalWrite(8, HIGH);// MS3
  
}

void Motor2Setup(){
  
  digitalWrite(STEP_PIN2, LOW); // Step
  digitalWrite(DIR_PIN1, LOW); // Direction
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

  windowStartTime = millis();
  
  setPoint = 0;

  myPid.SetOutputLimits(20, windowSize);

  myPid.SetMode(AUTOMATIC);

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

  if(NormAcZ + 100 > AcZ && AcZ > NormAcZ - 100)
  {
    offsetAcZ = NormAcZ - AcZ;
    PositionSafe = true;
  }

  // Accel low pass filter.
  AcX = ((1-Alpha) * prevAcX ) + ((Alpha) * (AcX) );
  AcY = ((1-Alpha) * prevAcY ) + ((Alpha) * (AcY) );
  AcZ = ((1-Alpha) * prevAcZ ) + ((Alpha) * (AcZ) );
  GyX = ((1-Alpha) * prevGyX) + ((Alpha) * (GyZ) );

  prevAcX = AcX;
  prevAcY = AcY;
  prevAcZ = AcZ;
  prevGyX = GyX;

  AcZ = AcZ + offsetAcZ;

  // arz = (180/PI)*acos(AcZ/NormAcZ); //calculate Z angle
  arz = atan(sqrt(square(AcY)+square(AcX))/ AcZ) * (180 / PI);

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


  rz = (0.96 * arz) + (0.04 * grx); // Combining Acceleration and gyro data

  RobotModeCalc();
  input = rz;// no touchy
  myPid.Compute();// no touchy

  if(RobotMode == 0)
  {
    //Find the offsets of the Acceleromter as the average position of the robot while it is in the safe zone
    offsetAcZSum += NormAcZ - AcZ;
    
    //		Changes		//
    offsetAcZ = offsetAcZSum / numSafeCycles;
    offsetAcXSum += AcX;
    offsetAcX = -offsetAcXSum / numSafeCycles;
    //				//
    
    ++numSafeCycles;
  }
  else if (RobotMode == 1)
  {
    //		Changes		//
    //Reset variables for pervious loop
    offsetAcXSum = 0;
    offsetAcZSum = 0;
    numSafeCycles = 0;

    //Calculate Direction
    if ((AcX + offsetAcX) >= 0)
    {
      reqDir = true;
    }
    else
    {
      reqDir = false;
    }
    //				//
    
    
    if(ControlCounter >= 1000 / (2 * reqVel))
    {
        ControlCounter = 0;
        if(MotorsHigh)
          MotorsHigh = false;
        else
          MotorsHigh = true; 
    }

    //! Jacob Do not touch this please !
    if(millis() - windowStartTime > windowSize){
      windowStartTime += windowSize;
    }

    if(output < millis() - windowStartTime && MotorsHigh) 
    {
      digitalWrite(STEP_PIN1, HIGH);
      digitalWrite(STEP_PIN2, HIGH);    
    }  
    else
    {
      digitalWrite(STEP_PIN1, LOW);
      digitalWrite(STEP_PIN2, LOW);
    }
    // this ^

    Serial.println(reqDir);
    
    ControlCounter += TIMESTEP;
    delay(TIMESTEP);
  }
}
