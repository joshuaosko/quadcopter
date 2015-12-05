/*
 * IMU Integration
 * EN.605.715.91 - Homework 6
 * 
 * Authors:
 * Nathanael Mcconnell
 * Joshua Osko
 * Kendra Thurmond
 * 
 * The technique used to read bits from the low order registers was modeled 
 * from Arduino Playground. Source: http://playground.arduino.cc/Main/MPU-6050
 */
 
#include<Wire.h>
#include<Servo.h>
#include <PID_v1.h>

#define PIN_INPUT_ROLL 8
#define PIN_INPUT_THROTTLE 9
#define PIN_INPUT_PITCH 10
#define PIN_INPUT_YAW 11

#define PIN_OUTPUT_FL 4
#define PIN_OUTPUT_FR 5
#define PIN_OUTPUT_BL 6
#define PIN_OUTPUT_BR 7

//Define Variables we'll be connecting to
double SetpointRoll, InputRoll, OutputRoll;
double SetpointPitch, InputPitch, OutputPitch;
double SetpointYaw, InputYaw, OutputYaw;

//Define the aggressive and conservative Tuning Parameters
double aggKpRoll=4, aggKiRoll=0.2, aggKdRoll=1;  
double consKpRoll=1, consKiRoll=0.05, consKdRoll=0.25;
int gapRoll = 100; 

double aggKpPitch=4, aggKiPitch=0.2, aggKdPitch=1;
double consKpPitch=1, consKiPitch=0.05, consKdPitch=0.25;
int gapPitch = 100; 

double aggKpYaw=4, aggKiYaw=0.2, aggKdYaw=1;
double consKpYaw=1, consKiYaw=0.05, consKdYaw=0.25;
int gapYaw = 100; 

//Specify the links and initial tuning parameters
PID rollPID(&InputRoll, &OutputRoll, &SetpointRoll, consKpRoll, consKiRoll, consKdRoll, DIRECT);
PID pitchPID(&InputPitch, &OutputPitch, &SetpointPitch, consKpPitch, consKiPitch, consKdPitch, DIRECT);
PID yawPID(&InputYaw, &OutputYaw, &SetpointYaw, consKpYaw, consKiYaw, consKdYaw, DIRECT);

const int MPU=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
#define IMU_COUNT 75

int gyroX[IMU_COUNT], gyroY[IMU_COUNT], gyroZ[IMU_COUNT], accelX[IMU_COUNT], accelY[IMU_COUNT], accelZ[IMU_COUNT];
int gX, gY, gZ, aX, aY, aZ; 

  Servo servoFL; 
  Servo servoFR; 
  Servo servoBL; 
  Servo servoBR; 

void setup(){
    Serial.begin(9600);
  //Servo setup
  servoFL.attach(PIN_OUTPUT_FL);
  servoFR.attach(PIN_OUTPUT_FR);
  servoBL.attach(PIN_OUTPUT_BL);
  servoBR.attach(PIN_OUTPUT_BR);

  //int val = map(777, 0,1023, 0, 179); 
  for(int i=0; i<1024; i++){
     servoFL.write(i); 
     servoFR.write(i);  
     servoBL.write(i);  
     servoBR.write(i);   
  }
  // IMU Setup 
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     //get MPU 6050 out of sleep mode
  Wire.endTransmission(true);
  getValues();

 Serial.println("**** Begin Sensor Readings ****");
 
 //PID Input Setup for Remote Control
  pinMode(PIN_INPUT_ROLL, INPUT); // Set our input pins as such
  pinMode(PIN_INPUT_THROTTLE, INPUT);
  pinMode(PIN_INPUT_PITCH, INPUT);
  pinMode(PIN_INPUT_YAW, INPUT);

}//end setup

void loop(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request 14 registers
  delay(1330);
  //getValues();
  runPID(PIN_INPUT_ROLL, &rollPID, &SetpointRoll, &InputRoll, &OutputRoll, consKpRoll, consKiRoll, consKdRoll, aggKpRoll, aggKiRoll, aggKdRoll, gapRoll, gX, aX);
  runPID(PIN_INPUT_PITCH, &pitchPID, &SetpointPitch, &InputPitch, &OutputPitch, consKpPitch, consKiPitch, consKdPitch, aggKpPitch, aggKiPitch, aggKdPitch, gapPitch, gY, aY);
  runPID(PIN_INPUT_YAW, &yawPID, &SetpointYaw, &InputYaw, &OutputYaw, consKpYaw, consKiYaw, consKdYaw, aggKpYaw, aggKiYaw, aggKdYaw, gapRoll, gZ, aZ);
  int throttle = pulseIn(PIN_INPUT_THROTTLE, HIGH, 25000); // Read the pulse width of 
  //Serial.println("THROTTLE");
  //Serial.println(throttle);
  //Motor Control
  double MOTOR_FL = throttle - OutputRoll - OutputPitch - OutputYaw ;
  double MOTOR_BL = throttle - OutputRoll + OutputPitch + OutputYaw ; 
  double MOTOR_FR = throttle + OutputRoll - OutputPitch + OutputYaw; 
  double MOTOR_BR = throttle + OutputRoll + OutputPitch - OutputYaw ;


  servoFL.write(MOTOR_FL); 
  servoFR.write(MOTOR_FR); 
  servoBL.write(MOTOR_BL); 
  servoBR.write(MOTOR_BR); 

  Serial.print("," );
  Serial.println(MOTOR_FL);


/*
  //Input from remote is the setpoint for the first PID
  SetpointRoll = pulseIn(PIN_INPUT_ROLL, HIGH, 25000); // Read the pulse width of 
  InputRoll = gX; 
  double gap = abs(SetpointRoll-InputRoll); //note need to make the GYRO and REMOTE have similar values
  if(gap < gapRoll){
     rollPID.SetTunings(consKpRoll, consKiRoll, consKdRoll); 
  }
  else 
  {
     rollPID.SetTunings(aggKpRoll, aggKiRoll, aggKdRoll); 
  }
  rollPID.Compute();
  SetpointRoll = OutputRoll;
  InputRoll = aX; 
  //Not sure how the aggressive tuning will help, so for now just the constant
  rollPID.SetTunings(consKpRoll, consKiRoll, consKdRoll); 
  rollPID.Compute(); 
  */
}//end loop
void runPID(int pinInput, PID *myPID, double *Setpoint, double *Input, double *Output, double consKp, double consKi, double consKd, double aggKp, double aggKi, double aggKd, int gap, int gyro, int accel){
  //Setpoint is the Input from remote for first PID computation
  *Setpoint = pulseIn(pinInput, HIGH, 25000); //read the pulse width
  //Serial.println("REMOTE IN"); 
  //Serial.println(pinInput); 
  //Serial.println(*Setpoint); 
  //Input is the kinematics angle
  *Input = gyro; 
  double diff = abs(Setpoint - Input);
  //If very far away from the desired value, then aggressively change otherwise only moderate
  if(diff < gap)
  {
     myPID->SetTunings(consKp, consKi, consKd); 
  }
  else
  {
     myPID->SetTunings(aggKp, aggKi, aggKd); 
  }
  myPID->Compute();
  //output from first computation is the Setpoint
  *Setpoint = *Output; 
  //input is the gyroscope rate value
  *Input = accel; 
  //Not sure how the aggressive tuning will help, so for now just the constant
  myPID->SetTunings(consKp, consKi, consKd); 
  myPID->Compute(); 
  
}
void getValues() {
  //Serial.println("begin getValues"); 
  int count = 0; 
  while(count < IMU_COUNT) {
    delay(1);
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,14,true);
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    gyroX[count] = GyX;
    gyroY[count] = GyY;
    gyroZ[count] = GyZ;
    accelX[count] = AcX;
    accelY[count] = AcY;
    accelZ[count] = AcZ;
    count++;
  }//end while
  count = 0;
  SortValues();
 // Serial.println("OUT OF SORTING"); 
  gX = gyroX[49]; aX = accelX[49];
  gY = gyroY[49]; aY = accelY[49];
  gZ = gyroZ[49]; aZ = accelZ[49];
  Serial.print(gX); 
  Serial.print(", ");
   Serial.print(gY); 
  Serial.print(", ");
  Serial.print(gZ); 
  Serial.println(" "); 
  //  Serial.println("end getValues"); 
}

/*
 * Used to apply filter to sensor values
 */
void BubbleSort(int num[], int numLength)
{
  int i, j, flag = 1;    // set flag to 1 to start first pass
  int temp;             
  for(i = 1; (i <= numLength) && flag; i++)
  {
     flag = 0;
     for (j=0; j < (numLength -1); j++)
     {
      if (num[j+1] > num[j]) // ascending order simply changes to <
      { 
        temp = num[j]; // swap
        num[j] = num[j+1];
        num[j+1] = temp;
        flag = 1;   // indicates that a swap occurred.
      }
     }
 }
 return;   //arrays are passed to functions by address; nothing is returned
}
void SortValues(){
  //Serial.println("SORTING"); 
  BubbleSort(gyroX, IMU_COUNT); //roll
  BubbleSort(gyroY, IMU_COUNT); //pitch
  BubbleSort(gyroZ, IMU_COUNT); //yaw
  BubbleSort(accelX, IMU_COUNT);
  BubbleSort(accelY, IMU_COUNT);
  BubbleSort(accelZ, IMU_COUNT);
 //   Serial.println("END SORTING"); 

}

