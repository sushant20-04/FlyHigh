/*
  @file receiver_nrf24.ino
  @brief Interfacing NRF24L01 with ARIES Board using SPI.
  @detail The demo to receive the data from transmitter controller using NRF24L01

   Aries V2 pinout: https://vegaprocessors.in/blog/interfacing-8x8-led-dot-matrix-to-aries-v2-board/
   NRF24L01 pi88nout: https://howtomechatronics.com/wp-content/uploads/2017/02/NRF24L01-Pinout-NRF24L01-PA-LNA-.png

   * Library Name : RF24 (by TMRH20, Avamander)

   *** A Radio Transceiver Module (NRF24L01+PA/LNA) ***

   Connections:
   NRF24L01      Aries Board
   VCC          -   3.3V
   GND          -   GND
   CE           -   GPIO-9
   CNS          -   GPIO-10
   SCK          -   SCLK0
   MISO         -   MISO0
   MOSI         -   MOSI0
*/

#include <Servo.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>
#include <nRF24L01.h>
#include "MPU6050_res_define.h"
#include <math.h>
#include <pwm.h>


SPIClass SPI(0);

// Define the I2C object
TwoWire Wire(1);
const int mpu6050 = 0x68;    // Default address for MPU-6050

int redLed=24;    // Red LED
int greenLed=22; // Green LED
int blueLed=23;  // Blue LED

#define TRUE 1
#define FALSE 0

uint32_t LoopTimer;

RF24 receiver(9, 10); // CE, CSN
const byte address[6] = "101000";
// boolean button_state = 0;
// int led_pin = 23;

Servo bldc;

struct Gyro {
  float ax;
  float ay;
  float az;
  float temp;
  float wx;
  float wy;
  float wz;
};

// struct procGyro{
//   float Ax;
//   float Ay;
//   float Az;
//   float Temp;
//   float Wx;
//   float Wy;
//   float Wz;
// };


struct PID {
  float Kp;
  float Ki;
  float Kd;

};

struct Signal {
  int height;
  int  frwrvr;
  int  rgtlft;
  int yaw;
};

// Servo objects for four motors
Servo motorFr; // Front Right Motor
Servo motorFl; // Front Left Motor
Servo motorBr; // Back Right Motor
Servo motorBl; // Back Left Motor

// Pins for the motors (ESCs)
#define MOTOR_FR 3
#define MOTOR_FL 2
#define MOTOR_BR 4
#define MOTOR_BL 1



// Struct to carry the Roll, Pitch and Yaw data
struct RPY {
  float roll;
  float pitch;
  float yaw;
};

// Struct to carry the reference Throttle, Roll, Pitch, Yaw , KalmanROll and KalmanPitch data
struct REF_RPY {
  float throttle;
  float roll;
  float pitch;
  float yaw;
  float KalmanRoll;
  float KalmanPitch;
};

// Struct to carry the PID output
struct PIDoutput {
  float roll;
  float pitch;
  float yaw;
};

struct rate{
  float roll;
  float pitch;
  float yaw;
};

struct caliberation{
  float roll;
  float pitch;
  float yaw;

};

// Variabkes to keep track of the PID errors


// Signal object to carry the data
Signal data;

// Gyro object to carry the raw Gyroscope data
Gyro gyro;

// RPY object to carry the Roll, Pitch and Yaw data
RPY rpy;

// PRY object to carry the reference Roll, Pitch and Yaw data
REF_RPY ref_rpy;

// procGyro object to carry the processed Gyroscope data
// procGyro pGyro;

// PID object to carry the PID constants
PID pid_roll;
PID pid_pitch;
PID pid_yaw;
PID pid_kalman_roll;
PID pid_kalman_pitch;


float integral_roll = 0.0;
float integral_pitch = 0.0;
float integral_yaw = 0.0;
float derivative_roll = 0.0;
float derivative_pitch = 0.0;
float derivative_yaw = 0.0;
float integral_kalman_roll = 0.0;
float integral_kalman_pitch = 0.0;
float derivative_kalman_roll = 0.0;
float derivative_kalman_pitch = 0.0;

float prev_error_roll = 0.0;
float prev_error_pitch = 0.0;
float prev_error_yaw = 0.0;
float prev_error_kalman_roll = 0.0;
float prev_error_kalman_pitch = 0.0

void setconst(){
pid_roll.Kp=500;
pid_roll.Ki=0;
pid_roll.Kd=0;

pid_pitch.Kp=500; 
pid_pitch.Ki=0;
pid_pitch.Kd=0;

pid_yaw.Kp=500;
pid_yaw.Ki=0;
pid_yaw.Kd=0;

pid_kalman_roll.Kp=10;
pid_kalman_roll.Ki=0;
pid_kalman_roll.Kd=0;

pid_kalman_pitch.Kp=10;
pid_kalman_pitch.Ki=0;
pid_kalman_pitch.Kd=0;

}


// Carrying caliberation values

caliberation CALIBER;

// Carrying rates
rate Rate;


// PIDoutput object to carry the PID output
PIDoutput pidOutput;

unsigned long currentRFTime = 0;
unsigned long previousRFTime = 0;

// Variables to keep track of timestamps for PID calculation
unsigned long currentPIDTime = 0;
unsigned long previousPIDTime = 0;

float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;

// Function to implement the PID
float calcPID(float ref, PID &pid, float actual, float &prev_error, float &integral, float &derivative, float &dt) {

  // Calculate the error
  float error = ref - actual;

  // Calculate the integral
  integral = integral + error*dt;

  
  // Calculate the derivative
  derivative = (error - prev_error)/dt;

  // Calculate the PID output
  float output = pid.Kp * error + pid.Ki * integral + pid.Kd * derivative;

//  if (output>90){
//     output=90;
//  }
//  if (output<-90){
//     output=-90; 
//   }

  // Update the previous error
  prev_error = error;
  return output;

  if (output<0){
    output=0;
  
  }

}

signed short Ax,Ay,Az,Temp,Wx,Wy,Wz;
float a = 0.00;
float b = 0.00;
float c = 0.00;
float p = 0.00;
float q = 0.00;
float r = 0.00;

float gyroX,gyroY,gyroZ;
float accelX,accelY,accelZ;
float AngleRoll, AnglePitch;

void (Gyro &gyro) {

// for (int z=0;z<20;z++)
// {

  Wire.beginTransmission(mpu6050); // Start the transmission
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false); // End the transmission
  Wire.requestFrom(mpu6050, 14, true); // request a total of 14 registers
  Ax = (Wire.read() << 8 | Wire.read()); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  Ay = (Wire.read() << 8 | Wire.read()); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  Az = (Wire.read() << 8 | Wire.read()); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Temp = (Wire.read() << 8 | Wire.read()); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  Wx =( Wire.read() << 8 | Wire.read()); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  Wy = (Wire.read() << 8 | Wire.read()); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  Wz = (Wire.read() << 8 | Wire.read()); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Wire.endTransmission(true); // End the transmission
  // delayMicroseconds(50);
  a = (float)Wx/65.0;
  b = (float)Wy/65.0;
  c = (float)Wz/65.0;
  d= (float)Ax/4096.0;
  e= (float)Ay/4096.0;
  f= (float)Az/4096.0;

// }
  gyroX = a;
  gyroY = b;
  gyroZ = c;
  accelX = d;
  accelY = e;
  accelZ = f;

AngleRoll=atan(accelY/sqrt(accelX*accelX+accelZ*accelZ))*(180/3.142);
AnglePitch=-atan(accelX/sqrt(accelY*accelY+accelZ*accelZ))*(180/3.142);

// Serial.print("Angle Roll: ");
// Serial.print(AngleRoll);
// Serial.print(" Angle Pitch: ");
// Serial.println(AnglePitch);
}





void calib (rate &rate,caliberation &caliberation)
  {

    for ( int iterations=0;iterations<2000;iterations++){
      gyroData(gyro);

  // rate.roll = gyro.wx; // Angular velocity in X-axis in degrees per second
  // rate.pitch = gyro.wy; // Angular velocity in Y-axis in degrees per second
  // rate.yaw = gyro.wz; // Angular velocity in Z-axis in degrees per second

  
      caliberation.roll += gyroX;
      caliberation.pitch += gyroY;
      caliberation.yaw += gyroZ;
      delay(10);

    }

      caliberation.roll = (float)caliberation.roll/2000.0;
      caliberation.pitch = (float)caliberation.pitch/2000.0;
      caliberation.yaw = (float)caliberation.yaw/2000.0;


  }


// void calcReferenceInput(Signal &signal, REF_RPY &rpy) {
//   // rpy.throttle = signal.height;
//   // rpy.roll = signal.rg/


//   rpy.throttle = 97000;
//   rpy.roll = 0;
//   rpy.pitch = 0 ;
//   rpy.yaw =0;

// }




void calcInput(Signal &signal, Gyro &gyro, PIDoutput &pidOutput,REF_RPY &ref_rpy) {

  // Calculate the time difference
  currentPIDTime = micros();
  float dt = (currentPIDTime - previousPIDTime)/1000000.0;
  previousPIDTime = currentPIDTime;
  // Serial.printl

  
  // low pass filter for rate.roll,pitch and yaw data to remove noise

  Rate.roll = gyroX - CALIBER.roll; // Angular velocity in X-axis in degrees per second
  Rate.pitch = gyroY - CALIBER.pitch; // Angular velocity in Y-axis in degrees per second
  Rate.yaw = gyroZ  -CALIBER.yaw; // Angular velocity in Z-axis in degrees per second


  if (abs(Rate.roll)<0.10){
    Rate.roll=0;
  }
  if (abs(Rate.pitch)<0.10){
    Rate.pitch=0;
  }
  if (abs(Rate.yaw)<0.10){
    Rate.yaw=0;
  }

  // unsigned long a1,a2;
  // a1=micros();
  // Serial.print("Rate roll: ");
  // Serial.print(Rate.roll);
  // Serial.print(" Rate pitch: ");
  // Serial.print(Rate.pitch);
  // Serial.print(" Rate yaw: ");
  // Serial.println(Rate.yaw);

  // a2=micros();
  // Serial.print("Time taken by printing rate calculation: ");
  // Serial.println(a2-a1);
  // Calculate the reference input
  // calcReferenceInput(signal, ref_rpy);

// Add signal values to ref_rpy


  ref_rpy.throttle = signal.height;
  ref_rpy.roll = 0;
  ref_rpy.pitch = 0 ;
  ref_rpy.yaw =0;
  
  // PID calculation
  // unsigned long t1,t2;

  // t1=micros();

kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, Rate.Roll, AngleRoll);

KalmanAngleRoll=Kalman1DOutput[0]; KalmanUncertaintyAngleRoll=Kalman1DOutput[1];

kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, Rate.Pitch, AnglePitch);

KalmanAnglePitch=Kalman1DOutput[0]; KalmanUncertaintyAnglePitch=Kalman1DOutput[1];



ref_rpy.KalmanRoll=KalmanAngleRoll;
ref_rpy.KalmanPitch=KalmanAnglePitch;
ref_rpy.roll  = calcPID(ref_rpy.KalmanRoll, pid_kalman_roll, AngleRoll, prev_error_kalman_roll, integral_kalman_roll, derivative_kalman_roll, dt);
ref_rpy.pitch = calcPID(ref_rpy.KalmanPitch, pid_kalman_pitch, AnglePitch, prev_error_kalman_pitch, integral_kalman_pitch, derivative_kalman_pitch, dt);
  
  pidOutput.roll = calcPID(ref_rpy.roll, pid_roll, Rate.roll, prev_error_roll, integral_roll, derivative_roll, dt);
  // Serial.println("PID roll: ");
  // Serial.print(pidOutput.roll);

  pidOutput.pitch = calcPID(ref_rpy.pitch, pid_pitch, Rate.pitch, prev_error_pitch, integral_pitch, derivative_pitch, dt);
  // Serial.print(" PID pitch: ");
  // Serial.print(pidOutput.pitch);

  pidOutput.yaw = calcPID(ref_rpy.yaw, pid_yaw, Rate.yaw, prev_error_yaw, integral_yaw, derivative_yaw, dt);

  // Serial.print(" PID yaw: ");
  // Serial.println(pidOutput.yaw);
  // t2=micros();

  // Serial.print("Time taken by PID calculation: ");
  // Serial.println(t2-t1);
  // motorControl(ref_rpy, pidOutput);

// a1=micros();

  int motorFrSpeed = ref_rpy.throttle + pidOutput.roll + pidOutput.pitch  + pidOutput.yaw -14000 ;
  int motorFlSpeed = ref_rpy.throttle - pidOutput.roll + pidOutput.pitch  - pidOutput.yaw ;
  int motorBrSpeed = ref_rpy.throttle + pidOutput.roll - pidOutput.pitch  - pidOutput.yaw ;
  int motorBlSpeed = ref_rpy.throttle - pidOutput.roll - pidOutput.pitch  + pidOutput.yaw -14000;




// Limiting motor speeds to 0
  if (motorFrSpeed<0){
    motorFrSpeed=0;
  }
  if (motorFlSpeed<0){
    motorFlSpeed=0;
  }
  if (motorBrSpeed<0){
    motorBrSpeed=0;
  }
  if (motorBlSpeed<0){
    motorBlSpeed=0;
  }

  // Limiting motor speeds to 22000
  if (motorFrSpeed>142000){
    motorFrSpeed=142000;
  } 
  if (motorFlSpeed>142000){
    motorFlSpeed=142000;
  }
  if (motorBrSpeed>142000){
    motorBrSpeed=142000;
  }
  if (motorBlSpeed>142000){
    motorBlSpeed=142000;
  }



  // Print the motor speeds in one line instead of using 8 lines of print functions
  
  // Serial.print("1=");
  // Serial.print(motorFrSpeed);
  // Serial.print(" 2=");
  // Serial.print(motorFlSpeed);
  // Serial.print(" 3=");
  // Serial.print(motorBrSpeed);
  // Serial.print(" 4=");
  // Serial.println(motorBlSpeed);


  // Write the motor speeds

  // motorFr.writeMicroseconds(motorFrSpeed);
  // motorFl.writeMicroseconds(motorFlSpeed);
  // motorBr.writeMicroseconds(motorBrSpeed);
  // motorBl.writeMicroseconds(motorBlSpeed);
// unsigned long x1,x2;
//   x1=micros();

  PWM.PWMC_Set_Period(1, SERVO_PERIOD);
  PWM.PWMC_Set_OnOffTime(1, motorBlSpeed);
  PWM.PWMC_init(1);
  PWM.PWMC_Enable();
  delayMicroseconds(100);

  PWM.PWMC_Set_Period(2, SERVO_PERIOD);
  PWM.PWMC_Set_OnOffTime(2, motorFlSpeed);
  PWM.PWMC_init(2);
  PWM.PWMC_Enable();
  delayMicroseconds(100);

  PWM.PWMC_Set_Period(3, SERVO_PERIOD);
  PWM.PWMC_Set_OnOffTime(3, motorFrSpeed);
  PWM.PWMC_init(3);
  PWM.PWMC_Enable();
  delayMicroseconds(100);

  PWM.PWMC_Set_Period(4, SERVO_PERIOD);
  PWM.PWMC_Set_OnOffTime(4, motorBrSpeed);
  PWM.PWMC_init(4);
  PWM.PWMC_Enable();
  delayMicroseconds(100);

// x2=micros();

// Serial.print("Time taken by motorControl function: ");
// Serial.println(x2-x1);
  // a2=micros();
  // Serial.print("Time taken by motorControl function: ");
  // Serial.println(a2-a1);
}




void receiveData() {

  
  if (receiver.available()) {
    // Serial.println("DATA AVAILABLE");
    
    digitalWrite(blueLed,1);
    digitalWrite(redLed,1);
    digitalWrite(greenLed,0);

    // delay(100);

    receiver.read(&data, sizeof(data));

  }


  else{
    Serial.println("NO DATA");
    digitalWrite(redLed,0);
    digitalWrite(greenLed,1);
    digitalWrite(blueLed,1);
    // delay(100);
      data.height = data.height;
      data.frwrvr = data.frwrvr;
      data.rgtlft = data.rgtlft;
      data.yaw = data.yaw;
  }

}




void setup() {

 // set the serial monitor baud rate
digitalWrite(blueLed,0);
digitalWrite(greenLed,1);
digitalWrite(redLed,1);


Serial.begin(115200);
// digitalWrite(greenLed,0);

 Wire.beginTransmission(mpu6050); // Start with device write address 
  Wire.write(SMPLRT_DIV);   //Write to sample rate register 0x19
  Wire.write(0x07);     //1KHz sample rate
  delayMicroseconds(100);
  Wire.endTransmission(true);
  delayMicroseconds(100); 

  Wire.beginTransmission(mpu6050);
  Wire.write(PWR_MGMT_1);    //Write to power management register 0x6B
  Wire.write(0x01);    //X axis gyroscope reference frequency
  Wire.endTransmission(true);
  delayMicroseconds(100); 
  
  
  Wire.beginTransmission(mpu6050);
  Wire.write(CONFIG);      // Write to Configuration register 0x1A
  Wire.write(0x00);    //Fs = 8KHz
  Wire.endTransmission(true);
  delayMicroseconds(100);
  
  
  Wire.beginTransmission(mpu6050);
  Wire.write(GYRO_CONFIG);   //Write to Gyro configuration register 0x1B
  Wire.write(0x8);    //Full scale range +/- 1000 degree/C
  Wire.endTransmission(true);
  delayMicroseconds(100);

// digitalWrite(redLed,0);

receiver.begin();
if (!receiver.begin()) { 
  Serial.println(F("radio hardware not responding!"));
  }
else {
  Serial.println(F("radio hardware is responding!"));
  }

receiver.openReadingPipe(0, address);   //Setting the address at which we will receive the data
receiver.setPALevel(RF24_PA_MAX);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
receiver.startListening();       //This sets the module as receiver
Serial.print("WORKING PROPERLY");


setconst();
// makeZero(pidOutput);




int j,k;

for (int i=1;i<5;i++){
  if (i%2==0){
      k=9000;
    }
  else{
    k=0;
  }
   for (j = 60000; j<104000;j+=1000){
    
    PWM.PWMC_Set_Period(i, SERVO_PERIOD);
    PWM.PWMC_Set_OnOffTime(i, j+k);
    PWM.PWMC_init(i);
    PWM.PWMC_Enable();
    delay(100);
    digitalWrite(redLed,0);

   }
   PWM.PWMC_Set_Period(i, SERVO_PERIOD);
    PWM.PWMC_Set_OnOffTime(i, 0);
    PWM.PWMC_init(i);
    PWM.PWMC_Enable();
    delay(500);

}

  delay(8000);

calib(Rate, CALIBER);


  // for (int i=1;i<5;i++){
  //   PWM.PWMC_Set_Period(i, SERVO_PERIOD);
  //   PWM.PWMC_Set_OnOffTime(i, 0);
  //   PWM.PWMC_init(i);
  //   PWM.PWMC_Enable();
  //   delay(100);
  // }



// // Print Caliberation values
// Serial.print("Caliberation roll: ");
// Serial.print(CALIBER.roll);
// Serial.print(" Caliberation pitch: ");
// Serial.print(CALIBER.pitch);
// Serial.print(" Caliberation yaw: ");
// Serial.println(CALIBER.yaw);

// delay(2000);

} 

float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);

  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);

  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}



unsigned long p1,p2;


void loop(){


// delay(2000);
receiveData();

// previousRFTime = micros();
// gyroData(gyro);
// currentRFTime = micros();
// for (int z=0;z<20;z++)
// {

  Wire.beginTransmission(mpu6050); // Start the transmission
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false); // End the transmission
  Wire.requestFrom(mpu6050, 14, true); // request a total of 14 registers
  Ax = (Wire.read() << 8 | Wire.read()); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  Ay = (Wire.read() << 8 | Wire.read()); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  Az = (Wire.read() << 8 | Wire.read()); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Temp = (Wire.read() << 8 | Wire.read()); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  Wx =( Wire.read() << 8 | Wire.read()); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  Wy = (Wire.read() << 8 | Wire.read()); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  Wz = (Wire.read() << 8 | Wire.read()); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Wire.endTransmission(true); // End the transmission
  // delayMicroseconds(50);
  a = (float)Wx/65.0;
  b = (float)Wy/65.0;
  c = (float)Wz/65.0;
  d= (float)Ax/4096.0;
  e= (float)Ay/4096.0;
  f= (float)Az/4096.0;

// }

  gyroX = a;
  gyroY = b;
  gyroZ = c;
  accelX = d;
  accelY = e;
  accelZ = f;

AngleRoll =  atan(accelY/sqrt(accelX*accelX+accelZ*accelZ))*(180/3.142);
AnglePitch= -atan(accelX/sqrt(accelY*accelY+accelZ*accelZ))*(180/3.142);


// Serial.print("Angle Roll: ");
// Serial.print(AngleRoll);
// Serial.print(" Angle Pitch: ");
// Serial.println(AnglePitch);


calcInput(data, gyro, pidOutput,ref_rpy);
// Serial.print("Time taken by calc() function: ");
// Serial.println(currentRFTime - previousRFTime);



// delay(7);



// Serial.print("Time taken by loop function: ");

// p2=micros();
delay(3);

// Serial.println(p2-p1);

// p1=micros();

}







