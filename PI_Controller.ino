#include <Servo.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>
#include <nRF24L01.h>
#include "MPU6050_res_define.h"
// #include <math.h>
#include <pwm.h>


SPIClass SPI(0);
TwoWire Wire(0);
const int MPU6050 = 0x68;
int redLed=24;    // Red LED
int greenLed=22; // Green LED
int blueLed=23;  // Blue LED

#define TRUE 1
#define FALSE 0

uint32_t LoopTimer;

RF24 receiver(9, 10);
const byte address[6] = "101000";

#define MOTOR_1 1
#define MOTOR_2 2
#define MOTOR_3 3
#define MOTOR_4 4

// #define FILTER_SIZE 1500  // Adjust this value as needed
#define FILTER_SIZE 90  // Adjust this value as needed

// Motor PWMs
int Motor1 = 0;
int Motor2 = 0;
int Motor3 = 0;
int Motor4 = 0;

struct Signal {
  int height;
  int frwrvr;
  int rgtlft;
  int yaw;
};

struct Kalman {
  float Q_angle; // Process noise variance for the accelerometer
  float Q_bias;  // Process noise variance for the gyro bias
  float R_measure; // Measurement noise variance
  float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
  float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
  float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
};

Kalman kalmanX, kalmanY;
float kalmanAngleRoll = 0, kalmanAnglePitch = 0;
float kalman_uncertainity_roll = 2*2;
float kalman_uncertainity_pitch = 2*2;
float kalman_1d_output[] = {0, 0};

// Filter parameters (adjust cutoff frequency as needed)
int lowpass_cutoff_hz = 10.0;  // Adjust this value for desired filtering strength
float lowpass_alpha = lowpass_cutoff_hz / (lowpass_cutoff_hz + 250.0);
float lowpass_roll = 0;
float lowpass_pitch = 0;

Signal rx_data;

// Complementary filter constant
float alpha = 0.98;
const float dt = 0.002;
signed short Ax, Ay, Az, Tmp, Gx, Gy, Gz;
float Ax_cal, Ay_cal, Az_cal, Tmp_cal, Gx_cal, Gy_cal, Gz_cal;
float Ax_offset, Ay_offset, Az_offset, Tmp_offset, Gx_offset, Gy_offset, Gz_offset;

float roll, pitch, rate_yaw;
float roll_offset, pitch_offset, rate_yaw_offset;

float roll_setpoint = 0;
float pitch_setpoint = 0;
float rate_yaw_setpoint = 0;

float roll_error, pitch_error, rate_yaw_error;

float roll_integral = 0;
float pitch_integral = 0;
float rate_yaw_integral = 0;

float roll_derivative = 0;
float pitch_derivative = 0;
float rate_yaw_derivative = 0;

float roll_output, pitch_output, rate_yaw_output;

float roll_filter_buffer[FILTER_SIZE] = {0};
float pitch_filter_buffer[FILTER_SIZE] = {0};
int filter_index = 0;

float Kp_roll = 420.0;
float Ki_roll = 16.5;
// float Kd_roll = 0.5;

float Kp_pitch = 420.0;
float Ki_pitch = 16.5;
// float Kd_pitch = 0.5;

float Kp_rate_yaw = 0.0;
float Ki_rate_yaw = 0.0;
// float Kd_yaw = 0.5;

template <int order> // order is 1 or 2
class LowPass
{
  private:
    float a[order];
    float b[order+1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order+1]; // Raw values
    float y[order+1]; // Filtered values

  public:  
    LowPass(float f0, float fs, bool adaptive){
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.
      
      omega0 = 6.28318530718*f0;
      dt = 1.0/fs;
      adapt = adaptive;
      tn1 = -dt;
      for(int k = 0; k < order+1; k++){
        x[k] = 0;
        y[k] = 0;        
      }
      setCoef();
    }

    void setCoef(){
      if(adapt){
        float t = micros()/1.0e6;
        dt = t - tn1;
        tn1 = t;
      }
      
      float alpha = omega0*dt;
      if(order==1){
        a[0] = -(alpha - 2.0)/(alpha+2.0);
        b[0] = alpha/(alpha+2.0);
        b[1] = alpha/(alpha+2.0);        
      }
      if(order==2){
        float alphaSq = alpha*alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq*beta[0] + 2*alpha*beta[1] + 4*beta[2];
        b[0] = alphaSq/D;
        b[1] = 2*b[0];
        b[2] = b[0];
        a[0] = -(2*alphaSq*beta[0] - 8*beta[2])/D;
        a[1] = -(beta[0]*alphaSq - 2*beta[1]*alpha + 4*beta[2])/D;      
      }
    }

    float filt(float xn){
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if(adapt){
        setCoef(); // Update coefficients if necessary      
      }
      y[0] = 0;
      x[0] = xn;
      // Compute the filtered values
      for(int k = 0; k < order; k++){
        y[0] += a[k]*y[k+1] + b[k]*x[k];
      }
      y[0] += b[order]*x[order];

      // Save the historical values
      for(int k = order; k > 0; k--){
        y[k] = y[k-1];
        x[k] = x[k-1];
      }
  
      // Return the filtered value    
      return y[0];
    }
};



// Filter instance
LowPass<2> lp_roll(1,3.33e2,true);
LowPass<2> lp_pitch(1,3.33e2,true);


void setup(){
    digitalWrite(blueLed, LOW);
    delay(1000);
    Serial.begin(115200);
    // Wire.begin();
    // Wire.setClock(400000);
    Wire.beginTransmission(MPU6050);
    Wire.write(SMPLRT_DIV);
    Wire.write(0x07);
    delayMicroseconds(100);
    Wire.endTransmission(true);
    delayMicroseconds(100);
    Wire.beginTransmission(MPU6050);
    Wire.write(PWR_MGMT_1);    
    Wire.write(0x00);
    Wire.endTransmission(true);
    delayMicroseconds(100);
    Wire.beginTransmission(MPU6050);
    Wire.write(CONFIG);
    Wire.write(0x00);
    Wire.endTransmission(true);
    delayMicroseconds(100);
    Wire.beginTransmission(MPU6050);
    Wire.write(GYRO_CONFIG);
    Wire.write(0x08);
    Wire.endTransmission(true);
    delayMicroseconds(100);
    Wire.beginTransmission(MPU6050);
    Wire.write(ACCEL_CONFIG);
    Wire.write(0x10);
    Wire.endTransmission(true);
    delayMicroseconds(100);

    receiver.begin();
    if (!receiver.begin()){
        Serial.println("Receiver failed to start");
        digitalWrite(redLed, LOW);
        digitalWrite(blueLed, HIGH);
    }
    receiver.openReadingPipe(0, address);
    receiver.setPALevel(RF24_PA_HIGH);
    receiver.startListening();

    int j,k;
    digitalWrite(redLed,LOW);
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
            digitalWrite(redLed,LOW);
        }
        PWM.PWMC_Set_Period(i, SERVO_PERIOD);
        PWM.PWMC_Set_OnOffTime(i, 61000);
        PWM.PWMC_init(i);
        PWM.PWMC_Enable();
        delay(500);

    }
    digitalWrite(redLed,HIGH);
    delay(1000);

    for (int i = 0; i < 2000; i++){
        digitalWrite(greenLed, LOW);
        Wire.beginTransmission(MPU6050);
        Wire.write(ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU6050, 14, true);
        Ax = ((Wire.read() << 8) | Wire.read());
        Ay = ((Wire.read() << 8) | Wire.read());
        Az = ((Wire.read() << 8) | Wire.read());
        Tmp = ((Wire.read() << 8) | Wire.read());
        Gx = ((Wire.read() << 8) | Wire.read());
        Gy = ((Wire.read() << 8) | Wire.read());
        Gz = ((Wire.read() << 8) | Wire.read());
        Ax_cal = (float)Ax/4096.0;
        Ay_cal = (float)Ay/4096.0;
        Az_cal = (float)Az/4096.0 ;
        Tmp_cal = (float)Tmp/340.0 + 36.53;
        Gx_cal = (float)Gx/65.5;
        Gy_cal = (float)Gy/65.5;
        Gz_cal = (float)Gz/65.5;
        Ax_offset += Ax_cal;
        Ay_offset += Ay_cal;
        Az_offset += Az_cal;
        Tmp_offset += Tmp_cal;
        Gx_offset += Gx_cal;
        Gy_offset += Gy_cal;
        Gz_offset += Gz_cal;
        roll_offset +=  atan(Ay_cal/sqrt(Ax_cal*Ax_cal+Az_cal*Az_cal))*(180/3.142);
        pitch_offset += -atan(Ax_cal/sqrt(Ay_cal*Ay_cal+Az_cal*Az_cal))*(180/3.142);
        delay(5);
    }
    Ax_offset /= 2000;
    Ay_offset /= 2000;
    Az_offset /= 2000;
    Tmp_offset /= 2000;
    Gx_offset /= 2000;
    Gy_offset /= 2000;
    Gz_offset /= 2000;
    roll_offset /= 2000;
    pitch_offset /= 2000;
    rate_yaw_offset /= 2000;
    Serial.println("Calibration done");
    Serial.print("Ax_offset: ");
    Serial.print(Ax_offset);
    Serial.print(" Ay_offset: ");
    Serial.print(Ay_offset);
    Serial.print(" Az_offset: ");
    Serial.print(Az_offset);
    Serial.print(" Tmp_offset: ");
    Serial.print(Tmp_offset);
    Serial.print(" Gx_offset: ");
    Serial.print(Gx_offset);
    Serial.print(" Gy_offset: ");
    Serial.print(Gy_offset);
    Serial.print(" Gz_offset: ");
    Serial.print(Gz_offset);
    Serial.print(" Roll_offset: ");
    Serial.print(roll_offset);
    Serial.print(" Pitch_offset: ");
    Serial.println(pitch_offset);
    digitalWrite(greenLed, HIGH);

    // Initialize Kalman filters
    initKalman(kalmanX);
    initKalman(kalmanY);

    for (int l = 0; l < 5; l++){
        PWM.PWMC_Set_Period(l, SERVO_PERIOD);
        PWM.PWMC_Set_OnOffTime(l, 0);
        PWM.PWMC_init(l);
        PWM.PWMC_Enable();
        delay(100);
        digitalWrite(redLed, LOW);
    }
    delay(1000);

    digitalWrite(redLed,HIGH);
    delay(1000);
    digitalWrite(blueLed,HIGH);
    delay(1000);
}

unsigned long a1 = 0;
unsigned long a2 = 0;
unsigned long a3 = 0;

void loop() {
    a1 = micros();
    if (receiver.available()) {
        digitalWrite(redLed,HIGH);
        digitalWrite(greenLed,LOW);
        digitalWrite(blueLed,HIGH);
        receiver.read(&rx_data, sizeof(rx_data));

    } else {
        digitalWrite(redLed,LOW);
        digitalWrite(greenLed,HIGH);
        digitalWrite(blueLed,HIGH);
        rx_data.height = rx_data.height;
        rx_data.frwrvr = rx_data.frwrvr;
        rx_data.rgtlft = rx_data.rgtlft;
        rx_data.yaw = rx_data.yaw;
    }


    // a1 = micros();
    Wire.beginTransmission(MPU6050);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050, 14, true);
    Ax = ((Wire.read() << 8) | Wire.read());
    Ay = ((Wire.read() << 8) | Wire.read());
    Az = ((Wire.read() << 8) | Wire.read());
    Tmp = ((Wire.read() << 8) | Wire.read());
    Gx = ((Wire.read() << 8) | Wire.read());
    Gy = ((Wire.read() << 8) | Wire.read());
    Gz = ((Wire.read() << 8) | Wire.read());
    Wire.endTransmission(true);

    Ax_cal = (float)Ax/4096.0 ;
    Ay_cal = (float)Ay/4096.0 ;
    Az_cal = (float)Az/4096.0 ;
    Tmp_cal = (float)Tmp/340.0 + 36.53;
    Gx_cal = (float)Gx/65.5 - Gx_offset;
    Gy_cal = (float)Gy/65.5 - Gy_offset;
    Gz_cal = (float)Gz/65.5 - Gz_offset;

    float current_roll =  atan(Ay_cal/sqrt(Ax_cal*Ax_cal+Az_cal*Az_cal))*(180/3.142) - roll_offset;
    float current_pitch= -atan(Ax_cal/sqrt(Ay_cal*Ay_cal+Az_cal*Az_cal))*(180/3.142) - pitch_offset;

  Serial.print("Current Roll: ");
  Serial.print(current_roll);
  Serial.print(", Current Pitch: ");
  Serial.print(current_pitch);

    // float com_roll = 0.98*(roll + Gx_cal*dt) + (1-alpha)*current_roll;
    // float com_pitch = 0.98*(pitch + Gy_cal*dt) + (1-alpha)*current_pitch;
    // // a2 = micros();
    // float kal_roll = kalmanFilter(kalmanX, current_roll, Gx_cal, dt);
    // float kal_pitch = kalmanFilter(kalmanY, current_pitch, Gy_cal, dt);

    // kalman_1d(kalmanAngleRoll, kalmanAnglePitch, Gx_cal, current_roll);
    // kalmanAngleRoll = kalman_1d_output[0];
    // kalman_uncertainity_roll = kalman_1d_output[1];

    // kalman_1d(kalmanAnglePitch, kalmanAnglePitch, Gy_cal, current_pitch);
    // kalmanAnglePitch = kalman_1d_output[0];
    // kalman_uncertainity_pitch = kalman_1d_output[1];

    // roll = com_roll;
    // pitch = com_pitch;
    // lowpass_roll = lowpass_alpha*(current_roll + Gx_cal);
    // lowpass_pitch = lowpass_alpha*(current_pitch + Gy_cal);
    // roll = lowpass_roll;
    // Compute the filtered signal
    // Serial.print("Roll: ");
    // Serial.print(current_roll);
    // Serial.print(", Pitch: ");
    // Serial.print(current_pitch);
    float roll_lp = lp_roll.filt(current_roll);
    roll = roll_lp;
    float pitch_lp = lp_pitch.filt(current_pitch);
    pitch = pitch_lp;

    // roll = (1/2.05)*(1.05*kalmanAngleRoll + kal_roll);
    // pitch = (1/2.05)*(1.05*kalmanAnglePitch + kal_pitch);
    // roll = kalmanAngleRoll;
    // pitch = kalmanAnglePitch;
    // roll = 0.6*com_roll + 0.2*kalmanAngleRoll + 0.2*kal_roll;
    // pitch = 0.6*com_pitch + 0.2*kalmanAnglePitch + 0.2*kal_pitch;
    // roll = current_roll;
    // pitch = current_pitch;

    // Update filter buffer
    // if (abs(roll)<1.0)roll=0;
    // if (abs(pitch)<1.0)pitch=0;

    roll_filter_buffer[filter_index] = roll;
    pitch_filter_buffer[filter_index] = pitch;

    // Calculate moving average
    float roll_filtered = 0.0;
    float pitch_filtered = 0.0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        roll_filtered += roll_filter_buffer[i];
        pitch_filtered += pitch_filter_buffer[i];
    }
    roll_filtered /= FILTER_SIZE;
    pitch_filtered /= FILTER_SIZE;

    // Update filter index (circular buffer)
    filter_index++;
    if (filter_index >= FILTER_SIZE) {
        filter_index = 0;
    }


    // roll = roll_filtered;
    // pitch = pitch_filtered;
    rate_yaw = Gz_cal;

    // Add dead zone
    // if (abs(roll) < 0.5) roll = 0;

    roll = roll_filtered;
    pitch = pitch_filtered;

  if(abs(roll)<0.6)roll=0;
  if(abs(pitch)<0.6)pitch=0;

    // kalman_1d(kalmanAngleRoll, kalmanAnglePitch, Gx_cal, current_roll);
    // kalmanAngleRoll = kalman_1d_output[0];
    // kalman_uncertainity_roll = kalman_1d_output[1];

    // kalman_1d(kalmanAnglePitch, kalmanAnglePitch, Gy_cal, current_pitch);
    // kalmanAnglePitch = kalman_1d_output[0];
    // kalman_uncertainity_pitch = kalman_1d_output[1];
    
    // roll = kalmanAngleRoll;
    // pitch = kalmanAnglePitch;

    // Serial.print(", Roll_filtered: ");
    // Serial.print(roll);
    // Serial.print(", Pitch filtered: ");
    // Serial.println(pitch);
    // rate_yaw = Gz_cal;

    // Add dead zone
    // if (abs(roll) < 1.5) roll = 0;
    // if (abs(pitch) < 1.5) pitch = 0;
    if (abs(rate_yaw) < 0.2) rate_yaw = 0;

    roll_error = roll_setpoint - roll;
    pitch_error = pitch_setpoint - pitch;
    rate_yaw_error = rate_yaw_setpoint - rate_yaw;

    // PI Controller for Roll
    if (rx_data.height > 102000)roll_integral += roll_error;
    if (abs(roll_integral) > 900)roll_integral = 900*(roll_integral/abs(roll_integral));
    roll_output = Kp_roll * roll_error + Ki_roll * roll_integral;

    // PI Controller for Pitch
    if (rx_data.height > 102000)pitch_integral += pitch_error;
    if (abs(pitch_integral) > 900)pitch_integral = 900*(pitch_integral/abs(pitch_integral));
    pitch_output = Kp_pitch * pitch_error + Ki_pitch * pitch_integral;

    // PI Controller for Rate Yaw
    // if (rx_data.height > 100000) 
    rate_yaw_integral += rate_yaw_error;
    if (abs(rate_yaw_integral) > 900)rate_yaw_integral = 900*(rate_yaw_integral/abs(rate_yaw_integral));
    rate_yaw_output = Kp_rate_yaw * rate_yaw_error + Ki_rate_yaw * rate_yaw_integral;

    // Motor 1 = Height - Pitch + Rate Yaw - Roll
    // Motor 2 = Height + Pitch - Rate Yaw - Roll
    // Motor 3 = Height + Pitch + Rate Yaw + Roll
    // Motor 4 = Height - Pitch - Rate Yaw + Roll

    // Motor 1
    Motor1 = rx_data.height * 0.920 - pitch_output + rate_yaw_output - roll_output;
    if (Motor1 > 127000)Motor1 = 127000;
    // Motor1 = Motor1;
    if (Motor1 < 0)Motor1 = 0;
    PWM.PWMC_Set_Period(1, SERVO_PERIOD);
    PWM.PWMC_Set_OnOffTime(1, Motor1);
    PWM.PWMC_init(1);
    PWM.PWMC_Enable();
    delayMicroseconds(100);

    // Motor 2
    Motor2 = rx_data.height * 1.030 + pitch_output - rate_yaw_output - roll_output;
    if (Motor2 > 127000)Motor2 = 127000;
    // Motor2 = Motor2 ;
    if (Motor2 < 0)Motor2 = 0;
    PWM.PWMC_Set_Period(2, SERVO_PERIOD);
    PWM.PWMC_Set_OnOffTime(2, Motor2);
    PWM.PWMC_init(2);
    PWM.PWMC_Enable();
    delayMicroseconds(100);

    // Motor 3
    Motor3 = rx_data.height * 0.920 + pitch_output + rate_yaw_output + roll_output;
    if (Motor3 > 127000)Motor3 = 127000;
    // Motor3 = Motor3 ;
    if (Motor3 < 0)Motor3 = 0;
    PWM.PWMC_Set_Period(3, SERVO_PERIOD);
    PWM.PWMC_Set_OnOffTime(3, Motor3);
    PWM.PWMC_init(3);
    PWM.PWMC_Enable();
    delayMicroseconds(100);

    // Motor 4
    Motor4 = rx_data.height * 1.0090 - pitch_output - rate_yaw_output+ roll_output;
    if (Motor4 > 127000)Motor4 = 127000;
    // Motor4 = Motor4 ;
    if (Motor4 < 0)Motor4 = 0;
    PWM.PWMC_Set_Period(4, SERVO_PERIOD);
    PWM.PWMC_Set_OnOffTime(4, Motor4);
    PWM.PWMC_init(4);
    PWM.PWMC_Enable();
    delayMicroseconds(100);

    debugPrint();
    a2 = micros();
    if (a2-a1 < 3000) delayMicroseconds(3000-(a2-a1));
    // a3 = micros();
    // Serial.println(a2-a1);
}


void debugPrint() {
    // Serial.print("Ax: ");
    // Serial.print((Ax_cal)*9.8);
    // Serial.print(" Ay: ");
    // Serial.print((Ay_cal)*9.8);
    // Serial.print(" Az: ");
    // Serial.print(Az_cal*9.8);
    // Serial.print(" Tmp: ");
    // Serial.print(Tmp_cal);
    // Serial.print(" Gx: ");
    // Serial.print(Gx_cal);
    // Serial.print(" Gy: ");
    // Serial.print(Gy_cal);
    // Serial.print(" Gz: ");
    // Serial.print(Gz_cal);
    // Serial.print(" Height: ");
    // Serial.print(rx_data.height);
    Serial.print(", Roll: ");
    Serial.print(roll);
    Serial.print(", Pitch: ");
    Serial.print(pitch);
    // Serial.print(" Rate Yaw: ");
    // Serial.print(rate_yaw);
    // Serial.print(" Motor1: ");
    // Serial.print(Motor1);
    // Serial.print(" Motor2: ");
    // Serial.print(Motor2);
    // Serial.print(" Motor3: ");
    // Serial.print(Motor3);
    // Serial.print(" Motor4: ");
    // Serial.print(Motor4);
    Serial.println();
}

void initKalman(Kalman &kalman) {
  kalman.Q_angle = 0.007;
  kalman.Q_bias = 0.007;
  kalman.R_measure = 0.05;
  kalman.angle = 0;
  kalman.bias = 0;
  kalman.P[0][0] = 0;
  kalman.P[0][1] = 0;
  kalman.P[1][0] = 0;
  kalman.P[1][1] = 0;
}


float kalmanFilter(Kalman &kalman, float newAngle, float newRate, float dt) {
  // Predict
  float rate = newRate - kalman.bias;
  kalman.angle += dt * rate;

  kalman.P[0][0] += dt * (dt * kalman.P[1][1] - kalman.P[0][1] - kalman.P[1][0] + kalman.Q_angle);
  kalman.P[0][1] -= dt * kalman.P[1][1];
  kalman.P[1][0] -= dt * kalman.P[1][1];
  kalman.P[1][1] += kalman.Q_bias * dt;

  // Update
  float y = newAngle - kalman.angle;
  float S = kalman.P[0][0] + kalman.R_measure;
  float K[2];
  K[0] = kalman.P[0][0] / S;
  K[1] = kalman.P[1][0] / S;

  kalman.angle += K[0] * y;
  kalman.bias += K[1] * y;

  float P00_temp = kalman.P[0][0];
  float P01_temp = kalman.P[0][1];

  kalman.P[0][0] -= K[0] * P00_temp;
  kalman.P[0][1] -= K[0] * P01_temp;
  kalman.P[1][0] -= K[1] * P00_temp;
  kalman.P[1][1] -= K[1] * P01_temp;

  return kalman.angle;
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  kalman_1d_output[0]=KalmanState; 
  kalman_1d_output[1]=KalmanUncertainty;
}
