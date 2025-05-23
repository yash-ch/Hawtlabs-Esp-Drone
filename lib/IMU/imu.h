#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

#define LED1 13
// #define LED2 14
// #define LED3 21

#define MOTOR1_PIN 11
#define MOTOR2_PIN 12
#define MOTOR3_PIN 35
#define MOTOR4_PIN 36
#define PWM_CHANNEL 0
#define PWM_FREQ 500
#define PWM_RESOLUTION 8 
#define PID_RATE 250 // PID loop rate in Hz

extern float RateRoll, RatePitch, RateYaw;
extern float rateCalibrationRoll, rateCalibrationPitch, rateCalibrationYaw;
extern int RateCalibrationNumber;
extern volatile uint32_t LoopTimer;

// all the PID variables
extern float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
extern float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
extern float InputRoll, InputThrottle, InputPitch, InputYaw;
extern float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
extern float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
extern float PIDReturn[];
extern float volatile PRateRoll;
extern float volatile PRatePitch;
extern float volatile PRateYaw;
extern float volatile IRateRoll;
extern float volatile IRatePitch;
extern float volatile IRateYaw;
extern float volatile DRateRoll;
extern float volatile DRatePitch;
extern float volatile DRateYaw;

extern float AccX, AccY, AccZ;
extern float AngleRoll, AnglePitch;

// kalman variables
extern float KalmanAngleRoll, KalmanUncertaintyAngleRoll;
extern float KalmanAnglePitch, KalmanUncertaintyAnglePitch;
extern float Kalman1DOutput[];

extern float DesiredAngleRoll, DesiredAnglePitch;
extern float ErrorAngleRoll, ErrorAnglePitch;
extern float PrevErrorAngleRoll, PrevErrorAnglePitch;
extern float PrevItermAngleRoll, PrevItermAnglePitch;
extern float PAngleRoll; 
extern float PAnglePitch;
extern float IAngleRoll; 
extern float IAnglePitch;
extern float DAngleRoll; 
extern float DAnglePitch;

extern volatile float trimRoll;   // in degrees
extern volatile float trimPitch;  // in degrees

// motor outputs
extern float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

extern Servo mot1;
extern Servo mot2;
extern Servo mot3;
extern Servo mot4;

void gyro_signals(void);

void pid_equation(float Error, float P, float I, float D,
                  float PrevError, float PrevIterm);

void imuPIDTask(void *pv);

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement);
void reset_pid(void);