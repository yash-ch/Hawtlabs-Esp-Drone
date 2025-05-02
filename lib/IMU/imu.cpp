#include "imu.h"
#include "elrs.h"
#include "config.h"

// PID variables
float RateRoll = 0, RatePitch = 0, RateYaw = 0;
float rateCalibrationRoll = 0, rateCalibrationPitch = 0, rateCalibrationYaw = 0;
int RateCalibrationNumber = 0;
volatile uint32_t LoopTimer = 0;
float DesiredRateRoll = 0, DesiredRatePitch = 0, DesiredRateYaw = 0;
float ErrorRateRoll = 0, ErrorRatePitch = 0, ErrorRateYaw = 0;
float InputRoll = 0, InputThrottle = 0, InputPitch = 0, InputYaw = 0;
float PrevErrorRateRoll = 0, PrevErrorRatePitch = 0, PrevErrorRateYaw = 0;
float PrevItermRateRoll = 0, PrevItermRatePitch = 0, PrevItermRateYaw = 0;
float MotorInput1 = 0, MotorInput2 = 0, MotorInput3 = 0, MotorInput4 = 0;

float PIDReturn[] = {0, 0, 0};
float volatile PRateRoll = 0;
float volatile PRatePitch = PRateRoll;
float volatile PRateYaw = 0;
float volatile IRateRoll = 0;
float volatile IRatePitch = IRateRoll;
float volatile IRateYaw = 0;
float volatile DRateRoll = 0;
float volatile DRatePitch = DRateRoll;
float volatile DRateYaw = 0;

float AccX = 0, AccY = 0, AccZ = 0;
float AngleRoll = 0, AnglePitch = 0;

// kalman variables
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = {0, 0};

// cascade controller
float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;
float PAngleRoll = 2;
float PAnglePitch = 0;
float IAngleRoll = 0;
float IAnglePitch = 0;
float DAngleRoll = 0;
float DAnglePitch = 0;

float NewMotorInput1 = 0;
float NewMotorInput2 = 0;
float NewMotorInput3 = 0;
float NewMotorInput4 = 0;

Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;

volatile float trimRoll = 0.0f;
volatile float trimPitch = 0.0f;

void imuPIDTask(void *pv)
{
    pinMode(LED1, OUTPUT);

    Wire.setClock(400000); // Set I2C clock speed
    Wire.begin();
    delay(250);
    Wire.beginTransmission(0x68);
    Wire.write(0x6B); // power management register
    Wire.write(0x00);
    Wire.endTransmission();
    delay(1000);

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    delay(1000);
    mot1.attach(MOTOR1_PIN, 1000, 2000);
    delay(1000);
    mot1.setPeriodHertz(PWM_FREQ);
    delay(100);
    mot2.attach(MOTOR2_PIN, 1000, 2000);
    delay(1000);
    mot2.setPeriodHertz(PWM_FREQ);
    delay(100);
    mot3.attach(MOTOR3_PIN, 1000, 2000);
    delay(1000);
    mot3.setPeriodHertz(PWM_FREQ);
    delay(100);
    mot4.attach(MOTOR4_PIN, 1000, 2000);
    delay(1000);
    mot4.setPeriodHertz(PWM_FREQ);
    delay(100);

    mot1.writeMicroseconds(1000);
    mot2.writeMicroseconds(1000);
    mot3.writeMicroseconds(1000);
    mot4.writeMicroseconds(1000);
    delay(500);

    while (1)
    {
        if (calibration_mode && !imu_ready)
        {
            for (int i = 0; i < 1000; i++)
            {
                gyro_signals();
                rateCalibrationRoll += RateRoll;
                rateCalibrationPitch += RatePitch;
                rateCalibrationYaw += RateYaw;
                delay(1);
            }
            rateCalibrationRoll /= 1000;
            rateCalibrationPitch /= 1000;
            rateCalibrationYaw /= 1000;
            calibration_mode = false;
            imu_ready = true; // Set IMU ready flag after calibration
            gyro_signals();
            Serial.println(RateRoll - rateCalibrationRoll);
            Serial.println(RatePitch - rateCalibrationPitch);
            Serial.println(RateYaw - rateCalibrationYaw);
            Serial.print("angle roll: ");
            Serial.println(AngleRoll);
            Serial.print(" angle pitch: ");
            Serial.println(AnglePitch);

            digitalWrite(LED1, HIGH);
        }
        else if (!imu_ready)
        {
            // blink the LED if the IMU is not ready
            digitalWrite(LED1, HIGH);
            delay(100);
            digitalWrite(LED1, LOW);
            delay(100);
        }

        if (armed)
        {
            gyro_signals();
            RateRoll -= rateCalibrationRoll;
            RatePitch -= rateCalibrationPitch;
            RateYaw -= rateCalibrationYaw;

            kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
            KalmanAngleRoll = Kalman1DOutput[0];
            KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

            kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
            KalmanAnglePitch = Kalman1DOutput[0];
            KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

            DesiredAngleRoll = 0.10 * (rcChannels[0] - 1500) + trimRoll;
            DesiredAnglePitch = 0.10 * (rcChannels[1] - 1500) + trimPitch;
            InputThrottle = rcChannels[2];
            DesiredRateYaw = 0.15 * (rcChannels[3] - 1500);
            ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
            ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;

            pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
            DesiredRateRoll = PIDReturn[0];
            PrevErrorAngleRoll = PIDReturn[1];
            PrevItermAngleRoll = PIDReturn[2];

            pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
            DesiredRatePitch = PIDReturn[0];
            PrevErrorAnglePitch = PIDReturn[1];
            PrevItermAnglePitch = PIDReturn[2];

            ErrorRateRoll = DesiredRateRoll - RateRoll;
            ErrorRatePitch = DesiredRatePitch - RatePitch;
            ErrorRateYaw = DesiredRateYaw - RateYaw;

            pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
            InputRoll = PIDReturn[0];
            PrevErrorRateRoll = PIDReturn[1];
            PrevItermRateRoll = PIDReturn[2];

            pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
            InputPitch = PIDReturn[0];
            PrevErrorRatePitch = PIDReturn[1];
            PrevItermRatePitch = PIDReturn[2];

            pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
            InputYaw = PIDReturn[0];
            PrevErrorRateYaw = PIDReturn[1];
            PrevItermRateYaw = PIDReturn[2];
            if (InputThrottle > 1900)
                InputThrottle = 1900;

            MotorInput1 = (InputThrottle - InputRoll - InputPitch - InputYaw);
            MotorInput2 = (InputThrottle - InputRoll + InputPitch + InputYaw);
            MotorInput3 = (InputThrottle + InputRoll + InputPitch - InputYaw);
            MotorInput4 = (InputThrottle + InputRoll - InputPitch + InputYaw);

            if (MotorInput1 > 2000)
                MotorInput1 = 1999;
            if (MotorInput2 > 2000)
                MotorInput2 = 1999;
            if (MotorInput3 > 2000)
                MotorInput3 = 1999;
            if (MotorInput4 > 2000)
                MotorInput4 = 1999;

            int ThrottleIdle = 1180;
            if (MotorInput1 < ThrottleIdle)
                MotorInput1 = ThrottleIdle;
            if (MotorInput2 < ThrottleIdle)
                MotorInput2 = ThrottleIdle;
            if (MotorInput3 < ThrottleIdle)
                MotorInput3 = ThrottleIdle;
            if (MotorInput4 < ThrottleIdle)
                MotorInput4 = ThrottleIdle;

            int ThrottleCutOff = 1000;
            if (rcChannels[2] < 1050)
            {
                MotorInput1 = ThrottleCutOff;
                MotorInput2 = ThrottleCutOff;
                MotorInput3 = ThrottleCutOff;
                MotorInput4 = ThrottleCutOff;
                reset_pid();
            }
            // write function to map the motor input to 0 to 255
            NewMotorInput1 = map(MotorInput1, 1000, 2000, 0, 255);
            NewMotorInput2 = map(MotorInput2, 1000, 2000, 0, 255);
            NewMotorInput3 = map(MotorInput3, 1000, 2000, 0, 255);
            NewMotorInput4 = map(MotorInput4, 1000, 2000, 0, 255);

            mot1.writeMicroseconds(MotorInput1);
            mot2.writeMicroseconds(MotorInput2);
            mot3.writeMicroseconds(MotorInput3);
            mot4.writeMicroseconds(MotorInput4);

            while (micros() - LoopTimer < (1000000 / PID_RATE))
                ;
            LoopTimer = micros();
        }
        else
        {
            reset_pid();
            mot1.writeMicroseconds(0);
            mot2.writeMicroseconds(0);
            mot3.writeMicroseconds(0);
            mot4.writeMicroseconds(0);
        }
    }
}

void gyro_signals(void)
{
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x8);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    int16_t GyroX = Wire.read() << 8 | Wire.read();
    int16_t GyroY = Wire.read() << 8 | Wire.read();
    int16_t GyroZ = Wire.read() << 8 | Wire.read();
    RateRoll = (float)GyroX / 65.5;
    RatePitch = (float)GyroY / 65.5;
    RateYaw = (float)GyroZ / 65.5;

    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    int16_t AccXLSB = Wire.read() << 8 | Wire.read();
    int16_t AccYLSB = Wire.read() << 8 | Wire.read();
    int16_t AccZLSB = Wire.read() << 8 | Wire.read();

    AccX = (float)AccXLSB / 4096 - 0.01;
    AccY = (float)AccYLSB / 4096 - 0.02;
    AccZ = (float)AccZLSB / 4096 - 0.035;

    // AccX = (float)AccXLSB / 4096;
    // AccY = (float)AccYLSB / 4096;
    // AccZ = (float)AccZLSB / 4096;
    // Serial.print("AccX: ");
    // Serial.print(AccX, 4);
    // Serial.print(" AccY: ");
    // Serial.print(AccY, 4);
    // Serial.print(" AccZ: ");
    // Serial.println(AccZ, 4);

    AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
    AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
}

void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
    float Pterm = P * Error;
    float Iterm = PrevIterm + I * (Error + PrevError) * (1 / PID_RATE) / 2.0f;
    if (Iterm > 400)
        Iterm = 400;
    else if (Iterm < -400)
        Iterm = -400;
    float Dterm = D * (Error - PrevError) / (1 / PID_RATE);
    float PIDOutput = Pterm + Iterm + Dterm;
    if (PIDOutput > 400)
        PIDOutput = 400;
    else if (PIDOutput < -400)
        PIDOutput = -400;
    PIDReturn[0] = PIDOutput;
    PIDReturn[1] = Error;
    PIDReturn[2] = Iterm;
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement)
{
    KalmanState = KalmanState + (1 / PID_RATE) * KalmanInput;
    KalmanUncertainty = KalmanUncertainty + (1 / PID_RATE) * (1 / PID_RATE) * 4 * 4;
    float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0] = KalmanState;
    Kalman1DOutput[1] = KalmanUncertainty;
}

void reset_pid(void)
{
    PrevErrorRateRoll = 0.0f;
    PrevErrorRatePitch = 0.0f;
    PrevErrorRateYaw = 0.0f;
    PrevItermRateRoll = 0.0f;
    PrevItermRatePitch = 0.0f;
    PrevItermRateYaw = 0.0f;
    PrevErrorAngleRoll = 0.0f;
    PrevErrorAnglePitch = 0.0f;
    PrevItermAngleRoll = 0.0f;
    PrevItermAnglePitch = 0.0f;
}