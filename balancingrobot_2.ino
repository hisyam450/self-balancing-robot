#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PID_v1.h>
#include <LMotorController.h>

#define MIN_ABS_SPEED 30

Adafruit_MPU6050 mpu;

// PID
double originalSetpoint = 172.50;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

// Adjust these values to fit your own design
double Kp = 60;   
double Kd = 2.2;
double Ki = 270;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.6;
double motorSpeedFactorRight = 0.5;

// MOTOR CONTROLLER
#define ENA 18
#define IN1 5
#define IN2 17

#define ENB 2
#define IN3 16
#define IN4 4

LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }

    // Set the accelerometer range
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    // Set the gyroscope range
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    // Set the filter bandwidth
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate the angle from the accelerometer data
    float angle = atan2(a.acceleration.y, a.acceleration.x) * 180 / PI;

    // Update input for PID
    input = angle + 180; // Adjust angle to be in the range of 0-360

    // Perform PID calculations and output to motors
    pid.Compute();
    motorController.move(output, MIN_ABS_SPEED);

    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print(" Output: ");
    Serial.println(output);

    delay(10);
}