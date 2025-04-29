#include <Arduino.h>
#include <DRV8835MotorShield.h>
#include "LF_SData.h"

LF_SData sensorData;

#define CALIBRATION_FLAG false
#define DEBUG_FLAG true

/*
    -- sensor pins --
*/
#define S0 D11   // Multiplexer select pin S0
#define S1 D12   // Multiplexer select pin S1
#define S2 D13   // Multiplexer select pin S2
#define S3 D14   // Multiplexer select pin S3
#define SIG A6   // Analog signal pin from multiplexer

/*
    -- motor pins --
*/
#define M1DIR D4  // PWM pin for M1A
#define M1PWM D8  // GPIO pin for M2A
#define M2DIR D5  // PWM pin for M1B
#define M2PWM D9  // GPIO pin for M2B
DRV8835MotorShield motors(M1DIR, M1PWM, M2DIR, M2PWM);

// PID constants
#define KP 0.22
#define KI 0.008
#define KD 9.2

float previousError = 0;
float integral = 0;

int setBaseSpeed = 0, baseSpeed = 0;
int setMaxSpeed = 0, maxSpeed = 0;

// Variables to store the last six error values for smoother KI calculation
int error1 = 0, error2 = 0, error3 = 0, error4 = 0, error5 = 0, error6 = 0;
unsigned long startTime = 0, currentTime = 0, elapsedTime = 0, accelerationTime = 500;

void setup() {
    Serial.begin(115200);
    sensorData.setupLineSensors(S0, S1, S2, S3, SIG);

    if (CALIBRATION_FLAG) {
        // Calibration mode
        unsigned long startTime = millis();
        unsigned long calibrationTime = 5000;  // 5 seconds
    
        Serial.println("Starting calibration...");
    
        while (millis() - startTime < calibrationTime) {
          sensorData.calibrateSensors(true);
        }

        Serial.println("Calibration complete.");
    } else {
        sensorData.calibrateSensors(false);
    }

}

void loop() {
    if (startTime == 0) {
        startTime = millis();
    }

    // long position = sensorData.getLinePosition();
    // Serial.print("Line Position: ");
    // Serial.println(position);

    // // Print sensor values
    // sensorData.getLiveSerialPrint(true, false);

    elapsedTime = millis() - startTime;
    if (elapsedTime <= accelerationTime) {
        baseSpeed = map(elapsedTime, 0, accelerationTime, 0, setBaseSpeed);
        maxSpeed = map(elapsedTime, 0, accelerationTime, 0, setMaxSpeed);
    }

    long linePosition = sensorData.getLinePosition();
    float error = linePosition - 7500;

    // Update integral with last six errors for smoother calculation
    // error6 = error5;
    // error5 = error4;
    // error4 = error3;
    // error3 = error2;
    // error2 = error1;
    // error1 = error;
    //integral = error6 + error5 + error4 + error3 + error2 + error1 + error;
    integral = integral + error;

    float derivative = error - previousError;
    previousError = error;

    float correction = (KP * error) + (KI * integral) + (KD * derivative);

    int leftMotorSpeed = constrain(baseSpeed - correction, 0, maxSpeed);
    int rightMotorSpeed = constrain(baseSpeed + correction, 0, maxSpeed);

    if (DEBUG_FLAG) {
        // Debug mode: print sensor data
        Serial.print("Line Position: ");
        Serial.println(linePosition);
    }

    if ((linePosition > 2000 && linePosition <= 4000) || (linePosition >= 11000 && linePosition < 13000)) {  // Medium turn
        baseSpeed = constrain(setBaseSpeed * 0.9, 50, setBaseSpeed);
        maxSpeed = constrain(setMaxSpeed * 0.9, 60, setMaxSpeed);
    } else {                     // Straight line
        baseSpeed = setBaseSpeed;  // Restore full speed
        maxSpeed = setMaxSpeed;
    }

    if (linePosition <= 2500) {
        // xmotion.MotorControl(80, -70);
        // Serial.print("left = ");
        // Serial.println(left);

        // Serial.print("right = ");
        // Serial.println(right);

        motors.setM1Speed(80);
        motors.setM2Speed(-70);
    } else if (linePosition >= 12500) {
        // xmotion.MotorControl(-70, 80);
        motors.setM1Speed(-70);
        motors.setM2Speed(80);
    } else {
        // xmotion.MotorControl(map(leftMotorSpeed, 0, 100, 0, 255), map(rightMotorSpeed, 0, 100, 0, 255));
        int left = map(leftMotorSpeed, 0, 100, 0, 255);
        int right = map(rightMotorSpeed, 0, 100, 0, 255);

        motors.setM1Speed(left);
        motors.setM2Speed(right);
    }

    delay(5);
}