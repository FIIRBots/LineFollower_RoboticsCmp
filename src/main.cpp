#include <Arduino.h>
#include <WiFi.h>
#include "LF_SData.h"
#include "MotorDriver.h"
#include "WebServerHandler.h"
#include "arduino_secrets.h"

WiFiServer server(80);

LF_SData sensorData;

#define CALIBRATION_FLAG false
#define DEBUG_FLAG true
#define SETUP_WIFI false

/*
    -- sensor pins --
*/
#define S0 D0  // Multiplexer select pin S0
#define S1 D1   // Multiplexer select pin S1
#define S2 D2   // Multiplexer select pin S2
#define S3 D3   // Multiplexer select pin S3
#define SIG A6   // Analog signal pin from multiplexer

#define M1DIR D4  // GPIO
#define M1PWM D5  // PWM 
#define M2DIR D7  // GPIO
#define M2PWM D6  // PWM 
MotorDriver motors(M1DIR, M1PWM, M2DIR, M2PWM);

float KP = 0.1;
float KI = 0.001;
float KD = 2.0;

float previousError = 0;
float integral = 0;

float setBaseSpeed = 50;
float baseSpeed = 0;

int setMaxSpeed = 70;
int maxSpeed = 0;

// Variables to store the last six error values for smoother KI calculation
// int error1 = 0, error2 = 0, error3 = 0, error4 = 0, error5 = 0, error6 = 0;
unsigned long startTime = 0, currentTime = 0, elapsedTime = 0, accelerationTime = 500;


void setup() {
    if (DEBUG_FLAG) {
        Serial.begin(115200);
        while (!Serial) {
            ;
        }

        if (SETUP_WIFI) {
            delay(5000);
        }

        // Create Wi-Fi Access Point
        Serial.println("Starting Access Point...");
    }
    
    if (WiFi.beginAP(SECRET_SSID, SECRET_PASS) != WL_AP_LISTENING) {
        if (DEBUG_FLAG) Serial.println("Failed to start AP");
        while (true); // halt
    }

    if (DEBUG_FLAG) {
        Serial.println("Access Point started");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP()); // This will usually be 192.168.3.1
    }
    server.begin();

    if (SETUP_WIFI) {
        delay(10000);
    }

    sensorData.setupLineSensors(S0, S1, S2, S3, SIG);

    if (CALIBRATION_FLAG) {
        // Calibration mode
        unsigned long startTime = millis();
        unsigned long calibrationTime = 15000;  // 5 seconds

        Serial.begin(115200);
        while(!Serial) {
            ; // wait for serial port to connect. Needed for native USB port only
        }

        delay(5000);
    
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
    handleWeb();

    if (startTime == 0) {
        startTime = millis();
    }

    elapsedTime = millis() - startTime;
    if (elapsedTime <= accelerationTime) {
        baseSpeed = map(elapsedTime, 0, accelerationTime, 0, setBaseSpeed);
        maxSpeed = map(elapsedTime, 0, accelerationTime, 0, setMaxSpeed);
    }

    long linePosition = sensorData.getLinePosition();
    float error = linePosition - 7500;

    if (DEBUG_FLAG) {
        Serial.print("error = ");
        Serial.println(error);
    }

    // Update integral with last six errors for smoother calculation
    // error6 = error5;
    // error5 = error4;
    // error4 = error3;
    // error3 = error2;
    // error2 = error1;
    // error1 = error;
    //integral = error6 + error5 + error4 + error3 + error2 + error1 + error;
    integral = integral + error;
    integral = constrain(integral, -10000, 10000);  // or reset when error is big
    if (abs(error) > 800) 
        integral = 0;

    float derivative = error - previousError;
    previousError = error;

    float correction = (KP * error) + (KI * integral) + (KD * derivative);
    correction = constrain(correction, -150, 150);  // or some reasonable bound


    if (DEBUG_FLAG) {
        Serial.print("correction = ");
        Serial.println(correction);
    }

    // int leftMotorSpeed = constrain(baseSpeed - correction, 0, maxSpeed);
    // int rightMotorSpeed = constrain(baseSpeed + correction, 0, maxSpeed);

    int rawLeft = baseSpeed - correction;
    int rawRight = baseSpeed + correction;

    int minEffectiveSpeed = 20; // motors may not even move below this

    int leftMotorSpeed = constrain(abs(rawLeft), 0, maxSpeed);
    int rightMotorSpeed = constrain(abs(rawRight), 0, maxSpeed);

    if (leftMotorSpeed > 0 && leftMotorSpeed < minEffectiveSpeed) leftMotorSpeed = minEffectiveSpeed;
    if (rightMotorSpeed > 0 && rightMotorSpeed < minEffectiveSpeed) rightMotorSpeed = minEffectiveSpeed;

    if (rawLeft < 0) leftMotorSpeed *= -1;
    if (rawRight < 0) rightMotorSpeed *= -1;


    if (DEBUG_FLAG) {
        Serial.print(" | Left Motor Speed: ");
        Serial.print(leftMotorSpeed);
        Serial.print(" | Right Motor Speed: ");
        Serial.println(rightMotorSpeed);
    }

    if ((linePosition > 2000 && linePosition <= 4000) || (linePosition >= 11000 && linePosition < 13000)) {  // Medium turn
        baseSpeed = constrain(setBaseSpeed * 0.9, 50, setBaseSpeed);
        maxSpeed = constrain(setMaxSpeed * 0.9, 60, setMaxSpeed);
        Serial.print(" | base Speed: ");
        Serial.print(baseSpeed);
        Serial.print(" | max Speed: ");
        Serial.println(maxSpeed);
    } else {                     // Straight line
        baseSpeed = setBaseSpeed;  // Restore full speed
        maxSpeed = setMaxSpeed;
    }

    if (linePosition <= 2500) {
        leftMotorSpeed = 31;
        rightMotorSpeed = -27;
    } else if (linePosition >= 12500) {
        leftMotorSpeed = -27;
        rightMotorSpeed = 31;
    }

    motors.setMotor1Speed(leftMotorSpeed);
    motors.setMotor2Speed(rightMotorSpeed);

    if (DEBUG_FLAG) {
        Serial.print(" | Left Motor Speed: ");
        Serial.print(leftMotorSpeed);
        Serial.print(" | Right Motor Speed: ");
        Serial.println(rightMotorSpeed);

        sensorData.getLiveSerialPrint(true);
    }

    delay(500);
}