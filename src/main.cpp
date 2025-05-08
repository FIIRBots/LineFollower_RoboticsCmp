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
#define SLOW_MODE false

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

float KP = 0.22;
float KI = 0.008;
float KD = 9.2;

float previousError = 0;
float integral = 0;

float setBaseSpeed = 0, baseSpeed = 0;
int setMaxSpeed = 0, maxSpeed = 0;

// Variables to store the last six error values for smoother KI calculation
int error1 = 0, error2 = 0, error3 = 0, error4 = 0, error5 = 0, error6 = 0;
unsigned long startTime = 0, currentTime = 0, elapsedTime = 0, accelerationTime = 500;


void setup() {
    // Create Wi-Fi Access Point
    Serial.println("Starting Access Point...");
    if (WiFi.beginAP(SECRET_SSID, SECRET_PASS) != WL_AP_LISTENING) {
        Serial.println("Failed to start AP");
        while (true); // halt
    }

    
    Serial.println("Access Point started");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());  // This will usually be 192.168.3.1
    server.begin();

    sensorData.setupLineSensors(S0, S1, S2, S3, SIG);

    // Determine mode based on dip switch settings
    if (SLOW_MODE) {
        // Safe Run mode
        setBaseSpeed = 50;
        setMaxSpeed = 70;
    } else {
        setBaseSpeed = 70;
        setMaxSpeed = 90;
    }

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

    Serial.print("correction = ");
    Serial.println(correction);

    int leftMotorSpeed = constrain(baseSpeed - correction, 0, maxSpeed);
    int rightMotorSpeed = constrain(baseSpeed + correction, 0, maxSpeed);

    if ((linePosition > 2000 && linePosition <= 4000) || (linePosition >= 11000 && linePosition < 13000)) {  // Medium turn
        baseSpeed = constrain(setBaseSpeed * 0.9, 50, setBaseSpeed);
        maxSpeed = constrain(setMaxSpeed * 0.9, 60, setMaxSpeed);
    } else {                     // Straight line
        baseSpeed = setBaseSpeed;  // Restore full speed
        maxSpeed = setMaxSpeed;
    }

    int left = 0;
    int right = 0;

    if (linePosition <= 2500) {
        // 80% of 400, 70% of 400
        left = 31;
        right = -27;

        motors.setMotor1Speed(left);
        motors.setMotor2Speed(right);
    } else if (linePosition >= 12500) {
        // 70% of 400, 80% of 400
        left = -27;
        right = 31;

        motors.setMotor1Speed(left);
        motors.setMotor2Speed(right);
    } else {
        // xmotion.MotorControl(map(leftMotorSpeed, 0, 100, 0, 255), map(rightMotorSpeed, 0, 100, 0, 255));
        left = leftMotorSpeed;
        right = rightMotorSpeed;

        motors.setMotor1Speed(left);
        motors.setMotor2Speed(right);
    }

    delay(5);
}