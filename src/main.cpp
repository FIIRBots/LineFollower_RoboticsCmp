#include <Arduino.h>
#include <WiFi.h>
#include "LF_SData.h"
#include "MotorDriver.h"
#include "WebServerHandler.h"
#include "arduino_secrets.h"

WiFiServer server(80);

LF_SData sensorData;

#define CALIBRATION_FLAG false
#define DEBUG_FLAG false
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

double MAX_OUTPUT = 35;

bool robotActive = false;

double BASE_SPEED = 40; 

double KP = 0.0375;
double KI = 0.00;
double KD = 1.0;

const float alpha = 0.9; // Lower = smoother but slower response
double smoothed_error = 0;

double last_derivative = 0;
const float dAlpha = 0.85;

double integral = 0;
double previousError = 0;

double PID(double error) {
    integral += error;

    double raw_derivative = error - previousError;
    last_derivative = dAlpha * raw_derivative + (1 - dAlpha) * last_derivative;

    previousError = error;

    double correction = (KP * error) + (KI * integral) + (KD * last_derivative);
    return correction;
}

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

    if (DEBUG_FLAG) {
        sensorData.getLiveSerialPrint(true);
    }

    int raw_error = sensorData.getLinePosition() - 750;
    smoothed_error = alpha * raw_error + (1 - alpha) * smoothed_error;
    // smoothed_error = raw_error;
    double pid_output = constrain(PID(smoothed_error), -MAX_OUTPUT, MAX_OUTPUT);


    if (DEBUG_FLAG) {
        Serial.print("Sensor Error: ");
        Serial.println(smoothed_error);
        Serial.print("PID: ");
        Serial.println(pid_output);
        Serial.print("active?: ");
        Serial.println(robotActive);
    }

    double left = BASE_SPEED + pid_output; 
    double right = BASE_SPEED - pid_output;

    if (robotActive) {
        motors.setMotor1Speed(left);
        motors.setMotor2Speed(right);
    } else {
        motors.setMotor1Speed(0);
        motors.setMotor2Speed(0);
    }

    if (DEBUG_FLAG) {
        Serial.print("left = ");
        Serial.println(left);

        Serial.print("right = ");
        Serial.println(right);
    }

    delay(10);
}