/*
    * Copyright (C) 2025 by Dan-Dominic Staicu
    * 331CA UNSTPB ACS CTI
*/
#include <Arduino.h>
#include <IRremote.h>

#include "LF_SData.h"
#include "MotorDriver.h"
#include "arduino_secrets.h"

#define CALIBRATION_FLAG false

LF_SData sensorData;

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

#define IR_PIN D8

unsigned long startTime = 0;
unsigned long accelerationTime = 5000; // 5 seconds

bool robotActive = false;

double MAX_OUTPUT = 80;
double setBaseSpeed = 90;


double KP = 0.0018; // todo maybe a bit lower
double KI = 0.000002; // todo play
double KD = 0.60; //todo increase 

const float alpha = 0.88; // Lower = smoother but slower response
double smoothed_error = 0;

double last_derivative = 0;
const float dAlpha = 0.90;

double integral = 0;
double previousError = 0;

double BASE_SPEED = 0;

void start_stop() {
    // IR handling
    if (IrReceiver.decode()) {
        uint8_t cmd = IrReceiver.decodedIRData.command;

        if (cmd == 0x45) {
            robotActive = true;
            Serial.print("Robot started: ");
            Serial.println(cmd, HEX);
        } else if (cmd == 0x46) {
            robotActive = false;
            Serial.print("Robot stopped: ");
            Serial.println(cmd, HEX);
        } else if (cmd == 0x47) {
            robotActive = true;
            Serial.print("Drag remote signal: ");
            Serial.println(cmd, HEX);
        } else {
            Serial.print("Unknown command: ");
            Serial.println(cmd, HEX);
        }

        IrReceiver.resume(); // ready to receive the next command
    }
}

double PID(double error) {
    integral += error;

    double raw_derivative = error - previousError;
    last_derivative = dAlpha * raw_derivative + (1 - dAlpha) * last_derivative;
    // last_derivative = raw_derivative;

    previousError = error;

    double correction = (KP * error) + (KI * integral) + (KD * last_derivative);
    return correction;
}

void setup() {
    // Serial.begin(115200);
    motors.begin();

    sensorData.setupLineSensors(S0, S1, S2, S3, SIG);

    if (CALIBRATION_FLAG) {
        // Calibration mode
        unsigned long startTime = millis();
        unsigned long calibrationTime = 15000;  // 15 seconds

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

    IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK); // Init IR receiver
}

void loop() {
    start_stop();

    if (startTime == 0) {
        startTime = millis();
    }

    unsigned long elapsedTime = millis() - startTime;
    if (elapsedTime <= accelerationTime) {
        BASE_SPEED = map(elapsedTime, 0, accelerationTime, 0, setBaseSpeed);
    }

    // if (true) {
    //     sensorData.getLiveSerialPrint(true);
    // }

    int line_value = sensorData.getLinePosition();
    int raw_error = line_value - 7500;
    smoothed_error = alpha * raw_error + (1 - alpha) * smoothed_error;
    // smoothed_error = raw_error;
    double pid_output = constrain(PID(smoothed_error), -MAX_OUTPUT, MAX_OUTPUT);


    // if (true) {
    //     Serial.print("Sensor Error: ");
    //     Serial.println(smoothed_error);
    //     Serial.print("PID: ");
    //     Serial.println(pid_output);
    //     Serial.print("active?: ");
    //     Serial.println(robotActive);
    // }

    double left = BASE_SPEED + pid_output;
    double right = BASE_SPEED - pid_output;
    // double left = constrain(BASE_SPEED + pid_output, -maxSpeed, maxSpeed); // constrain(baseSpeed + correction, 0, maxSpeed);
    // double right = constrain(BASE_SPEED - pid_output, -maxSpeed, maxSpeed;


    if ((line_value > 2000 && smoothed_error <= 4000) || (smoothed_error >= 11000 && smoothed_error < 13000)) {  // Medium turn
        BASE_SPEED = setBaseSpeed * 0.9; // maybe 0.85
    } else {                     // Straight line
        BASE_SPEED = setBaseSpeed;  // Restore full speed
    }

    // unsigned long first_line_time = 1000; // 1 second ADJUST AS NEEDED

    if (robotActive) {
        // FOR DRAG race
        // if (elapsedTime < first_line_time) {
        //     if (line_value <= 2500) {
        //         left = -50;
        //         right = -50;

        //         // alternativa
        //         // motors.brakeAll();

        //     } else if (line_value >= 12500) {
        //         left = -50;
        //         right = -50;

        //         // alternativa
        //         // motors.brakeAll();

        //     } else {
        //         motors.setMotor1Speed(left);
        //         motors.setMotor2Speed(right);
        //     }
        // } else {
        //     motors.setMotor1Speed(left);
        //     motors.setMotor2Speed(right);
        // }


        if (line_value <= 1500) {
            left = -75;
            right = 90;
        } else if (line_value >= 14500) {
            left = 90;
            right = -75;
        } else if (line_value <= 2500) {
            left = -45;
            right = 75;
        } else if (line_value >= 13500) {
            left = 75;
            right = -45;
        } 
        // else {
        //     motors.setMotor1Speed(left);
        //     motors.setMotor2Speed(right);
        // }

        // FOR CLASIC
        motors.setMotor1Speed(left);
        motors.setMotor2Speed(right);

    } else {
        motors.setMotor1Speed(0);
        motors.setMotor2Speed(0);

        startTime = 0;
    }

    // if (DEBUG_FLAG) {
    //     Serial.print("left = ");
    //     Serial.println(left);

    //     Serial.print("right = ");
    //     Serial.println(right);
    // }

    delay(5);
    // delay(1000);
}