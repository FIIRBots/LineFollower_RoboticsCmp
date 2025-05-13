/*
    * Copyright (C) 2025 by Dan-Dominic Staicu
    * 331CA UNSTPB ACS CTI
*/
#ifndef LF_SDATA_H
#define LF_SDATA_H

#include <Arduino.h>
#include "kvstore_global_api.h"

class LF_SData {
public:
    void setupLineSensors(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3, uint8_t sig);
    void calibrateSensors(bool fullCalibration);
    void getLiveSerialPrint(bool printSensors);
    long getLinePosition();

private:
    uint8_t S0, S1, S2, S3, SIG;
    const int numSensors = 16;
    int sensorValues[16];
    long sensorMin[16];
    long sensorMax[16];
    long total;
    long weightedSum;
    long max_value;
    long lastLinePosition = 0;
    bool serialInitialized = false;
    const int noLineThreshold = 450;  // Threshold for determining no line detected

    const int positionValues[16] = {0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500};

    int readMultiplexer(int channel);
    void loadCalibration();
    void saveCalibration();
    void initializeSerial();
    bool isSerialConnected();
};

void LF_SData::setupLineSensors(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3, uint8_t sig) {
    S0 = s0;
    S1 = s1;
    S2 = s2;
    S3 = s3;
    SIG = sig;

    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);

    digitalWrite(S0, LOW);
    digitalWrite(S1, LOW);
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
}

void LF_SData::calibrateSensors(bool fullCalibration) {
    if (fullCalibration) {
        // Initialize calibration values
        for (int i = 0; i < numSensors; i++) {
            sensorMin[i] = 1023;
            sensorMax[i] = 0;
        }

        // Perform calibration for 5 seconds
        initializeSerial();
        Serial.println("Calibrating...");
        unsigned long startTime = millis();
        while (millis() - startTime < 5000) {
            for (int i = 0; i < numSensors; i++) {
                sensorValues[i] = readMultiplexer(i);
                sensorMin[i] = min(sensorMin[i], sensorValues[i]);
                sensorMax[i] = max(sensorMax[i], sensorValues[i]);
            }
            delay(10);
        }
        Serial.println("Calibration complete");

        // for (int i = 0; i < numSensors; i++) {
        //     Serial.print("Sensor ");
        //     Serial.print(i);
        //     Serial.print(": Min = ");
        //     Serial.print(sensorMin[i]);
        //     Serial.print(", Max = ");
        //     Serial.println(sensorMax[i]);
        // }

        saveCalibration();
    } else {
        Serial.println("Loading calibration data...");

        // for (int i = 0; i < numSensors; i++) {
        //     Serial.print("Sensor ");
        //     Serial.print(i);
        //     Serial.print(": Min = ");
        //     Serial.print(sensorMin[i]);
        //     Serial.print(", Max = ");
        //     Serial.println(sensorMax[i]);
        // }


        loadCalibration();
    }
}

void LF_SData::loadCalibration() {
    size_t actualSize;
    for (int i = 0; i < numSensors; i++) {
        String keyMin = "min" + String(i);
        String keyMax = "max" + String(i);
        kv_get(keyMin.c_str(), &sensorMin[i], sizeof(sensorMin[i]), &actualSize);
        kv_get(keyMax.c_str(), &sensorMax[i], sizeof(sensorMax[i]), &actualSize);
    }
}

void LF_SData::saveCalibration() {
    for (int i = 0; i < numSensors; i++) {
        String keyMin = "min" + String(i);
        String keyMax = "max" + String(i);
        kv_set(keyMin.c_str(), &sensorMin[i], sizeof(sensorMin[i]), 0);
        kv_set(keyMax.c_str(), &sensorMax[i], sizeof(sensorMax[i]), 0);
    }
}

void LF_SData::getLiveSerialPrint(bool printSensors) {
    total = 0;
    weightedSum = 0;

    if (!serialInitialized && isSerialConnected()) {
        initializeSerial();
    }

    if (serialInitialized) {
        bool lineDetected = false;

        if (printSensors) {
            // Print values
            Serial.print("| ");
            for (int i = 0; i < numSensors; i++) {
                sensorValues[i] = readMultiplexer(i);
                long normalizedValue = sensorValues[i] > 100 ? map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000) : 1;
                weightedSum += normalizedValue * positionValues[i];
                total += normalizedValue;

                if (sensorValues[i] > noLineThreshold) {
                    lineDetected = true;
                }

                Serial.print(sensorValues[i]);
                Serial.print(" | ");
            }

            long linePosition = getLinePosition();
            if (!lineDetected) {
                linePosition = lastLinePosition > 8000 ? 15000 : 0;
            } else {
                lastLinePosition = linePosition;
            }
            Serial.print(" pos = ");
            Serial.print(linePosition);
            Serial.print(" | ");
        }

        Serial.println();
    }
}

long LF_SData::getLinePosition() {
    total = 0;
    weightedSum = 0;
    max_value = 0;

    for (int i = 0; i < numSensors; i++) {
        sensorValues[i] = readMultiplexer(i);
        long normalizedValue = map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000);
        weightedSum += normalizedValue * positionValues[i];
        total += normalizedValue;
        max_value = max(max_value, sensorValues[i]);
    }

    long linePosition = (total > 0) ? weightedSum / total : -1;
    if (max_value <= 150 || linePosition == -1) {
        // No line detected
        linePosition = lastLinePosition > 8000 ? 15000 : 0;
    } else {
        lastLinePosition = linePosition;
    }

    return linePosition;
}

int LF_SData::readMultiplexer(int channel) {
    int controlPin[] = { S0, S1, S2, S3 };

    int muxChannel[16][4] = {
        { 0, 0, 0, 0 },  //channel 0
        { 1, 0, 0, 0 },  //channel 1
        { 0, 1, 0, 0 },  //channel 2
        { 1, 1, 0, 0 },  //channel 3
        { 0, 0, 1, 0 },  //channel 4
        { 1, 0, 1, 0 },  //channel 5
        { 0, 1, 1, 0 },  //channel 6
        { 1, 1, 1, 0 },  //channel 7
        { 0, 0, 0, 1 },  //channel 8
        { 1, 0, 0, 1 },  //channel 9
        { 0, 1, 0, 1 },  //channel 10
        { 1, 1, 0, 1 },  //channel 11
        { 0, 0, 1, 1 },  //channel 12   
        { 1, 0, 1, 1 },  //channel 13
        { 0, 1, 1, 1 },  //channel 14
        { 1, 1, 1, 1 }   //channel 15
    };  

    for (int i = 0; i < 4; i++) {
        digitalWrite(controlPin[i], muxChannel[channel][i]);
    }

    return analogRead(SIG);
}

void LF_SData::initializeSerial() {
    Serial.begin(115200);
    serialInitialized = true;
}

bool LF_SData::isSerialConnected() {
    return Serial;  // Returns true if Serial is connected
}

#endif // LF_SDATA_H