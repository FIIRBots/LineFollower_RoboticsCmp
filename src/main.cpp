#include <Arduino.h>
#include <DRV8835MotorShield.h>
#include "LF_SData.h"
#include <WiFi.h>
#include "arduino_secrets.h"   // put SSID/PASS in Secret tab

WiFiServer server(80);

LF_SData sensorData;

#define CALIBRATION_FLAG true
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

#define M1DIR D5  // GPIO
#define M1PWM D4  // PWM 
#define M2DIR D7  // GPIO
#define M2PWM D6  // PWM 
DRV8835MotorShield motors(M1DIR, M1PWM, M2DIR, M2PWM);


#define MIN_SPEED -400
#define MAX_SPEED 400
// #define REFERENCE_SPEED 255

// PID constants
// #define KP 0.22
// #define KI 0.008
// #define KD 9.2
float KP = 0.22;
float KI = 0.008;
float KD = 9.2;
int   REFERENCE_SPEED = 255;

float previousError = 0;
float integral = 0;

float setBaseSpeed = 0, baseSpeed = 0;
int setMaxSpeed = 0, maxSpeed = 0;

// Variables to store the last six error values for smoother KI calculation
int error1 = 0, error2 = 0, error3 = 0, error4 = 0, error5 = 0, error6 = 0;
unsigned long startTime = 0, currentTime = 0, elapsedTime = 0, accelerationTime = 500;

void handleWeb() {
    WiFiClient client = server.available();
    if (!client) return;
  
    // Read the request line
    String req = client.readStringUntil('\r');
    client.readStringUntil('\n'); // skip remainder
  
    // If it's a GET with params, e.g. "/?kp=0.3&ki=0.01&kd=8&ref=200"
    if (req.startsWith("GET /?")) {
      int p = req.indexOf('?') + 1;
      int e = req.indexOf(' ', p);
      String qs = req.substring(p, e);
  
      // split on '&'
      int i = 0;
      while (i < qs.length()) {
        int eq = qs.indexOf('=', i);
        int amp = qs.indexOf('&', i);
        if (amp < 0) amp = qs.length();
        String key   = qs.substring(i, eq);
        String value = qs.substring(eq+1, amp);
  
        if (key == "kp")  KP               = value.toFloat();
        else if (key== "ki")  KI            = value.toFloat();
        else if (key== "kd")  KD            = value.toFloat();
        else if (key== "ref") REFERENCE_SPEED = value.toInt();
  
        i = amp + 1;
      }
    }
  
    // Serve back a tiny HTML form
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println();
    client.println("<!DOCTYPE HTML><html><body>");
    client.println("<h2>Portenta PID Tuner</h2>");
    client.println("<form action=\"/\">");
    client  .print("KP: <input name=\"kp\" value=\""); client.print(KP); client.println("\"><br>");
    client  .print("KI: <input name=\"ki\" value=\""); client.print(KI); client.println("\"><br>");
    client  .print("KD: <input name=\"kd\" value=\""); client.print(KD); client.println("\"><br>");
    client  .print("Ref Speed: <input name=\"ref\" value=\""); client.print(REFERENCE_SPEED); client.println("\"><br><br>");
    client.println("<input type=\"submit\" value=\"Update\">");
    client.println("</form></body></html>");
    client.stop();
  }
  

void setup() {
    if (DEBUG_FLAG) {
        Serial.begin(115200);
        while (!Serial) {
            ; // wait for serial port to connect. Needed for native USB port only
        }
    }
    

    if (!Serial) {
        Serial.begin(115200);
    }
    
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

    if (SETUP_WIFI) {
        delay(10000);   
    }

    sensorData.setupLineSensors(S0, S1, S2, S3, SIG);

    if (CALIBRATION_FLAG) {
        // Calibration mode
        unsigned long startTime = millis();
        unsigned long calibrationTime = 5000;  // 5 seconds

        Serial.begin(115200);
        while(!Serial) {
            ; // wait for serial port to connect. Needed for native USB port only
        }
    
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

    elapsedTime = millis() - startTime;
    if (elapsedTime <= accelerationTime) {
        baseSpeed = map(elapsedTime, 0, accelerationTime, 0, setBaseSpeed);
        maxSpeed = map(elapsedTime, 0, accelerationTime, 0, setMaxSpeed);
    }

    handleWeb();

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

        // Print sensor values
        sensorData.getLiveSerialPrint(true, false);

        // long position = sensorData.getLinePosition();
        // Serial.print("Line Position: ");
        // Serial.println(position);
    }

    if ((linePosition > 2000 && linePosition <= 4000) || (linePosition >= 11000 && linePosition < 13000)) {  // Medium turn
        baseSpeed = constrain(setBaseSpeed * 0.9, 0.5 * REFERENCE_SPEED, setBaseSpeed); // 50% of REFERENCE speed
        maxSpeed = constrain(setMaxSpeed * 0.9, 0.6 * REFERENCE_SPEED, setMaxSpeed); // 60% of REFERENCE speed
    } else {                     // Straight line
        baseSpeed = setBaseSpeed;  // Restore full speed
        maxSpeed = setMaxSpeed;
    }

    if (linePosition <= 2500) {
        // 80% of 400, 70% of 400
        motors.setSpeeds(0.8 * REFERENCE_SPEED, -0.7 * REFERENCE_SPEED); 

        // xmotion.MotorControl(80, -70);

        // motors.setM1Speed(80);
        // motors.setM2Speed(-70);

    } else if (linePosition >= 12500) {
        // 70% of 400, 80% of 400
        motors.setSpeeds(-0.7 * REFERENCE_SPEED, 0.8 * REFERENCE_SPEED);

        // xmotion.MotorControl(-70, 80);

        // motors.setM1Speed(-70);
        // motors.setM2Speed(80);

    } else {
        // xmotion.MotorControl(map(leftMotorSpeed, 0, 100, 0, 255), map(rightMotorSpeed, 0, 100, 0, 255));
        int left = map(leftMotorSpeed, 0, 100, 0, REFERENCE_SPEED);
        int right = map(rightMotorSpeed, 0, 100, 0, REFERENCE_SPEED);

        // motors.setM1Speed(left);
        // motors.setM2Speed(right);

        if (DEBUG_FLAG) {
            Serial.print("left = ");
            Serial.println(left);

            Serial.print("right = ");
            Serial.println(right);
        }

        motors.setSpeeds(left, right);
    }
    
    delay(5);
}