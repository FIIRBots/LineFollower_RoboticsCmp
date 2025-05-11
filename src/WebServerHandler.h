#ifndef WEB_SERVER_HANDLER_H
#define WEB_SERVER_HANDLER_H

#include <WiFi.h>

// Externally defined PID variables
extern double KP, KI, KD;
extern int REFERENCE_SPEED;
extern WiFiServer server;
extern double BASE_SPEED;
extern bool robotActive;

// #define DEBUG_FLAG true

void handleWeb();

#endif
