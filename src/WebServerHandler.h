#ifndef WEB_SERVER_HANDLER_H
#define WEB_SERVER_HANDLER_H

#include <WiFi.h>

// Externally defined PID variables
extern float KP, KI, KD;
extern int REFERENCE_SPEED;
extern WiFiServer server;

void handleWeb();

#endif
