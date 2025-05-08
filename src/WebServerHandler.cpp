#include "WebServerHandler.h"

void handleWeb() {
    WiFiClient client = server.available();
    if (!client) return;

    String req = client.readStringUntil('\r');
    client.readStringUntil('\n'); // skip remainder

    if (req.startsWith("GET /?")) {
        int p = req.indexOf('?') + 1;
        int e = req.indexOf(' ', p);
        String qs = req.substring(p, e);

        unsigned int i = 0;
        while (i < qs.length()) {
            int eq = qs.indexOf('=', i);
            int amp = qs.indexOf('&', i);
            if (amp < 0) amp = qs.length();
            String key   = qs.substring(i, eq);
            String value = qs.substring(eq+1, amp);

            if (key == "kp")  KP               = value.toFloat();
            else if (key == "ki")  KI            = value.toFloat();
            else if (key == "kd")  KD            = value.toFloat();
            else if (key == "ref") REFERENCE_SPEED = value.toInt();

            i = amp + 1;
        }
    }

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println();
    client.println("<!DOCTYPE HTML><html><body>");
    client.println("<h2>Portenta PID Tuner</h2>");
    client.println("<form action=\"/\">");
    client.print("KP: <input name=\"kp\" value=\""); client.print(KP); client.println("\"><br>");
    client.print("KI: <input name=\"ki\" value=\""); client.print(KI); client.println("\"><br>");
    client.print("KD: <input name=\"kd\" value=\""); client.print(KD); client.println("\"><br>");
    client.print("Ref Speed: <input name=\"ref\" value=\""); client.print(REFERENCE_SPEED); client.println("\"><br><br>");
    client.println("<input type=\"submit\" value=\"Update\">");
    client.println("</form></body></html>");
    client.stop();
}
