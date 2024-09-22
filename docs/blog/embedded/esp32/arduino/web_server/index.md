---
tags:
    - esp32
    - arduino
    - web
    - wifi
---


## Access Point (AP)
ESP32 sets up its own WiFi network 

```cpp
#include <WiFi.h>
#include <WebServer.h>

/* Put your SSID & Password */
const char *ssid = "ESP32";        // Enter SSID here
const char *password = "12345678"; // Enter Password here

/* Put IP Address details */
IPAddress local_ip(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

int counter = 0;
void setup()
{
    Serial.begin(115200); // set up Serial library at 9600 bps
    Serial.println("Hello world!");
    WiFi.softAP(ssid, password);
    WiFi.softAPConfig(local_ip, gateway, subnet);
    delay(100);
    server.on("/", handle_OnConnect);


    server.begin();
    Serial.println("HTTP server started");
}

String SendHTML(){
    return "hello world";
}

void handle_OnConnect() {
  Serial.println("Server contected");
  server.send(200, "text/html", SendHTML()); 
}

void loop()
{
    server.handleClient();
    
}
```