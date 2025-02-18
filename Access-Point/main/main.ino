#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// AP credentials
const char* ssid = "ESP32-AP";
const char* password = "12345678";

// Create an async web server on port 80
AsyncWebServer server(80);

void setup() {
  Serial.begin(115200);

  // Start SoftAP
  WiFi.softAP(ssid, password);
  Serial.println("Access Point Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());  // Usually 192.168.4.1

  // Define an API route (GET /status)
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {

      // Print Query Parameters
    if (request->params()) {
        Serial.println("Query Parameters:");
        for (int i = 0; i < request->params(); i++) {
            const AsyncWebParameter* param = request->getParam(i);
            Serial.printf("  %s: %s\n", param->name().c_str(), param->value().c_str());
        }
    }

    // Print Headers
    Serial.println("Headers:");
    int headers = request->headers();
    for (int i = 0; i < headers; i++) {
        const AsyncWebHeader* h = request->getHeader(i);
        Serial.printf("  %s: %s\n", h->name().c_str(), h->value().c_str());
    }


    request->send(200, "application/json", "{\"status\": \"ESP32 running\"}");
  });

  // Start server
  server.begin();
}

void loop() {
  // No need for `server.handleClient();` (handled asynchronously)
}