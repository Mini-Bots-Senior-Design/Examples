#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// Wi-Fi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Create an async web server on port 80
AsyncWebServer server(80);

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi network
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Define an API route (GET /status)
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("Status Message Received");
    request->send(200, "application/json", "{\"status\": \"ESP32 running\"}");
  });

  // Start server
  server.begin();
}

void loop() {
  // No need for `server.handleClient();` (handled asynchronously)
}
