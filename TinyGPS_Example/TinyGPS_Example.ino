#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <math.h>  // Required for lround()


// Define GPS Serial Port (ESP32 has multiple hardware serials)
HardwareSerial SerialGPS(1);  // Use Serial1

// GPS Object
TinyGPSPlus gps;

// Define GPS Module TX/RX Pins
#define GPS_RX_PIN 34  // Connect GPS TX -> ESP32 RX (Receive)
#define GPS_TX_PIN 12  // Connect GPS RX -> ESP32 TX (Transmit)
#define GPS_BAUD_RATE 9600  // Adjust based on your GPS module

void displayInfo();

void setup() {
    Serial.begin(115200);  // Debugging Serial Monitor
    SerialGPS.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    Serial.println(F("GPS Module with TinyGPS++"));
    Serial.println(F("Ensure the GPS module has a clear sky view."));
    Serial.println(F("Waiting for GPS data...\n"));
}

void loop()
{
    // This sketch displays information every time a new sentence is correctly encoded.
    while (SerialGPS.available() > 0){
      if (gps.encode(SerialGPS.read())){
        displayInfo2();
        delay(1000);
      }
    }
}


void displayInfo2() {
  long lat_scaled = 0;
  long lng_scaled = 0;

  Serial.print(F("Location: "));
  if (gps.location.isValid()) {
      // Scale latitude and longitude by 1,000,000 for micro-degree accuracy
      lat_scaled = lround(gps.location.lat() * 1e6);
      lng_scaled = lround(gps.location.lng() * 1e6);

      Serial.print(lat_scaled);
      Serial.print(F(", "));
      Serial.println(lng_scaled);
  } else {
      Serial.println(F("INVALID"));
  }
}

// Default Example
void displayInfo()
{
    Serial.print(F("Location: "));
    if (gps.location.isValid()) {
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gps.location.lng(), 6);
    } else {
        Serial.print(F("INVALID"));
    }

    Serial.print(F("  Date/Time: "));
    if (gps.date.isValid()) {
        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.print(gps.date.day());
        Serial.print(F("/"));
        Serial.print(gps.date.year());
    } else {
        Serial.print(F("INVALID"));
    }

    Serial.print(F(" "));
    if (gps.time.isValid()) {
        if (gps.time.hour() < 10) Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        if (gps.time.minute() < 10) Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        if (gps.time.second() < 10) Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
        if (gps.time.centisecond() < 10) Serial.print(F("0"));
        Serial.print(gps.time.centisecond());
    } else {
        Serial.print(F("INVALID"));
    }

    Serial.println();
}
