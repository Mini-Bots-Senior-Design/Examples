#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <SparkFun_u-blox_GNSS_v3.h>
#include <HardwareSerial.h>

BNO08x imu;
SFE_UBLOX_GNSS gps;

#define BNO08X_INT 19     // IMU INT --> ESP32 GPIO 19
#define BNO08X_RST 23     // IMU RST --> ESP32 GPIO 23
#define BNO08X_ADDR 0x4B  // default address is 0x4B
#define SLAVE_ADDR 0x08

#define COMPILE_WITH_GPS 1

#ifdef COMPILE_WITH_GPS
#define GPS_ADDR
#endif

#ifndef DEFAULT_STACK_SIZE
#define DEFAULT_STACK_SIZE 4096
#endif

float latitude = 0;
float longitude = 0;

void gps_task(void* ignored) {
  while (1) {
    if (gps.getPVT() == true) {  // GPS only gets location data once per second
      int32_t latitude_as_int = gps.getLatitude();
      int32_t longitude_as_int = gps.getLongitude();

      latitude = (float)latitude_as_int / 10000000.0;
      longitude = (float)longitude_as_int / 10000000.0;

      Serial.print("Latitude (Degrees): ");
      Serial.println(latitude, 7);
      Serial.print("Longitude (Degrees): ");
      Serial.println(longitude, 7);
    }
  }
}


void setup() {
  Serial.begin(9600);

  Serial.println("UART1 initialized on Sender");
  while (!Serial) {
    delay(100);
  }

  
  for (unsigned int gps_attempt = 0; gps_attempt < 10; gps_attempt++) {
    if (gps.begin()) {
      break;
    }
    Serial.print("GPS not detected at default I2C address. Attempt ");
    Serial.print(gps_attempt);
    Serial.println(" of 10. Retrying...");
    delay(100);
  }

  gps.setI2COutput(COM_TYPE_UBX);
  xTaskCreate(gps_task, "GPS Sensor Data", DEFAULT_STACK_SIZE, (void*)NULL, 4, NULL);
}

void loop() {
  // nothing
}