#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <SparkFun_u-blox_GNSS_v3.h>
#include <HardwareSerial.h>

BNO08x imu;

#define BNO08X_INT 14     // IMU INT --> ESP32 GPIO 19
#define BNO08X_RST 13     // IMU RST --> ESP32 GPIO 23
#define BNO08X_ADDR 0x4B  // default address is 0x4B
#define SLAVE_ADDR 0x08


#ifndef DEFAULT_STACK_SIZE
#define DEFAULT_STACK_SIZE 4096
#endif

// Enable all sensor outputs
void setImuReports(void) {
  Serial.println("Setting desired reports");
  if (imu.enableAccelerometer()) {
    Serial.println(F("Accelerometer enabled"));
  } else {
    Serial.println("Could not enable accelerometer");
  }
  if (imu.enableRotationVector()) {
    Serial.println(F("Rotation vector enabled"));
  } else {
    Serial.println("Could not enable rotation vector");
  }
  if (imu.enableMagnetometer()) {
    Serial.println(F("Magnetometer enabled"));
  } else {
    Serial.println("Could not enable magnetometer");
  }
}

// Variables to store sensor data
float accelX = 0, accelY = 0, accelZ = 0;
float rotI = 0, rotJ = 0, rotK = 0, rotR = 0;
float magX = 0, magY = 0, magZ = 0;
float compass = 0;  // 0 degrees = magnetic north

static float heading_calculation(float Ri, float Rj, float Rk, float Rr, float Mx, float My, float Mz) {

  // Ensure vectors are normalized
  float Rnorm = sqrt(sq(Ri) + sq(Rj) + sq(Rk) + sq(Rr));
  Ri /= Rnorm;
  Rj /= Rnorm;
  Rk /= Rnorm;
  Rr /= Rnorm;
  float Mnorm = sqrt(sq(Mx) + sq(My) + sq(Mz));
  Mx /= Mnorm;
  My /= Mnorm;
  Mz /= Mnorm;

  // Numerator components
  float num1 = 2 * (Ri * Rj + Rk * Rr);
  float num2 = sq(Rr) - sq(Ri) + sq(Rj) - sq(Rk);
  float num3 = 2 * (Rj * Rk - Ri * Rr);
  float numerator = num1 * Mx + num2 * My + num3 * Mz;

  // Denominator components
  float den1 = sq(Rr) + sq(Ri) - sq(Rj) - sq(Rk);
  float den2 = 2 * (Ri * Rj - Rk * Rr);
  float den3 = 2 * (Ri * Rk + Rj * Rr);
  float denominator = den1 * Mx + den2 * My + den3 * Mz;

  // Final calculation
  float bearing = atan2(numerator, denominator) * 180 / M_PI + 360;
  while(bearing >= 360) bearing -= 360;
  while(bearing < 0) bearing += 360;
  return bearing;
}


// Task to print out all sensor data
void imu_task(void* ignored) {
  while (1) {
    if (imu.wasReset()) {
      Serial.print("Sensor was reset ");
      setImuReports();
    }

    // Flags to check if data was read
    bool accelRead = false, gyroRead = false, magRead = false;

    // Read sensor events and store values
    while (imu.getSensorEvent()) {
      uint8_t eventID = imu.getSensorEventID();
      if (eventID == SENSOR_REPORTID_ACCELEROMETER) {
        accelX = imu.getAccelX();
        accelY = imu.getAccelY();
        accelZ = imu.getAccelZ();
        accelRead = true;
      } else if (eventID == SENSOR_REPORTID_ROTATION_VECTOR) {
        rotI = imu.getQuatI();
        rotJ = imu.getQuatJ();
        rotK = imu.getQuatK();
        rotR = imu.getQuatReal();
        gyroRead = true;
      } else if (eventID == SENSOR_REPORTID_MAGNETIC_FIELD) {
        magX = imu.getMagX();
        magY = imu.getMagY();
        magZ = imu.getMagZ();
        magRead = true;
      }

      // Break if all sensor readings have been captured
      if (accelRead && gyroRead && magRead) {
        compass = heading_calculation(rotI, rotJ, rotK, rotR, magX, magY, magZ);
        break;
      }

    }

    // // Print out all sensor data together
    // if (accelRead || gyroRead || magRead) {
    //   if (accelRead) {
    //     Serial.print("Accelerometer (m/s^2) - X: ");
    //     Serial.print(accelX, 2);
    //     Serial.print(", Y: ");
    //     Serial.print(accelY, 2);
    //     Serial.print(", Z: ");
    //     Serial.println(accelZ, 2);
    //   }
    //   if (magRead) {
    //     Serial.print("Magnetometer (uT) - X: ");
    //     Serial.print(magX, 2);
    //     Serial.print(", Y: ");
    //     Serial.print(magY, 2);
    //     Serial.print(", Z: ");
    //     Serial.println(magZ, 2);
    //   }
    //   if (gyroRead) {
    //     Serial.print("Rotation Quaternion - I: ");
    //     Serial.print(rotI, 2);
    //     Serial.print(", J: ");
    //     Serial.print(rotJ, 2);
    //     Serial.print(", K: ");
    //     Serial.print(rotK, 2);
    //     Serial.print(", Real: ");
    //     Serial.println(rotR);
    //   }
    if(compass){
      // Serial.print("Compass (Degrees): ");
      Serial.println(compass, 2);
    }

    // Serial.println("--- End of Sensor Data Cycle ---");

    vTaskDelay(200);
  }
}



void setup() {
  Serial.begin(9600);

  while (!Serial) {
    delay(100);
  }

  Serial.print("Compass (Degrees): ");

  Wire.begin(21,22);

  for (unsigned int imu_attempt = 0; imu_attempt < 10; imu_attempt++) {
    if (imu.begin()) {
      break;
    }
    Serial.print("BNO08x not detected at default I2C address. Attempt ");
    Serial.print(imu_attempt);
    Serial.println(" of 10. Retrying...");
    delay(100);
  }
  Serial.println("BNO08x found!");


  setImuReports();


  xTaskCreate(imu_task, "IMU Sensor Data", DEFAULT_STACK_SIZE, (void*)NULL, 2, NULL);
}

void loop() {
  // nothing
}
