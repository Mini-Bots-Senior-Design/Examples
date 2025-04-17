// File to control all onboard software
#include <ESP32Servo.h> 
#include <math.h> // GPS Functions

#include "LoRaBoards.h" // LoRa 
#include <RadioLib.h>   // LoRa


#define EARTH_RADIUS_FEET 20902230.0 // Earth's radius in feet      // GPS Functions
#define RAD_TO_DEG (180.0 / M_PI) //    Convert radians to degrees  // GPS Functions

// Pin configuration for T-Beam SX1278
#define RADIO_CS_PIN 18
#define RADIO_DIO0_PIN 26
#define RADIO_RST_PIN 23
#define RADIO_DIO1_PIN 33
#define RADIO_DIO2_PIN 32  // Optional


///////////
// Modes //
///////////
// STARTUP
// MOV
// TEST
// MOVGPS
// MOVPWM
String STATE = "STARTUP";

Servo motor1;  // create servo object to control a servo
Servo motor2; 

int minUs = 1000;
int maxUs = 2000;
int frequency = 50; // 50Hz

int motorpin1 = 13;      // GPIO pin used to connect the servo control (digital out) 
int motorpin2 = 2;      // GPIO pin used to connect the servo control (digital out)  

const char* BOTID = "3";

// Sensor Values
long Current_GPS_Latitude = 0;
long Current_GPS_Longitude = 0;
float Current_Compass_Heading = 0;

long Target_GPS_Latitude = 0;
long Target_GPS_Longitude = 0;

// State
bool automaticMode = false;

// Switch to get real value
bool DEBUG = true;


// LoRa Start
// Set up the LoRa module for SX1278
SX1278 radio = new Module(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_DIO1_PIN);

String payload = "";
volatile bool receivedFlag = false;
volatile bool transmittedFlag = false;

SemaphoreHandle_t xSemaphore;
TaskHandle_t BotTaskHandle = NULL;
// LoRa end




#include <Wire.h> //Needed for I2C to GNSS
#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
#include "SparkFun_BNO08x_Arduino_Library.h"

// IMU Start

BNO08x imu;
#define BNO08X_ADDR 0x4B
int imuEnabled = 0;

// IMU END

// GPS Start
SFE_UBLOX_GNSS myGNSS;
long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
// GPS End

// Mutexes to protect shared resources
SemaphoreHandle_t xMutexSensor;
SemaphoreHandle_t xMutexAutomatic;


void setup() {
  // Start communication with the Serial Monitor (USB Serial)
  Serial.begin(115200); 

  // Motor Setup
  motor1.setPeriodHertz(frequency);// Standard 50hz servo
  motor2.setPeriodHertz(frequency);// Standard 50hz servo
  motor1.attach(motorpin1, minUs, maxUs);   // attaches the servo on pin 18 to the servo object
  motor2.attach(motorpin2, minUs, maxUs);   // attaches the servo on pin 18 to the servo object

  // GPS Start //
  Serial.println("Configuring GPS....");

  Wire.begin();
  delay(1000);

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    //while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
  Serial.println("GPS Configured");
  // GPS End // 

  // TD: BENJI
  // IMU SETUP START
  if (imu.begin() == false) {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }
  Serial.println("BNO08x found!");

  if (imu.enableGeomagneticRotationVector() == true) {
    Serial.println(F("Geomagnetic Rotation vector enabled"));
    Serial.println(F("Output in form roll, pitch, yaw"));
    int imuEnabled = 1;
  } else {
    Serial.println("Could not enable geomagnetic rotation vector");
  }

  // IMU SETUP End

  // Create Tasks, 

  delay(1000); // short delay

  // Calibrate Motors
  Serial.println("Calibrating Motors...");
  motor1.write(90);                 
  motor2.write(90);
  delay(5000); 
  Serial.println("Calibration Finished");

  setupLoRa();

  xSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(xSemaphore);

  // For sensors
  xMutexSensor = xSemaphoreCreateMutex();
  xMutexAutomatic = xSemaphoreCreateMutex();


  xTaskCreate(BotTask, "Bot Task", 2048, NULL, 1, NULL);
  //xTaskCreate(SensorTask, "Sensor Task", 2048, NULL, 1, NULL);
  xTaskCreate(AutomaticTask, "Automatic Task", 2048, NULL, 1, NULL);

  xTaskCreate(CompassTestTask, "Compass Task", 2048, NULL, 1, NULL);

  Serial.println("Startup Completed");
}

// Reads the Serial Port
void loop() {
  String inputString = "";  // String to hold the incoming data

  // Read bytes from the serial until a newline character is found
  while (Serial.available() > 0) {

    // TODO: Change this to read from LoRA, or have the option to...
    char incomingByte = Serial.read();  // Read a byte from Serial

    if (incomingByte == '\n') {

      parseInput(inputString);
    
      inputString = "";// Clear the string for the next line
    } else {
      // Otherwise, add the byte to the string
      inputString += incomingByte;
    }
    delay(100);
  }
}
 

/////////////////
// Tasks Start // 
/////////////////

// Bot Task: Listen for requests, then send ID when requested
void BotTask(void *pvParameters) {

  switchToReceiveMode();  // Start in receive mode
  String receivedMessage;

  while (1) {
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
      int state = radio.receive(receivedMessage);

      if (state == RADIOLIB_ERR_NONE && receivedFlag) {
        Serial.println("Received: " + receivedMessage);
        unsigned long sendStart = millis();

        if(parseInput(receivedMessage)){
          // Reset and setup trasmit setting
          transmittedFlag = false;
          switchToTransmitMode();

          // Setup Payload
          payload = createPayload();

          int transmissionState = radio.transmit(payload);
        }

        // Reset and setup receive setting
        switchToReceiveMode();  
        receivedFlag = false;
      }
      xSemaphoreGive(xSemaphore);
      vTaskDelay(100 / portTICK_PERIOD_MS);  // Reduced from 200ms to 100ms
    }
  }
}


// This will sample the GPS and IMU Sensors and write them to global variables 
void SensorTask(void *pvParameters){
  long local_GPS_Latitude;
  long local_GPS_Longitude;
  float local_Compass_Heading = 0;
  static float prev_Compass_Heading = 0;

  while(1){
    local_GPS_Latitude = myGNSS.getLatitude();
    local_GPS_Longitude = myGNSS.getLongitude();

    float newHeading = readIMU();
    float diff = angleDiff(prev_Compass_Heading, newHeading);
    Serial.println(String(prev_Compass_Heading) + " " + String(newHeading) + " " + String(diff));

    if (abs(diff) > 10) {
      local_Compass_Heading = prev_Compass_Heading;
      Serial.print("ABRUPT COMPASS MOVEMENT OF: ");
      Serial.println(diff);
    } 
    else {
      float alpha = 0.2;
      local_Compass_Heading = fmod((prev_Compass_Heading + alpha * diff + 360.0), 360.0);
      if (local_Compass_Heading < 0) local_Compass_Heading += 360.0;
    }
    prev_Compass_Heading = local_Compass_Heading;

    if (xSemaphoreTake(xMutexSensor, portMAX_DELAY) == pdTRUE) {
      Serial.println("Filtered Compass: " + String(local_Compass_Heading));
      Current_GPS_Latitude = local_GPS_Latitude;
      Current_GPS_Longitude = local_GPS_Longitude;
      Current_Compass_Heading = local_Compass_Heading; 
      xSemaphoreGive(xMutexSensor);
    } else {
      Serial.println("Failed to take mutex");
    }

    vTaskDelay(25 / portTICK_PERIOD_MS);
  }
}

void CompassTestTask(void *pvParameters) {
  static float prevHeading = 0;
  static bool firstRun = true;
  long local_GPS_Latitude;
  long local_GPS_Longitude;

  while (1) {
    local_GPS_Latitude = myGNSS.getLatitude();
    local_GPS_Longitude = myGNSS.getLongitude();
    float rawHeading = readIMU();

    if (firstRun) {
      prevHeading = rawHeading;
      firstRun = false;
      Serial.println("FIRST RUN — Setting initial heading");
    }

    float diff = fmod((rawHeading - prevHeading + 540.0), 360.0) - 180.0;

    // Clips the offset to a max if it is too much too fast
    float maxStep = 10.0;
    if (abs(diff) > maxStep) {
      diff = (diff > 0) ? maxStep : -maxStep;
    }

    // Smooth the transition
    float alpha = 0.2;
    float filteredHeading = fmod((prevHeading + alpha * diff + 360.0), 360.0);
    if (filteredHeading < 0) filteredHeading += 360.0;

    prevHeading = filteredHeading;

    Serial.println("RAW: " + String(rawHeading) +
                   " | DIFF: " + String(diff) +
                   " | FILTERED: " + String(filteredHeading));

    if (xSemaphoreTake(xMutexSensor, portMAX_DELAY) == pdTRUE) {
      Current_GPS_Latitude = local_GPS_Latitude;
      Current_GPS_Longitude = local_GPS_Longitude;
      Current_Compass_Heading = filteredHeading;
      xSemaphoreGive(xMutexSensor);
    }

    vTaskDelay(25 / portTICK_PERIOD_MS);
  }
}



void AutomaticTask(void *pvParameters){
  while(1){
    if(getAutomaticMode()){
      Serial.println("In Automatic Mode");
      String gpsString = ""; // Debug
      long currentLat;
      long currentLon;
      long targetLat;
      long targetLon;
      float currentCompass;

      if (xSemaphoreTake(xMutexSensor, portMAX_DELAY) == pdTRUE) {
        targetLat = Target_GPS_Latitude;
        targetLon = Target_GPS_Longitude;
        currentLat = Current_GPS_Latitude;
        currentLon = Current_GPS_Longitude;
        currentCompass = Current_Compass_Heading;
        xSemaphoreGive(xMutexSensor);
      }

      // DEBUG Print:
      // Convert the longitude and latitude to a string and concatenate them
      gpsString = String(currentLon) + "," + String(currentLat);
      Serial.print("Current GPS/Compass: ");
      Serial.print(gpsString);
      Serial.print(" ,Compass: ");
      Serial.println(String(currentCompass));

      gpsString = "Longitude: " + String(targetLon) + ", Latitude: " + String(targetLat);
      Serial.print("Target GPS: ");
      Serial.println(gpsString);


      // TD: make the calculations
      // TD: move the motors
    }
    else{
      //Serial.println("Not In Automatic Mode");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Reduced from 200ms to 100ms
  }
}

float angleDiff(float a, float b) {
  float diff = fmod((b - a + 540.0), 360.0) - 180.0;
  return diff;
}


///////////////
// Tasks End // 
///////////////
void setupLoRa() {
    setupBoards();

    int state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("LoRa init failed, code: "));
        Serial.println(state);
        while (true);
    }

    radio.setFrequency(433.0);
    radio.setBandwidth(125.0);
    radio.setSpreadingFactor(8);
    radio.setCodingRate(6);
    radio.setSyncWord(0x12);
    radio.setOutputPower(17);

    //radio.implicitHeader(255); //ONLY FOR SF6 (param is max packet size)

    radio.setPacketSentAction(setTransmissionFlag);
    radio.setPacketReceivedAction(setReceiverFlag);
}




/////////////////////////////////////////
// INPUT: {BotID, Command, Parameters} //
/////////////////////////////////////////

// BotID: {1,2,3,4}
// Command: {MOV, MOVPWM, MOVGPS, STARTUP}
//  MOV: {F,L,R,S,B}
//  MOVGPS: {LAT, LAN}
//  MOVPWM: {PWMLeft, PWMRight}

// Ex. 1,MOV,L
bool parseInput(String inputString){

  // Parse Commas
  int indexOfFirstComma = inputString.indexOf(',');   
  int indexOfSecondComma = inputString.indexOf(',', indexOfFirstComma + 1);  

  // Extract "BotID"
  String BotID = inputString.substring(0, indexOfFirstComma);  
  // Serial.println("BotID: " + BotID);

  if(BotID == BOTID){

    // Extract Command
    String Command = inputString.substring(indexOfFirstComma + 1, indexOfSecondComma); 


    if(Command == "MOVAUTO"){

      // Parse GPS Points
      
      // parse lat
      int indexOfThirdComma = inputString.indexOf(',',indexOfSecondComma + 1); 
      String latString = inputString.substring(indexOfSecondComma + 1, indexOfThirdComma); 

      // parse lon
      int indexOfFourthComma = inputString.indexOf(',',indexOfThirdComma + 1); 
      String lonString = inputString.substring(indexOfThirdComma + 1, indexOfFourthComma); 

      // Set Target Points
      setTargetGPSPoints(latString.toInt(), lonString.toInt());

      setAutomaticModeTrue();
    } else{
      setAutomaticModeFalse();
    }

    // MOV
    if(Command == "MOV"){
      // Serial.println("MOV");

      // Extract Command
      int indexOfThirdComma = inputString.indexOf(',',indexOfSecondComma + 1); 
      String Direction = inputString.substring(indexOfSecondComma + 1, indexOfThirdComma); 
      // Serial.println("Direction: " + Direction);

      // Move the Motors
      moveMotorsForMOV(Direction);
    }

    // MOVPWM
    if(Command == "MOVPWM"){
      // Serial.println("MOVPWM");

      // parse leftPWM
      int indexOfThirdComma = inputString.indexOf(',',indexOfSecondComma + 1); 
      String leftPWMString = inputString.substring(indexOfSecondComma + 1, indexOfThirdComma); 

      // parse rightPWM
      int indexOfFourthComma = inputString.indexOf(',',indexOfThirdComma + 1); 
      String rightPWMString = inputString.substring(indexOfThirdComma + 1, indexOfFourthComma); 

      moveMotorsForMOVPWM(leftPWMString.toInt(), rightPWMString.toInt());
    }

    // STARTUP
    if(Command == "STARTUP"){
      // Serial.println("STARTUP");
      stopMotors();
    }
    
    return true;
  } 
  else{
    Serial.println("Incorrect BotID"); // DEBUG
    return false;
  }
}


///////////////////// 
// Motor Functions // 
/////////////////////

void stopMotors(){
  motor1.write(90);                 
  motor2.write(90);
}

void moveMotorsForMOVPWM(int leftPWM, int rightPWM){
  Serial.println("Movin Via da PWM");
  motor1.write(leftPWM);                 
  motor2.write(rightPWM); 
}

void moveMotorsForMOV(String direction){
  if(direction == "F"){
    Serial.println("Movin Forward");
    motor1.write(110);                 
    motor2.write(110); 
  }
  else if(direction == "B"){
    Serial.println("Movin Back");
    motor1.write(70);                 
    motor2.write(70); 
  }
  else if(direction == "L"){
    Serial.println("Movin Left");
    motor1.write(110);                 
    motor2.write(70); 
  }
  else if(direction == "R"){
    Serial.println("Movin Right");
    motor1.write(70);                 
    motor2.write(110); 
  }
  else if(direction == "S"){
    Serial.println("Movin Stop");
    motor1.write(90);                 
    motor2.write(90); 
  }
  else{
    Serial.println("Incorrect Format");
  }
}



////////////////////
// LoRa Functions //
////////////////////

// Callback functions for RadioLib events
void setReceiverFlag() {receivedFlag = true;}
void setTransmissionFlag() {transmittedFlag = true;}

void switchToTransmitMode() {
    radio.standby();  // Ensure module is in standby before transmit
    transmittedFlag = false;
    //Serial.println("Switching to Transmit Mode");
}

void switchToReceiveMode() {
    radio.standby();
    receivedFlag = false;
    radio.startReceive();  // Start listening
    //Serial.println("Switching to Receive Mode");
}


//////////////////////////
// LoRa Setup Functions //
//////////////////////////

void handleTransmission(bool transmittedFlag, int transmissionState){
  if (transmittedFlag) {
    Serial.println("Bot transmitted: " + payload);
  } else {
    Serial.println("Transmit error, code " + String(transmissionState));
  }
}

// This function reads the IMU's Compass
// TD: BENJI
float readIMU(){
  float currentDegrees = 0;

  if (imu.wasReset()) {
    Serial.print("sensor was reset ");
    if (imu.enableGeomagneticRotationVector() == true) {
      Serial.println(F("Geomagnetic Rotation vector enabled"));
      Serial.println(F("Output in form roll, pitch, yaw"));
    } else {
      Serial.println("Could not enable geomagnetic rotation vector");
    }
  }

  if (imu.getSensorEvent() == true) {
    if (imu.getSensorEventID() == SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR) {
      float yaw = imu.getYaw() * 180.0 / PI;
      if (yaw < 0) yaw += 360.0;
      currentDegrees = yaw;
    }
  }
  Serial.println("READ: " + String(currentDegrees));

  return currentDegrees;
}




//////////////////////////////
// Payload Helper Functions //
//////////////////////////////

// Return a string
String getGPS_String() {
  long currentLon;
  long currentLat;

  if (xSemaphoreTake(xMutexAutomatic, portMAX_DELAY) == pdTRUE) {
    currentLon = Current_GPS_Longitude;
    currentLat = Current_GPS_Latitude;
    xSemaphoreGive(xMutexAutomatic);
  }

  // Convert the longitude and latitude to a string and concatenate them
  String gpsString = String(currentLat) + "," + String(currentLon);
  Serial.println(gpsString);
  return gpsString;  // Return the formatted string
}

String createPayload(){

  String latLon = getGPS_String();
  String createdString = "0," + String(BOTID) + "," + latLon; // Why Zero? ohh zero is SBC

  Serial.print("Payload created: ");
  Serial.println(createdString);

  return createdString;
}


////////////////////////////////////
// Automatic Mode Setters/Getters //
////////////////////////////////////

bool getAutomaticMode(){
  bool localAutomaticMode;

  if (xSemaphoreTake(xMutexAutomatic, portMAX_DELAY) == pdTRUE) {
    localAutomaticMode = automaticMode;

    xSemaphoreGive(xMutexAutomatic);
  }
   else {
    Serial.println("Failed to take mutex");
  }

  return localAutomaticMode;
}

void setAutomaticModeFalse(){
  if (xSemaphoreTake(xMutexAutomatic, portMAX_DELAY) == pdTRUE) {
    automaticMode = false;

    xSemaphoreGive(xMutexAutomatic);
  }
   else {
    Serial.println("Failed to take mutex");
  }
}

void setAutomaticModeTrue(){
  if (xSemaphoreTake(xMutexAutomatic, portMAX_DELAY) == pdTRUE) {
    automaticMode = true;

    xSemaphoreGive(xMutexAutomatic);
  }
   else {
    Serial.println("Failed to take mutex");
  }
}


////////////////////////////////
// Target GPS Setters/Getters //
////////////////////////////////
void setTargetGPSPoints(long lat, long lon){
    // Take the mutex to gain exclusive access to the global variables
  if (xSemaphoreTake(xMutexSensor, portMAX_DELAY) == pdTRUE) {
    
    Target_GPS_Longitude = lon;  // Longitude
    Target_GPS_Latitude = lat;   // Latitude 

    // Give the mutex back so other tasks can use it
    xSemaphoreGive(xMutexSensor);
    
  }
  else {
    Serial.println("Failed to take mutex");
  }
}




// 
// GPS Helper Fuctions //
//

// Function to convert degrees * 10^-7 to decimal degrees
double convertToDecimal(long value) {
    return value / 10000000.0;
}

// Haversine formula to compute distance in feet
double haversineDistance(long lat1, long lon1, long lat2, long lon2) {
  // Convert coordinates to decimal degrees
  double lat1_d = convertToDecimal(lat1);
  double lon1_d = convertToDecimal(lon1);
  double lat2_d = convertToDecimal(lat2);
  double lon2_d = convertToDecimal(lon2);

  // Convert decimal degrees to radians
  double lat1_rad = lat1_d * M_PI / 180.0;
  double lon1_rad = lon1_d * M_PI / 180.0;
  double lat2_rad = lat2_d * M_PI / 180.0;
  double lon2_rad = lon2_d * M_PI / 180.0;

  // Haversine formula
  double dlat = lat2_rad - lat1_rad;
  double dlon = lon2_rad - lon1_rad;
  
  double a = pow(sin(dlat / 2), 2) + cos(lat1_rad) * cos(lat2_rad) * pow(sin(dlon / 2), 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return EARTH_RADIUS_FEET * c; // Distance in feet
}


// Calculate the initial bearing (angle) between two GPS points
double calculateBearing(long lat1, long lon1, long lat2, long lon2) {
  // Convert to decimal degrees
  double lat1_d = convertToDecimal(lat1);
  double lon1_d = convertToDecimal(lon1);
  double lat2_d = convertToDecimal(lat2);
  double lon2_d = convertToDecimal(lon2);

  // Convert to radians
  double lat1_rad = lat1_d * M_PI / 180.0;
  double lon1_rad = lon1_d * M_PI / 180.0;
  double lat2_rad = lat2_d * M_PI / 180.0;
  double lon2_rad = lon2_d * M_PI / 180.0;

  // Compute differences
  double deltaLon = lon2_rad - lon1_rad;

  // Bearing formula
  double y = sin(deltaLon) * cos(lat2_rad);
  double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(deltaLon);
  double theta = atan2(y, x); // Angle in radians

  // Convert to degrees
  double bearing = theta * RAD_TO_DEG;

  // Normalize to 0-360 degrees
  if (bearing < 0) {
      bearing += 360.0;
  }

  return bearing;
}










