

#include <Wire.h> //Needed for I2C to GNSS
#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
#include <math.h>

#define EARTH_RADIUS_FEET 20902230.0 // Earth's radius in feet
#define RAD_TO_DEG (180.0 / M_PI) // Convert radians to degrees


SFE_UBLOX_GNSS myGNSS;
long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.

// Example Point Based sitting outside on the porch
// Lab:  Lat: 423493889 Long: -711059071 (degrees * 10^-7)
// Park: Lat: 423514134 Long: -711007750
// Parking Log: Lat: 423652517 Long: -711366268
long exLat =  423652517;
long exLon = -711366268;


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



void setup()
{
  Serial.begin(115200);
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
}

void loop()
{
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer
    
    long latitude = myGNSS.getLatitude();
    // Serial.print(F("Lat: "));
    // Serial.print(latitude);

    long longitude = myGNSS.getLongitude();
    // Serial.print(F(" Long: "));
    // Serial.print(longitude);

    // Viewing Format Start
    double distance = haversineDistance(latitude, longitude, exLat, exLon);
    // Serial.print(" Distance: ");
    // Serial.print(distance);
    // Serial.print(" ");


    double angle = calculateBearing(latitude, longitude, exLat, exLon);
    // Serial.print(" Bearing: ");
    // Serial.print(angle);
    // Serial.println(" degrees");

    // Serial.println();
    // Viewing Format End

    //// Monitoring Format Start
    Serial.print(latitude);
    Serial.print(" ");

    Serial.print(longitude);
    Serial.print(" ");

    Serial.print(angle);
    Serial.print(" ");

    Serial.println(distance);
    Serial.print(" ");

    // Serial.println(angle);
    // Monitoring Format End
  }
}