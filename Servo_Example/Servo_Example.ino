/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald

 modified for the ESP32 on March 2017
 by John Bennett
 
 see  http://www.arduino.cc/en/Tutorial/Knob for a description of the original code

 * Different servos require different pulse widths to vary servo angle, but the range is 
 * an approximately 500-2500 microsecond pulse every 20ms (50Hz). In general, hobbyist servos
 * sweep 180 degrees, so the lowest number in the published range for a particular servo
 * represents an angle of 0 degrees, the middle of the range represents 90 degrees, and the top
 * of the range represents 180 degrees. So for example, if the range is 1000us to 2000us,
 * 1000us would equal an angle of 0, 1500us would equal 90 degrees, and 2000us would equal 1800
 * degrees.
 * 
 * Circuit: (using an ESP32 Thing from Sparkfun)
 * Servo motors have three wires: power, ground, and signal. The power wire is typically red,
 * the ground wire is typically black or brown, and the signal wire is typically yellow,
 * orange or white. Since the ESP32 can supply limited current at only 3.3V, and servos draw
 * considerable power, we will connect servo power to the VBat pin of the ESP32 (located
 * near the USB connector). THIS IS ONLY APPROPRIATE FOR SMALL SERVOS. 
 * 
 * We could also connect servo power to a separate external
 * power source (as long as we connect all of the grounds (ESP32, servo, and external power).
 * In this example, we just connect ESP32 ground to servo ground. The servo signal pins
 * connect to any available GPIO pins on the ESP32 (in this example, we use pin 18.
 * 
 * In this example, we assume a Tower Pro SG90 small servo connected to VBat.
 * The published min and max for this servo are 500 and 2400, respectively.
 * These values actually drive the servos a little past 0 and 180, so
 * if you are particular, adjust the min and max values to match your needs.
 */

// Include the ESP32 Arduino Servo Library instead of the original Arduino Servo Library
#include <ESP32Servo.h> 

Servo motor1;  // create servo object to control a servo
Servo motor2; 

int minUs = 1000;
int maxUs = 2000;

int motorpin1 = 15;      // GPIO pin used to connect the servo control (digital out) 
int motorpin2 = 4;      // GPIO pin used to connect the servo control (digital out)  

void setup()
{
  Serial.begin(115200);

  motor1.setPeriodHertz(50);// Standard 50hz servo
  motor2.setPeriodHertz(50);// Standard 50hz servo

  motor1.attach(motorpin1, minUs, maxUs);   // attaches the servo on pin 18 to the servo object
  motor2.attach(motorpin2, minUs, maxUs);   // attaches the servo on pin 18 to the servo object
  

  Serial.println("StopTime");
  motor1.write(90);                  // set the servo position according to the scaled value
  motor2.write(90);                  // set the servo position according to the scaled value

  delay(20000); 

  Serial.println("GoTime");
  motor1.write(110);
  motor2.write(110);                  // set the servo position according to the scaled value

}

void loop() {
 
}
