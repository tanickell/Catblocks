/* 
 * Project: Catblocks
 * Description: IoT Cohort 17 Capstone -- Smart Pet Feeder and Monitoring System
 * Author: Tim Nickell
 * Date: 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "HC_SR04.h"

// Pins
const int TRIGPIN = D5;
const int ECHOPIN = D6;

double cm;
double inches;


// Objects
HC_SR04 ultrasonicSensor = HC_SR04(TRIGPIN, ECHOPIN, 1.0, 2500.0);


// System Mode
SYSTEM_MODE(SEMI_AUTOMATIC);


// setup() 
void setup() {

  // initialize variables
  cm = 0.0;
  inches = 0.0;

  // set up serial monitor
  Serial.begin(9600);
  waitFor(Serial.isConnected, 10000);
  delay(1000);
  Serial.printf("Ready to go!\n\n");

  // set up ultrasonic sensor
  Particle.variable("cm", &cm, DOUBLE);
  Particle.variable("inches", &inches, DOUBLE);
}


// loop() 
void loop() {
  cm = ultrasonicSensor.getDistanceCM();
  delay(100);
  inches = ultrasonicSensor.getDistanceInch();
  delay(100);
  Serial.printf("Distance in cm: %0.2f  |  Distance in in: %0.2f\n", cm, inches);
  delay(1000);
}
