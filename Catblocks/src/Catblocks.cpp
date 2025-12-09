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
#include "Adafruit_PWMServoDriver.h"
#include "IoTTimer.h"

// Pins
const int TRIGPIN = D5;
const int ECHOPIN = D6;

// Setup
const int SERIAL_SPEED = 9600;
const int SERIAL_TIMEOUT = 10000;
const int SERIAL_DELAY = 1000;

const int SERVO1 = 0; // SpringRC servo on channel 0
const int SERVO2 = 2; // Parallax servo on channel 2
const int SERVOMIN = 150;
const int SERVOMAX = 600;

double cm;
double inches;

bool servo1On;
bool servo2On;
int motorPhase;

uint8_t servonum;


// Objects
HC_SR04 ultrasonicSensor = HC_SR04(TRIGPIN, ECHOPIN, 1.0, 2500.0);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
IoTTimer distanceTimer;
IoTTimer servoTimer;

// functions
void setServo(int servoNum, bool on, bool fast, bool clockwise);

// System Mode
SYSTEM_MODE(SEMI_AUTOMATIC);


// setup() 
void setup() {

  // initialize variables
  cm = 0.0;     // HC-SR04 (Ultrasonic Sensor)
  inches = 0.0;
  servonum = 0; // initialize servo in channel 0 (first channel)
  servo1On = false;
  servo2On = false;
  motorPhase = 0;

  // set up serial monitor
  Serial.begin(SERIAL_SPEED);
  waitFor(Serial.isConnected, SERIAL_TIMEOUT);
  delay(SERIAL_DELAY);
  Serial.printf("Ready to go!\n\n");

  // set up servos
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  // set up timers
  distanceTimer.startTimer(0);
  servoTimer.startTimer(0);
}


// loop() 
void loop() {

  // 1. ultrasonic sensor
  if (distanceTimer.isTimerReady()) {
    inches = ultrasonicSensor.getDistanceInch();
    Serial.printf("Distance in in: %0.2f\n", inches);
    distanceTimer.startTimer(1000);
  }

  // 2. servo driver
  if (servoTimer.isTimerReady()) {

    if (motorPhase == 0) {
      setServo(SERVO1, true, false, true);
      setServo(SERVO2, true, false, true);
    }
    if (motorPhase == 1) {
      setServo(SERVO1, true, true, true);
      setServo(SERVO2, true, true, true);
    }
    if (motorPhase == 2) {
      setServo(SERVO1, true, false, true);
      setServo(SERVO2, true, false, true);
    }
    if (motorPhase == 3) {
      setServo(SERVO1, false, true, true);
      setServo(SERVO2, false, true, true);
    }
    if (motorPhase == 4) {
      setServo(SERVO1, true, false, false);
      setServo(SERVO2, true, false, false);
    }    
    if (motorPhase == 5) {
      setServo(SERVO1, true, true, false);
      setServo(SERVO2, true, true, false);
    }
    if (motorPhase == 6) {
      setServo(SERVO1, true, false, false);
      setServo(SERVO2, true, false, false);
    }
    if (motorPhase == 7) {
      setServo(SERVO1, false, true, true);
      setServo(SERVO2, false, true, true);
    }
    motorPhase++;
    if (motorPhase > 7) {
      motorPhase = 0;
    }

    servoTimer.startTimer(1000);
  }
}

// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void setServo(int servoNum, bool on, bool fast, bool clockwise) {
  if (on) {
    if (servoNum == 0) {
      if (fast && clockwise) {
        pwm.setPWM(servoNum, 0, 300);
      }
      if (!fast && clockwise) {
        pwm.setPWM(servoNum, 0, 320);
      }
      if (!fast && !clockwise) {
        pwm.setPWM(servoNum, 0, 360);
      }
      if (fast && ! clockwise) {
        pwm.setPWM(servoNum, 0, 380);
      }
    }
    if (servoNum == 2) {
      if (fast && clockwise) {
        pwm.setPWM(servoNum, 0, 330);
      }
      if (!fast && clockwise) {
        pwm.setPWM(servoNum, 0, 335);
      }
      if (!fast && !clockwise) {
        pwm.setPWM(servoNum, 0, 345);
      }
      if (fast && ! clockwise) {
        pwm.setPWM(servoNum, 0, 350);
      }
    }
  }
  else {
    pwm.setPWM(servoNum, 0, 340);
  }
}