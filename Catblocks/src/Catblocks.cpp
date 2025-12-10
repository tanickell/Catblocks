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
#include "neopixel.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"
#include "Adafruit_BME280.h"
#include "HX711.h"
#include "HC_SR04.h"
#include "Adafruit_PWMServoDriver.h"
#include "Colors.h"
#include "IoTTimer.h"


// pin constants
const int TRIGPIN = D5;
const int ECHOPIN = D6;

const int SCALE_DT  = D8;
const int SCALE_CLK = D9;

// setup constants
const int SERIAL_SPEED = 9600;
const int SERIAL_TIMEOUT = 10000;
const int SERIAL_DELAY = 1000;

const int PIXELCOUNT = 12; // just the ring for now
const int BRIGHTNESS = 50;
const int RAINBOW_SIZE = sizeof(rainbow) / sizeof(rainbow[0]);
const int INITIAL_COLOR = violet;

const int OLED_RESET = -1;

const int BME280_HEX_ADDRESS = 0x76;

const int CALFACTOR = 425; // scale
const int SAMPLES = 10;

const int PUBLISH_DELAY = 0; // MQTT

const int SERVO1 = 0; // SpringRC servo on channel 0
const int SERVO2 = 2; // Parallax servo on channel 2
const int SERVOMIN = 150;
const int SERVOMAX = 600;

const int DELAY_OFFSET = 800;

const int PIXEL_TIMER_DELAY = 1000 - DELAY_OFFSET; // Timer delays
const int OLED_TIMER_DELAY = 1000 - DELAY_OFFSET;
const int BME_TIMER_DELAY = 1000 - DELAY_OFFSET;
const int LOAD_TIMER_DELAY = 1000 - DELAY_OFFSET;
const int DISTANCE_TIMER_DELAY = 1000 - DELAY_OFFSET;
const int SERVO_TIMER_DELAY = 1000 - DELAY_OFFSET;

// constants

// variables (global)
int pixelColor; // neopixels
int pixelNum;
int brightness;

bool status;
float tempC;
float pressPA;
float humidRH;
float tempF;
float pressInHg;

float weight; // scale
float rawData;
float calibration;
int offset;
unsigned int last;

float pubValue; // MQTT

double cm;
double inches;
bool servo1On;
bool servo2On;
int motorPhase;

uint8_t servonum;

// objects
Adafruit_NeoPixel pixel(PIXELCOUNT, SPI1, WS2812B);
Adafruit_SSD1306 display1(OLED_RESET);
Adafruit_SSD1306 display2(OLED_RESET);
Adafruit_BME280 bme; // Define BME280 object (I2C device)
HX711 myScale(SCALE_DT, SCALE_CLK);
HC_SR04 ultrasonicSensor = HC_SR04(TRIGPIN, ECHOPIN, 1.0, 2500.0);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

IoTTimer pixelTimer;
IoTTimer oledTimer;
IoTTimer bmeTimer;
IoTTimer loadTimer;
IoTTimer distanceTimer;
IoTTimer servoTimer;

// functions
void setServo(int servoNum, bool on, bool fast, bool clockwise);
void pixelFill(int startPixel, int endPixel, int hexColor);

float celsiusToFahrenheit(float celsius);
float pascalsToInHg(float pascals);
int fahrenheitToPixel(float Fahrenheit);
int inHgToBrightness(float PressInHg);
int humidityToColor(float HumidRH);

// system mode
SYSTEM_MODE(SEMI_AUTOMATIC);


// setup() 
void setup() {

  // 1. initialize global variables
  pixelColor = 0;
  pixelNum = 0;
  brightness = BRIGHTNESS;
  cm = 0.0;     // HC-SR04 (Ultrasonic Sensor)
  inches = 0.0;
  servonum = 0; // initialize servo in channel 0 (first channel)
  servo1On = false;
  servo2On = false;
  motorPhase = 0;

  // 2. set up serial monitor
  Serial.begin(SERIAL_SPEED);
  waitFor(Serial.isConnected, SERIAL_TIMEOUT);
  delay(SERIAL_DELAY);
  Serial.printf("Ready to go!\n\n");

  // 4. set up neopixels
  pixelColor = 0;
  pixel.begin();
  pixel.setBrightness(brightness);
  pixel.show();
  pixelFill(0, PIXELCOUNT, INITIAL_COLOR);

  // 7. initialize OLED display + splash
  display1.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display2.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  display1.display();
  display2.display();
  delay(3000);
  display1.clearDisplay();
  display2.clearDisplay();

  // 7. Initialize BME280
  status = bme.begin(BME280_HEX_ADDRESS);
  if (!status) {
    Serial.printf("BME280 at addess 0x%02X failed to start.", BME280_HEX_ADDRESS);
  }

  // initialize temp, humidity, and pressure variables
  tempC = bme.readTemperature();
  pressPA = bme.readPressure();
  humidRH = bme.readHumidity();


  // 11. set up scale
  myScale.set_scale();          // initialize loadcell
  delay(5000);                  // let the loadcell settle
  myScale.tare();               // set the tare weight (or zero)
  myScale.set_scale(CALFACTOR); // adjust when calibrating scale to desired units

  // 13. set up servos
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  // 17. set up timers
  pixelTimer.startTimer(0);
  oledTimer.startTimer(0);
  bmeTimer.startTimer(0);
  loadTimer.startTimer(0);
  distanceTimer.startTimer(0);
  servoTimer.startTimer(0);
}


// loop() 
void loop() {

  // 1. ultrasonic sensor
  if (distanceTimer.isTimerReady()) {
    inches = ultrasonicSensor.getDistanceInch();
    Serial.printf("Distance in in: %0.2f\n", inches);
    distanceTimer.startTimer(DISTANCE_TIMER_DELAY);
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

    servoTimer.startTimer(SERVO_TIMER_DELAY);
  }

  // 3. OLED Stuff
  if (oledTimer.isTimerReady()) {

    display1.clearDisplay();
    display1.setRotation(0);
    display1.setTextSize(1);
    display1.setTextColor(WHITE);
    display1.setCursor(0,0);
    display1.printf("Dist:   %0.2f in\n", inches);
    display1.printf("Weight: %0.2f grams\n", weight);
    display1.display();

    display2.clearDisplay();
    display2.setRotation(0);
    display2.setTextSize(1);
    display2.setTextColor(WHITE);
    display2.setCursor(0,0);
    display2.printf("tempF\n%0.2f\n\npressInHg\n%0.2f\n\nhumidRH\n%0.2f\n\n", 
      tempF, pressInHg, humidRH);
    display2.display();

    oledTimer.startTimer(OLED_TIMER_DELAY);
  }

  // 4. NeoPixels
  if (pixelTimer.isTimerReady()) {

    pixelFill(pixelNum, pixelNum, rainbow[pixelColor]);
    pixelNum++;
    if (pixelNum > PIXELCOUNT - 1) {
      pixelNum = 0;
    }
    pixelColor++;
    if (pixelColor > RAINBOW_SIZE - 1) {
      pixelColor = 0;
    }

    pixelTimer.startTimer(PIXEL_TIMER_DELAY);
  }

  // 5. Load Cell
  if (loadTimer.isTimerReady()) {

    weight = myScale.get_units(SAMPLES); // return weight in units set by set_scale()
    Serial.printf("Weight (in grams) --> %0.2f\n", weight);

    loadTimer.startTimer(LOAD_TIMER_DELAY);
  }

  // 6. BME
  if (bmeTimer.isTimerReady()) {

    // get initial readings from BME280
    tempC = bme.readTemperature();
    pressPA = bme.readPressure();
    humidRH = bme.readHumidity();

    // convert sensor readings for temp and pressure to imperial units for display
    tempF = celsiusToFahrenheit(tempC);
    pressInHg = pascalsToInHg(pressPA);

    // print temp, pressure, and humidity to serial monitor
    Serial.printf("tempF: %0.2f\npressInHg: %0.2f\nhumidRH: %0.2f\n\n", 
      tempF, pressInHg, humidRH);


    bmeTimer.startTimer(BME_TIMER_DELAY);
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

void pixelFill(int startPixel, int endPixel, int hexColor) {
  pixel.clear();
  for (int i = startPixel; i <= endPixel; i++) {
    pixel.setPixelColor(i, hexColor);
  }
  pixel.show();
}

float celsiusToFahrenheit(float celsius) {
  return (9.0 / 5.0) * celsius + 32;
}

float pascalsToInHg(float pascals) {
  return (1.0 / 3386.39) * pascals;
}

int fahrenheitToPixel(float Fahrenheit) {
  return (45.0 / 135.0) * Fahrenheit;
}

int inHgToBrightness(float PressInHg) {
  return (255.0 / 10.0) * PressInHg - 510;
}

int humidityToColor(float HumidRH) {
  return rainbow[(int)(HumidRH / (100.0 / 6.0))];
}