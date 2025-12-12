/* 
 * Project: Catblocks
 * Description: IoT Cohort 17 Capstone -- Smart Pet Feeder and Monitoring System
 * Author: Tim Nickell
 * Date: 2025-12-12
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
#include "Grove_Air_quality_Sensor.h"
#include "HC_SR04.h"
#include "Adafruit_PWMServoDriver.h"
#include "Colors.h"
#include "IoTTimer.h"
#include "Button.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "credentials.h"

// pin constants
const int BUTTONR = D3;
const int BUTTONL = D4;
const int TRIGPIN = D5;
const int ECHOPIN = D6;
const int SCALE_DT  = D8;
const int SCALE_CLK = D9;
const int AQSPIN = D11;
const int HALLPIN  = D14;

// setup constants
const int SERIAL_SPEED = 9600; // serial
const int SERIAL_TIMEOUT = 10000;
const int SERIAL_DELAY = 1000;
const int PIXELCOUNT = 12; // neopixels -- just the ring for now
const int BRIGHTNESS = 50;
const int RAINBOW_SIZE = sizeof(rainbow) / sizeof(rainbow[0]);
const int INITIAL_COLOR = violet;
const int OLED_RESET = -1; // OLED
const int BME280_HEX_ADDRESS = 0x76; // BME
const int CALFACTOR = 1000; // scale
const int SAMPLES = 10;
const String airQualityMessages[5] = { // AQ Sensor
  "High pollution! (Force signal active.)\n",
  "High pollution!\n",
  "Low pollution.\n",
  "Fresh air.\n",
  "Something went wrong.\n"
};
const int AQ_SENSOR_TIMEOUT = 1000;
const int PUBLISH_DELAY = 0; // MQTT
const int SUB_FREQ = 1000;
const int SERVO1 = 0; // SpringRC servo on channel 0
const int SERVO2 = 2; // Parallax servo on channel 2
const int SERVOMIN = 150;
const int SERVOMAX = 600;
const int MPU_ADDR = 0x68; // accelerometer
const int MPU_REFRESH_FREQ = 200;
const int STARTING_ADDRESS = 0x00AE; // EEPROM
const int DELAY_OFFSET = 800;
const int PIXEL_TIMER_DELAY = 1000 - DELAY_OFFSET; // Timer delays
const int OLED_TIMER_DELAY = 1000 - DELAY_OFFSET;
const int BME_TIMER_DELAY = 1000 - DELAY_OFFSET;
const int AQ_TIMER_DELAY = 1000 - DELAY_OFFSET;
const int LOAD_TIMER_DELAY = 1000 - DELAY_OFFSET;
const int ACCEL_TIMER_DELAY = 1000 - DELAY_OFFSET;
const int CLOCK_TIMER_DELAY = 1000 - DELAY_OFFSET;
const int DISTANCE_TIMER_DELAY = 1000 - DELAY_OFFSET;
const int SERVO_TIMER_DELAY = 1000 - DELAY_OFFSET;
const int PUB_TIMER_DELAY = 30000 - DELAY_OFFSET;
const int SERIAL_TIMER_DELAY = 850;

// variables (global)
int pixelColor; // neopixels
int pixelNum;
int brightness;
int color;
byte buf[6];
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
int quality; // aq sensor
int aqValue;
String aqMessage;
float pubValue; // MQTT
double cm; // scale
double inches;
int addr; // EEPROM
int addr2;
byte hour, minute, second;
byte globalred, globalgreen, globalblue;
bool servo1On; // servos
bool servo2On;
int motorPhase;
uint8_t servonum; // servos
int hallSensorVal; // hall sensor
byte accel_x_h, accel_x_l; // MPU-6050
byte accel_y_h, accel_y_l;
byte accel_z_h, accel_z_l;
int16_t accel_x, accel_y, accel_z;
float accel_x_gs, accel_y_gs, accel_z_gs;
String dateTime, timeOnly; // real-time clock
bool manualColor;
int morning[4];
int evening[4];
int mornInd;
int evenInd;
bool isMorning;
bool isFeeding;

// objects
Adafruit_NeoPixel pixel(PIXELCOUNT, SPI1, WS2812B);
Adafruit_SSD1306 display1(OLED_RESET);
Adafruit_SSD1306 display2(OLED_RESET);
Adafruit_BME280 bme; // Define BME280 object (I2C device)
HX711 myScale(SCALE_DT, SCALE_CLK);
AirQualitySensor aqSensor(AQSPIN);
HC_SR04 ultrasonicSensor = HC_SR04(TRIGPIN, ECHOPIN, 1.0, 2500.0);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Button buttonR(BUTTONR);
Button buttonL(BUTTONL);

// timers
IoTTimer pixelTimer;
IoTTimer oledTimer;
IoTTimer bmeTimer;
IoTTimer aqTimer;
IoTTimer loadTimer;
IoTTimer accelTimer;
IoTTimer clockTimer;
IoTTimer distanceTimer;
IoTTimer servoTimer;
IoTTimer serialTimer;
IoTTimer pubTimer;

// functions
void setServo(int servoNum, bool on, bool fast, bool clockwise);
void pixelFill(int startPixel, int endPixel, int hexColor);
float celsiusToFahrenheit(float celsius);
float pascalsToInHg(float pascals);
int fahrenheitToPixel(float Fahrenheit);
int inHgToBrightness(float PressInHg);
int humidityToColor(float HumidRH);
int16_t getAccelFromRegisters(byte *high, byte *low, int addr);
float convertLSBsToGs(int lsbs);
void saveToEEPROM(int addr, byte h, byte m, byte s);
void hexColorToBytes(int myColor, byte *r, byte *g, byte *b);
void MQTT_connect();
bool MQTT_ping();

// system mode
SYSTEM_MODE(SEMI_AUTOMATIC);

// global state
TCPClient TheClient;

// set up MQTT client class
Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// feeds
Adafruit_MQTT_Subscribe colorPickerFeed     = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/color-picker");
Adafruit_MQTT_Subscribe buttonFeed          = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/button-feed");
Adafruit_MQTT_Publish   scalePubFeed        = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weights");
Adafruit_MQTT_Publish   aqValuePubFeed      = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/air-quality-value-feed");
Adafruit_MQTT_Publish   aqMessagePubFeed    = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/air-quality-message-feed");
Adafruit_MQTT_Publish   temperaturePubFeed  = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bme-temperature-feed");
Adafruit_MQTT_Publish   pressurePubFeed     = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bme-pressure-feed");
Adafruit_MQTT_Publish   humidityPubFeed     = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bme-humidity-feed");
Adafruit_MQTT_Publish   distancePubFeed     = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/distance-feed");

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
  quality = 0;
  hallSensorVal = 0;
  addr = STARTING_ADDRESS;
  addr2 = STARTING_ADDRESS + 3;
  manualColor = false;

  morning[0] = 0;
  morning[1] = 6;
  morning[2] = 0;
  morning[3] = 0;
  evening[0] = 1;
  evening[1] = 8;
  evening[2] = 0;
  evening[3] = 0;

  mornInd = 0;
  evenInd = 0;
  isMorning = true;
  isFeeding = false;

  // pinModes
  pinMode(HALLPIN, INPUT);

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
  tempC = bme.readTemperature();
  pressPA = bme.readPressure();
  humidRH = bme.readHumidity();

  // 8. Set up WiFi
  WiFi.on();
  WiFi.connect();
  while (WiFi.connecting()) {
    Serial.printf(".");
    delay(1000);
    Serial.printf("\n\n");
  }

  // Set up particle cloud
  while (!Particle.connected()) {
    Particle.connect();
    delay(100);
    Serial.printf("x");
  }
  Serial.printf("\n\n");
  delay(3000);

  // set up clock
  Time.zone(-7); // MST = -7; MDT = -6
  Particle.syncTime(); // sync time with particle cloud

  // 11. set up scale
  myScale.set_scale();          // initialize loadcell
  delay(5000);                  // let the loadcell settle
  myScale.tare();               // set the tare weight (or zero)
  myScale.set_scale(CALFACTOR); // adjust when calibrating scale to desired units

  // 11. set up air quality sensor
  Serial.printf("Waiting for aq sensor to init...\n");
  delay(AQ_SENSOR_TIMEOUT);
  if (aqSensor.init()) {
    Serial.printf("AQ Sensor ready.\n");
  }
  else {
    Serial.printf("AQ Sensor ERROR.\n");
  }

  // 12. Set up MQTT
  mqtt.subscribe(&colorPickerFeed);
  mqtt.subscribe(&buttonFeed);

  // 12. Retrieve time info from EEPROM
  addr = STARTING_ADDRESS;
  hour   = EEPROM.read(addr);
  minute = EEPROM.read(addr + 1);
  second  = EEPROM.read(addr + 2);

  // 13. set up MPU-6050
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Select and write to PWR_MGMT_1 register
  Wire.write(0x00); // wakes up MPU-6050
  Wire.endTransmission(true); // end transmission and close connection

  // 13. set up servos
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  // 17. set up timers
  pixelTimer.startTimer(0);
  oledTimer.startTimer(0);
  bmeTimer.startTimer(0);
  aqTimer.startTimer(0);
  loadTimer.startTimer(0);
  accelTimer.startTimer(0);
  distanceTimer.startTimer(0);
  servoTimer.startTimer(0);
  serialTimer.startTimer(0);
  pubTimer.startTimer(0);
}


// loop() 
void loop() {

  // First first: perform Adafruit subscription stuff
  MQTT_connect();
  MQTT_ping();
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(SUB_FREQ))) {
    if (subscription == &colorPickerFeed) {
      Serial.printf("Received from Adafruit: %s \n", (char *) colorPickerFeed.lastread);

      memcpy(buf, &colorPickerFeed.lastread[1], 6); // strip off the '#' and copy into buf
      Serial.printf("Buffer: %s \n", (char *)buf);
      color = strtol((char *)buf, NULL, 16); // converts string to int (hex, base 16)
      Serial.printf("Buffer: 0x%02X \n", color);

      pixelFill(0, PIXELCOUNT - 1, color);

      byte bytered, bytegreen, byteblue;
      hexColorToBytes(color, &bytered, &bytegreen, &byteblue);      
      // store values in eeprom
      saveToEEPROM(addr, bytered, bytegreen, byteblue);

      manualColor = true;
    }
    if (subscription == &buttonFeed) {
      Serial.printf("Received from Adafruit: %s \n", (char *) buttonFeed.lastread);

      // Numpad Code
      if (((char *) buttonFeed.lastread)[0] == '*') { // morning
        isMorning = true;
        mornInd = 0;
      }
      else if (((char *) buttonFeed.lastread)[0] == '#') { // morning
        isMorning = false;
        evenInd = 0;
      }
      else {
        if (isMorning) {
          morning[mornInd++] = ((char *) buttonFeed.lastread)[0];
          if (mornInd > 3) {
            mornInd = 0;
          }
        }
        else {
          evening[evenInd++] = ((char *) buttonFeed.lastread)[0];
          if (evenInd > 3) {
            evenInd = 0;
          }
        }
      }

      // Momentary Button Code
      int subValue = atoi((char *)buttonFeed.lastread);
      if (subValue == 1) {
        setServo(SERVO2, true, true, true);
      }
      else {
        setServo(SERVO2, false, true, true);
      }
    }
  }

  for (int i = 0; i < 4; i++) {
    Serial.printf("%d\n", morning[i]);
  }

  // first, get hall sensor reading for this loop
  hallSensorVal = digitalRead(HALLPIN);

  // Servo Buttons
    if (buttonL.isPressed()) {
      setServo(SERVO1, true, true, true);
    }
    else {
      setServo(SERVO1, false, true, true);
    }
    if (buttonR.isPressed()) {
      setServo(SERVO2, true, true, true);
    }
    else {
      setServo(SERVO2, false, true, true);
    }

  // 1. ultrasonic sensor
  if (distanceTimer.isTimerReady()) {

    if (hallSensorVal == 0) { // if the lid is closed...
      inches = ultrasonicSensor.getDistanceInch();

      // TODO: alert logic goes here
    }
    distanceTimer.startTimer(DISTANCE_TIMER_DELAY);
  }

  // 2. servo driver
  if (servoTimer.isTimerReady()) {

    servoTimer.startTimer(SERVO_TIMER_DELAY);
  }

  // 3. OLED Stuff
  if (oledTimer.isTimerReady()) {

    display1.clearDisplay();
    display1.setRotation(0);
    display1.setTextSize(1);
    display1.setTextColor(WHITE);
    display1.setCursor(0,1);
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

    if (!manualColor) {

      pixelFill(pixelNum, pixelNum, rainbow[pixelColor]);
      pixelNum++;
      if (pixelNum > PIXELCOUNT - 1) {
        pixelNum = 0;
      }
      pixelColor++;
      if (pixelColor > RAINBOW_SIZE - 1) {
        pixelColor = 0;
      }

    }

    pixelTimer.startTimer(PIXEL_TIMER_DELAY);
  }

  // 5. Load Cell
  if (loadTimer.isTimerReady()) {
    weight = myScale.get_units(SAMPLES); // return weight in units set by set_scale()
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

    bmeTimer.startTimer(BME_TIMER_DELAY);
  }

  // 7. AQ Sensor
  if (aqTimer.isTimerReady()) {

    quality = aqSensor.slope();
    aqValue = aqSensor.getValue();

    if (quality == AirQualitySensor::FORCE_SIGNAL) {
      aqMessage = airQualityMessages[0];
    }
    else if (quality == AirQualitySensor::HIGH_POLLUTION) {
      aqMessage = airQualityMessages[1];
    }
    else if (quality == AirQualitySensor::LOW_POLLUTION) {
      aqMessage = airQualityMessages[2];
    }
    else if (quality == AirQualitySensor::FRESH_AIR) {
      aqMessage = airQualityMessages[3];
    }
    else {
      aqMessage = airQualityMessages[4];
    }

    aqTimer.startTimer(AQ_TIMER_DELAY);
  }

  // 8. Accelerometer (MPU-6050)
  if (accelTimer.isTimerReady()) {

    // Request and then read 2 bytes
    Wire.requestFrom(MPU_ADDR, 2, true);
    accel_x = getAccelFromRegisters(&accel_x_h, &accel_x_l, 0x3B);
    accel_y = getAccelFromRegisters(&accel_y_h, &accel_y_l, 0x3D);
    accel_z = getAccelFromRegisters(&accel_z_h, &accel_z_l, 0x3F);

    accel_x_gs = convertLSBsToGs(accel_x);
    accel_y_gs = convertLSBsToGs(accel_y);
    accel_z_gs = convertLSBsToGs(accel_z);

    accelTimer.startTimer(MPU_REFRESH_FREQ);
  }

  // Serial Print
  if (serialTimer.isTimerReady()) {

    dateTime = Time.timeStr();
    timeOnly = dateTime.substring(11, 19);

    Serial.printf(
      "Distance in in: %0.2f\n"
      "Weight (in grams) --> %0.2f\n"
      "tempF: %0.2f\npressInHg: %0.2f\nhumidRH: %0.2f\n\n"
      "aqSensor value: %d\n"
      "%s\n\n"
      "x-axis accel: %0.6f \ny-axis accel: %0.6f \nz-axis accel: %0.6f \n\n"
      ,
      inches, weight, tempF, pressInHg, humidRH, aqValue, aqMessage.c_str(), accel_x_gs, accel_y_gs, accel_z_gs
    );

    Serial.printf("Date and time is %s\n", dateTime.c_str());
    Serial.printf("Time is %s\n\n", timeOnly.c_str());

    serialTimer.startTimer(SERIAL_TIMER_DELAY);
  }

  // Publish to MQTT/Adafruit
  if (pubTimer.isTimerReady()) {

    temperaturePubFeed.publish(tempF);
    pressurePubFeed.publish(pressInHg);
    humidityPubFeed.publish(humidRH);
    scalePubFeed.publish(weight);
    aqValuePubFeed.publish(aqValue);
    aqMessagePubFeed.publish(String::format("%s\n", aqMessage.c_str()).c_str());
    distancePubFeed.publish(8.0 - inches);

    pubTimer.startTimer(PUB_TIMER_DELAY);
  }
}


// functions

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}


// ping MQTT
bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
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

int16_t getAccelFromRegisters(byte *high, byte *low, int addr) {

  // set the pointer to the memory location at addr of the MPU and wait for data
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(addr); // starting with register 0x3B
  Wire.endTransmission(false); // send the set pointer command and keep active

  // Request and then read 2 bytes
  Wire.requestFrom(MPU_ADDR, 2, true);
  byte accel_h = Wire.read();
  byte accel_l = Wire.read();
  
  *high = accel_h;
  *low = accel_l;
  return accel_h << 8 | accel_l;
}

float convertLSBsToGs(int lsbs) {
  return lsbs / 16384.0;
}

void saveToEEPROM(int addr, byte h, byte m, byte s) {
  EEPROM.write(addr, h);
  EEPROM.write(addr + 1, m);
  EEPROM.write(addr + 2, s);
}

void hexColorToBytes(int myColor, byte *r, byte *g, byte *b) {
    *r = color >> 16 & 0x0000FF;
    *g = color >> 8  & 0x0000FF;
    *b = color       & 0x0000FF;
}