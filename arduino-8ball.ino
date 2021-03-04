//#define DEBUG 1

#include <Arduino.h>
#include <WString.h>
#include <math.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <SPI.h>
#include "_const.h"
#include <U8g2lib.h> // Library: U8g2 by oliver Version 2.28.10
#include "I2Cdev.h"
#include "MPU6050.h" // Library: MPU6050 by Electronic Cats Version0.2.1

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define INTERRUPT_PIN 2
#define LED_PIN 13

enum fizzlefade_mode { fill, clear };

/*
  -----------------FUNCTION PROTOTYPES START (bless the arduino builder)
*/
void loop(void);
void complementaryFilter(int16_t accData[3], int16_t gyrData[3], float *pitch, float *roll);
void displayIdleMessage();
void displayMessage(long index);
// fizzlefade.ino
boolean fizzlefade_message(char* charMessage, fizzlefade_mode mode);
void fizzle_message(u8g2_uint_t x, u8g2_uint_t y, char* charMessage, byte lineCount, fizzlefade_mode mode, u8g2_uint_t batch_count);
void drawFillPaged(void);
void drawFill(void);
void printMessage(char* charMessage, byte lineCount, fizzlefade_mode mode);
byte getLineCount(const char* charMessage);
char* subStr(char* str, char *delim, byte index);
//messages.ino
String getMessage(long index);
//random.ino
void reseedRandom(uint32_t* address);
void reseedRandomInit(uint32_t* address, uint32_t value);
inline void reseedRandomInit(unsigned short address, uint32_t value);
/*
  -----------------FUNCTION PROTOTYPES END
*/

bool blinkState = false;

// Mpu instance
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

// u8g2 constructor
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);

#define ACCELEROMETER_SENSITIVITY 8192.0
#define GYROSCOPE_SENSITIVITY 65.536
#define dt 0.01 // 10 ms sample rate!
#define ROLL_THRESHOLD 2.5
#define PITCH_THRESHOLD 2.0
#define TIME_THRESHOLD 3000
#define SHAKE_COUNT_TRIGGER 3

int messageIndex;
int sleepDelay = 1000 * dt;

unsigned int timeCount = 0;
unsigned short accelCount = 0;
unsigned short shakeCount = 0;
unsigned short currentShakes = 0;

unsigned long startTime = millis();
unsigned long elapsedTime = 0;

uint32_t reseedRandomSeed EEMEM = 0xFFFFFFFF;
void setup(void) {
  wdt_enable(WDTO_8S);
  reseedRandom(&reseedRandomSeed);

  // Join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 103; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(38400);

  DEBUG_PRINTLN();
  DEBUG_PRINTLN(F("Setup start"));

  DEBUG_PRINTLN(F("Initializing MPU6050"));
  accelgyro.initialize();
  if (!accelgyro.testConnection()) {
    DEBUG_PRINTLN("Failed to initialize MPU6050 chip");
    while (1) {
      delay(1000);
    }
  }

  DEBUG_PRINT(accelgyro.getXAccelOffset()); DEBUG_PRINT("\t"); // -76
  DEBUG_PRINT(accelgyro.getYAccelOffset()); DEBUG_PRINT("\t"); // -2359
  DEBUG_PRINT(accelgyro.getZAccelOffset()); DEBUG_PRINT("\t"); // 1688
  DEBUG_PRINT(accelgyro.getXGyroOffset()); DEBUG_PRINT("\t"); // 0
  DEBUG_PRINT(accelgyro.getYGyroOffset()); DEBUG_PRINT("\t"); // 0
  DEBUG_PRINT(accelgyro.getZGyroOffset()); DEBUG_PRINT("\t"); // 0
  DEBUG_PRINT("\n");
  accelgyro.setXGyroOffset(220);
  accelgyro.setYGyroOffset(76);
  accelgyro.setZGyroOffset(-85);
  DEBUG_PRINT(accelgyro.getXAccelOffset()); DEBUG_PRINT("\t"); // -76
  DEBUG_PRINT(accelgyro.getYAccelOffset()); DEBUG_PRINT("\t"); // -2359
  DEBUG_PRINT(accelgyro.getZAccelOffset()); DEBUG_PRINT("\t"); // 1688
  DEBUG_PRINT(accelgyro.getXGyroOffset()); DEBUG_PRINT("\t"); // 0
  DEBUG_PRINT(accelgyro.getYGyroOffset()); DEBUG_PRINT("\t"); // 0
  DEBUG_PRINT(accelgyro.getZGyroOffset()); DEBUG_PRINT("\t"); // 0
  DEBUG_PRINT("\n");

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  DEBUG_PRINTLN(F("Initializing u8g2 driver"));
  u8g2.setI2CAddress(0x3c << 1);
  u8g2.begin();
  u8g2.setPowerSave(0);
  u8g2.setFont(FONT);

  DEBUG_PRINTLN(F("Setup finished"));

  drawFillPaged();
  displayIdleMessage();
}

void resetCounters() {
  accelCount = 0;
  shakeCount = 0;
  startTime = millis();
}

void loop(void) {
  wdt_reset();


  // Reset counters if enough time has passed
  if (elapsedTime >= TIME_THRESHOLD) {
    DEBUG_PRINT("resetting after idle:\t");
    DEBUG_PRINT(elapsedTime);
    delay(500);
    resetCounters();
  }

  // Read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  int16_t accelData[3] = {ax, ay, az};
  int16_t gyroData[3] = {gx, gy, gz};

  // Calculate updated pitch and roll from current accel & gyro data
  float pval = 0.0;
  float* pitch = &pval;
  float rval = 0.0;
  float* roll = &rval;
  complementaryFilter(accelData, gyroData, pitch, roll);

#if OUTPUT_DISPLAY_MESSAGE
  DEBUG_PRINT("a/g/pitch/roll:\t");
  DEBUG_PRINT(ax); DEBUG_PRINT("\t");
  DEBUG_PRINT(ay); DEBUG_PRINT("\t");
  DEBUG_PRINT(az); DEBUG_PRINT("\t");
  DEBUG_PRINT(gx); DEBUG_PRINT("\t");
  DEBUG_PRINT(gy); DEBUG_PRINT("\t");
  DEBUG_PRINT(gz); DEBUG_PRINT("\t");
  DEBUG_PRINT(*pitch); DEBUG_PRINT("\t");
  DEBUG_PRINTLN(*roll);
#endif

  if ((*roll >= ROLL_THRESHOLD) && (*pitch >= PITCH_THRESHOLD)) {
    shakeCount++;
  }

  currentShakes = accelCount % 2;
  accelCount -= accelCount % 2;
  shakeCount += currentShakes;

  if (shakeCount == SHAKE_COUNT_TRIGGER) {
    DEBUG_PRINTLN(F("SHAKE"));

    messageIndex = random(0, 14);
    DEBUG_PRINT(F("Display message "));
    DEBUG_PRINTLN(messageIndex);
    displayMessage(messageIndex);
    delay(REFRESH_INTERVAL * 2);
    drawFillPaged();
    displayIdleMessage();
    resetCounters();
  }

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
  delay(sleepDelay); // calculate delay from sample rate

  // Calculate elapsed time and increment counter
  elapsedTime = millis() - startTime;
  timeCount += elapsedTime;
}

// complementaryFilterr reduces the IMU input into a 2-dimensional output (pitch, roll)
// Credit: Pieter-Jan Van de Maele: https://www.pieter-jan.com/node/11
void complementaryFilter(int16_t accData[3], int16_t gyrData[3], float *pitch, float *roll)
{
  float pitchAcc, rollAcc;

  // Integrate the gyroscope data -> int(angularSpeed) = angle
  *pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
  *roll -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis

  // Compensate for drift with accelerometer data if !bullshit
  // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
  int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
  if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
  {
    // Turning around the X axis results in a vector on the Y-axis
    pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
    *pitch = *pitch * 0.98 + pitchAcc * 0.02;

    // Turning around the Y axis results in a vector on the X-axis
    rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
    *roll = *roll * 0.98 + rollAcc * 0.02;
  }
}

void displayMessage(long index) {
  String randomMessage = getMessage(index);

  char charMessage[MAX_MESSAGE_LENGTH];
  randomMessage.toCharArray(charMessage, randomMessage.length() + 1);

#if OUTPUT_DISPLAY_MESSAGE
  DEBUG_PRINTLN(F("Random index picked: "));
  DEBUG_PRINT(index);
  DEBUG_PRINTLN(F(""));
  DEBUG_PRINT(F("Message is: ["));
  DEBUG_PRINT(charMessage);
  DEBUG_PRINTLN(F("]"));
#endif

  fizzlefade_message(charMessage, random(0, 100) > 50 ? fill : clear);
}

void displayIdleMessage() {
  u8g2.firstPage();
  do {
    u8g2.drawStr((SH1106_WIDTH / 2) - (3 * FONT_WIDTH), (SH1106_HEIGHT / 2) - (FONT_HEIGTH / 2), "Ask me");
  } while (u8g2.nextPage());
}
