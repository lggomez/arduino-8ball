#define DEBUG 0

#include <Arduino.h>
#include <WString.h>
#include <math.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <SPI.h>
#include "_const.h"
#include "src/lib/U8g2/src/U8g2lib.h"
#include "src/lib/I2Cdev.h"
#include "src/lib/MPU6050_6Axis_MotionApps20.h"

#ifdef U8X8_HAVE_HW_I2C || (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE)
#include <Wire.h>
#endif

#define INTERRUPT_PIN 2
#define LED_PIN 13

enum fizzlefade_mode { fill, clear };

/*
  -----------------FUNCTION PROTOTYPES START (bless the arduino builder)
*/
//main(ssh1106ex)
void loop(void);
long minl(long x, long y);
byte sign(long x);
long vectorNorm(VectorInt16 *vec);
void displayIdleMessage();
void displayMessage(byte index);
// fizzlefade.ino
boolean fizzlefade_message(char* charMessage, fizzlefade_mode mode);
void fizzle_message(u8g2_uint_t x, u8g2_uint_t y, char* charMessage, byte lineCount, fizzlefade_mode mode, u8g2_uint_t batch_count);
void drawFillPaged(void);
void drawFill(void);
void printMessage(char* charMessage, byte lineCount, fizzlefade_mode mode);
byte getLineCount(const char* charMessage);
char* subStr(char* str, char *delim, byte index);
//messages.ino
String getMessage(byte index);
/*
  -----------------FUNCTION PROTOTYPES END
*/

bool blinkState = false;

// Mpu instance
MPU6050 mpu;

// u8g2 constructor
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

int index;
byte accelAccum = 0;
uint16_t magnitudeThreshold = 500;
double previousMagnitude = 0;
byte shakeCount = 0;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup(void) {
  wdt_enable(WDTO_8S);
  // Join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 103; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  DEBUG_PRINTLN(F("Connected to I2C bus"));

  Serial.begin(115200);
  while (!Serial); // Wait for Leonardo enumeration, others continue immediately
  randomSeed(analogRead(0));

  DEBUG_PRINTLN(F("Initializing MPU6050 device..."));
  mpu.initialize();

  mpu.setFullScaleGyroRange(3);
  mpu.setFullScaleAccelRange(3);

  pinMode(INTERRUPT_PIN, INPUT);

  DEBUG_PRINTLN(F("Testing MPU6050 connection..."));
  DEBUG_PRINTLN(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // Load and configure the DMP
  DEBUG_PRINTLN(F("Initializing MPU DMP..."));
  devStatus = mpu.dmpInitialize();

  DEBUG_PRINTLN(F("Initializing gyro offsets"));

  // Gyro offsets
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  mpu.setSleepEnabled(0);

  // Make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Turn on the DMP
    DEBUG_PRINTLN(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // Enable Arduino interrupt detection
    DEBUG_PRINTLN(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // Set our DMP Ready flag so the main loop() function knows it's okay to use it
    DEBUG_PRINTLN(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // Get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    DEBUG_PRINT(F("DMP Initialization failed (code "));
    DEBUG_PRINT(devStatus);
    DEBUG_PRINTLN(F(")"));
  }

  DEBUG_PRINTLN(F("*Init finished"));

  pinMode(LED_PIN, OUTPUT);
  u8g2.begin();
  u8g2.setFont(FONT);
  drawFillPaged();
  displayIdleMessage();
}

void loop(void) {
  wdt_reset();
  // If programming failed, don't try to do anything
  if (!dmpReady) {
    DEBUG_PRINTLN(F("--ERROR: Setup failed. ignoring loop"));
    delay(5000);
    return;
  }

  // Reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // Get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // Check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    DEBUG_PRINTLN(F("Overflow!. Resetting FIFO buffer"));
    // Reset so we can continue cleanly
    mpu.resetFIFO();
    return;
    // Otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // Wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // Read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // Track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // Display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    long currentMagnitude = vectorNorm(&aaReal);

    // Overflow detection & stabilization
    long delta = abs(currentMagnitude - previousMagnitude);
    long margin = minl((long)abs(5 * previousMagnitude), 10000);
    bool currentSlope = sign(delta);

    if (previousMagnitude != 0.0) {
      if (delta > margin) {
#if OUTPUT_MPU_FIFO_OVERFLOW
        DEBUG_PRINT(F("curr "));
        DEBUG_PRINT(currentMagnitude);
        DEBUG_PRINT(F(", prev "));
        DEBUG_PRINT(previousMagnitude);
        DEBUG_PRINT(F(", delta "));
        DEBUG_PRINT(abs(delta));
        DEBUG_PRINT(F(", margin; "));
        DEBUG_PRINT(margin);
        DEBUG_PRINTLN(F(" - Overflow detected. Ignore iteration"));
#endif
        previousMagnitude = minl(currentMagnitude, previousMagnitude);
#if PAUSE_ON_OVERFLOW
        delay(500);
#endif
        return;
      }
    }

    // Movement accumulator for shake detection
    if (delta > magnitudeThreshold) {
      DEBUG_PRINTLN(F("ACCEL ACCUM"));
      accelAccum++;
    }

    if (accelAccum > 5) {
      DEBUG_PRINTLN(F("SHAKE"));
      shakeCount++;
      if (shakeCount > 3) {
        index = random(0, 14);
        DEBUG_PRINT(F("Display message "));
        DEBUG_PRINTLN(index);
        displayMessage(index);
        delay(REFRESH_INTERVAL * 2);
        drawFillPaged();
        displayIdleMessage();
        shakeCount = 0;
      }
      accelAccum = 0;
    }

#if OUTPUT_ACCEL_VECTORS
    DEBUG_PRINT(F("areal\t"));
    DEBUG_PRINT(aaReal.x);
    DEBUG_PRINT(F("\t"));
    DEBUG_PRINT(aaReal.y);
    DEBUG_PRINT(F("\t"));
    DEBUG_PRINT(aaReal.z);
    DEBUG_PRINT(F("\t"));
    DEBUG_PRINTLN(currentMagnitude);
#endif

    previousMagnitude = currentMagnitude;

    // Blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
#if OUTPUT_INVALID_MPU_STATE
  else {
    DEBUG_PRINT(F("unknown mpu status ("));
    DEBUG_PRINT(mpuIntStatus);
    DEBUG_PRINT(F(")"));
    DEBUG_PRINTLN(F(""));
  }
#endif
}

long vectorNorm(VectorInt16 *vec) {
  long a = (long)abs(vec->x);
  a = a * a;
  long b = (long)abs(vec->y);
  b = b * b;
  long c = (long)abs(vec->z);
  c = c * c;
  long sum = a + b + c;
  long root = sqrt(sum);
#if OUTPUT_NORMALIZED_VECTORS
  DEBUG_PRINT(F("norm\t");
              DEBUG_PRINT(a);
              DEBUG_PRINT(F("\t"));
              DEBUG_PRINT(b);
              DEBUG_PRINT(F("\t"));
              DEBUG_PRINT(c);
              DEBUG_PRINT(F("sum\t"));
              DEBUG_PRINT(sum);
              DEBUG_PRINT(F("\t"));
              DEBUG_PRINTLN(root);
#endif
              return root;
}

byte sign(long x) {
  return ((x > 0) - (x < 0));
}

long minl(long x, long y) {
  return x < y ? x : y;
}

void displayMessage(byte index) {
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

  fizzlefade_message(charMessage, fill/*random(0, 100) > 50 ? fill : clear*/);
}

void displayIdleMessage() {
  u8g2.firstPage();
  do {
    u8g2.drawStr((SH1106_WIDTH / 2) - (3 * FONT_WIDTH), (SH1106_HEIGHT / 2) - (FONT_HEIGTH / 2), "Ask me");
  } while (u8g2.nextPage());
}
