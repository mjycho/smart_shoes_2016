{\rtf1\ansi\ansicpg1252\cocoartf1671\cocoasubrtf600
{\fonttbl\f0\fmodern\fcharset0 Courier;}
{\colortbl;\red255\green255\blue255;}
{\*\expandedcolortbl;;}
\margl1440\margr1440\vieww10800\viewh8400\viewkind0
\deftab720
\pard\pardeftab720\ri0\partightenfactor0

\f0\fs18 \cf0 #include "I2Cdev.h"\
#include <SoftwareSerial.h>// import the serial library\
\
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE\
\'a0 \'a0 #include "Wire.h"\
#endif\
\
// class default I2C address is 0x68\
// specific I2C addresses may be passed as a parameter here\
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)\
// AD0 high = 0x69\
MPU6050 mpu;\
\
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)\
#define D12_PIN 12\
bool blinkState = false;\
\
// MPU control/status vars\
bool dmpReady = false; \'a0// set true if DMP init was successful\
uint8_t mpuIntStatus; \'a0 // holds actual interrupt status byte from MPU\
uint8_t devStatus; \'a0 \'a0 \'a0// return status after each device operation (0 = success, !0 = error)\
uint16_t packetSize; \'a0 \'a0// expected DMP packet size (default is 42 bytes)\
uint16_t fifoCount; \'a0 \'a0 // count of all bytes currently in FIFO\
uint8_t fifoBuffer[64]; // FIFO storage buffer\
\
// orientation/motion vars\
Quaternion q; \'a0 \'a0 \'a0 \'a0 \'a0 // [w, x, y, z] \'a0 \'a0 \'a0 \'a0 quaternion container\
VectorInt16 aa; \'a0 \'a0 \'a0 \'a0 // [x, y, z] \'a0 \'a0 \'a0 \'a0 \'a0 \'a0accel sensor measurements\
VectorInt16 aaReal; \'a0 \'a0 // [x, y, z] \'a0 \'a0 \'a0 \'a0 \'a0 \'a0gravity-free accel sensor measurements\
VectorInt16 aaWorld; \'a0 \'a0// [x, y, z] \'a0 \'a0 \'a0 \'a0 \'a0 \'a0world-frame accel sensor measurements\
VectorFloat gravity; \'a0 \'a0// [x, y, z] \'a0 \'a0 \'a0 \'a0 \'a0 \'a0gravity vector\
float euler[3]; \'a0 \'a0 \'a0 \'a0 // [psi, theta, phi] \'a0 \'a0Euler angle container\
float ypr[3]; \'a0 \'a0 \'a0 \'a0 \'a0 // [yaw, pitch, roll] \'a0 yaw/pitch/roll container and gravity vector\
int count;\
float abs_loc_x;\
float offset;\
float speed;\
float speed_div;\
int sum;\
// packet structure for InvenSense teapot demo\
uint8_t teapotPacket[14] = \{ '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\\r', '\\n' \};\
\
int inPin=4;\
long time = 0;\
long debounce = 200;\
int cali_button;\
int previous=LOW;\
float caligx=0;\
float caligy=0;\
float caligz=0;\
boolean cali=false;\
float caliaz=0;\
float angle=20;\
float az=0;\
const int Buzzer=9;\
\
//bluetooth\
\'a0SoftwareSerial Genotronex(10, 11); // RX, TX\
\'a0int ledpin=13; // led on D13 will show blink on / off\
\'a0int BluetoothData; // the data given from Computer\
\
// Smart Shose\
\'a0int right0left1 = 0;\
\'a0String rol="";\
\
// ================================================================\
// === \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 INTERRUPT DETECTION ROUTINE \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0===\
// ================================================================\
\
volatile bool mpuInterrupt = false; \'a0 \'a0 // indicates whether MPU interrupt pin has gone high\
void dmpDataReady() \{\
\'a0 \'a0 mpuInterrupt = true;\
\}\
\
// ================================================================\
// === \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0INITIAL SETUP \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 ===\
// ================================================================\
\
void setup() \{ \'a0 \'a0\
\'a0 \'a0\
\'a0 \'a0 // join I2C bus (I2Cdev library doesn't do this automatically)\
\'a0 \'a0 #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE\
\'a0 \'a0 \'a0 \'a0 Wire.begin();\
\'a0 \'a0 \'a0 \'a0 TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)\
\'a0 \'a0 #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE\
\'a0 \'a0 \'a0 \'a0 Fastwire::setup(400, true);\
\'a0 \'a0 #endif\
\
\'a0 \'a0 // initialize serial communication\
\'a0 \'a0 // (115200 chosen because it is required for Teapot Demo output, but it's\
\'a0 \'a0 // really up to you depending on your project)\
\'a0 \'a0 Serial.begin(115200);\
\'a0 \'a0 while (!Serial); // wait for Leonardo enumeration, others continue immediately\
\
\'a0 \'a0 // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio\
\'a0 \'a0 // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to\
\'a0 \'a0 // the baud timing being too misaligned with processor ticks. You must use\
\'a0 \'a0 // 38400 or slower in these cases, or use some kind of external separate\
\'a0 \'a0 // crystal solution for the UART timer.\
\
\'a0 \'a0 // initialize device\
\'a0 \'a0 Serial.println(F("Initializing I2C devices..."));\
\'a0 \'a0 mpu.initialize();\
\
\'a0 \'a0 // verify connection\
\'a0 \'a0 Serial.println(F("Testing device connections..."));\
\'a0 \'a0 Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));\
\
\'a0 \'a0 // wait for ready\
\'a0 \'a0 Serial.println(F("\\nSend any character to begin DMP programming and demo: "));\
/* \'a0 \'a0while (Serial.available() && Serial.read()); // empty buffer\
\'a0 \'a0 while (!Serial.available()); \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 // wait for data\
\'a0 \'a0 while (Serial.available() && Serial.read()); // empty buffer again\
\'a0 \'a0 */\
\'a0 \'a0 delay(5000);\
\
\'a0 \'a0 // load and configure the DMP\
\'a0 \'a0 Serial.println(F("Initializing DMP..."));\
\'a0 \'a0 devStatus = mpu.dmpInitialize();\
\
\'a0 \'a0 // supply your own gyro offsets here, scaled for min sensitivity\
\'a0 \'a0 mpu.setXGyroOffset(220);\
\'a0 \'a0 mpu.setYGyroOffset(76);\
\'a0 \'a0 mpu.setZGyroOffset(-85);\
\'a0 \'a0 mpu.setZAccelOffset(1788); // 1688 factory default for my test chip\
\
\'a0 \'a0 // make sure it worked (returns 0 if so)\
\'a0 \'a0 if (devStatus == 0) \{\
\'a0 \'a0 \'a0 \'a0 // turn on the DMP, now that it's ready\
\'a0 \'a0 \'a0 \'a0 Serial.println(F("Enabling DMP..."));\
\'a0 \'a0 \'a0 \'a0 mpu.setDMPEnabled(true);\
\
\'a0 \'a0 \'a0 \'a0 // enable Arduino interrupt detection\
\'a0 \'a0 \'a0 \'a0 Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));\
\'a0 \'a0 \'a0 \'a0 attachInterrupt(0, dmpDataReady, RISING);\
\'a0 \'a0 \'a0 \'a0 mpuIntStatus = mpu.getIntStatus();\
\
\'a0 \'a0 \'a0 \'a0 // set our DMP Ready flag so the main loop() function knows it's okay to use it\
\'a0 \'a0 \'a0 \'a0 Serial.println(F("DMP ready! Waiting for first interrupt..."));\
\'a0 \'a0 \'a0 \'a0 dmpReady = true;\
\
\'a0 \'a0 \'a0 \'a0 // get expected DMP packet size for later comparison\
\'a0 \'a0 \'a0 \'a0 packetSize = mpu.dmpGetFIFOPacketSize();\
\'a0 \'a0 \} else \{\
\'a0 \'a0 \'a0 \'a0 // ERROR!\
\'a0 \'a0 \'a0 \'a0 // 1 = initial memory load failed\
\'a0 \'a0 \'a0 \'a0 // 2 = DMP configuration updates failed\
\'a0 \'a0 \'a0 \'a0 // (if it's going to break, usually the code will be 1)\
\'a0 \'a0 \'a0 \'a0 Serial.print(F("DMP Initialization failed (code "));\
\'a0 \'a0 \'a0 \'a0 Serial.print(devStatus);\
\'a0 \'a0 \'a0 \'a0 Serial.println(F(")"));\
\'a0 \'a0 \}\
\
\'a0 \'a0 // configure LED for output\
\'a0 \'a0 pinMode(LED_PIN, OUTPUT);\
\
\'a0 \'a0 count = 0;\
\'a0 \'a0 sum =0;\
\
\'a0 \'a0// put your setup code here, to run once:\
\'a0 \'a0Genotronex.begin(9600);\
\'a0 \'a0Genotronex.println("Bluetooth On please press 1 or 0 blink LED ..");\
\'a0 \'a0pinMode(ledpin,OUTPUT);\
\
\'a0 \'a0// Read if I am left or right LEFT=1, RIGHT=0\
\'a0 \'a0pinMode(D12_PIN, INPUT);\
\'a0 \'a0right0left1 = digitalRead(D12_PIN);\
\
\'a0 \'a0Genotronex.print("AT+NAME=LEFT\\r\\n");\
\'a0 \'a0 pinMode(inPin, INPUT);\
\'a0 \'a0 pinMode(Buzzer, OUTPUT);\
\'a0 \'a0 digitalWrite(Buzzer, LOW);\
\}\
\
\
// ================================================================\
// === \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0MAIN PROGRAM LOOP \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 ===\
// ================================================================\
\
void loop() \{\
\'a0 \'a0 // if programming failed, don't try to do anything\
\'a0 \'a0 if (!dmpReady) return;\
\
\'a0 \'a0 // wait for MPU interrupt or extra packet(s) available\
\'a0 \'a0 while (!mpuInterrupt && fifoCount < packetSize) \{\
\'a0 \'a0 \'a0 \'a0 // other program behavior stuff here\
\'a0 \'a0 \'a0 \'a0 // .\
\'a0 \'a0 \'a0 \'a0 // .\
\'a0 \'a0 \'a0 \'a0 // .\
\'a0 \'a0 \'a0 \'a0 // if you are really paranoid you can frequently test in between other\
\'a0 \'a0 \'a0 \'a0 // stuff to see if mpuInterrupt is true, and if so, "break;" from the\
\'a0 \'a0 \'a0 \'a0 // while() loop to immediately process the MPU data\
\'a0 \'a0 \'a0 \'a0 // .\
\'a0 \'a0 \'a0 \'a0 // .\
\'a0 \'a0 \'a0 \'a0 // .\
\'a0 \'a0 \}\
\
\'a0 \'a0 // reset interrupt flag and get INT_STATUS byte\
\'a0 \'a0 mpuInterrupt = false;\
\'a0 \'a0 mpuIntStatus = mpu.getIntStatus();\
\
\'a0 \'a0 // get current FIFO count\
\'a0 \'a0 fifoCount = mpu.getFIFOCount();\
\
\'a0 \'a0 // check for overflow (this should never happen unless our code is too inefficient)\
\'a0 \'a0 if ((mpuIntStatus & 0x10) || fifoCount == 1024) \{\
\'a0 \'a0 \'a0 \'a0 // reset so we can continue cleanly\
\'a0 \'a0 \'a0 \'a0 mpu.resetFIFO();\
\'a0 \'a0 \'a0 \'a0 Serial.println(F("FIFO overflow!"));\
\
\'a0 \'a0 // otherwise, check for DMP data ready interrupt (this should happen frequently)\
\'a0 \'a0 \} else if (mpuIntStatus & 0x02) \{\
\'a0 \'a0 \'a0 \'a0 // wait for correct available data length, should be a VERY short wait\
\'a0 \'a0 \'a0 \'a0 while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();\
\
\'a0 \'a0 \'a0 \'a0 // read a packet from FIFO\
\'a0 \'a0 \'a0 \'a0 mpu.getFIFOBytes(fifoBuffer, packetSize);\
\'a0 \'a0 \'a0 \'a0\'a0\
\'a0 \'a0 \'a0 \'a0 // track FIFO count here in case there is > 1 packet available\
\'a0 \'a0 \'a0 \'a0 // (this lets us immediately read more without waiting for an interrupt)\
\'a0 \'a0 \'a0 \'a0 fifoCount -= packetSize;\
\'a0 \'a0 \'a0 \'a0\
// \'a0 \'a0 \'a0 \'a0 \'a0\'a0\
// display Euler angles in degrees\
//M\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 mpu.dmpGetQuaternion(&q, fifoBuffer);\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 mpu.dmpGetGravity(&gravity, &q);\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 if(right0left1==1) \{rol="l"; \}\'a0\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 else \{ rol="r"; \}\
\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0float gx = ypr[0] * 180/M_PI;\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0float gy = ypr[1] * 180/M_PI;\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0float gz = ypr[2] * 180/M_PI;\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0\'a0\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0\'a0\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 Serial.println(rol+"gx "+(gx-caligx));\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 Serial.println(rol+"gy "+(gy-caligy));\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 Serial.println(rol+"gz "+(gz-caligz));\
\'a0\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0\
\'a0 \'a0 \'a0 \'a0 \'a0 /*\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 Genotronex.println(rol+"gx "+(gx-caligx));\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 Genotronex.println(rol+"gy "+(gy-caligy));\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 Genotronex.println(rol+"gz "+(gz-caligz));\
*/\
\
// \'a0 \'a0 \'a0 \'a0 \'a0 \'a0\
// display real acceleration, adjusted to remove gravity\
//\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0\'a0\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 mpu.dmpGetQuaternion(&q, fifoBuffer);\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 mpu.dmpGetAccel(&aa, fifoBuffer);\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 mpu.dmpGetGravity(&gravity, &q);\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 /*\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 Serial.println(rol+"ax "+aaReal.x);\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 Serial.println(rol+"ay "+aaReal.y);\
\'a0 \'a0 \'a0 \'a0 \'a0	*/\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 az=aaReal.z;\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 Serial.println(rol+"az "+(az-caliaz)); \'a0\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0\'a0\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 /*\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 Genotronex.println(rol+"ax "+aaReal.x);\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 Genotronex.println(rol+"ay "+aaReal.y);\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 Genotronex.println(rol+"az "+aaReal.z);\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 */\
//\
// Calibration\
//\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 cali_button = digitalRead(inPin);\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0\'a0\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 if (cali_button == HIGH && previous == LOW && \
\pard\pardeftab720\li1440\ri0\partightenfactor0
\cf0 millis() - time > debounce) \{ \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0\
\pard\pardeftab720\ri0\partightenfactor0
\cf0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 caligx=gx;\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 caligy=gy;\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 caligz=gz;\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 caliaz=az;\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 time=millis();\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 Serial.println("CALIBRATED");\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 // \'a0Gentronex.println("CALIBRATED");\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 cali=true;\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \}\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 previous=cali_button;\
\
//\
// Checking angle\
//\
/*\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 if(ypr[0] * 180/M_PI-caligx>angle && cali==true)\{\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 tone(9, 500, 200);\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \}\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 else if(ypr[0] * 180/M_PI-caligx<angle*-1 && cali==true)\{\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 tone(9, 500, 200);\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \'a0 tone(9, 500, 200);\
\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 \}\
\'a0 \'a0 \'a0 \'a0 \'a0 \'a0 */\
\'a0 \'a0 \'a0\'a0\
\'a0 \'a0 \'a0 \'a0 // blink LED to indicate activity\
\'a0 \'a0 \'a0 \'a0 blinkState = !blinkState;\
\'a0 \'a0 \'a0 \'a0 digitalWrite(LED_PIN, blinkState);\
\'a0 \'a0 \}\
\}\
\
\
}