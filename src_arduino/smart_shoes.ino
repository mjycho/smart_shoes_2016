#include "I2Cdev.h"
#include <SoftwareSerial.h>// import the serial library

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define D12_PIN 12
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int count;
float abs_loc_x;
float offset;
float speed;
float speed_div;
int sum;
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

int inPin=4;
long time = 0;
long debounce = 200;
int cali_button;
int previous=LOW;
float caligx=0;
float caligy=0;
float caligz=0;
boolean cali=false;
float caliaz=0;
float angle=20;
float az=0;
const int Buzzer=9;

//bluetooth
 SoftwareSerial Genotronex(10, 11); // RX, TX
 int ledpin=13; // led on D13 will show blink on / off
 int BluetoothData; // the data given from Computer

// Smart Shose
 int right0left1 = 0;
 String rol="";

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {    
   
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
/*    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
    */
    delay(5000);

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    count = 0;
    sum =0;

   // put your setup code here, to run once:
   Genotronex.begin(9600);
   Genotronex.println("Bluetooth On please press 1 or 0 blink LED ..");
   pinMode(ledpin,OUTPUT);

   // Read if I am left or right LEFT=1, RIGHT=0
   pinMode(D12_PIN, INPUT);
   right0left1 = digitalRead(D12_PIN);

   Genotronex.print("AT+NAME=LEFT\r\n");
    pinMode(inPin, INPUT);
    pinMode(Buzzer, OUTPUT);
    digitalWrite(Buzzer, LOW);
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
       
//           
// display Euler angles in degrees
//M
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
           
            if(right0left1==1) {rol="l"; } 
            else { rol="r"; }

           float gx = ypr[0] * 180/M_PI;
           float gy = ypr[1] * 180/M_PI;
           float gz = ypr[2] * 180/M_PI;
            
            
           
            Serial.println(rol+"gx "+(gx-caligx));
            Serial.println(rol+"gy "+(gy-caligy));
            Serial.println(rol+"gz "+(gz-caligz));
 
               
          /*
            Genotronex.println(rol+"gx "+(gx-caligx));
            Genotronex.println(rol+"gy "+(gy-caligy));
            Genotronex.println(rol+"gz "+(gz-caligz));
*/

//            
// display real acceleration, adjusted to remove gravity
//
            
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            /*
            Serial.println(rol+"ax "+aaReal.x);
            Serial.println(rol+"ay "+aaReal.y);
         	*/
            az=aaReal.z;
            Serial.println(rol+"az "+(az-caliaz));  
            
            /*
            Genotronex.println(rol+"ax "+aaReal.x);
            Genotronex.println(rol+"ay "+aaReal.y);
            Genotronex.println(rol+"az "+aaReal.z);
            */
//
// Calibration
//
            cali_button = digitalRead(inPin);
              
              if (cali_button == HIGH && previous == LOW && 
millis() - time > debounce) {              
              caligx=gx;
              caligy=gy;
              caligz=gz;
              caliaz=az;
              time=millis();
              Serial.println("CALIBRATED");
            //  Gentronex.println("CALIBRATED");
              cali=true;
            }
            previous=cali_button;

//
// Checking angle
//
/*
            if(ypr[0] * 180/M_PI-caligx>angle && cali==true){
              tone(9, 500, 200);
            }
            else if(ypr[0] * 180/M_PI-caligx<angle*-1 && cali==true){
              tone(9, 500, 200);
              tone(9, 500, 200);

            }
            */
      
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}


