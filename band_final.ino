
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "printf.h"

MPU6050 mpu(0x68);
float pitch, lastPitch, yaw, roll;
 

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
bool dmpReady = false;  // set true if DMP init was successful


uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

 
void setup() {
  Serial.begin(115200);
  init_mpu6050();
}


double now,last;

void loop()
{  
     clc_mpu6050();   
}



void init_mpu6050(void)
{
Wire.begin();
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  delay(2);
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    Serial.println("Enabling DMP...");
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println("DMP ready! Waiting for first interrupt...");
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }
}

void clc_mpu6050(void)//9-11ms 更新一次数据  pid 取20ms
{
  if (!dmpReady) return;
  if (!mpuInterrupt && fifoCount < packetSize) return;
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println("FIFO overflow!");
  } 

  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);  

    yaw=-ypr[0] * 180/M_PI;
    pitch=-ypr[1] * 180/M_PI;
    roll=-ypr[2] * 180/M_PI;
    
    Serial.print(yaw);Serial.print('\t');
    Serial.print(pitch);Serial.print('\t');
    Serial.println(roll);

    lastPitch = pitch;
  }
}

void dmpDataReady() {

  mpuInterrupt = true;

}

