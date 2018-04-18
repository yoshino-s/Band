/*
 *Author: Yoshino-s (Cui Chenyang)
 *Date: 2018.4.18
 *Descrption: Program for Genix's band
*/
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


MPU6050 mpu(0x68);


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

float pitch, lastPitch, yaw, roll; // datas
int delta, lastDelta;
unsigned int statusLeft = 0x00;
unsigned int statusRight = 0x00;
unsigned int statusStop = 0x00;

unsigned long stopBeforeTime;
unsigned int stopTime = 0;

#define SHIFT_START 500
#define SHIFT_END 1000
#define STOP_BEFORE 500
#define STOP_MIDDLE 1000
#define STOP_END 1500
#define DEFAULT 0x00

//changeable datas
#define MAX_ROLL_VALUE (40) //if roll > MAX_ROLL_VALUE , it means "他垂着手"
#define PITCH_VALUE  (3000)

#define STOP_VALUE 1000
#define STOP_TIME_LENGTH 100
#define MOTION_TIME 5000

void setup() {
	Serial.begin(115200);
	// printf_begin();
	pinMode(5, OUTPUT);
	digitalWrite(5, LOW);

	init_mpu6050();
}


double now, last;

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

		yaw = -ypr[0] * 180 / M_PI;
		pitch = -ypr[1] * 180 / M_PI;
		roll = -ypr[2] * 180 / M_PI;

		check();
	}
}

void dmpDataReady() {

	mpuInterrupt = true;

}

void check() {
	// yaw 不准确
	//pitch 需要计算动作
	//roll 当手下垂是大约是 80 左右

	delta = (int)((lastPitch - pitch)*1000); //delta between last one and this one
	int op0 = delta;

	if (roll > MAX_ROLL_VALUE || roll < -MAX_ROLL_VALUE) {
		//motion wrong
		clearData();
	}
	else {
		if (delta < STOP_VALUE && delta > -STOP_VALUE) {// stop
			stopTime++;

			if (stopTime > STOP_TIME_LENGTH && statusStop == DEFAULT) { //stop before motion
				clearData();
				statusStop = STOP_BEFORE;
				stopBeforeTime = millis();
			}

			if (stopTime > STOP_TIME_LENGTH && statusStop == STOP_MIDDLE) {
				if (millis() - stopBeforeTime <= MOTION_TIME) {
					statusStop = STOP_END;
				}
				else {
					//timeout
					clearData();
				}
			}
		}
		else {
			if (statusStop == STOP_BEFORE) { // 动作前静置完成
				if (lastDelta < PITCH_VALUE && delta > PITCH_VALUE && statusRight == DEFAULT) {
					statusRight = SHIFT_START;
				}
				if (lastDelta > PITCH_VALUE && delta < PITCH_VALUE && statusRight == SHIFT_START) {
					statusRight = SHIFT_END;
				}


				if (lastDelta > -PITCH_VALUE && delta < -PITCH_VALUE && statusLeft == DEFAULT) {
					statusLeft = SHIFT_START;
				}
				if (lastDelta < -PITCH_VALUE && delta > -PITCH_VALUE && statusLeft == SHIFT_START) {
					statusLeft = SHIFT_END;
				}

				if (statusLeft == SHIFT_END && statusRight == SHIFT_END) {
					statusStop = STOP_MIDDLE;
				}
			}
		}

		if (statusStop == STOP_END) {
			digitalWrite(5, HIGH);
			delay(500);
			digitalWrite(5, LOW);
			// send the ir signal
			clearData();
		}
	}
	// printf("%d %d %d %d\n", op0, statusLeft, statusRight, statusStop);

	lastDelta = delta;
	lastPitch = pitch; //save last pitch
}

void clearData() {
	statusLeft = DEFAULT;
	statusRight = DEFAULT;
	statusStop = DEFAULT;
	stopTime = 0;
}

int serial_putc(char c, FILE *)
{
	Serial.write(c);

	return c;
}

void printf_begin(void)
{
	fdevopen(&serial_putc, 0);
}
