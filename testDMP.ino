#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

MPU6050 mpu;

#define INTERRUPT_PIN 2

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
float euler[3];
float ypr[3];

volatile bool mpuInterrupt = false;
void dmpDataReady(){
  mpuInterrupt = true;
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  TWBR = 24;

  Serial.begin(115200);

  Serial.println(F("Initializing Device..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing Device Connection..."));
  if(mpu.testConnection()){
    Serial.println("MPU6050 Connected");
  }else{
    Serial.println("MPU is not connected!");
    while(1);
  }

  Serial.println(F("\nSend any chars..."));
  while(Serial.available() && Serial.read());
  while(!Serial.available());
  while(Serial.available() && Serial.read());

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  //-2326  295 1752  -67 17  -79

  mpu.setXGyroOffset(-67);
  mpu.setYGyroOffset(17);
  mpu.setZGyroOffset(-79);
  mpu.setZAccelOffset(1752);
  mpu.setXAccelOffset(-2326);
  mpu.setYAccelOffset(295);

  if(devStatus == 0){
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("Enabling interrupt detection.."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP Ready! Waiting for first interrupt"));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  }else{
    Serial.print(F("DMP Init failed error code : "));
    Serial.print(devStatus);
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if(!dmpReady) return;

  while(!mpuInterrupt && fifoCount < packetSize){
    //other behaviour here...
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  if((mpuIntStatus & 0x10) || fifoCount == 1024){
    mpu.resetFIFO();
    Serial.println(F("FIFO Overflow!"));
  }else if(mpuIntStatus & 0x02){
    while(fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);

    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    Serial.print("Euler\t");
    Serial.print(euler[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180/M_PI);
  }
}
