//
// Created by Rikhil on 2/9/2025.
//
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

uint16_t packetSize;
uint8_t FIFOBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

MPU6050 mpu;

void initMPU() {
    Wire.begin();
    Wire.setClock(400000);
    // optional for Serial prints
    Wire.setWireTimeout(3000, true);

    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU connection failed!");
        exit(1);
    }

    int dmpStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);

    if (dmpStatus != 0) {
        Serial.println("MPU DMP failed!");
        // 1 = memory load fail
        // 2 = DMP config update failed
        exit(dmpStatus);
    }

    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

    mpu.setDMPEnabled(true);

    packetSize = mpu.dmpGetFIFOPacketSize();
}

// must be called in loop as many times as possible
void update() {
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
        mpu.dmpGetQuaternion(&q, FIFOBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
}

float *angles() {
    return ypr;
}



void setup() {
    Serial.begin(115200);
    initMPU();
}

void loop() {
    update();
    float *a = angles();
    Serial.print(a[0] * 180 / M_PI);
    Serial.print(" ");
    Serial.print(a[1]* 180 / M_PI);
    Serial.print(" ");
    Serial.print(a[2]* 180 / M_PI);
    Serial.println();
}


