//
// Created by Rikhil on 2/10/2025.
//

#ifndef GYRO_H
#define GYRO_H

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>

class Gyro {
    MPU6050 mpu;
    uint8_t FIFOBuffer[64];

    Quaternion q;
    VectorFloat gravity;
    float ypr[3];
    double yawAngle;


    PID gyroStraightController;
    double driveCorrectionOutput;
    double driveSetpoint;


    PID gyroTurnController;
    double turnValueOutput;
    double turnSetpoint;

    static double normalizeAngle(double angle) {
        // Wrap angle to [-180, 180]
        angle = fmod(angle + 180, 360);
        if (angle < 0) angle += 360;
        return angle - 180;
    }


public:
    Gyro() : gyroStraightController(&yawAngle, &driveCorrectionOutput, &driveSetpoint, 40, 0, 12, DIRECT),
             gyroTurnController(&yawAngle, &turnValueOutput, &turnSetpoint, 30, 0, 5, DIRECT) {
        gyroStraightController.SetMode(AUTOMATIC);
        gyroStraightController.SetSampleTime(50);
        gyroStraightController.SetOutputLimits(-255, 255);

        gyroTurnController.SetMode(AUTOMATIC);
        gyroTurnController.SetSampleTime(50);
        gyroTurnController.SetOutputLimits(-255, 255);
    }

    double calculateTurnValue() {
        gyroTurnController.Compute();

        return turnValueOutput;
    }

    double getTurnError() {
        double error = turnSetpoint - yawAngle;
        return normalizeAngle(error); // Ensure error is always in [-180, 180]
    }

    double calculateDriveCorrection() {
        gyroStraightController.Compute();

        return driveCorrectionOutput;
    }

    double targetedYaw() {
        return driveSetpoint;
    }

    void initMPU() {
        Serial.println("starting mpu init");
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
    }

// must be called in loop as many times as possible
    void update() {
        if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
            mpu.dmpGetQuaternion(&q, FIFOBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yawAngle = ypr[0] * (180 / M_PI);
        }
    }

    double yaw() const {
        return yawAngle;
    }

    double getNormalizedYawError() {
        double error = turnSetpoint - yawAngle;
        // Ensure error is in the range [-180, 180]
        error = fmod(error + 180, 360);
        if (error < 0) error += 360;
        error -= 180;

        return error;
    }

    void setTargetYaw(const double yaw) {
        driveSetpoint = yaw;
    }

    void turnTo(double degrees) {
        turnSetpoint = normalizeAngle(degrees);
    }

    void turnDegrees(double degrees) {
        turnSetpoint = normalizeAngle(yawAngle + degrees); // Just add the degrees
    }

    double getTurnSetpoint() const {
        return turnSetpoint;
    };
};

#endif
