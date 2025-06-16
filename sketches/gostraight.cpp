#include <Arduino.h>
#include <PID_v1.h>
#include "motor.h"
#include "gyro.h"

constexpr int TICKS_PER_REVOLUTION = 960;
constexpr double CM_PER_REVOLUTION = 6.5 * M_PI;

volatile int leftTickInterrupt = 0;
volatile int rightTickInterrupt = 0;


Motor leftMotor(2, 5, 11, 13); // switch
Motor rightMotor(3, 6, 10, 12);
Gyro gyro;

void leftMotorInterrupt() {
    int leftReading = digitalRead(leftMotor.encoderA);
    if ((leftMotor.encoderALastReading == LOW) && leftReading == HIGH) {
        leftTickInterrupt++;
    }
    leftMotor.encoderALastReading = leftReading;
}

void rightMotorInterrupt() {
    int rightReading = digitalRead(rightMotor.encoderA);
    if ((rightMotor.encoderALastReading == LOW) && rightReading == HIGH) {
        rightTickInterrupt++;
    }
    rightMotor.encoderALastReading = rightReading;
}

void resetWheelTicks() {
    leftTickInterrupt = 0;
    rightTickInterrupt = 0;
    leftMotor.setTicks(0);
    leftMotor.setTicks(0);
}



void turn() {
    double turnOutput = gyro.calculateTurnValue();

}

void setup() {
    Serial.begin(115200);//Initialize the serial port

    gyro.initMPU();

    attachInterrupt(digitalPinToInterrupt(leftMotor.encoderA), leftMotorInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rightMotor.encoderA), rightMotorInterrupt, CHANGE);

    // uncomment if right motor is being a lil too fast
    // rightMotor.setTunings(MOTOR_KP, MOTOR_KI, MOTOR_KD + 0.05);

    leftMotor.setTargetPosition(9600);
    rightMotor.setTargetPosition(9600);

    leftMotor.setDirection(HIGH);

    gyro.setTargetYaw(0);
}

void loop() {
    // Update methods
    gyro.update();
    leftMotor.setTicks(leftTickInterrupt);
    rightMotor.setTicks(rightTickInterrupt);

    // calculate the motor correction values based on the yaw
    double correction = gyro.calculateDriveCorrection();

    leftMotor.updatePositionWithGyro(correction);
    rightMotor.updatePositionWithGyro(-correction);
}
