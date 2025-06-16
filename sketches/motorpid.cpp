#include <PID_v1.h>
#include <Arduino.h>
#include "motor.h"


constexpr int TICKS_PER_REVOLUTION = 960;
constexpr double CM_PER_REVOLUTION = 6.5 * M_PI;

volatile int leftTickInterrupt = 0;
volatile int rightTickInterrupt = 0;


Motor leftMotor(2, 5, 11, 13); // switch
Motor rightMotor(3, 6, 10, 12);

void leftMotorInterrupt() {
    int Lstate = digitalRead(leftMotor.encoderA);
    if ((leftMotor.encoderALastReading == LOW) && Lstate == HIGH) {
        leftTickInterrupt++;
    }
    leftMotor.encoderALastReading = Lstate;
}

void rightMotorInterrupt() {
    int Lstate = digitalRead(rightMotor.encoderA);
    if ((rightMotor.encoderALastReading == LOW) && Lstate == HIGH) {
        rightTickInterrupt++;
    }
    rightMotor.encoderALastReading = Lstate;
}

void setup() {
    Serial.begin(115200);//Initialize the serial port
    attachInterrupt(digitalPinToInterrupt(leftMotor.encoderA), leftMotorInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rightMotor.encoderA), rightMotorInterrupt, CHANGE);

    leftMotor.setTargetPosition(4800);
    rightMotor.setTunings(MOTOR_KP, MOTOR_KI, MOTOR_KD + 0.05);
    rightMotor.setTargetPosition(4800);
    leftMotor.setDirection(HIGH);
}

void loop() {
    leftMotor.updateTicks(leftTickInterrupt);
    rightMotor.updateTicks(rightTickInterrupt);
    leftMotor.updatePosition();
    rightMotor.updatePosition();
}
