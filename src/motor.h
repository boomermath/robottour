#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <PID_v1.h>

extern constexpr int SAMPLING_TIME = 100; //in ms
extern constexpr double MOTOR_KP = 0.38;
extern constexpr double MOTOR_KI = 0;
extern constexpr double MOTOR_KD = 0.05;

class Motor {
    int encoderB;
    int ePin;
    int mPin;

    double ticks;
    double motorOutput;
    double setPoint;

    int forward = LOW;
    PID rotationController;

public:
    // must be an interrupt pin
    int encoderA;
    int encoderALastReading;

    Motor(int encoderA, int encoderB, int ePin, int mPin) : rotationController(&ticks, &motorOutput, &setPoint,
                                                                               MOTOR_KP, MOTOR_KI, MOTOR_KD, DIRECT) {
        this->encoderA = encoderA;
        this->encoderB = encoderB;
        this->ePin = ePin;
        this->mPin = mPin;

        pinMode(encoderB, INPUT);
        pinMode(ePin, OUTPUT);
        pinMode(mPin, OUTPUT);

        rotationController.SetMode(AUTOMATIC);
        rotationController.SetSampleTime(50);
        rotationController.SetOutputLimits(0, 245);
    }

    void setDriveTunings(double p, double i, double d) {
        rotationController.SetTunings(p, i, d);
    }

    void setTargetPosition(double position) {
        setPoint = position;
    }

    void setDirection(int forwardValue) {
        forward = forwardValue;
    }

    void setTicks(int motorTicks) {
        ticks = motorTicks;
    }

    [[deprecated("use updatePositionWithGyro")]]
    void updatePosition() {
        int rotationControllerResult = rotationController.Compute();

        if (rotationControllerResult) {
            if (motorOutput == 0) {
                stop();
            } else {
                setOutput(motorOutput);
            }

            //            Serial.print("output: ");
            //            Serial.print(motorOutput);
            //            Serial.print(" | ");
            //            Serial.print("ticks: ");
            //            Serial.print(ticks);
            //            Serial.println();
        }
    }

    int updatePositionWithGyro(double gyroOffset) {
        rotationController.Compute();
//        Serial.print("set pt: ");
//        Serial.print(setPoint);
//        Serial.print(" | ");
//        Serial.print("ticks: ");
//        Serial.print(ticks);
//        Serial.print(" | ");
//        Serial.print("output: ");
//        Serial.println(motorOutput);
        if (motorOutput == 0) {
            stop();
            return 1;
        }

        const int outputResult = constrain(motorOutput + gyroOffset, 100, 255);

        setOutput(outputResult);
        return 0;


        // Serial.print(" | ");
        // Serial.print("ticks: ");
        // Serial.print(ticks);
        // Serial.print(" | ");
        // Serial.print("offset: ");
        // Serial.print(gyroOffset);
        // Serial.println();
    }

    void setOutput(int output) {
        digitalWrite(mPin, output > 0 ? forward : (forward == LOW ? HIGH : LOW));
        analogWrite(ePin, abs(output));
    }

    void stop() {
        setOutput(LOW);
    }
};

#endif // MOTOR_H
