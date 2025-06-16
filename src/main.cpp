#include <Arduino.h>

#include "motor.h"
#include "gyro.h"

//#define DISTANCE_SENSOR_ENABLED true

constexpr int TICKS_PER_REVOLUTION = 960;
constexpr double CM_PER_REVOLUTION = 6.5 * M_PI;

volatile int leftTickInterrupt = 0;
volatile int rightTickInterrupt = 0;

constexpr int LINEAR_MOTION = 0;
constexpr int TURN_TO_POSITION = 1;
constexpr int TURN_DEGREES = 2;
constexpr int END = 3;


struct MotionReq
{
    int type;
    double target;
};

Motor leftMotor(2, 5, 11, 13); // switch
Motor rightMotor(3, 6, 10, 12);
Gyro gyro;

void stopAndExit()
{
    leftMotor.stop();
    rightMotor.stop();
    //  Serial.println("ended");
    while (1);
}

void leftMotorInterrupt()
{
    int leftReading = digitalRead(leftMotor.encoderA);
    if ((leftMotor.encoderALastReading == LOW) && leftReading == HIGH)
    {
        leftTickInterrupt++;
    }
    leftMotor.encoderALastReading = leftReading;
}

void rightMotorInterrupt()
{
    int rightReading = digitalRead(rightMotor.encoderA);
    if ((rightMotor.encoderALastReading == LOW) && rightReading == HIGH)
    {
        rightTickInterrupt++;
    }
    rightMotor.encoderALastReading = rightReading;
}

void resetWheelTicks()
{
    leftTickInterrupt = 0;
    rightTickInterrupt = 0;
    leftMotor.setTicks(0);
    leftMotor.setTicks(0);
}

int turn()
{
    double turnOutput = gyro.calculateTurnValue();
    Serial.print("Output:");
    Serial.print(turnOutput);
    Serial.print(" | ");
    Serial.print("Turn error:");
    Serial.print(gyro.getTurnError());
    Serial.print(" | ");
    Serial.print("Yaw:");
    Serial.print(gyro.yaw());
    Serial.print(" | ");
    Serial.print("Turn target:");
    Serial.println(gyro.getTurnSetpoint());
    double constrainedOutput = turnOutput == 0 ? 0 : constrain(abs(turnOutput), 100, 255);

    if (turnOutput < 0)
    {
        constrainedOutput = -constrainedOutput;
    }

    double turnError = gyro.getTurnError();

    if (turnError > 0)
    {
        leftMotor.setOutput(constrainedOutput);
        rightMotor.stop();
    }
    else
    {
        rightMotor.setOutput(-constrainedOutput);
        leftMotor.stop();
    }

    if (abs(gyro.getTurnError()) < 1)
    {
        leftMotor.stop();
        rightMotor.stop();
        //  delay(5000);
        return 1;
    }

    return 0;
}

MotionReq turnDegrees(const double x)
{
    return MotionReq{TURN_DEGREES, x};
}

MotionReq turnTo(const double x)
{
    return MotionReq{TURN_TO_POSITION, x};
}

MotionReq moveCm(const double x)
{
    return MotionReq{LINEAR_MOTION, (x / CM_PER_REVOLUTION) * TICKS_PER_REVOLUTION};
}

MotionReq end()
{
    return MotionReq{END, 0};
}

void prepareRequest(MotionReq request)
{
    if (request.type == LINEAR_MOTION)
    {
        resetWheelTicks();
        leftMotor.setTargetPosition(request.target);
        rightMotor.setTargetPosition(request.target);
        gyro.update();
        gyro.setTargetYaw(gyro.yaw());
    }
    else if (request.type == TURN_TO_POSITION)
    {
        gyro.turnTo(request.target);
    }
    else if (request.type == TURN_DEGREES)
    {
        gyro.turnDegrees(request.target);
    }
    else if (request.type == END)
    {
        stopAndExit();
    }
}


MotionReq motions[] = {
    moveCm(30),
    turnDegrees(90),
    moveCm(40),
    turnDegrees(-90),
    moveCm(30),
    turnDegrees(3),
    moveCm(5),
    turnDegrees(-90),
    moveCm(30),
    moveCm(30),
    turnDegrees(3),
    moveCm(30),
    moveCm(30),
    turnDegrees(-3),
    moveCm(30),
    turnDegrees(90),
    moveCm(30),
    turnDegrees(-90),
    moveCm(30),
    // moveCm(200),
    // turnDegrees(-90),
    // moveCm(30),
    end() // always END the motions array with this, so it doesn't keep checking for requests
};
int motionReqIndex = 0;

void setup()
{
    Serial.begin(115200); //Initialize the serial port

    gyro.initMPU();

    attachInterrupt(digitalPinToInterrupt(leftMotor.encoderA), leftMotorInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rightMotor.encoderA), rightMotorInterrupt, CHANGE);

    // uncomment if right motor is being a lil too fast
    // rightMotor.setDriveTunings(MOTOR_KP, MOTOR_KI, MOTOR_KD + 0.05);

    leftMotor.setDirection(HIGH);
    prepareRequest(motions[0]);
}

void loop()
{
    gyro.update();

    bool currentRequestFinished = false;
    const MotionReq activeRequest = motions[motionReqIndex];

    if (activeRequest.type == LINEAR_MOTION)
    {
        leftMotor.setTicks(leftTickInterrupt);
        rightMotor.setTicks(rightTickInterrupt);
        double gyroCorrection = gyro.calculateDriveCorrection();
        Serial.print("gyro set: ");
        Serial.print(gyro.targetedYaw());
        Serial.print(" | ");
        Serial.print("gyro y: ");
        Serial.print(gyro.yaw());
        Serial.print(" | ");

        Serial.print("gyro correction: ");
        Serial.println(gyroCorrection);
        int leftComplete = leftMotor.updatePositionWithGyro(gyroCorrection);
        int rightComplete = rightMotor.updatePositionWithGyro(-gyroCorrection);
        currentRequestFinished = leftComplete || rightComplete;

        //#ifndef DISTANCE_SENSOR_ENABLED
        //        delay(29);
        //            long dist = distanceSensor.ping_cm();
        //            if (dist > 0 && dist < 5)
        //            {
        //                bool adjustmentFinished;
        //                gyro.turnDegrees(gyro.yaw() < 0 ? 5 : -5);
        //                while (!adjustmentFinished)
        //                {
        //                    adjustmentFinished = turn();
        //                }
        //            }
        //#endif
    }
    else if (activeRequest.type == TURN_TO_POSITION || activeRequest.type == TURN_DEGREES)
    {
        currentRequestFinished = turn();
    }
    else if (activeRequest.type == END)
    {
        // shouldn't get here
        stopAndExit();
    }

    if (currentRequestFinished)
    {
        // Serial.println("request finished");
        // Serial.println(motionReqIndex);

        if (activeRequest.type == LINEAR_MOTION)
        {
            leftMotor.stop();
            rightMotor.stop();
        }

        prepareRequest(motions[++motionReqIndex]);
        delay(1000); //prevent drift from moving too fast
        //  Serial.println(motionReqIndex);
    }
}
