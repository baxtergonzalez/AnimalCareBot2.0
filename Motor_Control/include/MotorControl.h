// MotorControl.h
#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include <AccelStepper.h>

class MotorControl {
public:
    MotorControl(int enablePin, int pulsePin, int dirPin, int maxSpeed = 10000, int acceleration = 500);
    
    void moveNSteps(int steps);
    void runAtSpeedForTime(double speed, int timeRunning);
    void runAtSpeed(double speed);
    void stopMotor();
    void setSpeed(double speed);
    void setAcceleration(double accel);
    
private:
    AccelStepper stepper;
    int enablePin;
};

#endif