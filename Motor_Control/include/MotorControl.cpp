// MotorControl.cpp
#include "MotorControl.h"

MotorControl::MotorControl(int enablePin, int pulsePin, int dirPin, int maxSpeed, int acceleration)
    : stepper(AccelStepper::DRIVER, pulsePin, dirPin), enablePin(enablePin) {
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, LOW);  // Enable the motor (LOW typically enables)
    stepper.setMaxSpeed(maxSpeed);
    stepper.setAcceleration(acceleration);
}

void MotorControl::moveNSteps(int steps) {
    /*
     * Runs the motor to a desired relative position in a blocking fashion
     */
    stepper.move(steps);
    stepper.runToPosition();
}

void MotorControl::runAtSpeedForTime(double speed, int timeRunning) {
    /*
     * Runs the motor at a set speed for a set amount of time
     */
    unsigned long timeRunningMillis = timeRunning * 10;
    unsigned long startTime = millis();
    
    stepper.setSpeed(speed);
    while (millis() - startTime < timeRunningMillis) {
        stepper.runSpeed();
    }
    stepper.stop();
}

void MotorControl::runAtSpeed(double speed) {
    /*
     * Runs the motor at a set speed for a set amount of time
     */
    
    stepper.setSpeed(speed);
    stepper.run();
}

void MotorControl::stopMotor(){
    stepper.stop();
}

void MotorControl::setSpeed(double speed){
    stepper.setMaxSpeed(speed);
}

void MotorControl::setAcceleration(double accel){
    stepper.setAcceleration(accel);
}