#include <Arduino.h>
#include <ServoInput.h>

// Motor PWM pins (one direction only)
#define LEFT1_PWM_PIN 10
#define LEFT2_PWM_PIN 11
#define RIGHT1_PWM_PIN 12
#define RIGHT2_PWM_PIN 13

// RC receiver pins
#define CH1_PIN 3  // Throttle
#define CH2_PIN 2  // Steering

// Create PWM reader input
ServoInputPin<CH1_PIN> throttle;
ServoInputPin<CH2_PIN> steering;

// Motor constants
const int MOTOR_MAX_SPEED = 100;
const int MOTOR_MIN_SPEED = 0;

// Safety Timeout
const unsigned long TIMEOUT = 1000;
unsigned long lastSignalTime = millis();

// Smoothing
float smoothedSpeedLeft = 0.0;
float smoothedSpeedRight = 0.0;
const float smoothingFactor = 0.7;

// --- Function to map input scales ---
float mapBetweenScales(float value, float value_min, float value_max, float goal_min, float goal_max) {
    return goal_min + (value - value_min) * (goal_max - goal_min) / (value_max - value_min);
}

// --- Set motor speed (positive only) ---
void run_motor(int pwm_pin, float speed) {
    speed = constrain(speed, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
    analogWrite(pwm_pin, speed);
}

// --- Stop motor ---
void stop_motor(int pwm_pin) {
    analogWrite(pwm_pin, 0);
}

void setup() {
    Serial.begin(115200);

    throttle.attach();
    steering.attach();

    pinMode(LEFT1_PWM_PIN, OUTPUT);
    pinMode(LEFT2_PWM_PIN, OUTPUT);
    pinMode(RIGHT1_PWM_PIN, OUTPUT);
    pinMode(RIGHT2_PWM_PIN, OUTPUT);

    while (!throttle.available() || !steering.available()) {
        Serial.println("Waiting for RC signal...");
        delay(500);
    }

    Serial.println("Setup complete. RC signal detected.");
}

void loop() {
    if (throttle.available() && steering.available()) {
        lastSignalTime = millis();

        float throttle_val = mapBetweenScales(throttle.getAngle(), 90.0, 180.0, 0.0, 1.0); // 0 (stop) to 1 (full)
        float steering_val = mapBetweenScales(steering.getAngle(), 90.0, 180.0, -1.0, 1.0); // -1 (left) to +1 (right)

        float targetSpeed = throttle_val * MOTOR_MAX_SPEED;

        // // Basic steering: slow down one side
        // float leftSpeed = targetSpeed * (1.0 - max(0.0f, steering_val));   // steer right = slower left
        // float rightSpeed = targetSpeed * (1.0 + min(0.0f, steering_val));  // steer left = slower right
        float leftSpeed = targetSpeed;
        float rightSpeed = targetSpeed;

        // Smoothing
        smoothedSpeedLeft = smoothedSpeedLeft + smoothingFactor * (leftSpeed - smoothedSpeedLeft);
        smoothedSpeedRight = smoothedSpeedRight + smoothingFactor * (rightSpeed - smoothedSpeedRight);

        // Drive motors
        run_motor(LEFT1_PWM_PIN, smoothedSpeedLeft);
        run_motor(LEFT2_PWM_PIN, smoothedSpeedLeft);
        run_motor(RIGHT1_PWM_PIN, smoothedSpeedRight);
        run_motor(RIGHT2_PWM_PIN, smoothedSpeedRight);

    } else if (millis() - lastSignalTime > TIMEOUT) {
        // Timeout safety stop
        stop_motor(LEFT1_PWM_PIN);
        stop_motor(LEFT2_PWM_PIN);
        stop_motor(RIGHT1_PWM_PIN);
        stop_motor(RIGHT2_PWM_PIN);
        Serial.println("No RC signal. Motors stopped for safety.");
    }
}
