#include <Arduino.h>
// #include <ServoInput.h>

// Left side motors
#define LEFT1_DIR_PIN 33 // Green
#define LEFT1_PWM_PIN 10 // Blue

#define LEFT2_DIR_PIN 35
#define LEFT2_PWM_PIN 11 

// Right side motors
#define RIGHT1_DIR_PIN 37
#define RIGHT1_PWM_PIN 12 // Blue

#define RIGHT2_DIR_PIN 39
#define RIGHT2_PWM_PIN 13 // Blue

void setup() {
    Serial.begin(115200);

    // Set motor pins to output
    pinMode(LEFT1_PWM_PIN, OUTPUT);
    pinMode(LEFT2_PWM_PIN, OUTPUT);
    pinMode(RIGHT1_PWM_PIN, OUTPUT);
    pinMode(RIGHT2_PWM_PIN, OUTPUT);

    delay(1000);
    Serial.println("Setup complete. Motors initialized.");
}

int speed = 100;

void loop(){
    analogWrite(LEFT1_PWM_PIN, speed); // Set the motor speed
    analogWrite(LEFT2_PWM_PIN, speed); // Set the motor speed
    analogWrite(RIGHT1_PWM_PIN, speed); // Set the motor speed
    analogWrite(RIGHT2_PWM_PIN, speed); // Set the motor speed
    delay(10);
}



// #define LEFT2_DIR_PIN 11
// #define LEFT2_STOP_PIN 12
// #define LEFT2_BRAKE_PIN 13
// #define LEFT2_PWM_PIN 14 

// // Right side motors
// #define RIGHT1_DIR_PIN 32
// #define RIGHT1_STOP_PIN 34
// #define RIGHT1_BRAKE_PIN 35
// #define RIGHT1_PWM_PIN 36 // Blue


// #define RIGHT2_DIR_PIN 24
// #define RIGHT2_STOP_PIN 26
// #define RIGHT2_BRAKE_PIN 27
// #define RIGHT2_PWM_PIN 28 // Blue

// #define CH1_PIN 3
// #define CH2_PIN 2

// const int GEARBOX_REDUCTION = 10;

// // const int MOTOR_MAX_SPEED = 17928; //Steps per second
// const int MOTOR_MAX_SPEED = 255; //Steps per second
// const int MOTOR_MIN_SPEED = 0;

// const float MIN_PWM_INPUT = 0.0;
// const float MAX_PWM_INPUT = 180.0;



// //Create pwm reader input
// ServoInputPin<CH1_PIN> throttle;
// ServoInputPin<CH2_PIN> steering;

// float mapBetweenScales(float value, float value_min, float value_max, float goal_min, float goal_max){
//     return goal_min + (value - value_min) * (goal_max - goal_min) / (value_max - value_min);
// }

// // Safety Timeout
// const unsigned long TIMEOUT = 1000;  // 1000 milliseconds without a signal will trigger a stop
// unsigned long lastSignalTime = millis();

// void setup() {
//     Serial.begin(115200);

//     throttle.attach();
//     steering.attach();


//     // Set motor pins to output
//     pinMode(LEFT1_DIR_PIN, OUTPUT);
//     pinMode(LEFT1_STOP_PIN, OUTPUT);
//     pinMode(LEFT1_BRAKE_PIN, OUTPUT);
//     pinMode(LEFT1_PWM_PIN, OUTPUT);

//     pinMode(LEFT2_DIR_PIN, OUTPUT);
//     pinMode(LEFT2_STOP_PIN, OUTPUT);
//     pinMode(LEFT2_BRAKE_PIN, OUTPUT);
//     pinMode(LEFT2_PWM_PIN, OUTPUT);

//     pinMode(RIGHT1_DIR_PIN, OUTPUT);
//     pinMode(RIGHT1_STOP_PIN, OUTPUT);
//     pinMode(RIGHT1_BRAKE_PIN, OUTPUT);
//     pinMode(RIGHT1_PWM_PIN, OUTPUT);

//     pinMode(RIGHT2_DIR_PIN, OUTPUT);
//     pinMode(RIGHT2_STOP_PIN, OUTPUT);
//     pinMode(RIGHT2_BRAKE_PIN, OUTPUT);
//     pinMode(RIGHT1_PWM_PIN, OUTPUT);


//     while (throttle.available() == false || steering.available() == false){
//         Serial.println("Waiting for signal...");
//         delay(500);
//     }
// }

// void run_motor(int speed_pin, int dir_pin, int speed) {
//     // if (speed < 0) {
//     //     digitalWrite(dir_pin, LOW); // Set direction to reverse
//     // } else {
//     //     digitalWrite(dir_pin, HIGH); // Set direction to forward
//     // }

//     speed = abs(speed);

//     // if (speed > MOTOR_MAX_SPEED) {
//     //     speed = MOTOR_MAX_SPEED;
//     // } else if (speed < MOTOR_MIN_SPEED) {
//     //     speed = MOTOR_MIN_SPEED;
//     // }

//     Serial.println(speed);

//     analogWrite(speed_pin, speed);
// }

// void brake_motor(int pin) {
//     analogWrite(pin, 255);
// }

// float smoothedSpeedLeft = 0.0;
// float smoothedSpeedRight = 0.0;
// const float smoothingFactor = 0.7;


// void loop() {
//     if (throttle.available() && steering.available())
//     {
//         lastSignalTime = millis(); // Reset the last signal
//         // Example usage
//         float throttle_val = mapBetweenScales(throttle.getAngle(), 90.0, 180.0, 0.0, 1.0); // Map values to a percent
//         float steering_val = mapBetweenScales(steering.getAngle(), 90.0, 180.0, 0.0, 1.0); // Map values to a percent

//         Serial.println();
//         Serial.print("  Throttle || Steering  \n");
//         Serial.print("  ");
//         Serial.print(throttle_val);
//         Serial.print("         ");
//         Serial.print(steering_val);
//         Serial.print("\n");

//         float targetSpeed = throttle_val * MOTOR_MAX_SPEED;
//         float leftSpeedAdjust = (steering_val < 0) ? (1 + steering_val) : 1;
//         float rightSpeedAdjust = (steering_val > 0) ? (1 - steering_val) : 1;

//         smoothedSpeedLeft = smoothedSpeedLeft + smoothingFactor * (targetSpeed * leftSpeedAdjust - smoothedSpeedLeft);
//         smoothedSpeedRight = smoothedSpeedRight + smoothingFactor * (targetSpeed * rightSpeedAdjust - smoothedSpeedRight);

//         // Actuate motors based on smoothed speeds
//         if (abs(throttle_val) > 0.05)
//         {
//             run_motor(LEFT1_PWM_PIN, LEFT1_DIR_PIN, smoothedSpeedLeft);
//             run_motor(LEFT2_PWM_PIN, LEFT2_DIR_PIN, smoothedSpeedLeft);
//             run_motor(RIGHT1_PWM_PIN, RIGHT1_DIR_PIN, smoothedSpeedRight);
//             run_motor(RIGHT2_PWM_PIN, RIGHT2_DIR_PIN, smoothedSpeedRight);
//         }
//         else
//         {
//             brake_motor(LEFT1_BRAKE_PIN);
//             brake_motor(LEFT2_BRAKE_PIN);
//             brake_motor(RIGHT1_BRAKE_PIN);
//             brake_motor(RIGHT2_BRAKE_PIN);
//         }
//     } else if(millis() - lastSignalTime > TIMEOUT) {
//         // Stop all motors if the timeout period has elapsed without a signal
//         brake_motor(LEFT1_BRAKE_PIN);
//         brake_motor(LEFT2_BRAKE_PIN);
//         brake_motor(RIGHT1_BRAKE_PIN);
//         brake_motor(RIGHT2_BRAKE_PIN);

//         Serial.println("No signal received. Motors stopped for safety.");
//     }
// }