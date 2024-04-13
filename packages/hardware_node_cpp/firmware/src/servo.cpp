#include <Arduino.h>
#include <Servo.h>
#include <ACAN_T4.h>
#include <cstring>
#include <cassert>

#define SERIAL_SPEED 115200
#define LEFT_SERVO_PWM_PIN 2
#define RIGHT_SERVO_PWM_PIN 3
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERIAL_PORT Serial

Servo leftServo;
Servo rightServo;

#pragma pack(push, 1)
struct ServoAngles {
    float left_angle;
    float right_angle;
};
#pragma pack(pop))

float clamp(float x) {
    if (x < SERVO_MIN_ANGLE) return SERVO_MIN_ANGLE;
    if (x > SERVO_MAX_ANGLE) return SERVO_MAX_ANGLE;
    return x;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void writeFloat(Servo& servo, float angle) {
    int value = mapFloat(clamp(angle), 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    servo.writeMicroseconds(value);
}

void handleCommand(const ServoAngles& cmd) {
    if (cmd.left_angle > 0) writeFloat(leftServo, cmd.left_angle);
    if (cmd.right_angle > 0) writeFloat(rightServo, cmd.right_angle);
    SERIAL_PORT.println("ok");
}

void setup() {
    delay(1000);
    SERIAL_PORT.begin(SERIAL_SPEED);
    SERIAL_PORT.println("teensy init");
    leftServo.attach(LEFT_SERVO_PWM_PIN);
    rightServo.attach(RIGHT_SERVO_PWM_PIN);
    leftServo.write(90);
    rightServo.write(90);
    ACAN_T4_Settings settings(500 * 1000);
    const uint32_t errorCode = ACAN_T4::can1.begin(settings);
    if (0 == errorCode) {
        Serial.println ("can1 ok") ;
    } else {
        Serial.print ("Error can1: 0x") ;
        Serial.println (errorCode, HEX) ;
    }
}

// 0-3 bytes: left angle
// 4-7 bytes: right angle

void loop() {
    CANMessage message;
    if (ACAN_T4::can1.receive(message)) {
        if (message.id == 0x555) {
            Serial.print("recived message\n");
            struct ServoAngles cmd{};
            std::memcpy(&cmd.left_angle, message.data, sizeof(cmd.left_angle));
            std::memcpy(&cmd.right_angle, message.data + 4, sizeof(cmd.right_angle));
            Serial.print("left: ");
            Serial.println(cmd.left_angle);
            Serial.print("right: ");
            Serial.println(cmd.right_angle);
            handleCommand(cmd);
        }
    }
}