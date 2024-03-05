#include <Arduino.h>
#include <Servo.h>
#include <MsgPacketizer.h>

#define SERIAL_SPEED 500000
#define LEFT_SERVO_PWM_PIN 2
#define RIGHT_SERVO_PWM_PIN 3
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define MSGPACK_RECV_INDEX 0x01
#define MSGPACK_SEND_INDEX 0x02
#define SERIAL_PORT Serial1

Servo leftServo;
Servo rightServo;

struct ServoCommand {
    float l;
    float r;
    MSGPACK_DEFINE(l, r);
};

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

void handleCommand(const ServoCommand& cmd) {
    if (cmd.l > 0) writeFloat(leftServo, cmd.l);
    if (cmd.r > 0) writeFloat(rightServo, cmd.r);
    SERIAL_PORT.println("ok");
}

void setup() {
    SERIAL_PORT.begin(SERIAL_SPEED);
    SERIAL_PORT.println("teensy init");
    MsgPacketizer::subscribe(SERIAL_PORT, MSGPACK_RECV_INDEX, &handleCommand);
    leftServo.attach(LEFT_SERVO_PWM_PIN);
    rightServo.attach(RIGHT_SERVO_PWM_PIN);
    leftServo.write(90);
    rightServo.write(90);
}

void loop() {
    MsgPacketizer::update();
}
