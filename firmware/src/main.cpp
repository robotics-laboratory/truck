// ------------------------------------------------------------------
//  Packetizer configuration – MUST match the Python side
// ------------------------------------------------------------------
#define PACKETIZER_USE_CRC_AS_DEFAULT    // use CRC (enabled)
#define PACKETIZER_USE_INDEX_AS_DEFAULT  // use index‑based framing

#include <Arduino.h>
#include <Packetizer.h>
#include "encoders/esp32hwencoder/ESP32HWEncoder.h"

#define UART Serial0

#define OUT1 31
#define OUT2 51
#define OUT3 29
#define OUT4 30
#define OUT5 50
#define OUT6 28
#define OUT7 5
#define OUT8 49

#define IN1 27
#define IN2 32
#define IN3 33
#define IN4 46
#define IN5 47
#define IN6 48

#define ACCEL_PWM_PIN 26
#define STEERING_PWM_PIN 22
#define FORWARD_PIN OUT1
#define BACKWARD_PIN OUT2

#define STEERING_MIDPOINT 512

ESP32HWEncoder ENCODER1 = ESP32HWEncoder(IN1, IN2, 100);
ESP32HWEncoder ENCODER2 = ESP32HWEncoder(20, 21, 100);

// ====== Packed structs for payload ======

struct __attribute__((packed)) Command {
  uint8_t type;       // 1=activate, 2=deactivate, 3=control
  float   steering;
  float   speed;
};

struct __attribute__((packed)) Status {
  float   enc1_speed_ticks;
  float   enc1_angle_ticks;
  float   enc2_speed_ticks;
  float   enc2_angle_ticks;
  uint8_t active;     // 1 = active, 0 = inactive
};

// ====== Global state ======

bool          active              = false;
float         current_speed       = 0.0f;
float         current_steering    = 0.0f;
unsigned long last_command_time   = 0;

const unsigned long TIMEOUT_MS       = 500;
const unsigned long STATUS_PERIOD_US = 10000;   // 100 Hz

// ====== Placeholder robot functions ======

void activate() {
  active = true;

  digitalWrite(OUT3, HIGH);
  // TODO: enable motors
}

void deactivate() {
  active = false;
  current_speed    = 0.0f;
  current_steering = 0.0f;

  digitalWrite(OUT3, LOW);
  digitalWrite(FORWARD_PIN, LOW);
  digitalWrite(BACKWARD_PIN, LOW);
  ledcWrite(ACCEL_PWM_PIN, 0);
  ledcWrite(STEERING_PWM_PIN, STEERING_MIDPOINT);
  // TODO: disable motors
}

void control(float steering, float speed) {
  // digitalWrite(OUT3, HIGH);
  current_steering = steering;
  current_speed    = speed;
  // TODO: send commands to motor controllers
}

// ====== Packet callbacks (called by Packetizer::parse()) ======

void onActivate(const uint8_t* data, const size_t size) {
  (void)data; (void)size;
  last_command_time = millis();
  if (!active) activate();
}

void onDeactivate(const uint8_t* data, const size_t size) {
  (void)data; (void)size;
  last_command_time = millis();
  if (active) deactivate();
}

void onControl(const uint8_t* data, const size_t size) {
  if (size != sizeof(Command)) return;
  last_command_time = millis();
  const Command* cmd = reinterpret_cast<const Command*>(data);
  if (!active) return;
  // Note: if not active, control commands are simply ignored
  if (cmd->speed < -1.0 || cmd->speed > 1.0) return;
  if (cmd->steering < -1.0 || cmd->steering > 1.0) return;
  if (cmd->speed > 0.01) {
    digitalWrite(FORWARD_PIN, HIGH);
    digitalWrite(BACKWARD_PIN, LOW);
  } else if (cmd->speed < -0.01) {
    digitalWrite(FORWARD_PIN, LOW);
    digitalWrite(BACKWARD_PIN, HIGH);
  } else {
    digitalWrite(FORWARD_PIN, LOW);
    digitalWrite(BACKWARD_PIN, LOW);
  }
  float accel_value = abs(cmd->speed) * 1023.0;
  ledcWrite(ACCEL_PWM_PIN, (uint32_t) accel_value);
  float steering_value = (cmd->steering + 1.0) / 2.0 * 1023.0;
  ledcWrite(STEERING_PWM_PIN, (uint32_t) steering_value);
}

// ====== Setup ======

void setup() {
  UART.begin(115200);

  ENCODER1.init();
  ENCODER2.init();

  pinMode(FORWARD_PIN, OUTPUT);
  pinMode(BACKWARD_PIN, OUTPUT);
  pinMode(OUT3, OUTPUT);

  digitalWrite(FORWARD_PIN, LOW);
  digitalWrite(BACKWARD_PIN, LOW);

  ledcAttach(ACCEL_PWM_PIN, 5000, 10);
  ledcWrite(ACCEL_PWM_PIN, 0);
  ledcAttach(STEERING_PWM_PIN, 5000, 10);
  ledcWrite(STEERING_PWM_PIN, STEERING_MIDPOINT);

  // Subscribe to command indices – the callbacks receive only the data part
  Packetizer::subscribe(UART, 1, &onActivate);   // activate
  Packetizer::subscribe(UART, 2, &onDeactivate); // deactivate
  Packetizer::subscribe(UART, 3, &onControl);    // control
}

// ====== Main loop ======

void loop() {
  // Feed incoming bytes to the packetizer
  Packetizer::parse();

  // Watchdog: deactivate if no packet within 500 ms while active
  if (active && (millis() - last_command_time >= TIMEOUT_MS)) {
    deactivate();
  }


    ENCODER1.update();
    ENCODER2.update();

  // Send status at 100 Hz
  static unsigned long last_status_time = 0;
  if (micros() - last_status_time >= STATUS_PERIOD_US) {
    last_status_time = micros();

    float enc1_angle_rads = ENCODER1.getAngle();
    float enc1_angle_ticks = enc1_angle_rads / 2.0 / PI * 100.0;
    float enc1_rads_per_second = ENCODER1.getVelocity();
    float enc1_tics_per_second = enc1_rads_per_second / 2.0 / PI * 100.0;

    float enc2_angle_rads = ENCODER2.getAngle();
    float enc2_angle_ticks = enc2_angle_rads / 2.0 / PI * 100.0;
    float enc2_rads_per_second = ENCODER2.getVelocity();
    float enc2_tics_per_second = enc2_rads_per_second / 2.0 / PI * 100.0;

    Status status;
    status.enc1_speed_ticks = enc1_tics_per_second;
    status.enc1_angle_ticks = enc1_angle_ticks;
    status.enc2_speed_ticks = enc2_tics_per_second;
    status.enc2_angle_ticks = enc2_angle_ticks;
    status.active           = active ? 1 : 0;

    // Send status with index 0x10
    Packetizer::send(UART, 0x10,
                     reinterpret_cast<const uint8_t*>(&status),
                     sizeof(Status));
  }
}

