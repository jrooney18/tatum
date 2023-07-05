#include "arm.h"
#include <Dynamixel2Arduino.h>

Arm::Arm() {
}

Arm::Arm(Dynamixel2Arduino dxl, byte motor_id, byte button_pin, int offset, float min_length, float max_length) {
  this->dxl = dxl;
  this->motor_id = motor_id;
  this->button_pin = button_pin;
  this->offset = offset;
  this->min_length = min_length;
  this->max_length = max_length;
  init();
}

void Arm::init() {
  dxl.ping(motor_id);
  dxl.torqueOff(motor_id);
  dxl.setOperatingMode(motor_id, OP_EXTENDED_POSITION);
  dxl.torqueOn(motor_id);
  pinMode(button_pin, INPUT_PULLUP);
  float current_length = 0;
}

void Arm::reboot() {
  dxl.reboot(motor_id);
  delay(800);
  dxl.setOperatingMode(motor_id, OP_EXTENDED_POSITION);
  dxl.torqueOn(motor_id);
}

void Arm::move_home() {
  dxl.setGoalPosition(motor_id, offset);
  current_length = min_length;
}

void Arm::move_mm(int dist) {
  int current_pos = dxl.getPresentPosition(motor_id);
  int num_revs = -dist * revs_per_mm;
  dxl.setGoalPosition(motor_id, current_pos + num_revs);
  current_length = current_length + dist;
}

void Arm::freeze() {
  int current_pos = dxl.getPresentPosition(motor_id);
  dxl.setGoalPosition(motor_id, current_pos);
}

bool Arm::is_limit_hit() {
  bool button_state = digitalRead(button_pin);
  if (button_state == LOW) {
    return true;
  }
  return false;
}

float Arm::get_current_length() {
  return current_length;
}

bool Arm::set_length(float len) {
  if (len < min_length or len > max_length) {
    return false;
  }
  float move_length = len - current_length;
  move_mm(move_length);
  return true;
}
