#include "arm.h"
#include <Dynamixel2Arduino.h>

Arm::Arm(Dynamixel2Arduino dxl, byte motor_id, byte button_pin, int offset) {
  this->dxl = dxl;
  this->motor_id = motor_id;
  this->button_pin = button_pin;
  this->offset = offset;
  init();
}

void Arm::init() {
  dxl.ping(motor_id);
  dxl.torqueOff(motor_id);
  dxl.setOperatingMode(motor_id, OP_EXTENDED_POSITION);
  dxl.torqueOn(motor_id);
  pinMode(button_pin, INPUT_PULLUP);
}

void Arm::move_mm(int dist) {
  int current_pos = dxl.getPresentPosition(motor_id);
  int num_revs = dist * revs_per_mm;
  dxl.setGoalPosition(motor_id, current_pos + num_revs);
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
