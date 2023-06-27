#include <Dynamixel2Arduino.h>
#include "arm.h"

#define motor_serial Serial1
#define debug_serial Serial
const int dxl_dir_pin = -1;


const float protocol_version = 2.0;
const int baud_rate = 57600;
const byte num_arms = 6;

const byte button_pins[num_arms] = {0, 2, 4, 6, 8, 10};
byte button_state;

// Rotation value offsets for level platform that clears limits
const int arm_offsets[num_arms] = {-2100, -7000, -7800, -5400, -1000, -1000};

Dynamixel2Arduino dxl(motor_serial, dxl_dir_pin);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // Use UART port of DYNAMIXEL Shield to debug.
  debug_serial.begin(115200);
  while(!debug_serial);
  debug_serial.println("Initializing Arm Prototype 0.5");

  // Set Port baudrate and protocol version.
  dxl.begin(baud_rate);
  dxl.setPortProtocolVersion(protocol_version);

  // Configure each arm
  debug_serial.print("Configuring motors...");
  Arm arms[] = {
    Arm(dxl, 1, button_pins[0], arm_offsets[0]),
    Arm(dxl, 2, button_pins[1], arm_offsets[1]),
    Arm(dxl, 3, button_pins[2], arm_offsets[2]),
    Arm(dxl, 4, button_pins[3], arm_offsets[3]),
    Arm(dxl, 5, button_pins[4], arm_offsets[4]),
    Arm(dxl, 6, button_pins[5], arm_offsets[5])
  };
  
//  for (byte motor_id = 1; motor_id <= num_arms; motor_id++) {
//    // Get motor info
//    dxl.ping(motor_id);
//    // Set motor operating mode
//    dxl.torqueOff(motor_id);
//    dxl.setOperatingMode(motor_id, OP_EXTENDED_POSITION);
//    dxl.torqueOn(motor_id);
//    // Set motor speed (0 = max. speed)
//    // dxl.writeControlTableItem(PROFILE_VELOCITY, motor_id, 120);
//  }
  debug_serial.println("done.");

  // Configure button pins as digital input
//  for (int i = 0; i < num_arms; i++) {
//    pinMode(button_pins[i], INPUT_PULLUP);
//  }

  debug_serial.print("Moving to lower limit...");
  delay(1000);
  arms[3].move_mm(10);
  while (!arms[3].is_limit_hit()) {
    delay(5);
  }
  arms[3].freeze();
  debug_serial.println("done");
  // Move all motors towards home
//  int motor_position;
//  for (byte motor_id = 1; motor_id <= num_arms; motor_id++) {
//    motor_position = dxl.getPresentPosition(motor_id);
//    dxl.setGoalPosition(motor_id, motor_position + 200000);
//  }
//
//  // While moving, check if any limit stop is hit
//  bool is_at_limit = false;
//  while (is_at_limit == false) {
//    delay(3);
//    for (byte motor_id = 1; motor_id <= num_arms; motor_id++) {
//      button_state = digitalRead(button_pins[motor_id - 1]);
//      if (button_state == LOW) {
//        is_at_limit = true;
//        break;
//      }
//    }
//  }
//
//  // When limit stop is hit, disable all motors
//  for (byte motor_id = 1; motor_id <= num_arms; motor_id++) {
//    dxl.torqueOff(motor_id);
//  }
//  debug_serial.println("done.");
//  delay(1500);
//
//  // Individually home each motor
//  debug_serial.println("Finding lower limits");
//  for (byte motor_id = 1; motor_id <= num_arms; motor_id++) {
//    dxl.torqueOn(motor_id);
//    debug_serial.print("   Arm ");
//    debug_serial.print(motor_id);
//    debug_serial.print("...");
//    delay(1000);
//    
//    // Turn arm one turn away from limit
//    int motor_position = dxl.getPresentPosition(motor_id);
//    motor_position = motor_position - 4096;
//    dxl.setGoalPosition(motor_id, motor_position);
//    delay(800);
//    
//    // Move motor maximum distance towards lower limit
//    motor_position = motor_position + 200000;
//    dxl.setGoalPosition(motor_id, motor_position);
//    
//    // If limit stop is hit, kill motor
//    button_state = HIGH;
//    while (button_state != LOW) {
//      delay(10);
//      button_state = digitalRead(button_pins[motor_id - 1]);
//    }
//    dxl.torqueOff(motor_id);
//    delay(500);
//
//    // Reboot motor to clear turn counter and set position to zero
//    dxl.reboot(motor_id);
//    delay(1000);
//    dxl.torqueOn(motor_id);
//    dxl.setGoalPosition(motor_id, 0);
//    delay(500);
//    debug_serial.println("done.");
//  }
//  debug_serial.println("Lower limits set");
//
//  // Move to home, defined by arm_offsets
//  debug_serial.print("Moving to home...");
//  for (byte motor_id = 1; motor_id <= num_arms; motor_id++) {
//    dxl.setGoalPosition(motor_id, arm_offsets[motor_id - 1]);
//  }
//  delay(1500);
//  debug_serial.println("done. Platform homing complete.");
}

void loop() {
  while(!debug_serial.available());
  String message = debug_serial.readString();
  debug_serial.println("Hello!");
}
