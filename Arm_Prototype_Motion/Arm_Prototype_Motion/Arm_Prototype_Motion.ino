#include <Dynamixel2Arduino.h>
#include "arm.h"

#define motor_serial Serial1
#define debug_serial Serial
const int dxl_dir_pin = -1;


const float protocol_version = 2.0;
const int baud_rate = 57600;
const byte num_arms = 6;

byte button_state;

Arm arms[num_arms];

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


  debug_serial.print("Configuring motors...");

  // Define arm parameters: overall length limits, motor rotation offsets from zero, and
  // GPIO pins for hard limits
  const float min_length = 226.8;
  const float max_length = 310;
  const int arm_offsets[num_arms] = {-2100, -7000, -7800, -5400, -1000, -1000};
  const byte button_pins[num_arms] = {0, 2, 4, 6, 8, 10};

  for (byte i = 0; i < num_arms; i++) {
    arms[i] = Arm(dxl, i+1, button_pins[i], arm_offsets[i], min_length, max_length);
  }
  debug_serial.println("done.");

  debug_serial.println("Homing platform");
  set_home();
  debug_serial.println("Platform homing complete.");
  debug_serial.println("Ready.");
}
  
void loop() {
  delay(10);
  float values[num_arms + 1];
  while (!debug_serial.available()) {}
  for (byte i = 0; i < num_arms + 1; i++) {
     values[i] = handle_serial_input();
  }
  float array_sum = 0;
  for (byte i = 0; i < num_arms; i++) {
    array_sum = array_sum + values[i];
  }
  if (array_sum - values[num_arms] < 0.001) {
    debug_serial.println("Data good");
  }
  for (byte i = 0; i < num_arms; i++) {
    arms[i].set_length(values[i]);
    delay(10);
  }
}

float handle_serial_input() {
  if (debug_serial.available() > 0) {
    String incoming_string = debug_serial.readStringUntil('\n');
    incoming_string.trim();
    float incoming_value = atof(incoming_string.c_str());
    debug_serial.print("Received: ");
    debug_serial.println(incoming_value);
    return incoming_value;
  }
}

void set_home() {
  // Move all arms towards limit
  debug_serial.print("Moving to lower limit...");
  delay(1000);
  for (byte i = 0; i < num_arms; i++) {
    arms[i].move_mm(-200);
  }

  // When one arm reaches limit, freeze all arms
  bool is_at_limit = false;
  while (is_at_limit == false) {
    delay(3);
    for (byte i = 0; i < num_arms; i++) {
      if (arms[i].is_limit_hit()){
        is_at_limit = true;
        break;
      }
    }
  }
  for (byte i = 0; i < num_arms; i++) {
    arms[i].freeze();
  }
  debug_serial.println("done");
  delay(500);

  // Individually home each motor
  debug_serial.println("Finding lower limits");
  for (byte i = 0; i < num_arms; i++) {
    debug_serial.print("   Arm ");
    debug_serial.print(i + 1);
    debug_serial.print("...");
    delay(1000);
    
    // Turn arm one turn away from limit
    arms[i].move_mm(2);
    delay(800);
    
    // Move motor maximum distance towards lower limit
    arms[i].move_mm(-200);
    
    // If limit stop is hit, kill motor
    while (!arms[i].is_limit_hit()) {}
    arms[i].freeze();
    delay(200);

    // Reboot motor to clear turn counter, and go to home position
    arms[i].reboot();
    arms[i].move_home();
    delay(500);
    debug_serial.println("done.");
  }
  debug_serial.println("Lower limits set.");
}
