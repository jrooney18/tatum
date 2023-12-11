#include <Dynamixel2Arduino.h>
// #include "serial_arm.h"

#define motor_serial Serial1
#define debug_serial Serial
const int dxl_dir_pin = -1;


const float protocol_version = 2.0;
const int baud_rate = 57600;

const int num_motors = 7;
byte dxl_ids[num_motors] = {1, 2, 3, 4, 5, 6, 7};

float home_pose[num_motors] = {2050, 1200, 1850, 1700, 3000, 2000, 2000};

// Arm arms[num_arms];

Dynamixel2Arduino dxl(motor_serial, dxl_dir_pin);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

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

void motors_on() {
  // Turn off torque when configuring items in EEPROM area
  for (byte i = 0; i < num_motors; i++) {
    dxl.ping(dxl_ids[i]);
    dxl.torqueOff(dxl_ids[i]);
    dxl.setOperatingMode(dxl_ids[i], OP_POSITION);
    dxl.writeControlTableItem(DRIVE_MODE, dxl_ids[i], 4);  // Configure time-based profile
    dxl.torqueOn(dxl_ids[i]);
  }
  debug_serial.println("Power on");
}

void motors_off() {
  for (byte i = 0; i < num_motors; i++) {
    dxl.torqueOff(dxl_ids[i]);
  }
  delay(200);
  debug_serial.println("Power off");
}

void pose_arm(float pose[num_motors], int move_time, int accel = 0, int wait_time = 0) {
  for (byte i = 0; i < num_motors; i++) {
    dxl.writeControlTableItem(PROFILE_VELOCITY, dxl_ids[i], move_time);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, dxl_ids[i], accel);
  }
  for (byte i = 0; i < num_motors; i++) {
    dxl.setGoalPosition(dxl_ids[i], pose[i]);
  }
  
  if(wait_time == 0){
    wait_time = move_time;
  }
  delay(wait_time);

//  for (byte i = 0; i < num_motors; i++) {
//    debug_serial.println(dxl.getPresentPosition(dxl_ids[i]));
//  }
}

void setup() {
  // Use UART port of DYNAMIXEL Shield to debug.
  debug_serial.begin(115200);
  while(!debug_serial);
  debug_serial.println("Initializing Serial Arm V1");

  // Initialize Dynamixel communictaion
  dxl.begin(baud_rate);
  dxl.setPortProtocolVersion(protocol_version);

  // Disable motor power from the OpenRB
  pinMode(31, OUTPUT);
  digitalWrite(31, LOW);

  debug_serial.print("Configuring motors...");
  motors_on();
  debug_serial.println("done.");

  debug_serial.print("Moving to home...");
  pose_arm(home_pose, 800, 400);
  delay(1000);
  debug_serial.println("done.");
  debug_serial.println("Ready.");
}
  
void loop() {
  delay(10);
  float values[num_motors + 1];
  while (!debug_serial.available()) {}
  for (byte i = 0; i < num_motors + 1; i++) {
    values[i] = handle_serial_input();
  }
  float array_sum = 0;
  for (byte i = 0; i < num_motors; i++) {
    array_sum = array_sum + values[i];
  }
  if (array_sum - values[num_motors] < 0.001) {
    debug_serial.println("Data good");
  }
  
  pose_arm(values, 800, 400);
  delay(10);
}
