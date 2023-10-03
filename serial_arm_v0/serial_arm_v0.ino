/*
  SERIAL ARM CONTROLLER V0
  2 October 2023
  Runs simple, pre-programmed signs on the 7-motor serial arm prototype.
  To add a sign, find its keypoints in the Dynamixel wizard, then add them as a new pose.
  Each pose should have a corresponding serial command in the main body of the code.

  Currently, sign 1 is "Hello" and sign 2 is "Hello" (improved).
*/

#include <Dynamixel2Arduino.h>

#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = -1;

const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

const int num_motors = 7;
int dxl_ids[num_motors] = {1, 2, 3, 4, 5, 6, 7};

int home_pose[num_motors] = {2050, 1200, 1850, 1700, 3000, 2000, 2000};

// Poses for sign 1
int pose011[num_motors] = {1200, 1600, 2500, 1800, 3700, 1600, 2000};
int pose012[num_motors] = {1200, 1600, 2500, 1250, 3500, 2200, 2000};

// Poses for sign 2
int pose021[num_motors] = {1750, 1300, 1700, 1600, 2900, 2000, 1650};
int pose022[num_motors] = {1850, 1300, 1900, 1650, 2800, 2000, 2300};
int pose023[num_motors] = {1950, 1300, 2100, 1450, 2700, 2000, 1700};

void motors_on() {
  // Turn off torque when configuring items in EEPROM area
  for (int i = 0; i < num_motors; i++) {
    dxl.ping(dxl_ids[i]);
    dxl.torqueOff(dxl_ids[i]);
    dxl.setOperatingMode(dxl_ids[i], OP_POSITION);
    dxl.writeControlTableItem(DRIVE_MODE, dxl_ids[i], 4);  // Configure time-based profile
    dxl.torqueOn(dxl_ids[i]);
    dxl.writeControlTableItem(PROFILE_VELOCITY, dxl_ids[i], 800);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, dxl_ids[i], 500);
    dxl.setGoalPosition(dxl_ids[i], home_pose[i]);
  }
  delay(1000);
  DEBUG_SERIAL.println("Power on");
}

void motors_off() {
  for (int i = 0; i < num_motors; i++) {
    dxl.torqueOff(dxl_ids[i]);
  }
  delay(200);
  DEBUG_SERIAL.println("Power off");
}

void pose_arm(int pose[num_motors], int move_time, int accel = 0, int wait_time = 0) {
  for (int i = 0; i < num_motors; i++) {
    dxl.writeControlTableItem(PROFILE_VELOCITY, dxl_ids[i], move_time);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, dxl_ids[i], accel);
  }
  for (int i = 0; i < num_motors; i++) {
    dxl.setGoalPosition(dxl_ids[i], pose[i]);
  }
  
  if(wait_time == 0){
    wait_time = move_time;
  }
  delay(wait_time);

  for (int i = 0; i < num_motors; i++) {
    DEBUG_SERIAL.println(dxl.getPresentPosition(dxl_ids[i]));
  }
}

void setup() {
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Disable motor power from the OpenRB
  pinMode(31, OUTPUT);
  digitalWrite(31, LOW);

  motors_on();
}

void loop() {
  // Wait for serial input
  while(DEBUG_SERIAL.available() == 0) {}
  int sign = DEBUG_SERIAL.read(); 
  DEBUG_SERIAL.println(sign);

  // "9" turns power on
  if(sign == 57) {
    motors_on();
  }

  // "0" turns power off
  if(sign == 48) {
    motors_off();
  }

  // Serial input 1 makes first sign shape
  if(sign == 49){
    pose_arm(pose011, 750, 250);
    delay(900);
    pose_arm(pose012, 1000, 250);
  }

  // Serial input 2 makes second sign shape
  if(sign == 50){
    pose_arm(pose021, 1000, 200);
    delay(450);
    pose_arm(pose022, 500, 250, 150);
    pose_arm(pose023, 500, 250);
  }

  // "h" returns to home
  if(sign == 104){
    pose_arm(home_pose, 1000, 400);
  }
}
