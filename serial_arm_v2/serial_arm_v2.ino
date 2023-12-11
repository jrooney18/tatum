#include <Dynamixel2Arduino.h>
// #include "serial_arm.h"

#define motor_serial Serial1
#define debug_serial Serial
const int dxl_dir_pin = -1;


const float protocol_version = 2.0;
const int baud_rate = 57600;

const int num_motors = 22;
byte dxl_ids[num_motors] = {21, 22, 23, 24, 18, 17, 16, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

float home_pose[num_motors] = {2050, 600, 2050, 800, // Arm
                               2050, 2050, 2050,     // Wrist
                               1000, 1000, 2050,     // Pinkie
                               1200, 1200, 2050,     // Ring finger
                               1300, 1200, 2050,     // Middle finger
                               1000, 1000, 2050,     // Index finger
                               1000, 1400, 1000};    // Thumb
                               
float hello1[num_motors] = {1400, 1000, 2400, 2000,
                            4000, 2050, 2050,
                            1000, 1000, 2050,
                            1200, 1200, 2050,
                            1300, 1200, 2050,
                            1000, 1000, 2050,
                            1000, 1700, 1000};
float hello2[num_motors] = {1300, 1100, 2400, 1500,
                            4000, 2050, 2050,
                            1000, 1000, 2050,
                            1200, 1200, 2050,
                            1300, 1200, 2050,
                            1000, 1000, 2050,
                            1000, 1700, 1000};

float my1[num_motors] = {1500, 700, 1400, 1900,
                         2900, 2050, 2050,
                         1000, 1000, 2300,
                         1200, 1200, 1800,
                         1300, 1200, 2050,
                         1000, 1000, 2050,
                         1300, 2200, 1000};

float name1[num_motors] = {1500, 700, 1800, 1850,
                           3600, 2050, 2050,
                           2800, 2500, 2050,
                           2800, 2500, 2050,
                           1300, 1200, 2050,
                           1000, 1000, 2050,
                           2500, 2400, 1500};
float name2[num_motors] = {1500, 700, 1500, 1770,
                           3200, 2050, 2050,
                           2800, 2500, 2050,
                           2800, 2500, 2050,
                           1300, 1200, 2050,
                           1000, 1000, 2050,
                           2500, 2400, 1500};
float name3[num_motors] = {1500, 700, 1800, 1850,
                           3600, 2050, 2050,
                           2800, 2500, 2050,
                           2800, 2500, 2050,
                           1300, 1200, 2050,
                           1000, 1000, 2050,
                           2500, 2400, 1500};
float name4[num_motors] = {1500, 700, 1500, 1770,
                           3200, 2050, 2050,
                           2800, 2500, 2050,
                           2800, 2500, 2050,
                           1300, 1200, 2050,
                           1000, 1000, 2050,
                           2500, 2400, 1500};
float name5[num_motors] = {1500, 700, 1800, 1850,
                           3600, 2050, 2050,
                           2800, 2500, 2050,
                           2800, 2500, 2050,
                           1300, 1200, 2050,
                           1000, 1000, 2050,
                           2500, 2400, 1500};

float robot1[num_motors] = {1750, 787, 1140, 1440,
                            2800, 2050, 2050,
                            1000, 1000, 2050,
                            1200, 1200, 2050,
                            1300, 1200, 2050,
                            1000, 1000, 2050,
                            1500, 2400, 1400};

float robot2[num_motors] = {1600, 750, 1320, 1820,
                            2800, 2050, 2050,
                            1000, 1000, 2050,
                            1200, 1200, 2050,
                            1300, 1200, 2050,
                            1000, 1000, 2050,
                            1500, 2400, 1400};

float how1[num_motors] = {1800, 720, 1400, 1570,
                          400, 2150, 2050,
                          3500, 2400, 2050,
                          4000, 3000, 2050,
                          4000, 3000, 2050,
                          4000, 3000, 2050,
                          1000, 1000, 1000}; 

float how2[num_motors] = {1800, 720, 1400, 1570,
                          4000, 2150, 2050,
                          3400, 2400, 2050,
                          4000, 3000, 2050,
                          4000, 3000, 2050,
                          4000, 3000, 2050,
                          1000, 1000, 1000}; 

float you1[num_motors] = {2000, 710, 2000, 1350,
                          4000, 2150, 2050,
                          3300, 2400, 2050,
                          4000, 3000, 2050,
                          4000, 3000, 2050,
                          1000, 1000, 2050,
                          2000, 2000, 2000}; 
                            
float spelling[num_motors] = {1830, 760, 1920, 1930,
                              200, 2050, 2050,
                              1000, 1000, 2050,
                              1000, 1000, 2050,
                              1000, 1000, 2050,
                              1000, 1000, 2050,
                              1000, 1000, 1000}; // Fingerspelling pose

//int pose021[num_motors] = {1750, 1300, 1700, 1600, 2900, 2000, 1650};
//int pose022[num_motors] = {1850, 1300, 1900, 1650, 2800, 2000, 2300};
//int pose023[num_motors] = {1950, 1300, 2100, 1450, 2700, 2000, 1700};

// Arm arms[num_arms];

Dynamixel2Arduino dxl(motor_serial, dxl_dir_pin);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void motors_on() {
  // Turn off torque when configuring items in EEPROM area
  for (byte i = 0; i < num_motors; i++) {
    dxl.ping(dxl_ids[i]);
    dxl.torqueOff(dxl_ids[i]);
    dxl.setOperatingMode(dxl_ids[i], OP_POSITION);
    dxl.writeControlTableItem(DRIVE_MODE, dxl_ids[i], 4);  // Configure time-based profile
    dxl.torqueOn(dxl_ids[i]);
    dxl.writeControlTableItem(POSITION_P_GAIN, dxl_ids[i], 8000);
    dxl.writeControlTableItem(POSITION_I_GAIN, dxl_ids[i], 1500);
    dxl.writeControlTableItem(POSITION_D_GAIN, dxl_ids[i], 10000);
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
//    dxl.writeControlTableItem(PROFILE_ACCELERATION, dxl_ids[i], accel);
  }
  for (byte i = 0; i < num_motors; i++) {
    dxl.setGoalPosition(dxl_ids[i], pose[i]);
    delay(25);
  }
  
  if(wait_time == 0){
    wait_time = move_time;
  }
  delay(wait_time);

//  for (byte i = 0; i < num_motors; i++) {
//    debug_serial.println(dxl.getPresentPosition(dxl_ids[i]));
//  }
}

void reset_motors() {
  for(byte i = 0; i < num_motors; i++) {
    dxl.reboot(dxl_ids[i]);
    delay(500);
    dxl.torqueOn(dxl_ids[i]);
    if(dxl_ids[i] != 24){
      dxl.writeControlTableItem(POSITION_P_GAIN, dxl_ids[i], 8000);
      dxl.writeControlTableItem(POSITION_I_GAIN, dxl_ids[i], 1500);
      dxl.writeControlTableItem(POSITION_D_GAIN, dxl_ids[i], 10000);
    }
  }
}

void setup() {
  // Use UART port of DYNAMIXEL Shield to debug.
  debug_serial.begin(115200);
  while(!debug_serial);
  debug_serial.println("Initializing Serial Arm V2");

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
  pose_arm(home_pose, 1000, 400);
  delay(1000);
  debug_serial.println("done.");
  debug_serial.println("Ready.");
}
  
void loop() {
  while(debug_serial.available() == 0) {}
  int sign = debug_serial.read();
//  debug_serial.println(sign);

  switch(sign) {
    // Serial input "1" makes first sign shape
    case 49:
      debug_serial.print("Forming sign 1...");
      pose_arm(hello1, 1250, 400);
      delay(500);
      pose_arm(hello2, 1000, 250);
      delay(900);
      pose_arm(my1, 1300, 300);
      delay(600);
      pose_arm(name1, 800, 300);
      delay(200);
      pose_arm(name2, 250, 100, 50);
      pose_arm(name3, 250, 100, 50);
      pose_arm(name4, 250, 100, 50);
      pose_arm(name5, 250, 100, 50);
      delay(600);
      for(byte i = 0; i < 3; i++) {
        pose_arm(robot1, 300, 300, 20);
        pose_arm(robot2, 300, 300, 20);
      }
      delay(3000);
      pose_arm(home_pose, 1500, 500);
      debug_serial.println("done.");
      break;

    // Serial input "2" makes second sign shape
    case 50:
      debug_serial.print("Forming sign 2...");
      pose_arm(how1, 1000, 250);
      delay(400);
      pose_arm(how2, 500, 150);
      delay(200);
      pose_arm(you1, 500, 200);
      delay(1500);
      pose_arm(home_pose, 1000, 400);
      debug_serial.println("done.");
      break;

    // Serial input "3" makes third sign shape
    case 51:
      debug_serial.print("Forming sign 3...");
      pose_arm(hello1, 1250, 400);
      delay(500);
      pose_arm(hello2, 1000, 250);
      delay(3000);
      pose_arm(home_pose, 1500, 500);
      debug_serial.println("done.");
      break;
      
    // Serial input "h" returns home
    case 104:
      debug_serial.print("Moving to home...");
      pose_arm(home_pose, 1500, 500);
      debug_serial.println("done.");
      break;

    // Serial input "r" resets motors
    case 114:
      debug_serial.print("Resetting motors...");
      reset_motors();
      pose_arm(home_pose, 1500, 500);
      debug_serial.println("done.");
      break;

    // Serial input "x" disables motors
    case 120:
      debug_serial.println("Disabling motors");
      motors_off();
      break;
  }
}
