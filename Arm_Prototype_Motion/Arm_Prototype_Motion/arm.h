#ifndef ARM_H
#define ARM_H

#include <Arduino.h>
#include <Dynamixel2Arduino.h>

class Arm {
  // Class for control of a single arm of a parallel-platform robot.
  
  private:
    uint16_t ticks_per_turn = 2048;
    Dynamixel2Arduino dxl;
    byte motor_id;
    byte button_pin;
    int offset;
    float min_length;
    float max_length;
    int max_rpm;
    float current_length;
    int pitch;

  public:
    Arm();
    Arm(Dynamixel2Arduino dxl, byte motor_id, byte button_pin, int offset, float min_length, float max_length, int max_rpm, int pitch);
    void init();
    void reboot();
      // Reboots motor, enables torque, and sets operating mode to Extended Position.
    void move_home();
      // Moves motor to home position defined by "offset".
    void move_mm(int);
      // Moves arm by a given length in mm. Positive value lengthens arm,
      // negative value shortens it. Returns duration of move in milliseconds.
    void freeze();
      // Stops arm at current position.
    bool is_limit_hit();
      // Returns true if arm's limit stop is pressed; else returns false.
    float get_current_length();
      // Returns current length
    bool set_length(float);
      // Sets overall arm length in mm. Rejects input outside of defined range.
      // Returns true if a valid length is entered, or false if not.
    int get_move_duration(float);
      // Returns duration of movement from current length to entered length, at max RPM
    void set_move_duration(int);
      // Sets duration of movement in milliseconds
};

#endif
