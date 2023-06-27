#ifndef ARM_H
#define ARM_H

#include <Arduino.h>
#include <Dynamixel2Arduino.h>

class Arm {
  
  private:
    uint16_t revs_per_mm = 2048;
    Dynamixel2Arduino dxl;
    byte motor_id;
    byte button_pin;
    int offset;

  public:
    Arm(Dynamixel2Arduino dxl, byte motor_id, byte button_pin, int offset);
    void init();
    void move_mm(int);
    void freeze();
    bool is_limit_hit();

};

#endif
