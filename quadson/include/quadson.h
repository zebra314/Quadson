#ifndef INC_QUADSON_H
#define INC_QUADSON_H

#include "actuator.h"
#include "can_interface.h"
#include "leg_group.h"

class Quadson {
public:
  Quadson(Can_interface *can_device);
  ~Quadson();
  void update();
  void scan();
  void moving_test();

private:
  Can_interface *can_device;

  Leg_group FR;
  Leg_group FL;
  Leg_group RR;
  Leg_group RL;
  Actuator *actuator[12];
  void process_cmd();

  void get_actuator_echo(int);
  void get_actuator_zero_state(int id, int value);
  void get_actuator_present_deg(int id, int value);
};
#endif // !INC_QUADSON_H