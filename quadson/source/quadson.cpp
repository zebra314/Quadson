#include "quadson.h"
#include "can_interface.h"

Quadson::Quadson(Can_interface *can_device) {
  this->can_device = can_device;
  can_device->can_init();
  can_device->can_open();

  for (size_t id = 0; id < 12; id++) {
    this->actuator[id - 1] = new Actuator(this->can_device, id);
  }
}

Quadson::~Quadson() {}

void Quadson::scan() { can_device->can_send_cmd(0, 0, 0); }

void Quadson::update() { process_cmd(); }

void Quadson::process_cmd() {
  while (!can_cmd_buffer.empty()) {
#ifdef DEBUG
    std::cout << "buffer not empty\n";
#endif
    can_cmd_t rec_cmd = can_cmd_buffer.front();
    can_cmd_buffer.pop_front();
    printf("receive id: %d, cmd: %d\n", rec_cmd.id, rec_cmd.cmd);

    switch (rec_cmd.cmd) {
    case CAN_STDID_ECHO:
      get_actuator_echo(rec_cmd.id);
      break;
    case CAN_STDID_ALL_PARAM:
    case CAN_STDID_INFO_PARAM:

    case CAN_STDID_TORQUE_ENABLE:
    case CAN_STDID_STATE_MACHINE:
    case CAN_STDID_CONTROL_MODE:

    case CAN_STDID_GOAL_REVOLUTION:
    case CAN_STDID_GOAL_POSITION_DEG:
    case CAN_STDID_GOAL_VELOCITY_DPS:
    case CAN_STDID_GOAL_TORQUE_CURRENT_MA:
    case CAN_STDID_GOAL_FLUX_CURRENT_MA:
    case CAN_STDID_PRESENT_REVOLUTION:
    case CAN_STDID_PRESENT_POSITION_DEG:
    case CAN_STDID_PRESENT_VELOCITY_DPS:
    case CAN_STDID_PRESENT_TORQUE_CURRENT_MA:
    case CAN_STDID_PRESENT_FLUX_CURRENT_MA:
    case CAN_STDID_PRESENT_VOLTAGE:
    case CAN_STDID_PRESENT_TEMPERATURE:
    case CAN_STDID_ZERO_DONE:
    case CAN_STDID_GROUP0:
    case CAN_STDID_GROUP1:
    case CAN_STDID_GROUP2:
    case CAN_STDID_GROUP3:
      break;
    }
  }
}

void Quadson::get_actuator_echo(int id) {
  this->actuator[id - 1]->connect(true);
}