#include "actuator.h"

Actuator::Actuator(Can_interface *can_device, int id) {
  this->can_device = can_device;
  this->id = id;
  this->connected = false;
  this->zeroed = false;
}

Actuator::~Actuator() {}

void Actuator::connect(bool connect) { this->connected = connect; }
bool Actuator::isConnected() { return this->connected; }

void Actuator::zero(bool zero) { this->zeroed = zero; }
bool Actuator::isZeroed() { return this->zeroed; }

void Actuator::torque_enable(int num){
  this->can_device->can_send_cmd(this->id, CAN_STDID_TORQUE_ENABLE, num);
}

void Actuator::goal_revolution(int num){
  this->can_device->can_send_cmd(this->id, CAN_STDID_GOAL_REVOLUTION, num);
}

void Actuator::goal_position_deg(int num){
  this->can_device->can_send_cmd(this->id, CAN_STDID_GOAL_POSITION_DEG, num);
}

void Actuator::goal_velocity_dps(int num){
  this->can_device->can_send_cmd(this->id, CAN_STDID_GOAL_VELOCITY_DPS, num);
}

void Actuator::check_zero_done(void){
  this->can_device->can_send_cmd(this->id, CAN_STDID_ZERO_DONE, 0);
}

void Actuator::get_info_param(void){
  this->can_device->can_send_cmd(this->id, CAN_STDID_INFO_PARAM, 0);
}

void Actuator::get_all_param(void){
  this->can_device->can_send_cmd(this->id, CAN_STDID_ALL_PARAM, 0);
}

void Actuator::control_mode(int num){
  this->can_device->can_send_cmd(this->id, CAN_STDID_CONTROL_MODE, num);
}