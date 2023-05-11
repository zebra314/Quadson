#include "actuator.h"

Actuator::Actuator(Can_interface *can_device, int id) {
  this->can_device = can_device;
  this->id = id;
  this->connected = false;
}

Actuator::~Actuator() {}

void Actuator::connect(bool connect) { this->connected = connect; }
bool Actuator::isConnected() { return this->connected; }