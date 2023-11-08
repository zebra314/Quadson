#include "quadson.h"
#include "can_interface.h"
#include <unistd.h>
#include <math.h>

Quadson::Quadson(Can_interface *can_device) {
  this->can_device = can_device;
  can_device->can_init();
  can_device->can_open();

  for (size_t id = 0; id < 12; id++) {
    this->actuator[id] = new Actuator(this->can_device, id+1);
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
    printf("receive id: %d, cmd: %d, return value:%d\n", rec_cmd.id, rec_cmd.cmd, rec_cmd.value);

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
      get_actuator_zero_state(rec_cmd.id, rec_cmd.value);

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

void Quadson::get_actuator_zero_state(int id, int value) {
  this->actuator[id - 1]->zero(bool(value));
}

void Quadson::moving_test(){
  Leg_group leg_test(this->actuator[0], this->actuator[1]);

  // #define ZERO
  // #define Input
  // #define Square
  // #define Triangle
  // #define Circle

  // Reset the position
  #ifdef ZERO
  std::cout<<"----------------Start Zero----------------\n";    
  leg_test.torque_enable(1);
  this->actuator[0]->control_mode(1);
  this->actuator[1]->control_mode(1);
  while (1){
    this->actuator[0]->check_zero_done();  
    this->actuator[1]->check_zero_done();     
    update();
    if (this->actuator[0]->isZeroed() == 1 and this->actuator[1]->isZeroed() == 1){
      this->actuator[0]->goal_velocity_dps(0);
      this->actuator[1]->goal_velocity_dps(0);
      break;
    }
      
    this->actuator[0]->goal_velocity_dps(30);
    this->actuator[1]->goal_velocity_dps(30);
  }
  sleep(1);
  this->actuator[0]->control_mode(0);
  this->actuator[1]->control_mode(0);
  std::cout<<"----------Zero done------------"<<'\n';
  #endif

  // Move to start position
  // leg_test.torque_enable(1);
  // leg_test.moveTo_coordinate(6,-17);
  // sleep(5);
  // leg_test.torque_enable(0);

  #ifdef Input
  std::cout<<"----------Input test-----------"<<'\n';
  sleep(3);
  float x, y;
  while(1) {
    update();
    std::cout<<"Input x : "; std::cin>>x;
    std::cout<<"Input y : "; std::cin>>y;
    leg_test.torque_enable(1);
    leg_test.moveTo_coordinate(x,y);
    sleep(2);
    leg_test.torque_enable(0);
  }
  std::cout<<"---------- End Input test -----------"<<'\n';
  #endif

  #ifdef Square
  std::cout<<"---------- Square route -----------"<<'\n';
  // Along -x direction
  float toe_X = 14;
  float toe_Y = -14;
  while(1) {
    if(toe_X <= 3.0)
      break;
    toe_X-=0.5;  
    leg_test.torque_enable(1);
    if(toe_X == 13.5) {
      leg_test.moveTo_coordinate(toe_X, toe_Y);
      sleep(2);
    }
    leg_test.moveTo_coordinate(toe_X, toe_Y);
    usleep(80*1000);
  }

  // Along -y direction
  while(1) {
    if(toe_Y <= -20.0)
      break;
    toe_Y-=0.5;  
    leg_test.torque_enable(1);
    leg_test.moveTo_coordinate(toe_X, toe_Y);
    usleep(80*1000);
  }

  while(1) {
    if(toe_X >= 14.0)
      break;
    toe_X+=0.5;  
    leg_test.torque_enable(1);
    leg_test.moveTo_coordinate(toe_X, toe_Y);
    usleep(80*1000);
  }

  // Along y direction
  while(1) {
    if(toe_Y >= -14.0)
      break;
    toe_Y+=0.5;  
    leg_test.torque_enable(1);
    leg_test.moveTo_coordinate(toe_X, toe_Y);
    usleep(80*1000);
  }
  std::cout<<"---------- End Square route -----------"<<'\n';
  #endif

  #ifdef Triangle
  std::cout<<"---------- Triangle route -----------"<<'\n';

  float toe_X = 14;
  float toe_Y = -14;

  // (14, -14) -> (7,-14) 
  while(1) {
    if(toe_X <= 7)
      break;
    toe_X-=0.5;
    leg_test.torque_enable(1);
    if(toe_X == 13.5) {
      sleep(2);
    }
    leg_test.moveTo_coordinate(toe_X, toe_Y);
    usleep(80*1000);
  }

  // (7, -14) -> (0, -20)
  while(1) {
    if(toe_X <=0 and toe_Y <=-20 )
      break;
    toe_X-=0.38;  
    toe_Y-=0.325;
    leg_test.torque_enable(1);
    leg_test.moveTo_coordinate(toe_X, toe_Y);
    usleep(80*1000);
  }

  // (0, -20) -> (14, -14)
  while(1) {
    if(toe_X >=14 and toe_Y >=-14 )
      break;
    toe_X+=0.46;  
    toe_Y+=0.197;
    leg_test.torque_enable(1);
    leg_test.moveTo_coordinate(toe_X, toe_Y);
    usleep(80*1000);
  }
  std::cout<<"---------- End Triangle route -----------"<<'\n';
  #endif

  #ifdef Circle
  std::cout<<"---------- Circle route -----------"<<'\n';
  float ox = 6;
  float oy = -17;
  float r = 3;
  float x, y;
  double theta = 0;
  sleep(2);
  while(1){  
    theta += 3;
    x = ox + r * float( cos(theta * M_PI / 180) );
    y = oy + r * float( sin(theta * M_PI / 180) );

    x = std::round(x * 100) / 100;
    y = std::round(y * 100) / 100;

    std::cout<<"("<<x<<", "<<y<<")\n";
    leg_test.torque_enable(1);
    leg_test.moveTo_coordinate(x, y);
    usleep(80*1000);
    if(theta == 1440) break;
  }
  std::cout<<"---------- End Circle route -----------"<<'\n';
  #endif 

  leg_test.torque_enable(0);
}