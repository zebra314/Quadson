#include "quadson.h"
#include "can_interface.h"
#include <unistd.h>
#include <math.h>

void print_progress(std::string progress){
  std::cout<<"\n+";
  for(int i = 0; i<progress.length()+2; i++)
    std::cout<<'-';
  std::cout<<"+\n| "<<progress<<" |\n+";
  for(int i = 0; i<progress.length()+2; i++)
    std::cout<<'-';
  std::cout<<"+\n\n";
}

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
        get_actuator_present_deg(rec_cmd.id, rec_cmd.value);
        break;
      case CAN_STDID_PRESENT_VELOCITY_DPS:
      case CAN_STDID_PRESENT_TORQUE_CURRENT_MA:
      case CAN_STDID_PRESENT_FLUX_CURRENT_MA:
      case CAN_STDID_PRESENT_VOLTAGE:
      case CAN_STDID_PRESENT_TEMPERATURE:
      case CAN_STDID_ZERO_DONE:
        get_actuator_zero_state(rec_cmd.id, rec_cmd.value);
        break;
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

void Quadson::get_actuator_present_deg(int id, int value){
  this->actuator[id -1]->angle(value);
}

void Quadson::moving_test(){
  Leg_group leg_test(this->actuator[0], this->actuator[1]);

  #define ZERO
  // #define TEST
  // #define Input
  #define GAIT

  #ifdef ZERO
  print_progress("Start Zero");
  leg_test.torque_enable(1);
  while(!leg_test.leg_reset_pos()){
    update();
    usleep(30*1000);
  }
  leg_test.torque_enable(0);
  print_progress("End Zero");
  #endif

  #ifdef TEST
  print_progress("Start Test Angle Read");
  int ang;
  while(1){
    update();
    this->actuator[1]->present_position_deg();
    ang = this->actuator[1]->getAngle();
    std::cout<<'\n';
    std::cout<<std::hex<<ang<<'\n'; 
    std::cout<<std::dec<<ang<<'\n';
    std::cout<<'\n';
    usleep(50*1000);
  }
  print_progress("End Test Angle Read");
  #endif

  #ifdef Input
  print_progress("Start Input test");
  sleep(3);
  float x, z;
  while(1) {
    update();
    std::cout<<"Input x : "; std::cin>>x;
    std::cout<<"Input z : "; std::cin>>z;
    leg_test.torque_enable(1);
    leg_test.leg_move_pos(x/100,0,z/100);
    sleep(2);
    leg_test.torque_enable(0);
  }
  print_progress("End Input test");
  #endif

  #ifdef GAIT
  print_progress("Start Gait");
  sleep(2);
  leg_test.torque_enable(1);
  leg_test.leg_move_pos(-0.0057/100, 0, -16.54/100);
  sleep(5);
  
  std::cout << "Press Enter to continue...\n";
  std::string line;
  std::getline(std::cin, line);

  for(int time = 0; time<5; time++){

    // Curve section
    for(int i = 0; i<100; i++){
      update();
      leg_test.leg_move_gait(static_cast<float>(i) / 100.0f);
      usleep(10*1000);
    }

    // Horzion section
    for(int i = 0; i<100; i++){
      update();
      float x = 0.0875561 - (0.0875561/100) * i;
      float y = 0;
      float z = -0.165362;
      leg_test.leg_move_pos(x,y,z);
      usleep(10*1000);
    }
  }

  leg_test.torque_enable(0);
  print_progress("End Gait");
  #endif 

}