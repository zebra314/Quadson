#ifndef INC_LEG_GROUP_H
#define INC_LEG_GROUP_H

#include "utility.h"
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <unistd.h>

#include "actuator.h"

#define MOTOR_DISTANCE 0.082
#define ARM_A 0.08
#define ARM_B 0.13
#define ARM_C 0.08
#define ARM_D 0.10
#define ARM_E 0.066

// Gait parameters
#define BODY_VELOCITY 0.03
#define BODY_PERIOD 4
#define DELTA_T 0.1
#define LIFT_HEIGHT 0.04
#define LEG_PERIOD (BODY_PERIOD/4 - DELTA_T)

class Leg_group {
private:
  Actuator *mMotorAlpha;
  Actuator *mMotorBeta;
  Actuator *mMotorGamma;

  Eigen::Vector3f pos;
  Eigen::Vector3f vel;
  Eigen::Vector3f ang;
  Eigen::Vector3f omg;

public:
  Leg_group();
  Leg_group(Actuator *motorAlpha, Actuator *mMtorBeta, Actuator *motorGamma);
  ~Leg_group();

  Leg_group(Actuator *motorAlpha, Actuator *mMtorBeta);
  void torque_enable(int num);
  bool leg_reset_pos();
  void leg_move_ang(float angle_1, float angle_2, float angle_3);
  void leg_move_omg(float omega_1, float omega_2, float omega_3);
  void leg_move_pos(float x, float y, float z);
  void leg_move_vel(float x, float y, float z, float dx_dt, float dy_dt, float dz_dt);
  void leg_move_gait(float time);
};
#endif
