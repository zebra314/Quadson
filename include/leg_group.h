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

#define ANG_KP 0
#define ANG_KI 0
#define ANG_KD 0

class Leg_group {
private:
  Actuator *mMotorAlpha;
  Actuator *mMotorBeta;
  Actuator *mMotorGamma;

  Eigen::Vector3f pos;
  Eigen::Vector3f vel;
  Eigen::Vector3f ang;
  Eigen::Vector3f omg;

  Eigen::Vector2f calc_pos2ang(Eigen::Vector2f position);

  /* PID params */
  double ang_kp;
  double ang_ki;
  double ang_kd;

  double prev_ang_time;
  double prev_ang_error;
  double ang_error_integral;

public:
  Leg_group();
  Leg_group(Actuator *motorAlpha, Actuator *mMtorBeta, Actuator *motorGamma);
  ~Leg_group();

  Leg_group(Actuator *motorAlpha, Actuator *mMtorBeta);
  void leg_enable_torque(int num);
  bool leg_reset_pos();
  void leg_pos_ctrl(float x, float y); // position control
  void leg_pos_ctrl_pid(float x, float y, float z); // velocity control
  void leg_ang_ctrl(float alpha, float beta);
};
#endif
