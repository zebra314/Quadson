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

class Leg_group {
private:
  Actuator *mMotorAlpha;
  Actuator *mMotorBeta;
  Actuator *mMotorGamma;

  Eigen::Vector3f pos;
  Eigen::Vector3f vel;
  Eigen::Vector3f ang;
  Eigen::Vector3f omg;

  Eigen::Vector2f toe_pos2motor_angle(Eigen::Vector2f position);

public:
  Leg_group();
  Leg_group(Actuator *motorAlpha, Actuator *mMtorBeta, Actuator *motorGamma);
  ~Leg_group();

  // For 2D test
  Leg_group(Actuator *motorAlpha, Actuator *mMtorBeta);
  void torque_enable(int num);
  bool reset_pos();
  void moveTo_coordinate(float x, float y);
  void moveTo_angle(float alpha, float beta);
};
#endif
