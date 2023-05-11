#include "leg_group.h"

Eigen::Vector3f leg_rad2pos(Eigen::Vector3f angle);
Eigen::Vector3f leg_pos2rad(Eigen::Vector3f position);
Eigen::Matrix3f leg_pos_grad(Eigen::Vector3f position);
Eigen::Matrix3f leg_angle_grad(Eigen::Vector3f angle);

Leg_group::Leg_group() {}
Leg_group::Leg_group(Actuator *motorAlpha, Actuator *motorBeta,
                     Actuator *motorGamma) {
  mMotorAlpha = motorAlpha;
  mMotorBeta = motorBeta;
  mMotorGamma = motorGamma;

  pos.Zero();
  vel.Zero();
  ang.Zero();
  omg.Zero();
}

Leg_group::~Leg_group() {}

Eigen::Vector3f leg_pos2rad(Eigen::Vector3f position) {
  float x = position(0);
  float y = position(1);
  float z = position(2);

  float angle_m1 = atan2(y, -z);
  float r_m3 = CALC_R((MOTOR_DISTANCE - x), -CALC_R(y, z));
  float theta_m3 = atan2((MOTOR_DISTANCE - x), CALC_R(y, z));
  float psi_m3 = acos((pow((ARM_D + ARM_E), 2) - pow(ARM_C, 2) - pow(r_m3, 2)) /
                      (-2 * ARM_C * r_m3));
  float angle_m3 = psi_m3 - theta_m3;
  float beta_m3 =
      acos((pow(r_m3, 2) - pow((ARM_D + ARM_E), 2) - pow(ARM_C, 2)) /
           (-2 * ARM_C * (ARM_D + ARM_E)));
  float r_m3p =
      sqrt(pow(ARM_C, 2) + pow(ARM_D, 2) - 2 * ARM_C * ARM_D * cos(beta_m3));
  float psi_m3p = acos((pow(ARM_D, 2) - pow(ARM_C, 2) - pow(r_m3p, 2)) /
                       (-2 * ARM_C * r_m3p));
  float theta_m3p = psi_m3p - angle_m3;
  float r_m2 = CALC_R((MOTOR_DISTANCE - r_m3p * sin(theta_m3p)),
                      -r_m3p * cos(theta_m3p));
  float psi_m2 = acos((pow(ARM_B, 2) - pow(ARM_A, 2) - pow(r_m2, 2)) /
                      (-2 * ARM_A * r_m2));
  float theta_m2 =
      atan2((MOTOR_DISTANCE - r_m3p * sin(theta_m3p)), r_m3p * cos(theta_m3p));
  float angle_m2 = psi_m2 - theta_m2;

#ifdef DEBUG_LEGGROUP
  // # print(degrees(angle_m1))
  // # print(degrees(angle_m2))
  // # print(degrees(angle_m3))
#endif
  return Eigen::Vector3f(angle_m1, angle_m2, angle_m3);
}

Eigen::Vector3f leg_rad2pos(Eigen::Vector3f angle) {
  float angle_m1 = angle(0);
  float angle_m2 = angle(1);
  float angle_m3 = angle(2);

  double f = sqrt(pow(ARM_A, 2) + pow(MOTOR_DISTANCE, 2) -
                  2 * ARM_A * MOTOR_DISTANCE * cos(angle_m2 + M_PI / 2));

  float g =
      CALC_R(-ARM_A * sin(angle_m2) - (MOTOR_DISTANCE + ARM_C * sin(angle_m3)),
             -ARM_A * cos(angle_m2) - (-ARM_C * cos(angle_m3)));
  float psi = acos((pow(f, 2) - pow(ARM_C, 2) - pow(g, 2)) / (-2 * ARM_C * g));
  float psi_1 =
      acos((pow(ARM_B, 2) - pow(ARM_D, 2) - pow(g, 2)) / (-2 * ARM_D * g));
  psi = psi + psi_1;

  float x = MOTOR_DISTANCE + ARM_C * sin(angle_m3) -
            (ARM_D + ARM_E) * cos(psi - (M_PI / 2 - angle_m3));
  float zp = -ARM_C * cos(angle_m3) -
             (ARM_D + ARM_E) * sin(psi - (M_PI / 2 - angle_m3));
  float z = zp * cos(angle_m1);
  float y = -zp * sin(angle_m1);

#ifdef DEBUG_LEGGROUP
#print(x)
#print(y)
#print(z)
#endif
  return Eigen::Vector3f(x, y, z);
}

Eigen::Matrix3f leg_pos_grad(Eigen::Vector3f position) {
  double shift = 1e-8;

  float x = position[0];
  float y = position[1];
  float z = position[2];

  Eigen::Vector3f x_plus = leg_pos2rad(Eigen::Vector3f(x + shift, y, z));
  Eigen::Vector3f x_minus = leg_pos2rad(Eigen::Vector3f(x - shift, y, z));
  Eigen::Vector3f x_grad = (0.5 * (x_plus - x_minus)) / shift;

  Eigen::Vector3f y_plus = leg_pos2rad(Eigen::Vector3f(x, y + shift, z));
  Eigen::Vector3f y_minus = leg_pos2rad(Eigen::Vector3f(x, y - shift, z));
  Eigen::Vector3f y_grad = (0.5 * (y_plus - y_minus)) / shift;

  Eigen::Vector3f z_plus = leg_pos2rad(Eigen::Vector3f(x, y, z + shift));
  Eigen::Vector3f z_minus = leg_pos2rad(Eigen::Vector3f(x, y, z - shift));
  Eigen::Vector3f z_grad = (0.5 * (z_plus - z_minus)) / shift;

  Eigen::Matrix3f jacobian;
  jacobian << x_grad, y_grad, z_grad;
#ifdef DEBUG_LEGGROUP
#print(x_grad)
#print(y_grad)
#print(z_grad)
#print(jacobian)
#endif
  return jacobian;
}

Eigen::Matrix3f leg_angle_grad(Eigen::Vector3f angle) {
  double shift = 1e-8;

  float angle_m1 = angle[0];
  float angle_m2 = angle[1];
  float angle_m3 = angle[2];

  Eigen::Vector3f m1_plus =
      leg_rad2pos(Eigen::Vector3f(angle_m1 + shift, angle_m2, angle_m3));
  Eigen::Vector3f m1_minus =
      leg_rad2pos(Eigen::Vector3f(angle_m1 - shift, angle_m2, angle_m3));
  Eigen::Vector3f m1_grad = (0.5 * (m1_plus - m1_minus)) / shift;

  Eigen::Vector3f m2_plus =
      leg_rad2pos(Eigen::Vector3f(angle_m1, angle_m2 + shift, angle_m3));
  Eigen::Vector3f m2_minus =
      leg_rad2pos(Eigen::Vector3f(angle_m1, angle_m2 - shift, angle_m3));
  Eigen::Vector3f m2_grad = (0.5 * (m2_plus - m2_minus)) / shift;

  Eigen::Vector3f m3_plus =
      leg_rad2pos(Eigen::Vector3f(angle_m1, angle_m2, angle_m3 + shift));
  Eigen::Vector3f m3_minus =
      leg_rad2pos(Eigen::Vector3f(angle_m1, angle_m2, angle_m3 - shift));
  Eigen::Vector3f m3_grad = (0.5 * (m3_plus - m3_minus)) / shift;

  Eigen::Matrix3f jacobian;
  jacobian << m1_grad, m2_grad, m3_grad;
#ifdef DEBUG_LEGGROUP
#print(m1_grad)
#print(m2_grad)
#print(m3_grad)
#print(jacobian)
#endif
  return jacobian;
}

void matrixTest() {
  Eigen::Vector3f x_grad(0, 0, 0);
  Eigen::Vector3f y_grad(1, 0, 0);
  Eigen::Vector3f z_grad(0, 3, 0);
  Eigen::Matrix3f jacobian;
  jacobian << x_grad, y_grad, z_grad;
  std::cout << jacobian;
}
