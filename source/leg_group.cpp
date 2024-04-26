#include "leg_group.h"

Eigen::Vector3f leg_rad2pos(Eigen::Vector3f angle);
Eigen::Vector3f leg_pos2rad(Eigen::Vector3f position);
Eigen::Matrix3f leg_pos_grad(Eigen::Vector3f position);
Eigen::Matrix3f leg_angle_grad(Eigen::Vector3f angle);

Leg_group::Leg_group() {}
Leg_group::Leg_group(Actuator *motorAlpha, Actuator *motorBeta,
                     Actuator *motorGamma) {
  this->mMotorAlpha = motorAlpha;
  this->mMotorBeta = motorBeta;
  this->mMotorGamma = motorGamma;

  pos.Zero();
  vel.Zero();
  ang.Zero();
  omg.Zero();

  ang_kp = ANG_KP;
  ang_ki = ANG_KI;
  ang_kd = ANG_KD;

  prev_ang_time = 0;
  prev_ang_error = 0;
  ang_error_integral = 0;
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

// For 2D test
Leg_group::Leg_group(Actuator *motorAlpha, Actuator *motorBeta) {
  mMotorAlpha = motorAlpha;
  mMotorBeta = motorBeta;
}

void Leg_group::leg_enable_torque(int num){
  this->mMotorAlpha->torque_enable(num); 
  this->mMotorBeta->torque_enable(num);
  // this->mMotorGamma->torque_enable(num);
}

bool Leg_group::leg_reset_pos() {
  if( !(mMotorAlpha->isZeroed() and mMotorBeta->isZeroed()) ) {
    this->mMotorAlpha->control_mode(1);
    this->mMotorBeta->control_mode(1);
    this->mMotorAlpha->goal_velocity_dps(50);
    this->mMotorBeta->goal_velocity_dps(50);
    this->mMotorAlpha->check_zero_done();
    this->mMotorBeta->check_zero_done();
    return true; 
  } else {
    this->mMotorAlpha->goal_velocity_dps(0);
    this->mMotorBeta->goal_velocity_dps(0);
    this->mMotorAlpha->control_mode(0);
    this->mMotorBeta->control_mode(0);
    return false; // if complete reseting, break from reset while.
  }
}

// For 2D test
void Leg_group::leg_ang_ctrl(float alpha, float beta){
  // Protect the legs from rotate too much angle.
  if(alpha > 180 or alpha < 0) {
    std::cout<<"Target angle too high.\n";
    return;
  }
  
  if(beta > 180 or beta < 0) {
    std::cout<<"Target angle too high.\n";
    return;
  }

  // The zero position of the motor during reset is different from 
  // the zero position in the derivation.
  float motor_angle_1 =  90 - alpha;
  float motor_angle_2 = 120 - beta;

  // Tranfer motor_angle to can signal.
  int can_signal_1 = int(motor_angle_1 * 32768 / 180);
  int can_signal_2 = int(motor_angle_2 * 32768 / 180); 

  this->mMotorAlpha->goal_position_deg(can_signal_1);
  this->mMotorBeta->goal_position_deg(can_signal_2);
}

// For 2D test
void Leg_group::leg_pos_ctrl(float x, float y){
  Eigen::Vector2f position = Eigen::Vector2f(x, y);
  Eigen::Vector2f motor_angle = calc_pos2ang(position);

  float DEG_1 = motor_angle(0);
  float DEG_2 = motor_angle(1);

  moveTo_angle(DEG_1, DEG_2);
}

void Leg_group::leg_pos_ctrl_pid(float x, float y float z = 0){
  Eigen::Vector2f position = Eigen::Vector2f(x, y);
  Eigen::Vector2f motor_angle = calc_pos2ang(position);
}

// For further information and plots, see the ppt in the connection below.
// https://1drv.ms/p/s!AogEDeJiKy9qoiUlYLLq_KLRA7Cr?e=AXuawB
Eigen::Vector2f Leg_group::calc_pos2ang(Eigen::Vector2f position) {
  float D = 8.16, L1 = 8, L2 = 13, L3 = 10, d = 8;
  float Tx = position(0);
  float Ty = position(1);

  float x1 = sqrt(pow(Tx, 2) + pow(Ty, 2));

  // Tx > 0, 0 < atan < PI / 2
  // Tx < 0, 0 > atan > - PI / 2  
  float alpha1 = atan( (-Ty) / Tx);
  if (alpha1 < 0)
    alpha1 += M_PI;
  float beta1 = acos( (pow(x1, 2) + pow(L1, 2) - pow(L3+d, 2)) / (2 * x1 * L1));

  float Qx = L1 * cos(alpha1 + beta1);
  float Qy = -L1 * sin(alpha1 + beta1);
  float Px = (Qx * d + Tx * L3) / (d + L3);
  float Py = (Qy * d + Ty * L3) / (d + L3);

  float x2 = sqrt(pow(D-Px, 2) + pow(Py, 2));

  // D > Px, 0 < alpha2 < PI / 2
  // D < Px, 0 > alpha2 > -PI / 2
  float alpha2 = atan( (-Py) / (D-Px));
  if (alpha2 < 0)
    alpha2 += M_PI;
  float beta2 = acos( (pow(x2, 2) + pow(L1, 2) - pow(L2, 2)) / (2 * x2 * L1));

  // NOTICE
  // The motor number in derivation is opposite to that in motor commands.
  float motor_angle_1 = (alpha2 + beta2) * 180 / M_PI;
  float motor_angle_2 = (alpha1 + beta1) * 180 / M_PI;

  return Eigen::Vector2f(motor_angle_1, motor_angle_2);
}