#include "leg_group.h"

Eigen::Vector3f leg_ang2pos(Eigen::Vector3f angle);
Eigen::Vector3f leg_pos2ang(Eigen::Vector3f position);
Eigen::Matrix3f leg_pos_grad(Eigen::Vector3f position);
Eigen::Matrix3f leg_ang_grad(Eigen::Vector3f angle);
Eigen::Vector3f leg_vel2omg(Eigen::Vector3f position, Eigen::Vector3f velocity);
Eigen::Matrix3f get_gait_status(float time);

// Eigen::Vector3f leg_pos2rad(Eigen::Vector3f position);

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

/* Breif introduction of the leg coordinate sys.
/* Unit : 
    Length : meter 
    Angle  : rad

    Z        Y
    ↑      ↗ 
    ∣   ／
    |／
    ------> X

  motor_1
    o
          motor_2        motor_3
            o--------------o、
           /                 ˋ、    
          /                   ˋ、  
         o                     o
          ˋ、                  /
            ˋ、               / 
              ˋ、            / 
                ˋ、         /    
                  ˋ、      /
                    ˋ、   /
                      ˋ、/
                        o                  
                          ˋ、
                            ˋ、   
                              ˋo (Tx, Ty, Tz)
  ---------------Ground -----------------------------
  For further information and plots, see the ppt in the connection below.
  https://1drv.ms/p/s!AogEDeJiKy9qoiUlYLLq_KLRA7Cr?e=AXuawB */

Eigen::Vector3f leg_ang2pos(Eigen::Vector3f angle) {
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

  // Transform the leg position from the coordinate in derivation 
	// to the coordinate in real world.
	x = -x + 0.08378

#ifdef DEBUG_LEGGROUP
#print(x)
#print(y)
#print(z)
#endif
  return Eigen::Vector3f(x, y, z);
}

Eigen::Vector3f leg_pos2ang(Eigen::Vector3f position) {

Eigen::Matrix3f leg_pos_grad(Eigen::Vector3f position) {
  double shift = 1e-8;

  float x = position[0];
  float y = position[1];
  float z = position[2];

  Eigen::Vector3f x_plus = leg_pos2ang(Eigen::Vector3f(x + shift, y, z));
  Eigen::Vector3f x_minus = leg_pos2ang(Eigen::Vector3f(x - shift, y, z));
  Eigen::Vector3f x_grad = (0.5 * (x_plus - x_minus)) / shift;

  Eigen::Vector3f y_plus = leg_pos2ang(Eigen::Vector3f(x, y + shift, z));
  Eigen::Vector3f y_minus = leg_pos2ang(Eigen::Vector3f(x, y - shift, z));
  Eigen::Vector3f y_grad = (0.5 * (y_plus - y_minus)) / shift;

  Eigen::Vector3f z_plus = leg_pos2ang(Eigen::Vector3f(x, y, z + shift));
  Eigen::Vector3f z_minus = leg_pos2ang(Eigen::Vector3f(x, y, z - shift));
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

Eigen::Matrix3f leg_ang_grad(Eigen::Vector3f angle) {
  double shift = 1e-8;

  float angle_m1 = angle[0];
  float angle_m2 = angle[1];
  float angle_m3 = angle[2];

  Eigen::Vector3f m1_plus =
      leg_ang2pos(Eigen::Vector3f(angle_m1 + shift, angle_m2, angle_m3));
  Eigen::Vector3f m1_minus =
      leg_ang2pos(Eigen::Vector3f(angle_m1 - shift, angle_m2, angle_m3));
  Eigen::Vector3f m1_grad = (0.5 * (m1_plus - m1_minus)) / shift;

  Eigen::Vector3f m2_plus =
      leg_ang2pos(Eigen::Vector3f(angle_m1, angle_m2 + shift, angle_m3));
  Eigen::Vector3f m2_minus =
      leg_ang2pos(Eigen::Vector3f(angle_m1, angle_m2 - shift, angle_m3));
  Eigen::Vector3f m2_grad = (0.5 * (m2_plus - m2_minus)) / shift;

  Eigen::Vector3f m3_plus =
      leg_ang2pos(Eigen::Vector3f(angle_m1, angle_m2, angle_m3 + shift));
  Eigen::Vector3f m3_minus =
      leg_ang2pos(Eigen::Vector3f(angle_m1, angle_m2, angle_m3 - shift));
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

Eigen::Vector3f leg_vel2omg(Eigen::Vector3f position, Eigen::Vector3f velocity) {
  Eigen::Matrix3f jacobian = leg_pos_grad(position);
  Eigen::Vector3f omega = jacobian * velocity;
  return omega;
}

Eigen::Matrix3f get_gait_status(float time) {
  Eigen::Vector3f leave_point(0, 0, 0);
  Eigen::Vector3f leave_velocity(-BODY_VELOCITY, 0, 0);
  Eigen::Vector3f entry_point(BODY_VELOCITY * (3 * LEG_PERIOD + 4 * DELTA_T), 0, 0);
  Eigen::Vector3f peak_point(0, 0, LIFT_HEIGHT);
  Eigen::Vector3f entry_velocity(-BODY_VELOCITY, 0, 0);

  // 5th order bezier curve
	Eigen::Vector3f p0 = leave_point;
	Eigen::Vector3f p1 = (leave_velocity * LEG_PERIOD + 4 * leave_point) / 4;
	Eigen::Vector3f p2 = leave_point * 0.1 + entry_point * 0.9 + peak_point;
	Eigen::Vector3f p3 = (-1 * entry_velocity * LEG_PERIOD + 4 * entry_point) / 4;
	Eigen::Vector3f p4 = entry_point;

  float t = time;
  float x = (
    1 * p0.x() * pow((1 - t), 4) +
    4 * p1.x() * t * pow((1 - t), 3) +
    6 * p2.x() * pow(t, 2) * pow((1 - t), 2) +
    4 * p3.x() * pow(t, 3) * (1 - t) +
    1 * p4.x() * pow(t, 4) );

	float y = 0;

	float z = (
    1 * p0.z() * pow((1 - t), 4) +
    4 * p1.z() * t * pow((1 - t), 3) +
    6 * p2.z() * pow(t, 2) * pow((1 - t), 2) +
    4 * p3.z() * pow(t, 3) * (1 - t) +
    1 * p4.z() * pow(t, 4) );

  float dx_dt = (
   -4 * p0.x() * pow((1 - t), 3) +
    4 * p1.x() * pow((1 - t), 3) +
   12 * p2.x() * t * pow((1 - t), 2) +
   12 * p3.x() * pow(t, 2) * (1 - t) -
    4 * p4.x() * pow(t, 3));

	float dy_dt = 0;

	float dz_dt = (
   -4 * p0.z() * pow((1 - t), 3) +
    4 * p1.z() * pow((1 - t), 3) +
   12 * p2.z() * t * pow((1 - t), 2) +
   12 * p3.z() * pow(t, 2) * (1 - t) -
    4 * p4.z() * pow(t, 3) );

	// Align the midpoint of the trajectory with the centerline of the two motors.
	z = z - 0.1654;
	x = x - 0.0057;

  Eigen::Vector3f gait_pos(x, y, z);
  Eigen::Vector3f gait_vel(dx_dt, dy_dt, dz_dt);
  Eigen::Vector3f blank(0, 0, 0);
  Eigen::Matrix3f gait_status;
  gait_status << gait_pos, gait_vel, blank;

  return gait_status;
}

Leg_group::Leg_group(Actuator *motorAlpha, Actuator *motorBeta) {
  mMotorAlpha = motorAlpha;
  mMotorBeta = motorBeta;
}

void Leg_group::torque_enable(int num){
  this->mMotorAlpha->torque_enable(num); 
  this->mMotorBeta->torque_enable(num);
}

bool Leg_group::leg_reset_pos() {
  int check_list[3] = {0, 0, 1};

  this->mMotorAlpha->check_zero_done();  
  this->mMotorBeta->check_zero_done();

  if (this->mMotorAlpha->isZeroed() == 1){
    this->mMotorAlpha->goal_velocity_dps(0);
    this->mMotorAlpha->control_mode(0);
    check_list[0] = 1;
  } else if (this->mMotorAlpha->isZeroed() == 0){
    this->mMotorAlpha->control_mode(1);
    this->mMotorAlpha->goal_velocity_dps(55);
  }
  
  if(this->mMotorBeta->isZeroed() == 1){
    this->mMotorBeta->goal_velocity_dps(0);
    this->mMotorBeta->control_mode(0);
    check_list[1] = 1;
  } else if (this->mMotorBeta->isZeroed() == 0){
    this->mMotorBeta->control_mode(1); 
    this->mMotorBeta->goal_velocity_dps(55);
  }

  if(check_list[0] * check_list[1] * check_list[2] == 1) 
    return true;
  return false;
}

void Leg_group::leg_move_ang(float angle_1, float angle_2, float angle_3){
  // Set limits to the motor angles.
  if( angle_1 > M_PI/2  or angle_1 < -M_PI/2 or 
      angle_2 > M_PI or angle_2 < 0   or
      angle_3 > M_PI or angle_3 < 0   ) {        
    std::cout<<"Target angle too high.\n Abort\n";
    return;
  }

  // Calibration the offset
  float motor_angle_1 = 0;
  float motor_angle_2 = 127 * M_PI / 180 - angle_2;
  float motor_angle_3 = M_PI / 2 - angle_3;

  int can_signal_1 = int(motor_angle_1 * 32768 / M_PI);
  int can_signal_2 = int(motor_angle_2 * 32768 / M_PI);
  int can_signal_3 = int(motor_angle_3 * 32768 / M_PI);

  this->mMotorAlpha->goal_position_deg(can_signal_3);
  this->mMotorBeta->goal_position_deg(can_signal_2);
  // this->mMotorGamma->goal_position_deg(can_signal_1);
}

  float DEG_1 = motor_angle(0);
  float DEG_2 = motor_angle(1);

  moveTo_angle(DEG_1, DEG_2);
}

// For further information and plots, see the ppt in the connection below.
// https://1drv.ms/p/s!AogEDeJiKy9qoiUlYLLq_KLRA7Cr?e=AXuawB
Eigen::Vector2f Leg_group::toe_pos2motor_angle(Eigen::Vector2f position) {
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