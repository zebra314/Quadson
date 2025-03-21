from math import *
import numpy as np
import warnings

class LegKinematics:
  """
  Represents a single leg of the quadruped robot. The leg is a five-bar linkage mechanism with three motors.
  
  Attributes:
    Ls            : [L0, L1, L2, L3, L4, L5], length of the links, cm
    _motor_angles : [angle0, angle1, angle5], angles of the motors, rad
    _ee_point     : [x, y, z], position of the end effector, cm
    _points       : (6, 3), points of the joints in the leg's frame, cm
    _angles       : (6,), angles of the joints, rad
    _velocities   : [vx, vy, vz], linear velocity of the end effector, cm/s
    _omegas       : [omega0, omega1, omega5], angular velocities of the motors, rad/s
    _points_alt   : (6, 3), the second possible points of the joints, result from the calculation of the inverse kinematics
    _angles_alt   : (6,), the second possible angles of the joints, result from the calculation of the inverse kinematics

  Methods:
    calc_ang2pnt       : Forward kinematics, (motor angles -> end effector point)
    calc_pnt2ang       : Inverse kinematics, (end effector point -> motor angles)
    calc_omg2vel       : Forward differential kinematics, (motor omega -> end effector velocity)
    calc_vel2omg       : Inverse differential kinematics, (end effector velocity -> motor omega)
    numerical_jacobian : Calculate the numerical Jacobian matrix for differential kinematics
  """

  def __init__(self):
    self.Ls = [8.16, 8.0, 10.0, 8.0, 13.0, 8.0]
    self._motor_angles = None
    self._ee_point = None
    self._points = None
    self._angles = None
    self._velocities = None
    self._omegas = None
    self._points_alt = None
    self._angles_alt = None

  @property
  def motor_angles(self):
    if self._motor_angles is None:
      raise warnings.warn("Error: _motor_angles is None")
    return self._motor_angles
  
  @motor_angles.setter
  def motor_angles(self, angles):
    self._motor_angles = angles
    self._ee_point = self.calc_ang2pnt(angles)

  @property
  def ee_point(self):
    if self._ee_point is None:
      raise warnings.warn("Error: _ee_point is None")
    return self._ee_point
  
  @ee_point.setter
  def ee_point(self, point):
    self._ee_point = point
    self._motor_angles = self.calc_pnt2ang(point)

  @property
  def points(self):
    if self._points is None:
      raise warnings.warn("Error: _points is None")
    return self._points

  @property
  def angles(self):
    if self._angles is None:
      raise warnings.warn("Error: _angles is None")
    return self._angles

  @property
  def velocities(self):
    if self._velocities is None:
      raise warnings.warn("Error: _velocities is None")
    return self._velocities
  
  @velocities.setter
  def velocities(self, velocities):
    self._velocities = velocities
    self._omegas = self.calc_vel2omg(velocities)
  
  @property
  def omegas(self):
    if self._omegas is None:
      raise warnings.warn("Error: _omegas is None")
    return self._omegas
  
  @omegas.setter
  def omegas(self, omegas):
    self._omegas = omegas
    self._velocities = self.calc_omg2vel(omegas)

  def calc_ang2pnt(self, angles):
    '''
    Forward kinematics
    Given the angles of the motors, calculate the end point of the leg and the state of the leg.

    @param motor_angles: Angles of the motors, size 3 [angle0, angle1, angle5]
    @return: state object
    '''
    angle0, angle1, angle5 = angles

    # Start from 2D kinematics
    p1 = np.array([0, 0])
    p2 = np.array([self.Ls[1]*cos(angle1), -self.Ls[1]*sin(angle1)])
    p5 = np.array([self.Ls[0], 0])
    p4 = np.array([self.Ls[5]*cos(angle5), -self.Ls[5]*sin(angle5)]) + p5
    
    L14 = np.linalg.norm(p4-p1)
    L24 = np.linalg.norm(p4-p2)

    # Check if the leg is reachable
    if(L24 > self.Ls[4] + self.Ls[2]):
      raise warnings.warn("Leg is not reachable")
    elif (L24 == self.Ls[4] + self.Ls[2]):
      warnings.warn("Leg reaches singularity", RuntimeWarning)

    # Calculate angle2
    angle_324 = acos((self.Ls[2]**2 + L24**2 - self.Ls[4]**2) / (2 * self.Ls[2] * L24))
    angle_421 = acos((self.Ls[1]**2 + L24**2 - L14**2) / (2 * self.Ls[1] * L24))
    angle2 = angle_324 + angle_421 - (np.pi - angle1)

    # Calculate pe and p3 by averaging the positions
    pe = np.array([(self.Ls[2]+self.Ls[3])*cos(angle2), -(self.Ls[2]+self.Ls[3])*sin(angle2)]) + p2
    p3 = (pe*self.Ls[2]+p2*self.Ls[3])/(self.Ls[2]+self.Ls[3])
    angle4 = acos((p3-p4)[0] / np.linalg.norm(p3-p4))

    # Translate points from 2D to 3D, rotate the current plane to the x-y plane
    transformation_matrix = np.array([
      [1, 0, 0],
      [0, cos(-angle0), -sin(-angle0)],
      [0, sin(-angle0), cos(-angle0)]
    ])
    points = np.array([p1, p2, p3, pe, p4, p5])
    points = np.concatenate([points, np.zeros((points.shape[0], 1))], axis=1)
    points = points @ transformation_matrix

    self._points = points
    self._angles = np.array([angle0, angle1, angle2, None, angle4, angle5])

    return points[3]
  
  def calc_pnt2ang(self, point):
    '''
    Inverse kinematics
    Given the end point of the leg, calculate the angles of the motors and the state of the leg.

    @param end_point: Position of the end point, size 3 [x, y, z]
    @return: state object
    '''
    x, y, z = point

    # Calculate the angle of the motor 0
    angle0 = -atan2(z, -y)

    # Translate the points from 3D to 2D
    offset_angle = -angle0
    transformation_matrix = np.array([
      [1, 0, 0],
      [0, cos(offset_angle), -sin(offset_angle)],
      [0, sin(offset_angle), cos(offset_angle)]
    ])
    [x, y, z] = transformation_matrix @ np.array([x, y, z])
    pe_2d = np.array([x, y])

    # Angle 1
    L1e = np.linalg.norm(pe_2d)
    angle_e15 = atan2(-y, x)
    if angle_e15 < 0:
      angle_e15 += np.pi
    angle_e12 = acos((self.Ls[1]**2 + L1e**2 - (self.Ls[2] + self.Ls[3])**2) / (2 * self.Ls[1] * L1e))
    angle1 = angle_e15 + angle_e12

    # Points based on angle 1
    p1 = np.array([0, 0])  
    p2 = np.array([self.Ls[1]*cos(angle1), -self.Ls[1]*sin(angle1)])
    p3 = (pe_2d * self.Ls[2] + p2 * self.Ls[3]) / (self.Ls[2] + self.Ls[3])
    p5 = np.array([self.Ls[0], 0])

    # Angle 5
    L35 = np.linalg.norm(p3 - p5)
    angle_350 = atan2(-p3[1], self.Ls[0] - p3[0])
    if angle_350 < 0:
      angle_350 += np.pi
    angle_354 = acos((L35**2 + self.Ls[5]**2 - self.Ls[4]**2) / (2 * L35 * self.Ls[5]))
    angle5 = np.pi - (angle_350 + angle_354)

    # Point 4
    p4 = np.array([self.Ls[5]*cos(angle5), -self.Ls[5]*sin(angle5)]) + p5

    # Alt Angle 1
    angle1_alt = angle_e15 - angle_e12

    # Alt Points based on angle 1
    p2_alt = np.array([self.Ls[1]*cos(angle1_alt), -self.Ls[1]*sin(angle1_alt)])
    p3_alt = (pe_2d * self.Ls[2] + p2_alt * self.Ls[3]) / (self.Ls[2] + self.Ls[3])

    # Alt Angle 5
    L35_alt = np.linalg.norm(p3_alt - p5)
    angle_350_alt = atan2(-p3_alt[1], self.Ls[0] - p3_alt[0])
    if angle_350_alt < 0:
      angle_350_alt += np.pi
    angle_354_alt = acos((L35_alt**2 + self.Ls[5]**2 - self.Ls[4]**2) / (2 * L35_alt * self.Ls[5]))
    angle5_alt = np.pi - (angle_350_alt + angle_354_alt)

    # Alt Point 4
    p4_alt = np.array([self.Ls[5]*cos(angle5_alt), -self.Ls[5]*sin(angle5_alt)]) + p5

    # Store the states
    angle2 = acos((pe_2d-p2)[0] / np.linalg.norm(pe_2d-p2))
    angle4 = acos((p2-p4)[0] / np.linalg.norm(p2-p4))
    angles = np.array([angle0, angle1, angle2, None, angle4, angle5])

    angle2_alt = acos((pe_2d-p2_alt)[0] / np.linalg.norm(pe_2d-p2_alt))
    angle4_alt = acos((p2_alt-p4_alt)[0] / np.linalg.norm(p2_alt-p4_alt))
    angles_alt = np.array([angle0, angle1_alt, angle2_alt, None, angle4_alt, angle5_alt])

    # Translate the points from 2D to 3D
    points = np.array([p1, p2, p3, pe_2d, p4, p5])
    points = np.concatenate([points, np.zeros((points.shape[0], 1))], axis=1)
    points_alt = np.array([p1, p2_alt, p3_alt, pe_2d, p4_alt, p5])
    points_alt = np.concatenate([points_alt, np.zeros((points_alt.shape[0], 1))], axis=1)
    transformation_matrix = np.array([
      [1, 0, 0],
      [0, cos(-angle0), -sin(-angle0)],
      [0, sin(-angle0), cos(-angle0)]
    ])
    points = points @ transformation_matrix
    points_alt = points_alt @ transformation_matrix

    self._points = points
    self._angles = angles
    self._points_alt = points_alt
    self._angles_alt = angles_alt

    return [angle0, angle1, angle5]
  
  def calc_omg2vel(self, omegas):
    '''
    Given the angular velocities of the motors, calculate the linear velocity of the end effector.

    @param omegas: Angular velocities of the motors, size 3 [omega0, omega1, omega5]
    @return: Linear velocity of the end effector, size 3 [vx, vy, vz]
    '''
    J = self.numerical_jacobian()
    return J @ omegas
  
  def calc_vel2omg(self, velocities):
    '''
    Given the linear velocity of the end effector, calculate the angular velocities of the motors.

    @param velocities: Linear velocity of the end effector, size 3 [vx, vy, vz]
    @return: Angular velocities of the motors, size 3 [omega0, omega1, omega5]
    '''
    J = self.numerical_jacobian()
    return np.linalg.pinv(J) @ velocities
  
  def numerical_jacobian(self, delta=1e-4):
    motor_angles = self._motor_angles
    ee_point = self._ee_point
    J = np.zeros((6, 3))
    for i in range(3):
      angles_perturbed = motor_angles.copy()
      angles_perturbed[i] += delta
      pe_perturbed = self.calc_ang2pnt(angles_perturbed)
      J[0:3, i] = (pe_perturbed - ee_point) / delta
      J[3:6, i] = [1 if i == 0 else 0, 0, 1 if i != 0 else 0] # Angular part simplified (adjust based on axes)
    return J
  