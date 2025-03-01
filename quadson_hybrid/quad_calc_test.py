import math
import numpy as np

class Leg(object):
	"""docstring for Leg"""
	def __init__(self, 
				# motor_1_obj, 
				# motor_2_obj,
				# motors_distance=0.082,
				motors_distance=0.0816,
				arm_a=0.08, 
				arm_b=0.13, 
				arm_c=0.08, 
				arm_d=0.10,
				# arm_e=0.066
				arm_e=0.08
				):

		# self.motor_1 = motor_1_obj
		# self.motor_2 = motor_2_obj
		self.motors_distance = motors_distance
		self.arm_a = arm_a
		self.arm_b = arm_b
		self.arm_c = arm_c
		self.arm_d = arm_d
		self.arm_e = arm_e

		self.pos = [0, 0, 0]
		self.vel = [0, 0, 0]
		self.angle = [0, 0, 0]
		self.omega = [0, 0, 0]


	def calc_r(self, a, b):
		return math.sqrt(a**2+b**2)

	def leg_pos2rad(self, pos):
		x = pos[0]
		y = pos[1]
		z = pos[2]
	
		# Transform the leg position from the coordinate in real world 
		# to the coordinate in derivation.
		# (The real world x axis is reversed to the derivation x axis)
		x = x / 100
		y = y / 100
		z = z / 100
		x = -x + 0.08378

		angle_m1 = math.atan2(y, -z)
		zp = -self.calc_r(y, z)
		r_m3 = self.calc_r((self.motors_distance-x), zp)
		theta_m3 = math.atan2((self.motors_distance-x), -zp) # (x,y); tan-1(y/x) = atan2(y,x)
		psi_m3 = math.acos(( (self.arm_d+self.arm_e)**2 - self.arm_c**2 - r_m3**2 )/( -2*self.arm_c*r_m3 ))
		angle_m3 = psi_m3 - theta_m3
		
		beta_m3 = math.acos(( r_m3**2 - (self.arm_d+self.arm_e)**2 - self.arm_c**2 )/( -2*self.arm_c*(self.arm_d+self.arm_e) ))
		r_m3p = math.sqrt(self.arm_c**2 + self.arm_d**2 - 2*self.arm_c*self.arm_d*math.cos(beta_m3))
		psi_m3p = math.acos(( self.arm_d**2 - self.arm_c**2 - r_m3p**2 )/( -2*self.arm_c*r_m3p ))
		theta_m3p = psi_m3p - angle_m3
		r_m2 = self.calc_r((self.motors_distance - r_m3p*math.sin(theta_m3p)), -r_m3p*math.cos(theta_m3p))
		psi_m2 = math.acos(( self.arm_b**2 - self.arm_a**2 - r_m2**2 )/( -2*self.arm_a*r_m2 ))
		theta_m2 = math.atan2((self.motors_distance - r_m3p*math.sin(theta_m3p)), r_m3p*math.cos(theta_m3p))
		angle_m2 = psi_m2 - theta_m2

		# Transform the leg angle from the coordinate in derivation
		# to the coordinate in real world.

		print(math.degrees(angle_m1))
		print(math.degrees(angle_m2))
		print(math.degrees(angle_m3))

		return np.array([angle_m1, angle_m2, angle_m3])


	def leg_pos_grad(self, pos):
		shift = 1e-8

		x = pos[0]
		y = pos[1]
		z = pos[2]
		
		x_plus = self.leg_pos2rad([x + shift, y, z])
		x_minus = self.leg_pos2rad([x - shift, y, z])
		x_grad = (0.5 * (x_plus - x_minus))/shift

		y_plus = self.leg_pos2rad([x, y + shift, z])
		y_minus = self.leg_pos2rad([x, y - shift, z])
		y_grad = (0.5 * (y_plus - y_minus))/shift

		z_plus = self.leg_pos2rad([x, y, z + shift])
		z_minus = self.leg_pos2rad([x, y, z - shift])
		z_grad = (0.5 * (z_plus - z_minus))/shift

		# print(x_grad)
		# print(y_grad)
		# print(z_grad)

		jacobian = np.matrix([x_grad, y_grad, z_grad])
		# print(jacobian)

		return jacobian

	def leg_vel2omega(self, pos, vel):
		jacobian = self.leg_pos_grad(pos)
		omega = vel*jacobian
		return omega


	def leg_rad2pos(self, angle):
		
		angle_m1 = angle[0]
		angle_m2 = angle[1]
		angle_m3 = angle[2]
		
		# f = math.sqrt(self.arm_a**2 + self.motors_distance**2 - 2*self.arm_a*self.motors_distance*math.cos(angle_m2+math.pi/2))
		f = math.sqrt(self.arm_a**2 + self.motors_distance**2 - 2*self.arm_a*self.motors_distance*math.cos(angle_m2+math.pi/2))
		g = self.calc_r(-self.arm_a*math.sin(angle_m2)-(self.motors_distance+self.arm_c*math.sin(angle_m3)), 
						-self.arm_a*math.cos(angle_m2)-(-self.arm_c*math.cos(angle_m3))) 
		psi = math.acos((f**2-self.arm_c**2-g**2)/(-2*self.arm_c*g))
		psi_1 = math.acos((self.arm_b**2-self.arm_d**2-g**2)/(-2*self.arm_d*g))
		psi = psi + psi_1

		x = self.motors_distance + self.arm_c*math.sin(angle_m3) - (self.arm_d+self.arm_e)*math.cos(psi-(math.pi/2-angle_m3))
		zp = -self.arm_c*math.cos(angle_m3) - (self.arm_d+self.arm_e)*math.sin(psi-(math.pi/2-angle_m3))
		z = zp*math.cos(angle_m1)
		y = -zp*math.sin(angle_m1)
		# print(x)
		# print(y)
		# print(z)

		# Transform the leg position from the coordinate in derivation 
		# to the coordinate in real world.
		# (The real world x axis is reversed to the derivation x axis)
		x = -x + 0.08378
		pos = 100 * np.array([x, y, z])

		return pos 

	def leg_angle_grad(self, angle):
		shift = 1e-8

		angle_m1 = angle[0]
		angle_m2 = angle[1]
		angle_m3 = angle[2]

		m1_plus = self.leg_rad2pos([angle_m1 + shift, angle_m2, angle_m3])
		m1_minus = self.leg_rad2pos([angle_m1 - shift, angle_m2, angle_m3])
		m1_grad = (0.5 * (m1_plus - m1_minus))/shift

		m2_plus = self.leg_rad2pos([angle_m1, angle_m2 + shift, angle_m3])
		m2_minus = self.leg_rad2pos([angle_m1, angle_m2 - shift, angle_m3])
		m2_grad = (0.5 * (m2_plus - m2_minus))/shift

		m3_plus = self.leg_rad2pos([angle_m1, angle_m2, angle_m3 + shift])
		m3_minus = self.leg_rad2pos([angle_m1, angle_m2, angle_m3 - shift])
		m3_grad = (0.5 * (m3_plus - m3_minus))/shift

		# print(m1_grad)
		# print(m2_grad)
		# print(m3_grad)

		jacobian = np.matrix([m1_grad, m2_grad, m3_grad])
		# print(jacobian)

		return jacobian

	def leg_omega2velocity(self, angle, omega):
		jacobian = self.leg_angle_grad(angle)
		velocity = omega*jacobian
		return velocity


	def leg_status_update(self, angle, omega):
		self.angle = angle
		self.omega = omega
		self.pos = self.leg_rad2pos(angle)
		self.vel = self.leg_vel2omega(angle, omega)

	def boundary(self):
		pass

leg = Leg()
rad = leg.leg_pos2rad([10.0, 0.0, -15.0])
print(rad)

# pos = leg.leg_rad2pos([0, math.radians(45), math.radians(45)])
# print(pos)