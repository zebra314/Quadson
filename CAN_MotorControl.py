# import sys
# from cando import *


# dev_lists = list_scan()


# if len(dev_lists):
#     for dev in dev_lists:

#         serial_number = dev_get_serial_number_str(dev)

#         dev_info = dev_get_dev_info_str(dev)

#         print("Serial Number: " + serial_number + ', Dev Info: ' + dev_info)
# else:
#     print("Device not found!")
#     sys.exit(0)


from enum import IntEnum, auto
import struct
import math
import numpy as np

ID_STD_OFFSET = 6
ID_EXT_OFFSET = 24


class CAN_ID_TYPE(IntEnum):
	STANDARD = 0
	EXTENDED = 1

class CAN_STD_TYPE(IntEnum):
	CAN_STDID_ECHO = 0
	CAN_STDID_ALL_PARAM = auto()
	CAN_STDID_INFO_PARAM = auto()

	CAN_STDID_TORQUE_ENABLE = auto()
	CAN_STDID_STATE_MACHINE = auto()
	CAN_STDID_CONTROL_MODE = auto()

	CAN_STDID_GOAL_REVOLUTION = auto()
	CAN_STDID_GOAL_POSITION_DEG = auto()
	CAN_STDID_GOAL_VELOCITY_DPS = auto()
	CAN_STDID_GOAL_TORQUE_CURRENT_MA = auto()
	CAN_STDID_GOAL_FLUX_CURRENT_MA = auto()

	CAN_STDID_PRESENT_REVOLUTION = auto()
	CAN_STDID_PRESENT_POSITION_DEG = auto()
	CAN_STDID_PRESENT_VELOCITY_DPS = auto()
	CAN_STDID_PRESENT_TORQUE_CURRENT_MA = auto()
	CAN_STDID_PRESENT_FLUX_CURRENT_MA = auto()

	CAN_STDID_PRESENT_VOLTAGE = auto()
	CAN_STDID_PRESENT_TEMPERATURE = auto()
	CAN_STDID_MOVING = auto()

	CAN_STDID_GROUP0 = auto()
	CAN_STDID_GROUP1 = auto()
	CAN_STDID_GROUP2 = auto()
	CAN_STDID_GROUP3 = auto()

class CAN_EXT_TYPE(IntEnum):
	CAN_EXTID_MODEL_NUMBER = 0
	CAN_EXTID_VERSION = auto()
	CAN_EXTID_ID = auto()
	CAN_EXTID_BAUD_RATE = auto()
	CAN_EXTID_RECALIBRATE = auto()
	CAN_EXTID_GEARBOX_RATIO = auto()
	CAN_EXTID_MIN_POSITION_DEG = auto()
	CAN_EXTID_MAX_POSITION_DEG = auto()
	CAN_EXTID_MAX_VELOCITY_DPS = auto()
	CAN_EXTID_MAX_ACCELERATION_DPSS = auto()
	CAN_EXTID_MAX_CURRENT_MA = auto()
	CAN_EXTID_RESERVED_RAM_1 = auto()
	CAN_EXTID_TEMPERATURE_LIMIT = auto()
	CAN_EXTID_LOW_VOLTAGE_LIMIT = auto()
	CAN_EXTID_HIGH_VOLTAGE_LIMIT = auto()
	CAN_EXTID_MOVING_THRESHOLD_DPS = auto()
	CAN_EXTID_STATUS_RETURN_LVL = auto()
	CAN_EXTID_ALARM_LED = auto()
	CAN_EXTID_ALARM_SHUTDOWN = auto()
	CAN_EXTID_MOTOR_POLE_PAIRS = auto()
	CAN_EXTID_ELEC_SYNCHRO = auto()
	CAN_EXTID_MOTOR_OFFSET = auto()
	CAN_EXTID_POSITION_DEADZONE = auto()
	CAN_EXTID_INV_PHASE_MOTOR = auto()
	CAN_EXTID_INV_ROTATION = auto()
	CAN_EXTID_PID_POSITION_KP = auto()
	CAN_EXTID_PID_POSITION_KI = auto()
	CAN_EXTID_PID_POSITION_KD = auto()
	CAN_EXTID_PID_VELOCITY_KP = auto()
	CAN_EXTID_PID_VELOCITY_KI = auto()
	CAN_EXTID_PID_VELOCITY_KD = auto()
	CAN_EXTID_PID_VELOCITY_KFF = auto()
	CAN_EXTID_PID_ACCELERATION_KFF = auto()
	CAN_EXTID_PID_FLUX_CURRENT_KP = auto()
	CAN_EXTID_PID_FLUX_CURRENT_KI = auto()
	CAN_EXTID_PID_FLUX_CURRENT_KFF = auto()
	CAN_EXTID_PID_TORQUE_CURRENT_KP = auto()
	CAN_EXTID_PID_TORQUE_CURRENT_KI = auto()
	CAN_EXTID_PID_TORQUE_CURRENT_KFF = auto()
	CAN_EXTID_CAL_PHASE1_CURRENT_SENSE_MA = auto()
	CAN_EXTID_CAL_PHASE1_CURRENT_SENSE_OFFSET = auto()
	CAN_EXTID_CAL_PHASE2_CURRENT_SENSE_MA = auto()
	CAN_EXTID_CAL_PHASE2_CURRENT_SENSE_OFFSET = auto()
	CAN_EXTID_CAL_PHASE3_CURRENT_SENSE_MA = auto()
	CAN_EXTID_CAL_PHASE3_CURRENT_SENSE_OFFSET = auto()
	CAN_EXTID_CAL_VOLTAGE_SENSOR = auto()
	CAN_EXTID_TORQUE_ENABLE = auto()
	CAN_EXTID_STATE_MACHINE = auto()
	CAN_EXTID_CONTROL_MODE = auto()
	CAN_EXTID_GOAL_POSITION_DEG = auto()
	CAN_EXTID_GOAL_VELOCITY_DPS = auto()
	CAN_EXTID_GOAL_TORQUE_CURRENT_MA = auto()
	CAN_EXTID_GOAL_FLUX_CURRENT_MA = auto()
	CAN_EXTID_GOAL_REVOLUTION = auto()
	CAN_EXTID_GOAL_RESERVED_RAM_2 = auto()
	CAN_EXTID_GOAL_CLOSED_LOOP = auto()
	CAN_EXTID_PRESENT_REVOLUTION = auto()
	CAN_EXTID_PRESENT_POSITION_DEG = auto()
	CAN_EXTID_PRESENT_VELOCITY_DPS = auto()
	CAN_EXTID_PRESENT_TORQUE_CURRENT_MA = auto()
	CAN_EXTID_PRESENT_FLUX_CURRENT_MA = auto()
	CAN_EXTID_PRESENT_VOLTAGE = auto()
	CAN_EXTID_PRESENT_TEMPERATURE = auto()
	CAN_EXTID_MOVING = auto()
	CAN_EXTID_ALL_PARAM = auto()
	CAN_EXTID_INFO_PARAM = auto()

class CMD_TYPE(IntEnum):
	MODEL_NUMBER = 0
	VERSION = auto()
	ID = auto()
	BAUD_RATE = auto()
	RECALIBRATE = auto()
	GEARBOX_RATIO = auto()
	MIN_POSITION_DEG = auto()
	MAX_POSITION_DEG = auto()
	MAX_VELOCITY_DPS = auto()
	MAX_ACCELERATION_DPSS = auto()
	MAX_CURRENT_MA = auto()
	RESERVED_RAM_1 = auto()
	TEMPERATURE_LIMIT = auto()
	LOW_VOLTAGE_LIMIT = auto()
	HIGH_VOLTAGE_LIMIT = auto()
	MOVING_THRESHOLD_DPS = auto()
	STATUS_RETURN_LVL = auto()
	ALARM_LED = auto()
	ALARM_SHUTDOWN = auto()
	MOTOR_POLE_PAIRS = auto()
	ELEC_SYNCHRO = auto()
	MOTOR_OFFSET = auto()
	POSITION_DEADZONE = auto()
	INV_PHASE_MOTOR = auto()
	INV_ROTATION = auto()
	PID_POSITION_KP = auto()
	PID_POSITION_KI = auto()
	PID_POSITION_KD = auto()
	PID_VELOCITY_KP = auto()
	PID_VELOCITY_KI = auto()
	PID_VELOCITY_KD = auto()
	PID_VELOCITY_KFF = auto()
	PID_ACCELERATION_KFF = auto()
	PID_FLUX_CURRENT_KP = auto()
	PID_FLUX_CURRENT_KI = auto()
	PID_FLUX_CURRENT_KFF = auto()
	PID_TORQUE_CURRENT_KP = auto()
	PID_TORQUE_CURRENT_KI = auto()
	PID_TORQUE_CURRENT_KFF = auto()
	CAL_PHASE1_CURRENT_SENSE_MA = auto()
	CAL_PHASE1_CURRENT_SENSE_OFFSET = auto()
	CAL_PHASE2_CURRENT_SENSE_MA = auto()
	CAL_PHASE2_CURRENT_SENSE_OFFSET = auto()
	CAL_PHASE3_CURRENT_SENSE_MA = auto()
	CAL_PHASE3_CURRENT_SENSE_OFFSET = auto()
	CAL_VOLTAGE_SENSOR = auto()
	TORQUE_ENABLE = auto()
	CONTROL_MODE = auto()
	GOAL_REVOLUTION = auto()
	GOAL_POSITION_DEG = auto()
	GOAL_VELOCITY_DPS = auto()
	GOAL_TORQUE_CURRENT_MA = auto()
	GOAL_FLUX_CURRENT_MA = auto()
	GOAL_RESERVED_RAM_2 = auto()
	GOAL_CLOSED_LOOP = auto()
	PRESENT_REVOLUTION = auto()
	PRESENT_POSITION_DEG = auto()
	PRESENT_VELOCITY_DPS = auto()
	PRESENT_TORQUE_CURRENT_MA = auto()
	PRESENT_FLUX_CURRENT_MA = auto()
	PRESENT_VOLTAGE = auto()
	PRESENT_TEMPERATURE = auto()
	MOVING = auto()

Param_Dict = {
	CMD_TYPE.MODEL_NUMBER : 0,
	CMD_TYPE.VERSION : 0,
	CMD_TYPE.ID : 0,
	CMD_TYPE.BAUD_RATE : 0,
	CMD_TYPE.RECALIBRATE : 0,
	CMD_TYPE.GEARBOX_RATIO : 0,
	CMD_TYPE.MIN_POSITION_DEG : 0,
	CMD_TYPE.MAX_POSITION_DEG : 0,
	CMD_TYPE.MAX_VELOCITY_DPS : 0,
	CMD_TYPE.MAX_ACCELERATION_DPSS : 0,
	CMD_TYPE.MAX_CURRENT_MA : 0,
	CMD_TYPE.RESERVED_RAM_1 : 0,
	CMD_TYPE.TEMPERATURE_LIMIT : 0,
	CMD_TYPE.LOW_VOLTAGE_LIMIT : 0,
	CMD_TYPE.HIGH_VOLTAGE_LIMIT : 0,
	CMD_TYPE.MOVING_THRESHOLD_DPS : 0,
	CMD_TYPE.STATUS_RETURN_LVL : 0,
	CMD_TYPE.ALARM_LED : 0,
	CMD_TYPE.ALARM_SHUTDOWN : 0,
	CMD_TYPE.MOTOR_POLE_PAIRS : 0,
	CMD_TYPE.ELEC_SYNCHRO : 0,
	CMD_TYPE.MOTOR_OFFSET : 0,
	CMD_TYPE.POSITION_DEADZONE : 0,
	CMD_TYPE.INV_PHASE_MOTOR : 0,
	CMD_TYPE.INV_ROTATION : 0,
	CMD_TYPE.PID_POSITION_KP : 0,
	CMD_TYPE.PID_POSITION_KI : 0,
	CMD_TYPE.PID_POSITION_KD : 0,
	CMD_TYPE.PID_VELOCITY_KP : 0,
	CMD_TYPE.PID_VELOCITY_KI : 0,
	CMD_TYPE.PID_VELOCITY_KD : 0,
	CMD_TYPE.PID_VELOCITY_KFF : 0,
	CMD_TYPE.PID_ACCELERATION_KFF : 0,
	CMD_TYPE.PID_FLUX_CURRENT_KP : 0,
	CMD_TYPE.PID_FLUX_CURRENT_KI : 0,
	CMD_TYPE.PID_FLUX_CURRENT_KFF : 0,
	CMD_TYPE.PID_TORQUE_CURRENT_KP : 0,
	CMD_TYPE.PID_TORQUE_CURRENT_KI : 0,
	CMD_TYPE.PID_TORQUE_CURRENT_KFF : 0,
	CMD_TYPE.CAL_PHASE1_CURRENT_SENSE_MA : 0,
	CMD_TYPE.CAL_PHASE1_CURRENT_SENSE_OFFSET : 0,
	CMD_TYPE.CAL_PHASE2_CURRENT_SENSE_MA : 0,
	CMD_TYPE.CAL_PHASE2_CURRENT_SENSE_OFFSET : 0,
	CMD_TYPE.CAL_PHASE3_CURRENT_SENSE_MA : 0,
	CMD_TYPE.CAL_PHASE3_CURRENT_SENSE_OFFSET : 0,
	CMD_TYPE.CAL_VOLTAGE_SENSOR : 0,
	CMD_TYPE.TORQUE_ENABLE : 0,
	CMD_TYPE.CONTROL_MODE : 0,
	CMD_TYPE.GOAL_REVOLUTION : 0,
	CMD_TYPE.GOAL_POSITION_DEG : 0,
	CMD_TYPE.GOAL_VELOCITY_DPS : 0,
	CMD_TYPE.GOAL_TORQUE_CURRENT_MA : 0,
	CMD_TYPE.GOAL_FLUX_CURRENT_MA : 0,
	CMD_TYPE.GOAL_RESERVED_RAM_2 : 0,
	CMD_TYPE.GOAL_CLOSED_LOOP : 0,
	CMD_TYPE.PRESENT_REVOLUTION : 0,
	CMD_TYPE.PRESENT_POSITION_DEG : 0,
	CMD_TYPE.PRESENT_VELOCITY_DPS : 0,
	CMD_TYPE.PRESENT_TORQUE_CURRENT_MA : 0,
	CMD_TYPE.PRESENT_FLUX_CURRENT_MA : 0,
	CMD_TYPE.PRESENT_VOLTAGE : 0,
	CMD_TYPE.PRESENT_TEMPERATURE : 0,
	CMD_TYPE.MOVING : 0,
}

def msg_error(rec_frame):
	if rec_frame.can_id & CANDO_ID_ERR:
		error_code, err_tx, err_rx = parse_err_frame(rec_frame)
		print("Error: ")
		print(error_code)
		if error_code & CAN_ERR_BUSOFF:
			print(" CAN_ERR_BUSOFF")
		if error_code & CAN_ERR_RX_TX_WARNING:
			print(" CAN_ERR_RX_TX_WARNING")
		if error_code & CAN_ERR_RX_TX_PASSIVE:
			print(" CAN_ERR_RX_TX_PASSIVE")
		if error_code & CAN_ERR_OVERLOAD:
			print(" CAN_ERR_OVERLOAD")
		if error_code & CAN_ERR_STUFF:
			print(" CAN_ERR_STUFF")
		if error_code & CAN_ERR_FORM:
			print(" CAN_ERR_FORM")
		if error_code & CAN_ERR_ACK:
			print(" CAN_ERR_ACK")
		if error_code & CAN_ERR_BIT_RECESSIVE:
			print(" CAN_ERR_BIT_RECESSIVE")
		if error_code & CAN_ERR_BIT_DOMINANT:
			print(" CAN_ERR_BIT_DOMINANT")
		if error_code & CAN_ERR_CRC:
			print(" CAN_ERR_CRC")
			print(" err_tx: " + str(err_tx))
			print(" err_rx: " + str(err_rx))

		return True

	else:
		return False



class Can_msg_package(object):
	def __init__(self, motor_id, id_type, msg_id, data):
		self.motor_id = motor_id
		self.id_type = id_type
		self.msg_id = msg_id 
		self.data = data

class Can_motor(object):
	def __init__(self, motor_id, exist):
		self.motor_id = motor_id
		self.exist = exist
		self.params = dict(Param_Dict)

	def can_motor_param_update(self, id_type, msg_id, value):
		if id_type == CAN_ID_TYPE.EXTENDED:
			self.params[msg_id] = value
		else:
			if msg_id == CAN_STD_TYPE.CAN_STDID_TORQUE_ENABLE:
				self.params[CMD_TYPE.TORQUE_ENABLE.value] = value
			elif msg_id == CAN_STD_TYPE.CAN_STDID_STATE_MACHINE:
				self.params[CMD_TYPE.STATE_MACHINE.value] = value
			elif msg_id == CAN_STD_TYPE.CAN_STDID_CONTROL_MODE:
				self.params[CMD_TYPE.CONTROL_MODE.value] = value

			elif msg_id == CAN_STD_TYPE.CAN_STDID_GOAL_REVOLUTION:
				self.params[CMD_TYPE.GOAL_REVOLUTION.value] = value
			elif msg_id == CAN_STD_TYPE.CAN_STDID_GOAL_POSITION_DEG:
				self.params[CMD_TYPE.GOAL_POSITION_DEG.value] = value
			elif msg_id == CAN_STD_TYPE.CAN_STDID_GOAL_VELOCITY_DPS:
				self.params[CMD_TYPE.GOAL_VELOCITY_DPS.value] = value
			elif msg_id == CAN_STD_TYPE.CAN_STDID_GOAL_TORQUE_CURRENT_MA:
				self.params[CMD_TYPE.GOAL_TORQUE_CURRENT_MA.value] = value
			elif msg_id == CAN_STD_TYPE.CAN_STDID_GOAL_FLUX_CURRENT_MA:
				self.params[CMD_TYPE.GOAL_FLUX_CURRENT_MA.value] = value

			elif msg_id == CAN_STD_TYPE.CAN_STDID_PRESENT_REVOLUTION:
				self.params[CMD_TYPE.PRESENT_REVOLUTION.value] = value
			elif msg_id == CAN_STD_TYPE.CAN_STDID_PRESENT_POSITION_DEG:
				self.params[CMD_TYPE.PRESENT_POSITION_DEG.value] = value
			elif msg_id == CAN_STD_TYPE.CAN_STDID_PRESENT_VELOCITY_DPS:
				self.params[CMD_TYPE.PRESENT_VELOCITY_DPS.value] = value
			elif msg_id == CAN_STD_TYPE.CAN_STDID_PRESENT_TORQUE_CURRENT_MA:
				self.params[CMD_TYPE.PRESENT_TORQUE_CURRENT_MA.value] = value
			elif msg_id == CAN_STD_TYPE.CAN_STDID_PRESENT_FLUX_CURRENT_MA:
				self.params[CMD_TYPE.PRESENT_FLUX_CURRENT_MA.value] = value

			elif msg_id == CAN_STD_TYPE.CAN_STDID_PRESENT_VOLTAGE:
				self.params[CMD_TYPE.PRESENT_VOLTAGE.value] = value
			elif msg_id == CAN_STD_TYPE.CAN_STDID_PRESENT_TEMPERATURE:
				self.params[CMD_TYPE.PRESENT_TEMPERATURE.value] = value
			elif msg_id == CAN_STD_TYPE.CAN_STDID_MOVING:
				self.params[CMD_TYPE.MOVING.value] = value

	def can_motor_get_param(self, param_id):

		return self.params[param_id.value]



import threading	
import os	
class Can_motor_manager(object):
	def __init__(self):
		self.dev_lists = None
		self.motor_list = [None]*12
		for x in range(12):
			self.motor_list[x] = Can_motor(x, False)
		self.rec_frame = Frame()

		self.connect_can_device()
		self.thread_pause_event = threading.Event()
		self.thread_stop_event = threading.Event()
		self.reading_thread = threading.Thread(target = self.can_read_handle)

		self.thread_stop_event.set()
		self.reading_thread.start()

	def connect_can_device(self):
		self.dev_lists = list_scan()

		if len(self.dev_lists) == 0:
			print("Device not found!")
			sys.exit(0)

		if os.name == 'posix':  # for linux os
			if self.dev_lists[0].is_kernel_driver_active(0):
				reattach = True
				self.dev_lists[0].detach_kernel_driver(0)

		# set baudrate: 500K, sample point: 87.5%
		dev_set_timing(self.dev_lists[0], 1, 12, 6, 1, 6)

		dev_start(self.dev_lists[0], CANDO_MODE_NORMAL|CANDO_MODE_NO_ECHO_BACK)

	def disconnect_can_device(self):
		self.thread_stop_event.clear()
		dev_stop(self.dev_lists[0])

	def set_auto_receive(self, enable):
		if enable:
			self.thread_pause_event.set()
		else:
			self.thread_pause_event.clear()

	def scan_motors(self):
		print("Scaning...")

		for i in range(1,13):
			send_frame = Frame()
			send_frame.can_id = 0x00 | (i<<6)
			send_frame.can_id |= CANDO_ID_RTR
			# send_frame.can_dlc = 8
			# send_frame.data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]
			dev_frame_send(self.dev_lists[0], send_frame)
			# print("scannnig "+str(i))
			time.sleep(0.001)

			if dev_frame_read(self.dev_lists[0], self.rec_frame, 1):
				if (not msg_error(self.rec_frame)):
					can_pack = self.decode_msg(self.rec_frame)
					if i == can_pack.motor_id:
						self.motor_list[i-1].exist = True
						print("Receive from motor " + str(i))
					else:
						print("Scanning error")
						self.disconnect_can_device()
						sys.exit(0)

			else:
				self.motor_list[i-1].exist = False
				print("Motor " + str(i) + " no response")

		for x in range(10):
			dev_frame_read(self.dev_lists[0], self.rec_frame, 1)


	def decode_msg(self, rec_frame):
		# print(" is_extend : " + ("True" if self.rec_frame.can_id & CANDO_ID_EXTENDED else "False"))
		# print(" is_rtr : " + ("True" if self.rec_frame.can_id & CANDO_ID_RTR	else "False"))
		# print(" can_id : " + str(self.rec_frame.can_id & CANDO_ID_MASK))
		can_id = self.rec_frame.can_id & CANDO_ID_MASK
		can_dlc = self.rec_frame.can_dlc
		group_mag = False
		if (self.rec_frame.can_id & CANDO_ID_EXTENDED):
			motor_id = can_id>>24
			id_type = CAN_ID_TYPE.EXTENDED
			msg_id = can_id & 0xFFFFF
			value = struct.unpack("<h",bytes(self.rec_frame.data[0:2]))[0]
			# print("from motor " + str(motor_id) + " get msg: " + str(CAN_EXT_TYPE(msg_id)) + " value: " + str(value))

		else:
			motor_id = can_id>>6
			if (motor_id>>10 > 0):
				group_mag = true
			id_type = CAN_ID_TYPE.STANDARD
			msg_id = can_id & 0x1F
			value = struct.unpack("<h",bytes(self.rec_frame.data[0:2]))[0]
			# print("from motor " + str(motor_id) + " get msg: " + str(CAN_STD_TYPE(msg_id)) + " value: " + str(value))

		if (group_mag):
			# TODO: add group msg handle
			pass

		self.motor_list[motor_id-1].can_motor_param_update(id_type, msg_id, value)
		
		return Can_msg_package(motor_id, id_type, msg_id, value)

	def can_read_handle(self):
		while self.thread_stop_event.isSet():
			self.thread_pause_event.wait()
			if dev_frame_read(self.dev_lists[0], self.rec_frame, 1):
				if (not msg_error(self.rec_frame)):
					self.decode_msg(self.rec_frame)
				else:
					print("reading error")
					self.disconnect_can_device()


	def send_motor_cmd(self, motor_index, cmd, value):

		if self.motor_list[motor_index-1].exist == False:
			print("Motor unconnected")
			return

		send_frame = Frame()
		
		if (cmd.__class__) == CAN_STD_TYPE:
			send_frame.can_id = 0x00 | (motor_index << ID_STD_OFFSET) | cmd.value
		else:
			send_frame.can_id = 0x00 | (motor_index << ID_EXT_OFFSET) | cmd.value
			send_frame.can_id |= CANDO_ID_EXTENDED

		send_frame.can_dlc = 2
		send_frame.data = [value>>8, value&0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
		dev_frame_send(self.dev_lists[0], send_frame)
		# print("sending message id:" + str(send_frame.can_id))

	def can_motor_manager_get_angle(self, motor_id):
		self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_PRESENT_REVOLUTION, 0)
		self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_PRESENT_POSITION_DEG, 0)
		revolution = self.motor_list[motor_id-1].can_motor_get_param(CMD_TYPE.PRESENT_REVOLUTION)
		angle_16 = self.motor_list[motor_id-1].can_motor_get_param(CMD_TYPE.PRESENT_POSITION_DEG)
		# print("rev: " + str(revolution) + "  angle: " + str(angle_16))

		angle = (revolution + ( (angle_16) / 65536))/71.96 * 2*math.pi
		# print("degree: " + str(math.degrees(angle)))
		return angle

	def can_motor_manager_get_omega(self, motor_id):
		self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_PRESENT_VELOCITY_DPS, 0)
		velocity_01 = self.motor_list[motor_id-1].can_motor_get_param(CMD_TYPE.PRESENT_VELOCITY_DPS)

		omega = (((velocity_01*10) / 65536))/71.96 * 2*math.pi
		# print("omega: " + str(omega))
		return omega

import numpy
class Leg(object):
	"""docstring for Leg"""
	def __init__(self, 
				# motor_1_obj, 
				# motor_2_obj,
				motors_distance=0.082,
				arm_a=0.08, 
				arm_b=0.13, 
				arm_c=0.08, 
				arm_d=0.10, 
				arm_e=0.066):

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

		angle_m1 = math.atan2(y, -z)
		zp = -self.calc_r(y, z)
		r_m3 = self.calc_r((self.motors_distance-x), zp)
		theta_m3 = math.atan2((self.motors_distance-x), -zp)
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
		# print(math.degrees(angle_m1))
		# print(math.degrees(angle_m2))
		# print(math.degrees(angle_m3))

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

		return np.array([x, y, z])

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
		self.pos = leg_rad2pos(angle)
		self.vel = leg_vel2omega(angle, omega)

	def boundary(self):
		pass
	
class Legs_manager(object):
	"""docstring for Legs_manager"""
	def __init__(self, motor_manager):
		self.motor_manager = motor_manager
		self.legs_list = [None]*4
		for i in range(1,5):
			self.leg_maganger_assign_motor(i, 3*(i-1)+1, 3*(i-1)+2, 3*(i-1)+3)
			pass

	def leg_maganger_assign_motor(self, leg_id, motor1_id, motor2_id, motor3_id):
		self.legs_list[leg_id-1] = Leg()

	def leg_manager_leg_group_update(self, leg_id):

		self.motor_manager
		pass

	def leg_manager_leg_update(self, leg_id):
		motor1_index = (leg_id-1)*3+1
		motor2_index = (leg_id-1)*3+2
		motor3_index = (leg_id-1)*3+3
		motor1_angle = self.motor_manager.can_motor_manager_get_angle(motor1_index)
		motor2_angle = self.motor_manager.can_motor_manager_get_angle(motor2_index)
		motor3_angle = self.motor_manager.can_motor_manager_get_angle(motor3_index)
		motor1_omega = self.motor_manager.can_motor_manager_get_omega(motor1_index)
		motor2_omega = self.motor_manager.can_motor_manager_get_omega(motor2_index)
		motor3_omega = self.motor_manager.can_motor_manager_get_omega(motor3_index)

		angle = [motor1_angle, motor2_angle, motor3_angle]
		omega = [motor1_omega, motor2_omega, motor3_omega]
		self.legs_list[leg_id-1].angle = angle
		self.legs_list[leg_id-1].omega = omega
		self.legs_list[leg_id-1].pos = self.legs_list[leg_id-1].leg_rad2pos(angle)
		self.legs_list[leg_id-1].vel = self.legs_list[leg_id-1].leg_omega2velocity(angle, omega)

	def leg_maganger_get_leg_pos(self, leg_id):
		print(self.legs_list[leg_id-1].pos)

		return self.legs_list[leg_id-1].pos

	def leg_maganger_get_leg_pos(self, leg_id):
		print(self.legs_list[leg_id-1].vel)

		return self.legs_list[leg_id-1].vel


import sys
import time
from cando import *

if __name__ == '__main__':
	
	cmm = Can_motor_manager()

	cmm.scan_motors()

	cmm.set_auto_receive(True)

	lm = Legs_manager(cmm)

	cmm.send_motor_cmd(1, CAN_STD_TYPE.CAN_STDID_TORQUE_ENABLE, True)
	cmm.send_motor_cmd(2, CAN_STD_TYPE.CAN_STDID_TORQUE_ENABLE, True)

	for x in range(10):
        lm.leg_manager_leg_update(1)

		cmm.send_motor_cmd(1, CAN_STD_TYPE.CAN_STDID_GOAL_REVOLUTION, 0)
		cmm.send_motor_cmd(2, CAN_STD_TYPE.CAN_STDID_GOAL_REVOLUTION, 0)
		time.sleep(0.1)
	# # pass

    ######
    #TEST#    
    ######
    for x in range(10):
        lm.leg_manager_leg_update(1)
        cmm.send_motor_cmd(1, CAN_STD_TYPE.CAN_STDID_GOAL_REVOLUTION, 0)
        time.sleep(0.1)
	time.sleep(5)
    cmm.send_motor_cmd(1, CAN_STD_TYPE.CAN_STDID_TORQUE_ENABLE, False)
	cmm.send_motor_cmd(2, CAN_STD_TYPE.CAN_STDID_TORQUE_ENABLE, False)
    
	cmm.disconnect_can_device()

	time.sleep(1)

	print("exit")
