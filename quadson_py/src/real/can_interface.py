import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

import struct
import math
from src.real.can_config import *

import time
from cando import *


dev_lists = list_scan()


if len(dev_lists):
    for dev in dev_lists:

        serial_number = dev_get_serial_number_str(dev)

        dev_info = dev_get_dev_info_str(dev)

        print("Serial Number: " + serial_number + ', Dev Info: ' + dev_info)
else:
    print("Device not found!")
    sys.exit(0)

ID_STD_OFFSET = 6
ID_EXT_OFFSET = 24

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
				group_mag = True
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
