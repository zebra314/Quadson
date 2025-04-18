import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from cando import *
from typing import Any
import math
import struct
from src.real.can_config import *

class Motor:
  ID_STD_OFFSET = 6
  ID_EXT_OFFSET = 24

  def __init__(self, motor_id: int, exist: bool, dev_handle: Any):
    self.motor_id = motor_id
    self.exist = exist
    self.dev_handle = dev_handle
    self.params = dict(Param_Dict)
  
  def send_cmd(self, cmd, value):
    if self.exist == False:
      print("[WARN] Command not send because motor not connect.")
      return

    send_frame = Frame()
    
    if (cmd.__class__) == CAN_STD_TYPE:
      send_frame.can_id = 0x00 | (self.motor_id << self.ID_STD_OFFSET) | cmd.value
    else:
      send_frame.can_id = 0x00 | (self.motor_id << self.ID_EXT_OFFSET) | cmd.value
      send_frame.can_id |= CANDO_ID_EXTENDED

    send_frame.can_dlc = 2
    send_frame.data = [value>>8, value&0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    dev_frame_send(self.dev_handle, send_frame)
    # print("Sending message id: " + str(send_frame.can_id))

  def decode_msg(self, receive_frame):
    print(" is_extend : " + ("True" if receive_frame.can_id & CANDO_ID_EXTENDED else "False"))
    print(" is_rtr : " + ("True" if receive_frame.can_id & CANDO_ID_RTR  else "False"))
    print(" can_id : " + str(receive_frame.can_id & CANDO_ID_MASK))
    can_id = receive_frame.can_id & CANDO_ID_MASK
    can_dlc = receive_frame.can_dlc
    group_msg = False
    if (receive_frame.can_id & CANDO_ID_EXTENDED):
      motor_id = can_id>>24
      id_type = CAN_ID_TYPE.EXTENDED
      msg_id = can_id & 0xFFFFF
      value = struct.unpack("<h",bytes(receive_frame.data[0:2]))[0]
      print("from motor " + str(motor_id) + " get msg: " + str(CAN_EXT_TYPE(msg_id)) + " value: " + str(value))

    else:
      motor_id = can_id>>6
      if (motor_id>>10 > 0):
        group_msg = True
      id_type = CAN_ID_TYPE.STANDARD
      msg_id = can_id & 0x1F
      value = struct.unpack("<h",bytes(receive_frame.data[0:2]))[0]
      print("from motor " + str(motor_id) + " get msg: " + str(CAN_STD_TYPE(msg_id)) + " value: " + str(value))

    if (group_msg):
      # TODO : add group msg handle
      pass
    
    return self.CanMessage(motor_id, id_type, msg_id, value)
  
  def get_angle(self):
    self.send_cmd(CAN_STD_TYPE.CAN_STDID_PRESENT_REVOLUTION, 0)
    self.send_cmd(CAN_STD_TYPE.CAN_STDID_PRESENT_POSITION_DEG, 0)
    revolution = self.get_param(CMD_TYPE.PRESENT_REVOLUTION)
    angle_16 = self.get_param(CMD_TYPE.PRESENT_POSITION_DEG)
    # print("Revolution: " + str(revolution) + " Angle: " + str(angle_16))

    angle = (revolution + (angle_16 / 65536)) / 71.96 * 2 * math.pi
    # print("Degree: " + str(math.degrees(angle)))

    return angle

  def update_param(self, id_type, msg_id, value):
    if id_type == CAN_ID_TYPE.EXTENDED:
      self.params[msg_id] = value
    else:
      if msg_id == CAN_STD_TYPE.CAN_STDID_TORQUE_ENABLE:
        self.params[CMD_TYPE.TORQUE_ENABLE.value] = value
      elif msg_id == CAN_STD_TYPE.CAN_STDID_STATE_MACHINE:
        self.params[CMD_TYPE.STATE_MACHINE.value] = value
      elif msg_id == CAN_STD_TYPE.CAN_STDID_CONTROL_MODE:
        self.params[CMD_TYPE.CONTROL_MODE.value] = value

      # Goal
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

      # Present
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

  def get_param(self, param_id):
    return self.params[param_id.value]