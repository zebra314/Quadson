import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

class CanMessage:
	def __init__(self, motor_id, id_type, msg_id, data):
		self.motor_id = motor_id
		self.id_type = id_type
		self.msg_id = msg_id 
		self.data = data