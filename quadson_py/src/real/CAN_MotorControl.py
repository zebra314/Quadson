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
