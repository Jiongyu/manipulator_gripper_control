#!/usr/bin/env python2.7
# -*- coding: UTF-8 -*-

import rospy
from gripper_control.srv import *
import can
from time import sleep

class gripper_control_server(object):

    def __init__(self):
        rospy.init_node('gripper_control_server')
        s = rospy.Service("gripper_control", gripper_control, self.handle_function)
        self.__bus = can.interface.Bus()
        self.__gripper_motion_time = 4

    def handle_function(self,req):
        if req.gripper_control_code == 0:
            temp = self.__close_gripper()
        elif req.gripper_control_code == 1:
            temp = self.__open_gripper()
        elif req.gripper_control_code == 2:
            temp = self.stop_motion_gripper()
        return gripper_controlResponse(temp)

    def __gripper_io_configure(self):
        can_msg = can.Message(arbitration_id=(0x605),
                                data=[0x21, 0x93, 0x21, 0x01, 0x06, 0x00, 0x00, 0x00],
                                extended_id=False)
        if self.__bus.send(can_msg) > 0:
            rospy.logwarn("Gripper Configuration Message NOT sent")
            return False
        can_msg = can.Message(arbitration_id=(0x605),
                                data=[0x03, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00],
                                extended_id=False)
        if self.__bus.send(can_msg) > 0:
            rospy.logwarn("Gripper Configuration Message NOT sent")
            return False
        can_msg = can.Message(arbitration_id=(0x605),
                                data=[0x21, 0x93, 0x21, 0x02, 0x06, 0x00, 0x00, 0x00],
                                extended_id=False)
        if self.__bus.send(can_msg) > 0:
            rospy.logwarn("Gripper Configuration Message NOT sent")
            return False
        can_msg = can.Message(arbitration_id=(0x605),
                                data=[0x03, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00],
                                extended_id=False)
        if self.__bus.send(can_msg) > 0:
            rospy.logwarn("Gripper Configuration Message NOT sent")
            return False
        return True

    def __open_gripper(self):
        rospy.loginfo("Open Gripper")
        self.__gripper_io_configure()

        can_msg = can.Message(arbitration_id=(0x605),
                                data=[0x2B, 0x94, 0x21, 0x00, 0x01, 0x00, 0x00, 0x00],
                                extended_id=False)
        if self.__bus.send(can_msg) > 0:
            rospy.logwarn("Gripper Open Message NOT sent")
            return False

        sleep(self.__gripper_motion_time)
        self.__stop_motion_gripper() 
        return True
   

    def __close_gripper(self):
        rospy.loginfo("Close Gripper")
        self.__gripper_io_configure()

        can_msg = can.Message(arbitration_id=(0x605),
                                data=[0x2B, 0x94, 0x21, 0x00, 0x02, 0x00, 0x00, 0x00],
                                extended_id=False)
        if self.__bus.send(can_msg) > 0:
            rospy.logwarn("Gripper Close Message NOT sent")
            return False

        sleep(self.__gripper_motion_time)
        self.__stop_motion_gripper()
        return True

    def __stop_motion_gripper(self):
        rospy.loginfo("Stop Motion Gripper")
        self.__gripper_io_configure()
        can_msg = can.Message(arbitration_id=(0x605),
                                data=[0x2B, 0x94, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00],
                                extended_id=False)
        if self.__bus.send(can_msg) > 0:
            rospy.logwarn("Gripper Stop Motion Message NOT sent")
            return False
        return True

if __name__ == "__main__":
    gripper = gripper_control_server()

    rospy.loginfo("Ready to handle the request:")
    # 阻塞程序结束
    rospy.spin()
    pass