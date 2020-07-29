#!/usr/bin/env python2.7
# -*- coding: UTF-8 -*-
import rospy
from gripper_control.srv import *

def client_srv():
    rospy.init_node('gripper_control_client')
    rospy.loginfo("Wait for Gripper_control Server")
    rospy.wait_for_service("gripper_control")
    gripper_control_client = rospy.ServiceProxy("gripper_control",gripper_control)
    
    # close gripper
    resp = gripper_control_client.call(0)
    # open gripper
    resp = gripper_control_client.call(1)


if __name__ == "__main__":
    client_srv()