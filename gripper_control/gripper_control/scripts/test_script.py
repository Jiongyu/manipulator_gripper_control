#!/usr/bin/env python2.7
# -*- coding: UTF-8 -*-
import sys
from rospkg import RosPack
sys.path.append(RosPack().get_path('canopen_communication') + "/modular/")
from modular_T85 import T85

from time import sleep

def main():
    eds_file = RosPack().get_path('canopen_communication') + "/file/Copley.eds"
    T5 = T85(5, eds_file)
    T5.start()
    T5.serve_on()
    print "close gripper"
    # T5.close_gripper()
    # sleep(15)

    print "open gripper"
    # T5.open_gripper()
    # sleep(15)

    print "stop motion gripper"
    # T5.gripper_stop_motion()
    sleep(500)
    T5.stop()

if __name__ == "__main__":
    main()
