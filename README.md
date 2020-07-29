## gripper control

> 1. start the can communication:
`rosrun gripper_control can_prepare.sh`


> 2. gripper control code define：[gripper_control.srv](/src/gripper_control/gripper_control/srv/gripper_control.srv)

> 3. gripper control example(with ros server):
>> start the gripper control server:
>>` roslaunch gripper_control gripper_control_server.launch`  
>> close and open gripper:
>> `rosrun gripper_control gripper_control_client.py`
