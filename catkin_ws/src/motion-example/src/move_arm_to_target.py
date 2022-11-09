import move_arm
import re
import yaml
import subprocess

# fanuc = move_arm.Robot()
    # fanuc.move_arm_to_home()
    # fanuc.move_joints()
    # print(fanuc.group.get_current_pose())
while True:
    result = subprocess.run(['sh', '/home/ros/catkin_ws/src/fanuc/fanuc_lrmate200ic_moveit_plugins/lrmate200ic5l_kinematics/src', '0 0 1 0.4 0 1 0 0.4 -1 0 0 0.7'], stdout=subprocess.PIPE)
    print(result.stdout)
    # joints = 1
    # fanuc.move_joints(joints)