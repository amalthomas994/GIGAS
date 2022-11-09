from ntpath import join
import move_arm
import re
import yaml
import subprocess
import time
import os
import yaml_to_mujoco
import numpy as np
fanuc = move_arm.Robot()

class FanucControl():
    
    def __init__(self, yaml_file):
        self.fanuc = move_arm.Robot()
        # os.chdir("/home/ros/catkin_ws/src/motion-example/src")
        print(os.getcwd())
    def open(self, yaml_file):
        print("Opening YAML")
        with open(yaml_file, 'r') as file:
            self.data = yaml.safe_load(file)
        self.target = self.data["target"]
        self.planner = self.data["planner"]
        self.range = self.data["range"]
        
        print(self.target["position"])
    
    def get_quaternion_from_euler(self, orientation):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        roll = orientation[0]
        pitch = orientation[1]
        yaw = orientation[2]
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]
    def get_target_joints(self, target):
        print("Getting Target Joints")
        position = list(map(float, target["position"].strip().split()))
        orientation = list(map(float, target.get('orientation', None).strip().split()))
        orientation = self.get_quaternion_from_euler(orientation)
        print(orientation)
        fanuc.move_arm_to_position(position, orientation)
        time.sleep(5)
        joint_values = fanuc.move_arm_to_position(position, orientation)
        print("Got Target Joints", joint_values)
        return joint_values

    def plan_path(self, joint_values):
        print("Planning Path")
        # os.chdir("../mjc")
        f = open("/home/ros/catkin_ws/src/motion-example/mjc/problems/fanuc_prob.yaml", "w")
        f.write('mujoco_config: "robot.xml" \n')
        f.write("algorithm: {} \n".format(self.planner))
        f.write("range: {} \n".format(self.range))
        f.write("start: [0, 0, 0, 0, 0, 0] \n")
        f.write("goal: {} \n".format(joint_values))
        f.close()
        print("Anyyt")
        n = yaml_to_mujoco.YAMLtoMuJoCo("/home/ros/catkin_ws/src/motion-example/src/plan.yaml")
        print('wjat')
        # os.chdir("../mjc")
        print(os.getcwd())
        n.createXML("/home/ros/catkin_ws/src/motion-example/mjc/problems/obs.xml")
        # time.sleep(5)
        os.chdir("/home/ros/catkin_ws/src/motion-example/mjc")
        
        subprocess.call(["sh", "/home/ros/catkin_ws/src/motion-example/mjc/fanuc_kin"])
        # os.chdir("/home/ros/catkin_ws/src/motion-example/src")
        print(os.getcwd())
        print("Planning Complete")
        joint_values = open("/home/ros/catkin_ws/src/motion-example/mjc/plan.out")
        for line in joint_values:
            print(line)
            values = line.split(" ")
            values.pop()
            values = [ float(x) for x in values ]

            print(values)
            fanuc.move_joints(values)
        
mover = FanucControl("/home/ros/catkin_ws/src/motion-example/src/plan.yaml")
mover.open("/home/ros/catkin_ws/src/motion-example/src/plan.yaml")
target = mover.target
joint_values = mover.get_target_joints(target)
mover.plan_path(joint_values)