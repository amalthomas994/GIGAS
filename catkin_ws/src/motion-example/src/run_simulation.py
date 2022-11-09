from ntpath import join
import move_arm
import re
import yaml
import subprocess
import time
import os
import yaml_to_mujoco

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
    
    def get_target_joints(self, target):
        print("Getting Target Joints")
        position = list(map(float, target["position"].strip().split()))
        orientation = list(map(float, target.get('orientation', None).strip().split()))
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
        # os.chdir("/home/ros/catkin_ws/src/motion-example/mjc")
        
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
# mover.plan_path(joint_values)
f = open("/home/ros/catkin_ws/src/motion-example/mjc/problems/joints.txt", "w")
f.write("{}".format(joint_values))