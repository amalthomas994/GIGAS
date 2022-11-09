#!/usr/bin/env python
import sys
import copy
import numpy as np
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped
from actionlib_msgs.msg import GoalStatusArray
import os
from realsense import Realsense
# import regiongrow

import time


class Robot():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.vacuum = rospy.Publisher('toggle_vacuum', Bool, queue_size=10)
        self.grasp_state = False
        self.robot_state = 0

        rospy.Subscriber('vacuum_state', Bool, self.vacuum_callback)
        rospy.Subscriber('/move_group/status',
                         GoalStatusArray, self.robot_callback)

        rospy.init_node('moving_robot_arm', anonymous=True)

        self.listener = tf.TransformListener()

        # Instantiate a RobotCommander object.  This object is an interface to
        # the self.robot as a whole.
        self.robot = moveit_commander.RobotCommander()

        # Instantiate a PlanningSceneInterface object.  This object is an interface
        # to the world surrounding the self.robot.
        self.scene = moveit_commander.PlanningSceneInterface()

        self.camera = Realsense()

        rospy.sleep(1)

        self.add_collision_workspace()

        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.group.set_end_effector_link("tool0")

    def add_collision_workspace(self):
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = 0
        p.pose.position.y = 0.
        p.pose.position.z = -0.011
        self.scene.add_box("table", p, (2, 2, 0.02))

        # p = geometry_msgs.msg.PoseStamped()
        # p.header.frame_id = self.robot.get_planning_frame()
        # p.pose.position.x = 0.593
        # p.pose.position.y = 0.037
        # p.pose.position.z = 0.012
        # self.scene.add_box("box", p, (0.70, 0.75, 0.048))

        # p = geometry_msgs.msg.PoseStamped()
        # p.header.frame_id = self.robot.get_planning_frame()
        # p.pose.position.x = 0
        # p.pose.position.y = -0.8
        # p.pose.position.z = 0.29
        # self.scene.add_box("tomato", p, (0.040, 0.045, 0.04))

    def move_joints(self, joints):
        joint_goal = self.group.get_current_joint_values()
        print(joints)
        joint_goal[0] = joints[0]
        joint_goal[1] = joints[1]
        joint_goal[2] = joints[2]
        joint_goal[3] = joints[3]
        joint_goal[4] = joints[4]
        joint_goal[5] = joints[5]

        self.group.go(joint_goal, wait=True)

        self.group.stop()

    def move_arm_to_home(self):
        # self.group.set_named_target('home')
        print('Moving Home')
        joint_goal = fanuc.group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = -0.2
        joint_goal[3] = -0.2
        joint_goal[4] = 0
        joint_goal[5] += np.pi
        fanuc.group.go(joint_goal, wait=True)

        # plan = self.group.go(wait=False)

        # print('wiaitng')
        # time.sleep(5)
        # print("Robot State before while loop", self.robot_state)

        # while self.robot_state != 3:
        #   print(self.robot_state)
        #   print('Waiting to finish...')

        # print('done')
        # self.group.stop()

    def move_arm_to_position(self, position, orientation=None):
        theta = 90 / 2
        print(orientation)
        print(position)
        pose_goal = self.group.get_current_pose().pose
        if orientation is None:
            pose_goal.orientation.w = np.cos(theta*np.pi/180)
            pose_goal.orientation.x = 0.0 * np.sin(theta*np.pi/180)
            pose_goal.orientation.y = 1.0 * np.sin(theta*np.pi/180)
            pose_goal.orientation.z = 0.0 * np.sin(theta*np.pi/180)
        else:
            pose_goal.orientation.w = orientation[3]
            pose_goal.orientation.x = orientation[0]
            pose_goal.orientation.y = orientation[1]
            pose_goal.orientation.z = orientation[2]
        pose_goal.position.x = position[0]
        pose_goal.position.y = position[1]
        pose_goal.position.z = position[2]
        print(pose_goal)

        self.group.set_pose_target(pose_goal)
        # self.group.plan(pose_goal)
        plan = self.group.go(wait=False)

        self.group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.group.clear_pose_targets()
        return self.group.get_current_joint_values()

    def move_arm_relative(self, relative_position):
        pose_goal = self.group.get_current_pose().pose

        pose_goal.position.x += relative_position[0]
        pose_goal.position.y += relative_position[1]
        pose_goal.position.z += relative_position[2]

        self.group.set_pose_target(pose_goal)

        self.group.plan(pose_goal)

        plan = self.group.go(wait=True)

        self.group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.group.clear_pose_targets()

    def vacuum_callback(self, state):
        if state.data == True:
            print('Grasped')

    def robot_callback(self, data):
        self.robot_state = data.status_list[0].status

    def activate_vacuum(self):
        self.vacuum.publish(True)

    def deactivate_vacuum(self):
        self.vacuum.publish(False)

    def transform_camera_to_base(self, location):
        (trans, rot) = self.listener.lookupTransform(
            '/base_link', '/camera_tool', rospy.Time(0))

        camera_point = PointStamped()
        camera_point.header.frame_id = "camera_origin"
        camera_point.header.stamp = rospy.Time(0)

        x, y, z = location

        camera_point.point.x = x
        camera_point.point.y = y
        camera_point.point.z = z
        world_msg = self.listener.transformPoint("base_link", camera_point)

        world_location = np.array(
            [world_msg.point.x, world_msg.point.y, world_msg.point.z])

        return world_location


if __name__ == '__main__':
    try:
        fanuc = Robot()
        # fanuc.move_arm_to_home()
        joints = [0.896055384571341, 1.683285515732639, 2.977573926381578, 0.914861419475263, -1.399418222850679, 2.923523573605125]
        fanuc.move_joints(joints)
        # print(fanuc.group.get_current_pose())
        # print (os.getcwd())
        # # with open("/home/ros/catkin_ws/src/motion-example/src/path_plan.txt", 'r') as file:
        # #     for line in file:
        # #         joints = list(map(float, (line.strip().split())))
        # #         fanuc.move_joints(joints)                
        # while True:
        #     position = list(map(float, raw_input(
        #             "Position (x, y, z): ").strip().split()))
        #     orientation = list(map(float, raw_input(
        #             "Orientation (x, y, z, w): ").strip().split()))
        #     print(orientation)
        #     if not orientation:
        #         orientation = None
                
        #     fanuc.move_arm_to_position(position, orientation)
        #     joint_values = fanuc.move_arm_to_position(position, orientation)
        #     print(joint_values)


    except rospy.ROSInterruptException:
        pass
