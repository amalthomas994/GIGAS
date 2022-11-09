#!/usr/bin/env python  
import rospy
import tf
from geometry_msgs.msg import PointStamped

if __name__ == '__main__':
    rospy.init_node('robot_state_listener')

    listener = tf.TransformListener()

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/camera_tool', rospy.Time(0))
            print(trans)

            camera_point = PointStamped()
            camera_point.header.frame_id = "camera_origin"
            camera_point.header.stamp = rospy.Time(0)

            x, y, z = [ 0.0511089,  -0.01716597,  0.55600005]

            camera_point.point.x = x
            camera_point.point.y = y
            camera_point.point.z = z     
            world_point = listener.transformPoint("base_link", camera_point)

            print(world_point)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
