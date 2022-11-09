#!/usr/bin/env python

import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2


class Realsense():
    def __init__(self):
        # rospy.init_node('realsense_subscriber', anonymous=True)

        self.colour_msg = None
        self.points_msg = None

        rospy.Subscriber('/camera/color/image_raw', Image, self.colour_callback)
        rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.points_callback)    

    def colour_callback(self, data):
        self.colour_msg = data

    def points_callback(self, data):
        self.points_msg = data

    def get_colour_image(self):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(self.colour_msg, desired_encoding="passthrough")    

        return cv_image

    def get_point_cloud(self):
        cloud_points = list(pc2.read_points(self.points_msg, skip_nans=True, field_names = ("x", "y", "z")))

        cloud = np.array(cloud_points)

        return cloud

if __name__ == '__main__':
    camera = Realsense()
    rospy.spin()

