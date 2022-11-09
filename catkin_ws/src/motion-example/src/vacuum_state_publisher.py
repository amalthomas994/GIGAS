#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Bool

def talker():
    pub = rospy.Publisher('toggle_vacuum', Bool, queue_size=10)

    rospy.init_node('set_vacuum', anonymous=True)

    pub.publish(False)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass