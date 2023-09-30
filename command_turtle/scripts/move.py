#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3

def talker():
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)


    rospy.init_node("robot_mover_test", anonymous=True)

    # Create a publisher for the robot's velocity commands
    rate = rospy.Rate(5)
    
    while not rospy.is_shutdown():
        pub.publish(Twist(linear=Vector3(x=0, y=0, z=0), angular=Vector3(x=0, y=0, z=-0.5)))
        rate.sleep()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass 