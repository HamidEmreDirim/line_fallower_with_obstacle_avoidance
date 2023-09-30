#!/usr/bin/env python

import rospy
from sensor_msgs.msg  import LaserScan






def checkcase(range):
    case = ''
    if ( range["right"] >0.3  and range["center"] > 0.3 and range["left"] >0.3):
        case = 'NO OBSTACLE!'
        linearx=0.6
        angularz=0
    elif ( range["right"] > 0.3  and range["center"] < 0.3 and range["left"] > 0.3 ):
        case = 'OBSTACLE CENTER!'
        linearx=0
        angularz=-0.5
    elif ( range["right"] < 0.3  and range["center"] > 0.3 and range["left"] > 0.3 ):
        case = 'OBSTACLE RIGHT!'
        linearx=0
        angularz=0.5
    elif ( range["right"] > 0.3  and range["center"] > 0.3 and range["left"] < 0.3 ):
        case = 'OBSTACLE LEFT!'
        linearx=0
        angularz=-0.5
    elif ( range["right"] < 0.3  and range["center"] > 0.3 and range["left"] < 0.3 ):
        case = 'OBSTACLE RIGHT AND LEFT!'
        linearx=0.6
        angularz=0
    elif ( range["right"] > 0.5  and range["center"] < 0.5 and range["left"] < 0.5 ):
        case = 'OBSTACLE CENTER!'
        linearx=0
        angularz=-0.5
    elif ( range["right"] < 0.3  and range["center"] < 0.3 and range["left"] > 0.3 ):
        case = 'OBSTACLE CENTER!'
        linearx=0
        angularz=0.5
    elif ( range["right"] < 0.3  and range["center"] < 0.3 and range["left"] < 0.3 ):
        case = 'OBSTACLE CENTER!'
        linearx=0
        angularz=0.8
    
    rospy.loginfo(case)

def callback(message):
    rospy.loginfo(len(message.ranges))
    range={
        "right" : min(min(message.ranges[300:350]) , 2),
        "center" : min(min(message.ranges[0:40]), min(message.ranges[351:360]), 2),
        "left" : min(min(message.ranges[41: 120]) , 2)
    }

    checkcase(range)

def listen_laser():
    rospy.init_node("laser_listener", anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback=callback)

    rospy.spin()

if __name__ == "__main__":
    listen_laser()




