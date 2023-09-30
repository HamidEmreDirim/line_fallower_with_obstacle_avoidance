#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Vector3


# Initialize ROS node
rospy.init_node("line_follower", anonymous=True)

# Create a publisher for the robot's velocity commands
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

# Create a rate object to control the loop rate
rate = rospy.Rate(5)

# Create a CvBridge object to convert ROS messages to OpenCV format
bridge = CvBridge()



obstacle_detected = False
line_detected = False



# Checking obstacles
def checkcase(range):
    global obstacle_detected
    global line_detected
    msg = Twist()
    linearx = 0
    angularz = 0
    case = ""
    if ( range["right"] >1  and range["center"] > 1 and range["left"] >1):
        case = 'NO OBSTACLE!'
        if obstacle_detected:
            angularz=-0.5
            angularx=0.2
            msg.linear.x = linearx
            msg.angular.z = angularz
            pub.publish(msg)

        obstacle_detected = False
        
    elif ( range["right"] > 1  and range["center"] < 1 and range["left"] > 1 ):
        case = 'OBSTACLE CENTER!'
        obstacle_detected = True
        linearx=0
        angularz=0.5
    elif ( range["right"] < 1  and range["center"] > 1 and range["left"] > 1 ):
        case = 'OBSTACLE RIGHT!'
        
        linearx=0
        angularz=0.5
    elif ( range["right"] > 1  and range["center"] > 1 and range["left"] < 1 ):
        case = 'OBSTACLE LEFT!'
        
        linearx=0.5
        angularz=0
    elif ( range["right"] < 1  and range["center"] > 1 and range["left"] < 1 ):
        case = 'OBSTACLE RIGHT AND LEFT!'
        
        linearx=0.6
        angularz=0
    elif ( range["right"] > 1  and range["center"] < 1 and range["left"] < 1 ):
        case = 'OBSTACLE CENTER AND LEFT!'
        
        linearx=0
        angularz=0.3
    elif ( range["right"] < 1  and range["center"] < 1 and range["left"] > 1 ):
        case = 'OBSTACLE CENTER AND RIGHT!'
        obstacle_detected = True
        linearx=0
        angularz=0.3
    elif ( range["right"] < 1  and range["center"] < 1 and range["left"] < 1 ):
        case = 'OBSTACLE AHEAD!'
        obstacle_detected = True
    
        linearx=0
        angularz=0.8

    print(case)
    if obstacle_detected:
        msg.linear.x = linearx
        msg.angular.z = angularz
        pub.publish(msg)  







def laser_callback(message):
    rospy.loginfo(len(message.ranges))
    range={
        "right" : min(min(message.ranges[300:350]) , 2),
        "center" : min(min(message.ranges[0:60]), 2),
        "left" : min(min(message.ranges[70: 120]) , 2)
    }

    checkcase(range)






# Function to process the camera image and control the robot
def follow_line(image):
    global obstacle_detected
    try:
        # Convert the ROS Image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(image, "bgr8")

        cv2.imshow("Camera Image", cv_image)
        cv2.waitKey(1)

        # Define the region of interest (ROI) where you expect to find the red line
        # Adjust the values according to your specific setup
        lower_red = np.array([0, 0, 100])
        upper_red = np.array([100, 100, 255])

        # Create a mask to extract only red pixels
        mask = cv2.inRange(cv_image, lower_red, upper_red)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        
        # Determine a limit to stop
        allowed_wait = 0
        
        # Check if any contours (red regions) were found

        if not obstacle_detected:
            if allowed_wait > 60:
                    # Stopping robot and going to waiting mod
                    cmd_vel = Twist(linear=Vector3(x=0, y=0, z=0), angular=Vector3(x=0, y=0, z=-0))
                    pub.publish(cmd_vel)
            else:

                if contours:
                    allowed_wait = 0
                    print("Redline detected!")
                    line_detected = True
                    # Get the largest contour (assuming it's the red line)
                    largest_contour = max(contours, key=cv2.contourArea)

                    # Calculate the centroid of the largest contour
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        # Check the position of the centroid and control the robot accordingly
                        if cx < cv_image.shape[1] // 2:
                            # Turn left
                            cmd_vel = Twist(linear=Vector3(x=0.3, y=0, z=0), angular=Vector3(x=0, y=0, z=0.5))
                        else:
                            # Turn right
                            cmd_vel = Twist(linear=Vector3(x=0.3, y=0, z=0), angular=Vector3(x=0, y=0, z=-0.5))
                        
                        pub.publish(cmd_vel)
                else:
                    allowed_wait += 1
                    print("No redline detected!")
                    line_detected = False
            
                


    except CvBridgeError as e:
        rospy.logerr(e)

# Subscribe to the camera image and laser topic
image_topic = "/camera/rgb/image_raw"
laser_topic = "/scan"
rospy.Subscriber(laser_topic, LaserScan, callback=laser_callback)
rospy.Subscriber(image_topic, Image, follow_line)


# Main loop
while not rospy.is_shutdown():
    rate.sleep()
