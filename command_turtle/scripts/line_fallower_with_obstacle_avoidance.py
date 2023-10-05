#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Vector3
from math import radians



# Initialize ROS node
rospy.init_node("line_follower", anonymous=True)

# Create a publisher for the robot's velocity commands
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

# Create a rate object to control the loop rate
rate = rospy.Rate(5)

# Create a CvBridge object to convert ROS messages to OpenCV format
bridge = CvBridge()

move_cmd = Twist()
move_cmd.linear.x = 0.2
	# by default angular.z is 0 so setting this isn't required

        #let's turn at 45 deg/s
turn_left_cmd = Twist()
turn_left_cmd.linear.x = 0
turn_left_cmd.angular.z = radians(45)


turn_right_cmd = Twist()
turn_right_cmd.linear.x = 0
turn_right_cmd.angular.z = -radians(45)




obstacle_detected = False
line_detected = False
case = ""


# Checking obstacles
def checkcase(range):
    global obstacle_detected
    global line_detected
    global case
   
    left_value = 0.5
    right_value = 0.5
    center_value = 0.5

    if ( range["right"] >0.4  and range["center"] > 0.4 and range["left"] >0.4):
        case = 'NO OBSTACLE!'
   
    elif ( range["right"] > center_value  and range["center"] < center_value and range["left"] > 0.3 ):
        case = 'OBSTACLE CENTER!'
        obstacle_detected = True
    
    elif ( range["right"] < right_value  and range["center"] > right_value and range["left"] > right_value ):
        case = 'OBSTACLE RIGHT!'
   
    elif ( range["right"] > left_value  and range["center"] > left_value and range["left"] < left_value ):
        case = 'OBSTACLE LEFT!'
    
    elif ( range["right"] < 0.3  and range["center"] > 0.3 and range["left"] < 0.3 ):
        case = 'OBSTACLE RIGHT AND LEFT!'
 
    elif ( range["right"] > center_value  and range["center"] < center_value and range["left"] < center_value ):
        case = 'OBSTACLE CENTER!'
        obstacle_detected = True

    elif ( range["right"] < center_value  and range["center"] < center_value and range["left"] > center_value):
        case = 'OBSTACLE CENTER!'
        obstacle_detected = True

    elif ( range["right"] < center_value  and range["center"] < center_value and range["left"] < center_value ):
        case = 'OBSTACLE CENTER!'

    print(case)  



def laser_callback(message):
    
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
                            cmd_vel = Twist(linear=Vector3(x=0.2, y=0, z=0), angular=Vector3(x=0, y=0, z=0.5))
                        else:
                            # Turn right
                            cmd_vel = Twist(linear=Vector3(x=0.2, y=0, z=0), angular=Vector3(x=0, y=0, z=-0.5))
                        
                        pub.publish(cmd_vel)
                else:
                    allowed_wait += 1
                    print("No redline detected!")
        else:

            rospy.loginfo("Turning")
            if case == 'OBSTACLE CENTER!':
                for _ in range(10):
                    pub.publish(turn_left_cmd)
                    rate.sleep()
                
                for _ in range(8):
                    pub.publish(move_cmd)
                    rate.sleep()

                for _ in range(10):
                    pub.publish(turn_right_cmd)
                    rate.sleep()

                for _ in range(14):
                    pub.publish(move_cmd)
                    rate.sleep()

                for _ in range(4):
                    pub.publish(turn_right_cmd)
                    rate.sleep()

                for _ in range(5):
                    pub.publish(move_cmd)
                    rate.sleep()

                # Reset the obstacle_detected flag
                obstacle_detected = False        
            
                


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