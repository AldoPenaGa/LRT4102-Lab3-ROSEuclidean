#!/usr/bin/env python

# Turtlesim: rosrun turtlesim turtlesim_node
# Reset: rosservice call reset

# Libraries
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# Global variables

max_distance_x = 11.
max_distance_y = 11.

desired_x = 0.
desired_y = 0.
desired_theta = 0.

current_x = 0.
current_y = 0.
current_theta = 0.
    
# Calculation of distance to go (DTG):

def dtg_operation(current_x, current_y):
    # Uses pythagoras for calculating the distance
    distance_tg = math.sqrt(((desired_x - current_x) ** 2) + ((desired_y - current_y) ** 2))

    return distance_tg

# Calculation of angle to go (ATG):

def atg_operation(current_x, current_y):

    global current_theta     # This is the global variable current_theta

    # Difference between the desired and the current x and y
    delta_x = desired_x - current_x
    delta_y = desired_y - current_y

    # Calculating the angle to go using atan2
    angle_rad = math.atan2(delta_y, delta_x)

    # Converting the angle to degrees so it is easier to read.
    angle_deg = math.degrees(angle_rad)

    # Ajust the angle depending using 180 deg as a threshold so 
    # decides for a easier angle

    delta_theta = angle_deg - current_theta
    if delta_theta > 180:
        delta_theta -= 360

    elif delta_theta < -180:
        delta_theta += 360

    return delta_theta

# Everytime a pose is obtained, then update the current variables.

def pose_callback(pose_msg):
    global current_x, current_y, current_theta
    current_x = pose_msg.x
    current_y = pose_msg.y
    current_theta = pose_msg.theta

# MAIN

if __name__ == "__main__":

    # Init node dtg_atg
    rospy.init_node('dtg_atg')
    
    # Declare a subscriber to '/turtle1/pose' and call pose_callback function everytime a message shows up
    pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

    try:
        
        # Ask the user for the three desired values
        input_str = input("Set the desired position to go separated by commas (x,y): ")
        # desired_pose_values is an array of the elements the user introduced,
        # map is used to float the integers and input_str.split is separating
        # the elements everytime a comma is found
        desired_pose_values = list(map(float, input_str.split(',')))
        desired_x = min(desired_pose_values[0], max_distance_x)  # Limit x to max_distance_x
        desired_y = min(desired_pose_values[1], max_distance_y)  # Limit y to max_distance_y

        # Wait for first pose
        rospy.wait_for_message('/turtle1/pose', Pose)
        
        # Obtain first pose
        pose = rospy.wait_for_message('/turtle1/pose', Pose)
        
        # Print first pose information
        rospy.loginfo("\nOriginal pose: %s\n", pose)
        
        # Print the desired and current poses
        print('\nThe desired position set was: ')
        print(desired_x, desired_y, sep=', ')
        print('\nAnd the current pose (x,y, theta) is: ')
        print(pose.x, pose.y, pose.theta, sep=', ')

        # Perform and print the DTG and ATG operation using the pose just obtained
        distance_tg = dtg_operation(pose.x, pose.y)

        print("\n Then, the distance to go is: ",distance_tg)

        angle_tg = atg_operation(pose.x, pose.y)

        print("\n Then, the angle to go is: ", angle_tg)
        

    except rospy.ROSInterruptException:
        pass
