#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import atan2, sqrt, radians, sin, cos
from std_srvs.srv import Empty

# Global variables for current and desired pose
current_pose = Pose()
desired_pose = Pose()
error_x = 0.
error_y = 0.
error_theta = 0.
distance_error = 0.

# Define maximum distances for x and y
max_distance_x = 11.0
max_distance_y = 11.0

# Threshold distance for reaching the desired position
threshold_distance = 0.1

# Callback function to handle turtle's pose updates
def pose_callback(pose):
    global current_pose
    current_pose = pose

# Function to control the turtle's movement
def control_turtle():
    global current_pose, desired_pose

    # Calculate errors in x, y coordinates and orientation angle
    global error_x 
    error_x = desired_pose.x - current_pose.x
    global error_y 
    error_y = desired_pose.y - current_pose.y

    # Calculate the Euclidean distance to the desired position
    distance_to_goal = sqrt(error_x ** 2 + error_y ** 2)

    # Calculate the desired orientation angle to the goal
    desired_theta = atan2(error_y, error_x)

    # Calculate the angle difference between the desired and current orientation
    # Ensure the angle is in the range [-pi, pi]
    global error_theta
    error_theta = atan2(sin(desired_theta - current_pose.theta), cos(desired_theta - current_pose.theta))

    # Proportional controller coefficients for linear and angular velocity
    Kp_linear = 0.5  # Adjust this value to control linear velocity
    Kp_angular = 1.1 # Adjust this value to control angular velocity

    # If the turtle hasn't rotated to face the desired orientation yet,
    # only rotate until it does
    if abs(error_theta) > 0.05:  # A small threshold for orientation error
        vel_linear = 0.0
        vel_angular = Kp_angular * error_theta
    else:
        # Once facing the desired orientation, use P controller for linear velocity
        vel_linear = Kp_linear * distance_to_goal
        vel_angular = Kp_angular * error_theta

    return vel_linear, vel_angular

# Main function
if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('dtg_atg_withcontroller')

    # Subscribe the callback function to the topic publishing the turtle's pose
    pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

    # Publish to the topic for controlling the turtle's velocity
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    # Message publishing rate (10 Hz)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Prompt the user to set the desired position as a vector separated by commas
        input_str = input("Set the desired position separated by commas (x,y): ")
        # desired_pose_values is an array of the elements the user introduced,
        # map is used to float the integers and input_str.split is separating
        # the elements every time a comma is found
        desired_pose_values = list(map(float, input_str.split(',')))
        desired_x = min(desired_pose_values[0], max_distance_x)  # Limit x to max_distance_x
        desired_y = min(desired_pose_values[1], max_distance_y)  # Limit y to max_distance_y
        desired_pose.x = desired_x
        desired_pose.y = desired_y


        # Control loop until the desired position is reached
        while not rospy.is_shutdown():
            # Calculate linear and angular velocities
            vel_linear, vel_angular = control_turtle()

            # Create a Twist message with the calculated velocities and publish it
            twist_msg = Twist()
            twist_msg.linear.x = vel_linear
            twist_msg.angular.z = vel_angular
            velocity_publisher.publish(twist_msg)

            # Print linear and angular velocities to the terminal
            rospy.loginfo("Current Position: X =%f, Y =%f, TH =%f", current_pose.x, current_pose.y, current_pose.theta)
            rospy.loginfo("Desired Position: X =%f, Y =%f, TH =%f", desired_pose.x, desired_pose.y, desired_pose.theta)
            rospy.loginfo("Error: X= %f, Y =%f, TH =%f", error_x, error_y, error_theta)

            # Check if the turtle has reached the desired position
            if sqrt((current_pose.x - desired_pose.x) ** 2 + (current_pose.y - desired_pose.y) ** 2) <= threshold_distance:
                rospy.loginfo("Position reached.")
                break  # Exit the control loop to prompt for a new position

            # Wait until the next iteration
            rate.sleep()
