#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import atan2, radians, sin, cos
from std_srvs.srv import Empty

# Callback function to handle turtle's pose updates
def pose_callback(pose):
    global current_pose
    current_pose = pose

# Function to calculate linear and angular velocities based on pose error
def calculate_velocities(desired_pose):
    global current_pose

    # Calculate errors in x, y coordinates and orientation angle
    error_x = desired_pose.x - current_pose.x
    error_y = desired_pose.y - current_pose.y

    # Calculate the angle difference between the desired and current orientation
    # Ensure the angle is in the range [-pi, pi]
    error_theta = atan2(sin(desired_pose.theta - current_pose.theta), cos(desired_pose.theta - current_pose.theta))

    # Calculate linear and angular velocities proportional to errors and proportional controller coefficients
    vel_linear = Kp_linear * error_x
    vel_angular = Kp_angular * error_theta

    return vel_linear, vel_angular


# Function to call the 'reset' service
def call_reset_service():
    # Wait for the 'reset' service to become available
    rospy.wait_for_service('/reset')
    try:
        # Create a proxy to call the service and call it
        rospy.ServiceProxy('/reset', Empty)()
        rospy.loginfo("Turtle reset successful")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to reset turtle: %s", e)

# Main function
if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('velocities')

    # Subscribe the callback function to the topic publishing the turtle's pose
    pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

    # Publish to the topic for controlling the turtle's velocity
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    # Message publishing rate (10 Hz)
    rate = rospy.Rate(10)

    # Proportional controller coefficients for linear and angular velocity
    Kp_linear = 4.0  # Adjust this value to control linear velocity
    Kp_angular = 1.1 # Adjust this value to control angular velocity

    # Initialize the current and desired turtle pose
    current_pose = Pose()
    desired_pose = Pose()
    desired_pose.x = 3.0  # Desired position in x
    desired_pose.y = 3.0  # Desired position in y
    desired_pose.theta = radians(270.0)  # Desired orientation in radians (converted from degrees)

    # Initialize the time of the last reset service call
    last_reset_time = rospy.get_time()

    # Main loop running until the program is interrupted
    while not rospy.is_shutdown():
        # Check if it's time to call the 'reset' service
        current_time = rospy.get_time()
        if current_time - last_reset_time >= 15.0:
            call_reset_service()
            last_reset_time = current_time

        # Calculate linear and angular velocities
        vel_linear, vel_angular = calculate_velocities(desired_pose)

        # Create a Twist message with the calculated velocities and publish it
        twist_msg = Twist()
        twist_msg.linear.x = vel_linear
        twist_msg.angular.z = vel_angular
        velocity_publisher.publish(twist_msg)

        # Print linear and angular velocities to the terminal
        rospy.loginfo("Linear Velocity: %f, Angular Velocity: %f", vel_linear, vel_angular)

        # Wait until the next iteration
        rate.sleep()
