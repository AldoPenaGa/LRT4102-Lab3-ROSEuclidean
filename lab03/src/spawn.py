#!/usr/bin/env python

# Turtlesim: rosrun turtlesim turtlesim_node
# Reset: rosservice call reset

# Libraries
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill  # Importa los servicios de Spawn y Kill

# Global variables
max_distance_x = 11.
max_distance_y = 11.

current_x = 0.
current_y = 0.
current_theta = 0.
    
# Calculation of distance to go (DTG):
def dtg_operation(current_x, current_y, desired_x, desired_y):
    distance_tg = math.sqrt(((desired_x - current_x) ** 2) + ((desired_y - current_y) ** 2))
    return distance_tg

# Calculation of angle to go (ATG):
def atg_operation(current_x, current_y, desired_x, desired_y):
    delta_x = desired_x - current_x
    delta_y = desired_y - current_y
    angle_rad = math.atan2(delta_y, delta_x)
    angle_deg = math.degrees(angle_rad)
    return angle_deg

# MAIN
if __name__ == "__main__":
    # Init node spawn
    rospy.init_node('spawn')
    
    # Declare a subscriber to '/turtle1/pose' and call pose_callback function everytime a message shows up
    pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose)

    try:
        while not rospy.is_shutdown():
            # Ask the user for the desired position and orientation
            input_str = input("Set the desired position and orientation separated by commas (x,y,theta): ")
            desired_pose_values = list(map(float, input_str.split(',')))
            desired_x = min(desired_pose_values[0], max_distance_x)
            desired_y = min(desired_pose_values[1], max_distance_y)
            desired_theta = math.radians(desired_pose_values[2])

            # Kill current turtle so we have only one turtle displayed
            rospy.wait_for_service('/kill')
            kill_turtle = rospy.ServiceProxy('/kill', Kill)
            kill_turtle("turtle1")  # Kill current turtle

            # Wait for the current turtle to be killed
            rospy.sleep(1)

            # Spawn a new turtle at the desired x, y and theta
            rospy.wait_for_service('spawn')
            spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
            spawn_turtle(desired_x, desired_y, desired_theta, "turtle1") 

    except rospy.ROSInterruptException:
        pass
