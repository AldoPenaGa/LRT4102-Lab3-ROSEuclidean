<p align="center">
  <h2 align="center">Understanging Robot Operating System (ROS) and euclidean distances in robotics</h2>

  <p align="justify">
  This is the third laboratory report for the course titled Robotic Systems Design (LRT4102). This report will settle a different approach using Euclidean techniques for moving the turtlesim.
	  
  <br>Universidad de las Américas Puebla (UDLAP) - Guided by professor Dr. César Martínez Torres. "https://www.linkedin.com/in/c%C3%A9sar-martinez-torres-617b5347/?originalSubdomain=mx>" 
  </p>
</p>
<be>

## Table of contents
- [Introduction](#introduction)
- [Problems](#problems)
- [Codes](#codes)
- [Conclusion](#conclusion)
- [Contributors](#codes)

<div align= "justify">

### Introduction

In this report, we explore Euclidean distances and deepen the Robot Operating System (ROS) environment. The goal of this practice is to clarify the foundations of ROS, in particular, keep practicing in basic structures like publishers and subscribers and exploring new ways of communication: services. Additionally, we'll go over how to compute Euclidean distances using algorithms, with an emphasis on 'Turtlesim'.

Optimizing the development process for robotic applications is the main goal of ROS. Among other things, ROS does hardware management, control algorithm execution, and inter-component communication. 'Turtlesim' is one of its main tools, mostly because it helps in the learning path for many robotic navigation and manipulation applications. It is a simulation software application accessible as an extension that features a turtle-shaped robot. This robot can be controlled via Python programs as well as a control interface, which allows particular locations to be set to form desired shapes and design paths to areas of interest.

A way of managing the difficulties of moving through its environment is to know where exactly is located and how it has changed its position over time. This is where solving using Euclidean distances shines. Euclidean distances are calculations of the distance between two points in Euclidean space. Practically, it is the length of the straight line that connects points "A" and "B" on a plane. In a two-dimensional Euclidean space, such as the X Y plane, the Euclidean distance(C) between two points: P(x1,y1) and P(x2,y2) can be calculated as c = √(a² + b²). 


### Problems
There were four tasks to be solved:

1- Calculate and display on the screen DTG (distance to go) and ATG (angle to go).

2- Spawn the robot, using services, at the goal position. (not moving, just spawning).

3- Explain the velocities mapping.

4- Use a free control system (P, I, D, or a combination) to move the turtle to the desired position in infinite loop.


### Codes

This time, the Python scripts are all included in the ROS package called lab03. It is important to highlight the max distance considered is 11 in x and 11 in y.

**dtg_atg**-

The code dtg_atg asks for a desired position within the turtlesim workspace, the values are saved in a vector which will serve to calculate the distance to go (dtg). This is calculated by using the Pythagorean theorem (d = sqrt((x2-x1)² + (y2-y1)²)) between the actual pose, obtained by reading the topic `/turtle1/pose` and the desired location introduced by the user. The angle to go (atg) is calculated using atan2(y2-y1, x2-1) doing the operation in radians and displaying the result in degrees. Both values are printed once the program has finished its execution.

**spawn**

The spawn script has a similar input than the dtg_atg, in which the values are asked and saved in a vector. Then the `rospy.ServiceProxy` called `/kill` is called in order to erase the current turtle. Once this has done, the service `spawn` is also called so it spawns the turtle in the desired x, y and theta.

*Important to import the services `Spawn` and `Kill` from `turtle.srv` or the program won't work as intended.

**velocities**

The velocities program will print continuously the linear and angular velocities based on the error and the controller adjustments. These velocities are obtained with the product of those characteristics.

This script just demonstrates the relation between these structures and how they affect the movement of the turtlesim, in future releases, the movement in y axis and a plotting graphic tool will be added.

**dtg_atg_withcontroller**

This script aims not only to calculate the distance to go and the angle to go but to move the robot to the desired pose introduced by the user, first does the same than the dtg_atg script and then, by using a P controller (iterating the proportional control constant for angular velocity to the error produced by the actual pose and the desired pose), rotates to achieve the position asked (this is done by setting the linear velocity to zero meanwhile it reaches a threshold for the rotation) and then moves forward to that point (comparing the actual pose to the desired pose and utilizing the proportional control constant for linear velocity). `Kp_linear` and `Kp_angular` can be adjusted to meet the expactations depending on the purpose, nevertheless, the default values are: 0.5 and 1.1 respectively. Finally, the current pose, the desired pose and the error are displayed continuously. Once it has achieved the desired position, it asks again for a new pose to reach.

### Conclusion


### Contributors

| Name                          | Github                               |
|-------------------------------|--------------------------------------|
| Aldo Oziel Peña Gamboa        | https://github.com/AldoPenaGa        |
| Joan Carlos Monfil Huitle     | https://github.com/JoanCarlosMonfilHuitle  |

