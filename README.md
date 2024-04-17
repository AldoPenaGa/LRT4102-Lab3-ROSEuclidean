<p align="center">
  <h2 align="center">Euclidean approach for turtlesim using Robot Operating System (ROS)</h2>

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

This program just demonstrates the relation between these structures and how they affect the movement of the turtlesim, in future releases, the movement in y axis and a plotting graphic tool will be added.

### Conclusion

To summarize, this laboratory report has provided an extensive review of ROS, from the basics of developing a ROS package to some advanced implementation of a PID controller. The journey through the challenges and codes has not only strengthened our comprehension of ROS communication system, but has also demonstrated the integration of theory and practice. As we conclude, we consider the impact of ROS and PID control on the future of robotic system design, leaving us encouraged and equipped with the tools to innovate in the field of robotics.

### Contributors

| Name                          | Github                               |
|-------------------------------|--------------------------------------|
| Aldo Oziel Peña Gamboa        | https://github.com/AldoPenaGa        |
| Joan Carlos Monfil Huitle     | https://github.com/JoanCarlosMonfilHuitle  |

