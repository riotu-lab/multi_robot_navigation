# multi_robot_navigation
ROS 2 package for multi-turtlebot navigation in Gazebo
This package is tested on ros2(galactic)

I.	Make sure that you have the following packages:
•	Gazebo_ros
•	Nav2
o	Sudo apt install ros-galactic-navigation2
o	Sudo apt install ros-galactic-nav2-bringup
•	Turtlebot3 packages
o	Sudo apt install ros-galactic-dynamixel-sdk
o	Sudo apt install ros-galactic-turtlebot3-msgs
o	Sudo apt install ros-galactic-turtlebot3-*
o	sudo apt install ros-galactic-turtlebot3-navigation2
II.	Python libraries
•	Jinja2 
o	sudo apt install python3-pip
o	pip install Jinja2
III.	Changing the number of robots 
This is done through the robot_specs.yaml file, located in the config file of the package.
IV.	Running the code 
You can run the code through:
i.	ros2 launch multi_robot_navigation main_launch_multi_robot_navigation.launch.py
ii.	Sending a goal 
1.	On the tools properties, change the namespace of the /goal_pose topic. Note, the robots will be named following the format /robot_0 , /robot_1, etc.
 
2.	After adding the namespace, define the goal pose through the upper bar menu.

 
![image](https://user-images.githubusercontent.com/63425641/229776761-4be4790b-7423-48a8-96cd-24ac702de977.png)
