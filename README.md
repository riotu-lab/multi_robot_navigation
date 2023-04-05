# multi_robot_navigation
ROS 2 package for multi-turtlebot navigation in Gazebo
This package is tested on ros2 (galactic)

## I.	Make sure that you have the following packages:

**•	gazebo_ros**
          sudo apt install ros-galatic-gazebo*

**•	Nav2** 

     	sudo apt install ros-galactic-navigation2
     	sudo apt install ros-galactic-nav2-bringup
          
**• Turtlebot3 packages**

     	sudo apt install ros-galactic-dynamixel-sdk
     	sudo apt install ros-galactic-turtlebot3-msgs
     	sudo apt install ros-galactic-turtlebot3-*
     	sudo apt install ros-galactic-turtlebot3-navigation2
     
## II.	Python libraries

**•	Jinja2** 
  
     	sudo apt install python3-pip
     	pip install Jinja2

## III.	Changing the number of robots

 This is done through the [robot_specs.yaml](https://github.com/riotu-lab/multi_robot_navigation/blob/main/config/robot_specs.yaml), located in the config file of the package.

## IV.	Running the code 

You can run the code through:

 **i.	ros2 launch multi_robot_navigation main_launch_multi_robot_navigation.launch.py**
 
 **ii.	Sending a goal** 

   1.On the tools properties, change the namespace of the /goal_pose topic. Note, the robots will be named following the format /robot_0 , /robot_1, etc.
   
<img width="333" alt="image" src="https://user-images.githubusercontent.com/63425641/229781042-cf347de9-2253-4d8a-be63-6e3b5a92090b.png">

   2.After adding the namespace, define the goal pose through the upper bar menu.
     

 <img width="103" alt="image" src="https://user-images.githubusercontent.com/63425641/229780985-6a8cbc78-ba0d-4126-b128-02ecc0ebb81c.png">
