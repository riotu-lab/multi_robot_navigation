#!/usr/bin/env python3
from jinja2 import Template
import yaml
from ament_index_python.packages import get_package_share_directory
import os 
from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    #Number of spawned turtlebot3 
    specs=os.path.join(
        get_package_share_directory('multi_robot_navigation'),
        'config',
        'robot_specs.yaml'
       )
    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)
        
    X=robot_specs['number_of_robots']
    
    
    #####
    path_main=''
    laser_main=''
    frames_main=''
    robotModel_main=''
    CostMap_main=''
    #Content
    frames='''
        {{ namespace }}/base_footprint:
          Value: true
        {{ namespace }}/base_link:
          Value: true
        {{ namespace }}/base_scan:
          Value: true
        {{ namespace }}/camera_depth_frame:
          Value: true
        {{ namespace }}/camera_depth_optical_frame:
          Value: true
        {{ namespace }}/camera_link:
          Value: true
        {{ namespace }}/camera_rgb_frame:
          Value: true
        {{ namespace }}/camera_rgb_optical_frame:
          Value: true
        {{ namespace }}/caster_back_left_link:
          Value: true
        {{ namespace }}/caster_back_right_link:
          Value: true
        {{ namespace }}/imu_link:
          Value: true'''   
    # Path topic 
    path='''
    - Alpha: 1
      Buffer Length: 1
      Class: rviz_default_plugins/Path
      Color: 252; 233; 79
      Enabled: true
      Head Diameter: 0.30000001192092896
      Head Length: 0.20000000298023224
      Length: 0.30000001192092896
      Line Style: Lines
      Line Width: 0.029999999329447746
      Name: Path
      Offset:
        X: 0
        Y: 0
        Z: 0
      Pose Color: 255; 85; 255
      Pose Style: None
      Radius: 0.029999999329447746
      Shaft Diameter: 0.10000000149011612
      Shaft Length: 0.10000000149011612
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /{{ namespace }}/plan'''
    #Laser 
    laser='''
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 0
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.05000000074505806
      Style: Flat Squares
      Topic:
        Depth: 1
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /{{ namespace }}/scan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true'''    

    robotModel='''
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description File: ""
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /{{ namespace }}/robot_description
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
        base_footprint:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        base_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        base_scan:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        camera_depth_frame:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        camera_depth_optical_frame:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        camera_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        camera_rgb_frame:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        camera_rgb_optical_frame:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        caster_back_left_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        caster_back_right_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        imu_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        wheel_left_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        wheel_right_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
      Name: RobotModel
      TF Prefix: {{ namespace }}
      Update Interval: 0
      Value: true
      Visual Enabled: true'''
    CostMap='''
    - Alpha: 0.699999988079071
      Class: rviz_default_plugins/Map
      Color Scheme: costmap
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /{{ namespace }}/global_costmap/costmap
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /{{ namespace }}/global_costmap/costmap_updates
      Use Timestamp: false
      Value: true
    - Alpha: 0.699999988079071
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /{{ namespace }}/local_costmap/costmap
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /{{ namespace }}/local_costmap/costmap_updates
      Use Timestamp: false
      Value: true'''              
    # Opening the Rviz template 
    
    rviz=os.path.join(
        get_package_share_directory('multi_robot_navigation'),
        'config',
        'multirobot_config.rviz'
       )

    with open(rviz, 'r') as infp:
        template = infp.read()
        

    
    FilesList=[]  
    #Loop to edit the content of varibles( frames, path,laser)
    for i in range(X):
        #generating content
        data = {
                "namespace": "robot_"+str(i),
               }
               
        #Creating a template form the varibles 
        j2_path_template = Template(path)
        j2_laser_template = Template(laser)
        j2_frames_template = Template(frames)
        j2_robotModel_template = Template(robotModel)
        j2_CostMap_template = Template(CostMap)
        #Replacing the namespace in each varible 
        frame_name=j2_frames_template.render(data)
        path_name=j2_path_template.render(data)
        laser_name=j2_laser_template.render(data)     
        robotModel_name=j2_robotModel_template.render(data)
        CostMap_name=j2_CostMap_template.render(data)
        # Making a record for all the namespeces          
        path_main=path_main+path_name
        laser_main=laser_main+laser_name
        frames_main=frames_main+frame_name
        robotModel_main=robotModel_main+robotModel_name
        CostMap_main=CostMap_main+CostMap_name
    data = {
             "path": path_main,
             "laser": laser_main,
             "frames": frames_main,
             "robotModel":robotModel_main,
             "costMap":CostMap_main,
           } 
  
    # Adding the content of path, laser and frames to the main template
     
    fileName='rviz_robot.rviz'
    j2_template=Template(template)
    with open(fileName, "w") as f:
         f.write(j2_template.render(data))
         
    dst=os.path.join(get_package_share_directory('multi_robot_navigation'),'config')
    os.rename(os.getcwd()+'/'+fileName, dst+'/'+fileName)
    
    rviz_config_file=os.path.join(get_package_share_directory('multi_robot_navigation'), 'config', 'rviz_robot.rviz')
    
    #Launch rviz
    
    return LaunchDescription([
    Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        ),
     ])   
