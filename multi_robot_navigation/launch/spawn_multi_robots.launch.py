#!/usr/bin/env python3
from jinja2 import Template
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import xacro
import yaml



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

    
    #Launch directory 
    pkg_share = FindPackageShare(package='multi_robot_navigation').find('multi_robot_navigation')
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    
    # Launch args
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    #World 
    world_path=os.path.join(FindPackageShare(package='turtlebot3_gazebo').find('turtlebot3_gazebo'), 'worlds', 'turtlebot3_world.world')
    
    #Setting the gazebo model path
    gazebo_models_path = os.path.join(pkg_share, 'models')
    gazebo_models_path2=os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + gazebo_models_path+ ':' + gazebo_models_path2
    else:
        os.environ['GAZEBO_MODEL_PATH'] =   gazebo_models_path+ ':' + gazebo_models_path2
    


    # Opening the model template 
    urdf=os.path.join(
        get_package_share_directory('multi_robot_navigation'),
        'models',
        'model.sdf'
       )
    with open(urdf, 'r') as infp:
        template = infp.read()
        
    #Generating a model file for each spawned robot( namely changing the frames' nampespace)
    FilesList=[]  
    for i in range(X):
        
        data = {
                "namespace": "robot_"+str(i),
               }
        
        j2_template = Template(template)
        
        fileName='model'+str(i)+'.sdf'
        with open(fileName, "w") as f:
            f.write(j2_template.render(data))
        dst=os.path.join(get_package_share_directory('multi_robot_navigation'),'models')

        os.rename(os.getcwd()+'/'+fileName, dst+'/'+fileName)
        # Don't recreate files
        FilesList.append(fileName)
        
        


    #URDF  arg
    urdf_file_name = 'turtlebot3_waffle.urdf'
    urdf_package=os.path.join(
        get_package_share_directory('turtlebot3_gazebo'))
    urdf = os.path.join(
        urdf_package,
        'urdf',
        urdf_file_name)

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    #Start Gazebo
    gazebo=ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path,
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen')


    #Creating the launch description
    ld = LaunchDescription()    
    ld.add_action(gazebo) 
         
    #Start other nodes   

    for i in range(X): 
      
      namespace='robot_'+str(i) 
      
      # start robot state publisher 
      publisher_node = Node(
          package="robot_state_publisher",
          executable="robot_state_publisher",
          name="robot_state_publisher",
          namespace=namespace,
          parameters=[{'use_sim_time': use_sim_time,
                       'frame_prefix':namespace+'/',
                       'robot_description': Command(['xacro ', urdf, ' robot_name:=', namespace])}],
          output="screen",

      )
      
      #Updating the model.sdf file name    
      fileName='model'+str(i)+'.sdf'
      model_path=os.path.join(
        get_package_share_directory('multi_robot_navigation'),
        'models',
        fileName
      )
      # Spawn robot 
      spawn_node=  Node(
          package='gazebo_ros',
          executable='spawn_entity.py',
          output='screen',
          arguments=['-entity','turtlebot3_waffle'+str(i),
                     '-file',model_path,
                     '-robot_namespace',namespace,
                     '-x', str(-i+1.6), '-y', '0.0', '-z', '0.0']
        
      )
       

      ld.add_action(publisher_node)
      ld.add_action(spawn_node) 



    return ld


