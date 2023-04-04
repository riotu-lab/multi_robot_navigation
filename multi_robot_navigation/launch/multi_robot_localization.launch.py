#!/usr/bin/env python3
from jinja2 import Template
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def generate_launch_description():
   
    #Map path 
    map_file = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml')
    
    #Args
    FilesList=[] 
    NodeList=['map_server']
    
    #Number of spawned turtlebot3 
    specs=os.path.join(
        get_package_share_directory('multi_robot_navigation'),
        'config',
        'robot_specs.yaml'
       )
    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)
        
    X=robot_specs['number_of_robots']
    
    #Opening the amcl template 
    amcl_config=os.path.join(
        get_package_share_directory('multi_robot_navigation'),
        'config','amcl_template.yaml')
    with open(amcl_config, 'r') as infp:
        amcl_config = infp.read()
        
    #Creating amcl configuration file for each robot
    for i in range(int(X)):
        
        data = {
                "namespace": "robot_"+str(i),
                "x":-i+1.6,
               }
        
        j2_template = Template(amcl_config)
        
        fileName='amcl_robot_'+str(i)+'.yaml'
        with open(fileName, "w") as f:
            f.write(j2_template.render(data))
        dst=os.path.join(get_package_share_directory('multi_robot_navigation'),'config')

        os.rename(os.getcwd()+'/'+fileName, dst+'/'+fileName)
        # Don't recreate files
        FilesList.append(fileName)    
    
    # Launch nodes 
    ld = LaunchDescription()
    
    #Map server node 
    map_server_node=Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'topic_name':"map"},
                        {'frame_id':"map"},
                        {'yaml_filename':map_file}]
        )
        
    ld.add_action(map_server_node)
    for i in range(X): 
      
      namespace='robot_'+str(i) 
      amcl_node=Node(
            namespace=namespace,
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('multi_robot_navigation'), 'config', FilesList[i])]
        )
      ld.add_action(amcl_node)  
      NodeList.append(namespace+'/amcl') 
    

    lifecycle_manager_localization=Node(
                       package='nav2_lifecycle_manager',
                       executable='lifecycle_manager',
                       name='lifecycle_manager_localization',
                       output='screen',
                       parameters=[{'use_sim_time': True},
                                   {'autostart': True},
                                   {'bond_timeout':0.0},
                                   {'node_names': NodeList}]
                      )
    ld.add_action(lifecycle_manager_localization)


    return ld
