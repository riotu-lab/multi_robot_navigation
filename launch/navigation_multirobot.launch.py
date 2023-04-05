#!/usr/bin/env python3
from jinja2 import Template
import os
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml

def generate_launch_description():

    ld = LaunchDescription()
    
    #Args
    controllerFiles=[] 
    plannerFiles=[]
    btNavigatorFiles=[]
    recoveryFiles=[]   
    NodeList=[]
        
    # Number of robots 
    specs=os.path.join(
        get_package_share_directory('multi_robot_navigation'),
        'config',
        'robot_specs.yaml'
       )
    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)
        
    X=robot_specs['number_of_robots']
     
     #Templates Path
    controller_config=os.path.join(get_package_share_directory('multi_robot_navigation'),'config','controller_template.yaml')
    planner_config=os.path.join(get_package_share_directory('multi_robot_navigation'),'config','planner_template.yaml')    
    bt_navigator_config=os.path.join(get_package_share_directory('multi_robot_navigation'),'config','bt_navigator_template.yaml')                
    recovery_config=os.path.join(get_package_share_directory('multi_robot_navigation'),'config','recovery_template.yaml')

    #Opening templates 
    
    with open(controller_config, 'r') as infp:
        controller_config = infp.read()
        
    with open(planner_config, 'r') as infp:
        planner_config = infp.read()
            
    with open(bt_navigator_config, 'r') as infp:
        bt_navigator_config = infp.read()
            
    with open(recovery_config, 'r') as infp:
        recovery_config = infp.read() 


    #Function to generate config file for each robot 
    def Generatefiles(fileName,j2_template,FilesList,data):
        
        with open(fileName, "w") as f:
            f.write(j2_template.render(data))
   
        dst=os.path.join(get_package_share_directory('multi_robot_navigation'),'config')

        os.rename(os.getcwd()+'/'+fileName, dst+'/'+fileName)
        FilesList.append(fileName)  
        
       
        
    for i in range(X):
        data = {
                "namespace": "robot_"+str(i),
                "source":str(get_package_share_directory('multi_robot_navigation')),
               }
               
        #Templates       
        j2_controller_template = Template(controller_config)
        j2_planner_template = Template(planner_config)
        j2_bt_navigator_template = Template(bt_navigator_config)
        j2_recovery_template = Template(recovery_config)
    
     
        #New Files Naming
        controller_fileName='controller_robot_'+str(i)+'.yaml'
        planner_fileName='planner_robot_'+str(i)+'.yaml'
        bt_navigator_fileName='bt_navigator_robot_'+str(i)+'.yaml'
        recovery_fileName='recovery_robot_'+str(i)+'.yaml'

       
        #Sending to the function
        Generatefiles(controller_fileName,j2_controller_template,controllerFiles,data)
        Generatefiles(planner_fileName,j2_planner_template,plannerFiles,data)
        Generatefiles(bt_navigator_fileName,j2_bt_navigator_template,btNavigatorFiles,data)
        Generatefiles(recovery_fileName,j2_recovery_template,recoveryFiles,data)

    #Launch nodes            
    for i in range(int(X)): 
      
      namespace='robot_'+str(i) 
 
      controller_node=Node(
            namespace=namespace,
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('multi_robot_navigation'), 'config', controllerFiles[i])])

      planner_node=Node(
            namespace=namespace,
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('multi_robot_navigation'), 'config', plannerFiles[i])])
            
      recoveries_node=Node(
            namespace=namespace,
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            parameters=[os.path.join(get_package_share_directory('multi_robot_navigation'), 'config',recoveryFiles[i])],
            output='screen')

      bt_navigator_node=Node(
            namespace=namespace,
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('multi_robot_navigation'), 'config',btNavigatorFiles[i])])

      NodeList.append(namespace+'/controller_server')  
      NodeList.append(namespace+'/planner_server')  
      NodeList.append(namespace+'/recoveries_server')  
      NodeList.append(namespace+'/bt_navigator')  
            
      
      lifecycle_manger_node=Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner_'+str(i),
            output='screen',
            parameters=[{'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': NodeList}])
                        
      ld.add_action(controller_node)
      ld.add_action(planner_node)
      ld.add_action(recoveries_node)
      ld.add_action(bt_navigator_node)
      ld.add_action(lifecycle_manger_node)
      NodeList=[]

    
    return ld    
