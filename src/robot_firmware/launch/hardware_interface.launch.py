from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os 

def generate_launch_description():
    robot_description_dir = get_package_share_directory("robot_description")
    bumperbot_controller_dir = get_package_share_directory("bumperbot_controller")
    
    robot_description = ParameterValue(Command([
        "xacro ",
        os.path.join(robot_description_dir,"urdf","bumperbot.urdf.xacro"),
        " is_sim:=False",
        ]),
        value_type=str
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":robot_description}]
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
             os.path.join(
                 bumperbot_controller_dir,
                 "config",
                 "bumperbot_controllers.yaml"
             )
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node
    ])