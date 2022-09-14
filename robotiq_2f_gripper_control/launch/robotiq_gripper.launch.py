# Copyright (c) 2021 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    comport_parameter_name = 'comport'
    baud_parameter_name = 'baud'
    stroke_parameter_name = 'stroke'
    sim_parameter_name = 'sim'
    rate_parameter_name = 'rate'
    joint_names_parameter_name = 'joint_names'
    comport = LaunchConfiguration(comport_parameter_name)
    baud = LaunchConfiguration(baud_parameter_name)
    stroke = LaunchConfiguration(stroke_parameter_name)
    sim = LaunchConfiguration(sim_parameter_name)
    rate = LaunchConfiguration(rate_parameter_name)
    joint_names = LaunchConfiguration(joint_names_parameter_name)

    gripper_config = os.path.join(get_package_share_directory('franka_gripper'), 'config',
                                  'franka_gripper_node.yaml')

    default_joint_name_postfix = '_finger_joint'
    arm_default_argument = ['[', arm_id, default_joint_name_postfix, '1', ',', arm_id,
                            default_joint_name_postfix, '2', ']']

    return LaunchDescription([
        DeclareLaunchArgument(
            comport_parameter_name,
            default_value='/dev/ttyUSB0',
            description='Com Port.'),
        DeclareLaunchArgument(
            baud_parameter_name,
            default_value='115200',
            description='Default baud rate for Robotiq Gripper.'),
        DeclareLaunchArgument(
            stroke_parameter_name,
            default_value='0.085',
            description='Stroke of the Robotiq Gripper'),
        DeclareLaunchArgument(
            sim_parameter_name,
            default_value='false',
            description='Whether to use a simulated gripper or not'),
        DeclareLaunchArgument(
            rate_parameter_name,
            default_value='50',
            description='Rate at which to publish joint states'),
        DeclareLaunchArgument(
            joint_names_parameter_name,
            default_value=arm_default_argument,
            description='Names of the gripper joints in the URDF'),
        Node(
            package='franka_gripper',
            executable='franka_gripper_node',
            name=[arm_id, '_gripper_', robot_num],
            parameters=[{'robot_ip': robot_ip, 'joint_names': joint_names}, gripper_config],
            condition=UnlessCondition(use_fake_hardware)
        ),
        Node(
            package='franka_ros_interface',
            executable='get_current_gripper_state_server',
            name=['get_current_gripper_state_server_node_', robot_num],
            parameters=[{'gripper_state_topic_name': ["/franka_gripper_",robot_num,"/joint_states"]}],
            condition=UnlessCondition(use_fake_hardware)
        ),
        Node(
            package='franka_gripper',
            executable='fake_gripper_state_publisher.py',
            name=[arm_id, '_gripper'],
            parameters=[{'robot_ip': robot_ip, 'joint_names': joint_names}, gripper_config],
            condition=IfCondition(use_fake_hardware)
        ),
    ])
