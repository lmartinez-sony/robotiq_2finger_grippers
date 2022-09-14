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
    gripper_id_parameter_name = 'robotiq'
    robot_num_parameter_name = '1'
    joint_names_parameter_name = 'joint_names'
    comport = LaunchConfiguration(comport_parameter_name)
    baud = LaunchConfiguration(baud_parameter_name)
    stroke = LaunchConfiguration(stroke_parameter_name)
    sim = LaunchConfiguration(sim_parameter_name)
    gripper_id = LaunchConfiguration(gripper_id_parameter_name)
    gripper_num = LaunchConfiguration(gripper_num_parameter_name)
    rate = LaunchConfiguration(rate_parameter_name)
    joint_names = LaunchConfiguration(joint_names_parameter_name)

    default_joint_name_postfix = '_finger_joint'
    gripper_default_argument = ['[', gripper_id, default_joint_name_postfix, '1', ',', gripper_id,
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
            gripper_id_parameter_name,
            default_value='robotiq',
            description='Robot name.'),
        DeclareLaunchArgument(
            gripper_num_parameter_name,
            default_value='1',
            description='Gripper number.'),
        DeclareLaunchArgument(
            rate_parameter_name,
            default_value='50',
            description='Rate at which to publish joint states'),
        DeclareLaunchArgument(
            joint_names_parameter_name,
            default_value=gripper_default_argument,
            description='Names of the gripper joints in the URDF'),
        Node(
            package='robotiq_2f_gripper_control',
            executable='robotiq_2f_action_server_node',
            name=[gripper_id, '_gripper_', gripper_num],
            parameters=[{'comport': comport, 'baud': baud, 'stroke': stroke, 'sim':sim, 'rate':rate, 'joint_names': joint_names}],
        ),
        Node(
            package='robotiq_2f_gripper_control',
            executable='get_current_gripper_state_server',
            name=['get_current_gripper_state_server_node_', gripper_num],
            parameters=[{'gripper_state_topic_name': ["/",gripper_id,"_gripper_",gripper_num,"/joint_states"]}],
        ),
    ])
