#! /usr/bin/env python
"""--------------------------------------------------------------------
COPYRIGHT 2015 Stanley Innovation Inc.

Software License Agreement:

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 file   robotiq_2f_gripper_driver.py

 brief  Driver for Robotiq 85 communication

 Platform: Linux/ROS Humble
--------------------------------------------------------------------"""

"""
This Module defines the `Robotiq2FingerGripperDriver` and `Robotiq2FingerSimulatedGripperDriver` classes, which 
allow for the control and operation (also simulation of operation) of the 2 finger adaptive grippers from robotiq.

The end user should not need to use this class direcly since an instance of it is created with every action server
controlling a given gripper, and commanded by the user commands puubished by an action client instance.  
"""

import time
from .robotiq_2f_gripper import Robotiq2FingerGripper
from robotiq_2f_gripper_msgs.msg import RobotiqGripperCommand, RobotiqGripperStatus
from robotiq_2f_gripper_msgs.action import CommandRobotiqGripper
from sensor_msgs.msg import JointState
import numpy as np
import rclpy
from rclpy.node import Node
from enum import Enum

WATCHDOG_TIME = 1.0   # Max Time without communication with gripper allowed

class Robotiq2FingerGripperDriver(Node):
    """
    This class represents an abstraction of a gripper driver, it handles the gripper connection, initialization,
    command send request, status request, and publishing of joint position to the `/joint_state` topic.   

    Args:
        comport: Name of the USB port to which the gripper is connected to.
        baud: Baudrate to use during Modbus protocol communication.
        stroke: Stroke of the gripper you are using, should be `0.085` or `0.140`.
        finger_joint: Name of the URDF joint to publish on the `/joint_state` topic. 

    Attributes:
        is_ready: Boolean indicating gripper is ready to take commands.
        _gripper: Instance of `robotiq_2f_gripper_control/Robotiq2FingerGripper` class representing the abstraction 
            of the real gripper.
        _gripper_joint_state_pub: Ros publisher for the `/joint_state` topic
        _driver_state: Internal machine state variable indicating the current gripper status
                            0: Driver not running
                            1: Driver is running
                            2: Gripper has been activated
    """
    def __init__(self, comport = '/dev/ttyUSB0', baud = '115200', stroke = 0.085, joint_names='finger_joint', rate=100):

        super().__init__('robotiq_2f_gripper_driver')
        self._comport = comport
        self._baud = baud
        self._joint_names = joint_names
        self._rate = rate

        # Instanciate and open communication with gripper.
        self._gripper = Robotiq2FingerGripper(device_id=0, stroke=stroke, comport=self._comport, baud=self._baud)
        
        self._max_joint_limit = 0.8
        if( self._gripper.stroke == 0.140 ):
            self._max_joint_limit = 0.7
        
        if not self._gripper.init_success:
            self.get_logger().error("Unable to open comport to {}".format(self._comport))
            return
        else:
            self.get_logger().info("Connection to gripper with stroke {}[m] on port {} successful".format(self._gripper.stroke, self._comport))

        self._gripper_joint_state_pub = self.create_publisher(JointState, "~/joint_states", 10)        

        self._prev_joint_pos = 0.0
        self._prev_joint_state_time = self.get_clock().now().nanoseconds / 1e9 
        self._driver_state = 0
        self.is_ready = False
        
        if not self._gripper.getStatus():
            self.get_logger().error("Failed to contact gripper on port {}... ABORTING".format(self._comport))
            return                
                
        self._run_driver()
        self._last_update_time = self.get_clock().now().nanoseconds / 1e9
        
    def _clamp_position(self,cmd):
        out_of_bounds = False
        if (cmd <= 0.0):
            out_of_bounds = True
            cmd_corrected = 0.0
        elif (cmd > self._gripper.stroke):
            out_of_bounds = True
            cmd_corrected = self._gripper.stroke
        if(out_of_bounds):
            self.get_logger().debug("Position ({%.3f}[m]) out of limits for {%d}[mm] gripper: \n- New position: {%.3f}[m]\n- Min position: {%.3f}[m]\n- Max position: {%.3f}[m]".format(cmd, self._gripper.stroke*1000, cmd_corrected, 0.0, self._gripper.stroke))
            cmd = cmd_corrected
        return cmd

    def _clamp_speed(self,cmd):
        out_of_bounds = False
        if (cmd <= 0.013):
            out_of_bounds = True
            cmd_corrected = 0.013
        elif (cmd > 0.101):
            out_of_bounds = True
            cmd_corrected = 0.1
        if(out_of_bounds):
            self.get_logger().debug("Speed ({%.3f}[m/s]) out of limits for {%d}[mm] gripper: \n- New speed: {%.3f}[m/s]\n- Min speed: {%.3f}[m/s]\n- Max speed: {%.3f}[m/s]".format(cmd, self._gripper.stroke*1000, cmd_corrected, 0.013, 0.1))
            cmd = cmd_corrected
        return cmd
    
    def _clamp_force(self,cmd):
        out_of_bounds = False
        if (cmd < 0.0):
            out_of_bounds = True
            cmd_corrected = 0.0
        elif (cmd > 100.0):
            out_of_bounds = True
            cmd_corrected = 100.0
        if(out_of_bounds):
            self.get_logger().debug("Force ({%.3f}[%]) out of limits for {%d}[mm] gripper: \n- New force: {%.3f}[%]\n- Min force: {%.3f}[%]\n- Max force: {%.3f}[%]".format(cmd, self._gripper.stroke*1000, cmd_corrected, 0, 100))
            cmd = cmd_corrected
        return cmd
    
    def update_gripper_command(self,cmd):
        """
        Updates the driver internal goal/setpoint, which will be use to command the robotiq gripper/ 

        Args:
            cmd: Instance of `robotiq_2f_gripper_msgs/RobotiqGripperCommand` message, holding the most recent 
            user provided command parameters. See the message declaration for fields description 
        """
        if (True == cmd.emergency_release):
            self._gripper.activate_emergency_release(open_gripper=cmd.emergency_release_dir)
            return
        else:
            self._gripper.deactivate_emergency_release()

        if (True == cmd.stop):
            self._gripper.stop()
            
        else:
            pos = self._clamp_position(cmd.position)
            vel = self._clamp_speed(cmd.speed)
            force = self._clamp_force(cmd.force)
            # self.get_logger().info(" {} {} {} ".format(pos, vel, force))
            self._gripper.goto(pos=pos,vel=vel,force=force)
            
    def get_current_gripper_status(self):
        """
        Public function to obtain the current gripper status.

        Returns:  Instance of `robotiq_2f_gripper_msgs/RobotiqGripperStatus` message. See the message declaration for fields description
        """
        status = RobotiqGripperStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.is_ready = self._gripper.is_ready()
        status.is_reset = self._gripper.is_reset()
        status.is_moving = self._gripper.is_moving()
        status.obj_detected = self._gripper.object_detected()
        status.fault_status = self._gripper.get_fault_status()
        status.position = self._gripper.get_pos()
        status.requested_position = self._gripper.get_req_pos()
        status.current = self._gripper.get_current()
        return status
        
    def _update_gripper_joint_state(self):
        """
        Private helper function to create a JointState message with the current gripper joint position
        """
        js = JointState()
        js.header.frame_id = ''
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self._joint_names]
        max_joint_limit = 0.8
        if( self._gripper.stroke == 0.140 ):
            max_joint_limit = 0.7
        pos = np.clip(max_joint_limit - ((max_joint_limit/self._gripper.stroke) * self._gripper.get_pos()), 0., max_joint_limit)
        js.position = [pos]
        dt = self.get_clock().now().nanoseconds / 1e9 - self._prev_joint_state_time
        self._prev_joint_state_time = self.get_clock().now().nanoseconds / 1e9
        js.velocity = [(pos-self._prev_joint_pos)/dt]
        self._prev_joint_pos = pos
        return js
        
    def _run_driver(self):
        """
        Private function to test the comunication with the gripper and initialize it.
        WARNING: 
            Initialization of the gripper will generate a close motion of the real
            gripper. 

        Raises: 
            IOError: If Modbus RTU communication with the gripper is not achieved.
        """
        last_time = self.get_clock().now().nanoseconds / 1e9
        while rclpy.utilities.ok() and self._driver_state != 2:
            # Check if communication is failing or taking too long
            dt = self.get_clock().now().nanoseconds / 1e9 - last_time
            if (0 == self._driver_state):
                if (dt < 0.5):
                    self._gripper.deactivate_gripper()
                else:
                    self._driver_state = 1
            # If driver is not running, activate gripper 
            elif (1 == self._driver_state):
                is_gripper_activated = True
                self._gripper.activate_gripper()
                is_gripper_activated &= self._gripper.is_ready()
                if (is_gripper_activated):
                    self.get_logger().info("Gripper on port {} activated".format(self._comport))
                    self._driver_state = 2
                        
            success = True
            success &= self._gripper.sendCommand()
            success &= self._gripper.getStatus()
            if not success and rclpy.utilities.ok():
                self.get_logger().error("Failed to initialize contact with gripper {}".format(self._gripper.device_id))
            else:
                stat = RobotiqGripperStatus()
                #js = JointState()
                stat = self.get_current_gripper_status()
                js = self._update_gripper_joint_state()
                if stat.is_ready:
                    self.is_ready = True 
                # self._gripper_pub.publish(stat)
                # self._gripper_joint_state_pub.publish(js)
                            
            time.sleep(1/self._rate)

    def update_driver(self):
        """
        Public function that:
            1. Sends the current driver command.
            2. Request the current gripper status.  
            3. Publish the gripper current joint position to the `/joint_state` topic.

        Raises:
            Log Error: If Modbus RTU communication with the gripper is not achieved.
        """
        # Try to update gripper command and status
        success = self._gripper.sendCommand()
        success &= self._gripper.getStatus()

        # Check if communication is broken
        update_time = self.get_clock().now().nanoseconds / 1e9
        if success:
            js = JointState()
            js = self._update_gripper_joint_state()
            self._gripper_joint_state_pub.publish(js)
            self._last_update_time = update_time
        
        # If communication failed, check if connection was truly lost
        elif (update_time - self._last_update_time) > WATCHDOG_TIME:
            self.get_logger().fatal("Failed to contact gripper on port: {}".format(self._comport))
            # self._gripper.shutdown()
            # self.get_logger().fatal("Communication to gripper lost")
            # self.shutdown()

    def from_distance_to_radians(self, linear_pose ):
      """
      Private helper function to convert a command in meters to radians (joint value)
      """
      return np.clip(self._max_joint_limit - ((self._max_joint_limit/self._gripper.stroke) * linear_pose), 0.0, self._max_joint_limit)
  
    def from_radians_to_distance(self, joint_pose):
      """
      Private helper function to convert a joint position in radians to meters (distance between fingers)
      """
      return np.clip(self._gripper.stroke - ((self._gripper.stroke/self._max_joint_limit) * joint_pose), 0.0, self._max_joint_limit)
      
    def get_current_joint_position(self):
      return self.from_distance_to_radians(self._gripper.get_pos())


class Robotiq2FingerSimulatedGripperDriver(Node):
    """
    This class represents an abstraction of a gripper driver for a SIMULATED gripper,
    by simulating the operation/response of the gripper to a given command and the publishing 
    of the simulated gripper joint position to the `/joint_state` topic.  

    Args:
        stroke: Stroke of the gripper you are using, should be `0.085` or `0.140`.
        finger_joint: Name of the URDF joint to publish on the `/joint_state` topic. 

    Attributes:
        is_ready: Boolean indicating gripper is ready to take commands.
        _gripper_joint_state_pub: Ros publisher for the `/joint_state` topic.
        _current_joint_pos: [radians] Position of the simulated joint in radians.
        _current_goal: Instance of `RobotiqGripperCommand` message holding the latest user command.
    """
    def __init__(self, stroke=0.085, joint_names='finger_joint'):
        super().__init__('robotiq_2f_simulated_gripper_driver')
        self._stroke = stroke
        self._joint_names = joint_names
        self._current_joint_pos = 0.0                                 
        self._prev_time = self.get_clock().now().nanoseconds / 1e9
        self._current_goal = CommandRobotiqGripper.Goal()
        self._current_goal.position = self._stroke
        self._gripper_joint_state_pub = self.create_publisher(JointState, "~/joint_states", 10) 
        self.is_ready = True
        self._is_moving = False
        self._max_joint_limit = 0.8
        if( self._stroke == 0.140 ):
            self._max_joint_limit = 0.7


    def update_driver(self):
        """
        Public function simulating the gripper state change given the current gripper command and the time
        difference between the last call to this function.
        """
        delta_time = self.get_clock().now().nanoseconds / 1e9 - self._prev_time
        linear_position_goal = self._current_goal.position                                                   #[mm]
        joint_goal = self.from_distance_to_radians(linear_position_goal)                                     #[rad]
        position_increase = (delta_time * self._current_goal.speed) * (self._max_joint_limit/self._stroke) 
        if( abs(joint_goal - self._current_joint_pos) > position_increase ):
            self._current_joint_pos += position_increase if (joint_goal - self._current_joint_pos) > 0 else -position_increase
            self._is_moving = True
        else:
            self._current_joint_pos = joint_goal
            self._is_moving = False
        js = self._update_gripper_joint_state()
        self._gripper_joint_state_pub.publish(js)
        self._prev_time = self.get_clock().now().nanoseconds / 1e9


    def update_gripper_command(self, goal_command):
        """
        Updates the driver internal goal/setpoint, which will be use to command the robotiq gripper/ 

        Args:
            goal_command: Instance of `robotiq_2f_gripper_msgs/RobotiqGripperCommand` message, holding the most recent 
            user provided command parameters. See the message declaration for fields description 
        """
        self._current_goal = goal_command
        self._current_goal.position = self._clamp_position(goal_command.position)
        self._current_goal.speed = self._clamp_speed(goal_command.speed)

    def get_current_gripper_status(self):
        """
        Public function to obtain the current gripper status.

        Returns:  Instance of `robotiq_2f_gripper_msgs/RobotiqGripperStatus` message. See the message declaration for fields description
        """
        status = RobotiqGripperStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.is_ready = True
        status.is_reset = False
        status.is_moving = self._is_moving
        status.obj_detected = False
        status.fault_status = False
        status.position = self.from_radians_to_distance(self._current_joint_pos) #[mm]
        status.requested_position = self._current_goal.position #[mm]
        status.current = 0.0
        return status

    def get_current_joint_position(self):
      return self._current_joint_pos

    def from_distance_to_radians(self, linear_pose ):
      """
      Private helper function to convert a command in meters to radians (joint value)
      """
      return np.clip(self._max_joint_limit - ((self._max_joint_limit/self._stroke) * linear_pose), 0.0, self._max_joint_limit)
  
    def from_radians_to_distance(self, joint_pose):
      """
      Private helper function to convert a joint position in radians to meters (distance between fingers)
      """
      return np.clip(self._stroke - ((self._stroke/self._max_joint_limit) * joint_pose), 0.0, self._max_joint_limit)

    def _update_gripper_joint_state(self):
        """
        Private helper function to create a JointState message with the current gripper joint position
        """
        js = JointState()
        js.header.frame_id = ''
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self._joint_names]
        pos = np.clip( self._current_joint_pos, 0.0, self._max_joint_limit)
        js.position = [pos]
        js.velocity = [self._current_goal.speed]
        self._prev_joint_pos = pos
        return js

    def _clamp_position(self,cmd):
        if (cmd <= 0.0):
            cmd = 0.0
        elif (cmd >= self._stroke):
            cmd = self._stroke
        return cmd
    
    def _clamp_speed(self,cmd):
        if (cmd < 0.0):
            cmd = 0.01
        elif (cmd > 0.101):
            cmd = 0.1
        return cmd

