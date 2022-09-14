#!/usr/bin/env python
"""--------------------------------------------------------------------
This Node/Script creates an instance of a `SimpleActionServer` dedicated to receive, 
process and execute user commands for the Robotiq 2 finger adaptive grippers.

The action type is defined in the `robotiq_2f_gripper_msgs` as `CommandRobotiqGripper.action` and
the default action name is `/command_robotiq_action` but you can namespace it for multiple grippers
control.

See the `robotiq_action_server.launch` file on this package for an example on how to call this node.

Parameters:
    comport: USB Communication port to which the gripper is connected to (not needed in `sim` mode).  
    baud: Baudrate of communication with gripper (not needed in `sim` mode).
    stroke: Maximum distance in meters, between the gripper fingers (Only 0,085 and 0.140 are currently supported)
    joint_name: Name of the URDF gripper actuated joints to publish on the `/joint_state` topic 
    sim: Boolean indicating whether to use a simulated gripper or try to connect to a real one.
    rate: Frequency in Herz to update the gripper variables (command, joint_state, gripper_state)

@author: Daniel Felipe Ordonez Apraez
@email: daniels.ordonez@gmail.com
--------------------------------------------------------------------"""
import time
import rclpy
from rclpy.node import Node
from .robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver, Robotiq2FingerSimulatedGripperDriver 
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from robotiq_2f_gripper_msgs.action import CommandRobotiqGripper

GOAL_DETECTION_THRESHOLD = 0.05 # Max deviation from target goal to consider as goal "reached"

class CommandGripperActionServer(Node):

    def __init__(self):
        super().__init__('robotiq_2f_gripper_action_server')

        self.declare_parameter('~comport','/dev/ttyUSB0')
        self.declare_parameter('~baud','115200')
        self.declare_parameter('~stroke', 0.085) # Default stroke is 85mm (Small C / 2 finger adaptive gripper model)
        self.declare_parameter('~joint_names', 'finger_joint')
        self.declare_parameter('~sim', False)
        self.declare_parameter('~rate', 50)

         # Get Node parameters
        self._comport = self.get_parameter('~comport').get_parameter_value().string_value
        self._baud = self.get_parameter('~baud').get_parameter_value().integer_value
        self._stroke = self.get_parameter('~stroke').get_parameter_value().double_value         
        self._joint_names = self.get_parameter('~joint_names').get_parameter_value().string_array_value
        self._sim = self.get_parameter('~sim').get_parameter_value().bool_value
        self._rate = self.get_parameter('~rate').get_parameter_value().integer_value      

        # Create instance of Robotiq Gripper Driver
        if self._sim: # Use simulated gripper
            self._driver = Robotiq2FingerSimulatedGripperDriver( stroke=self._stroke, joint_names=self._joint_names)    
        else:   # Try to connect to a real gripper 
            self._driver = Robotiq2FingerGripperDriver( comport=self._comport, baud=self._baud, stroke=self._stroke, joint_names=self._joint_names, rate=self._rate)

        self._action_server = ActionServer(self, CommandRobotiqGripperAction, "~/command", self.execute_cb)
        self._joint_trajectory_action_server = ActionServer(FollowJointTrajectoryAction, "~/robotiq_controller/follow_joint_trajectory", \
                                                            self.execute_joint_trajectory_cb)
        self._driver = driver       # Get handle to the Gripper Control Server
        
        # Wait until gripper driver is ready to take commands.
        watchdog = self.create_timer(15.0, self._connection_timeout)
        while rclpy.utilities.ok() and not self._driver.is_ready:
            time.sleep(0.5)
            self.get_logger().warn("Waiting for gripper to be ready...")
        
        watchdog.cancel() 
        if rclpy.utilities.ok():
            self._processing_goal = False
            self._is_stalled = False

            self.get_logger().info("Robotiq server started")

        # Send and Request data from gripper and update joint state every `r`[Hz]
        self._gripper_state_publisher_timer = self.create_timer(1/self._rate, self._driver.update_driver())
    
    def _connection_timeout(self, event):
        self.get_logger().fatal("Gripper on port {} seems not to respond".format(self._driver._comport))
        self.shutdown()
    
    def execute_cb(self, goal_handle):
      goal_command = goal_handle.request
      self.get_logger().debug( ("New goal received Pos:{%.3f} Speed: {%.3f} Force: {%.3f} Force-Stop: {%r}").format(goal_command.position, goal_command.speed, goal_command.force, goal_command.stop) )
      # Send incoming command to gripper driver
      self._driver.update_gripper_command(goal_command)
      # Wait until command is received by the gripper 
      time.sleep(0.1)
      # Set Action Server as active === processing goal...
      self._processing_goal = True        
      
      feedback = CommandRobotiqGripper.Feedback()
      result = CommandRobotiqGripper.Result()

      # Set timeout timer 
      watchdog = self.create_timer(5.0, self._execution_timeout)

      # Wait until goal is achieved and provide feedback
      rate = self.create_rate(self.get_parameter('~rate').get_parameter_value())

      while rclpy.utilities.ok() and self._processing_goal and not self._is_stalled:             # While moving and not stalled provide feedback and check for result
          feedback.feedback = self._driver.get_current_gripper_status()
          goal_handle.publish_feedback( feedback )
          self.get_logger().debug("Error = {%.5f} Requested position = {%.3f} Current position = {%.3f}".format(abs(feedback.requested_position - feedback.position), feedback.requested_position, feedback.position))
          # Check for completion of action 
          if( feedback.fault_status != 0 and not self._is_stalled):               # Check for errors
              self.get_logger().error("Fault status (gFLT) is: {%d}".format(feedback.fault_status))
              self._is_stalled = True
              goal_handle.abort()
              return feedback
          if( abs(feedback.requested_position - feedback.position) < GOAL_DETECTION_THRESHOLD or feedback.obj_detected):    # Check if position has been reached 
              watchdog.shutdown()                         # Stop timeout watchdog.
              self._processing_goal = False 
              self._is_stalled = False              
          rate.sleep()
      
      result = feedback                                   # Message declarations are the same 
      # Send result 
      if not self._is_stalled:
          self.get_logger().debug("Goal reached or object detected Pos: {%.3f} PosRequested: {%.3f} ObjectDetected: {%r}".format(goal_command.position, feedback.requested_position, feedback.obj_detected) )
          goal_handle.succeed()  
      else:
          self.get_logger().error("Goal aborted Pos: {%.3f} PosRequested: {%.3f} ObjectDetected: {%r}".format(goal_command.position, feedback.requested_position, feedback.obj_detected) )
          goal_handle.abort()  

      self._processing_goal = False 
      self._is_stalled = False 
      return result
    
    def execute_joint_trajectory_cb(self, goal_handle):
      goal = goal_handle.request
      self.get_logger().info("Trajectory received with {%d} points".format(len(goal.trajectory.points)))
      feedback = FollowJointTrajectory.Feedback()
      result = FollowJointTrajectory.Result()
      current_status = self._driver.get_current_gripper_status()

      # Check trajectory joint names
      joint_names = goal.trajectory.joint_names
      if len(joint_names) != 1 and joint_names[0] != self._driver._joint_name :
        msg = "Joint trajectory joints do not match gripper joint"
        self.get_logger().error(msg)
        result.error_code = result.INVALID_JOINTS
        result.error_string = msg
        goal_handle.abort()
        return result
      # Check trajectory points
      if len(goal.trajectory.points) == 0:
        msg = "Ignoring empty trajectory "
        self.get_logger().error(msg)
        result.error_code = result.INVALID_GOAL
        result.error_string = msg
        goal_handle.abort()
        return result
      
      # Process goal trajectory
      self._processing_goal = True  
      self._is_stalled = False

      goal_command = CommandRobotiqGripper.Goal()
      feedback.joint_names = goal.trajectory.joint_names      
      watchdog = self.create_timer((goal.trajectory.points[-1].time_from_start.to_sec() + 0.5), self._execution_timeout)

      # Follow trajectory points
      goal_trajectory_point = goal.trajectory.points[-1]
      
      # Validate trajectory point
      if len(goal_trajectory_point.positions) != 1:
        result.error_code = result.INVALID_GOAL
        result.error_string = "Invalid joint position on trajectory point "
        goal_handle.abort()
        return result
      target_speed = goal_trajectory_point.velocities[0] if len(goal_trajectory_point.velocities) > 0 else 0.01
      target_force = goal_trajectory_point.effort[0] if len(goal_trajectory_point.effort) > 0 else 0.1
      goal_command.position = self._driver.from_radians_to_distance(goal_trajectory_point.positions[0])
      goal_command.speed = abs(target_speed) # To-Do: Convert to rad/s
      goal_command.force = target_force
      # Send incoming command to gripper driver
      self._driver.update_gripper_command(goal_command)
      # Set feedback desired value 
      feedback.desired.positions = [goal_trajectory_point.positions[0]]
      
      while rclpy.utilities.ok() and self._processing_goal and not self._is_stalled:  
        current_status = self._driver.get_current_gripper_status()          
        feedback.actual.positions = [self._driver.get_current_joint_position()]
        error = abs(feedback.actual.positions[0] - feedback.desired.positions[0])
        self.get_logger().debug("Error : %.3f -- Actual: %.3f -- Desired: %.3f", error, self._driver.get_current_joint_position(), feedback.desired.positions[0])           

        feedback.error.positions = [error]
        goal_handle.publish_feedback( feedback )
        
        # Check for errors
        if current_status.fault_status != 0 and not self._is_stalled:              
          self._is_stalled = True
          self._processing_goal = False 
          self.get_logger().error(msg)
          result.error_code = -6
          result.error_string = "Gripper fault status (gFLT): " + current_status.fault_status
          goal_handle.abort()
          return result
        # Check if object was detected
        if current_status.obj_detected:     
          watchdog.shutdown()                         # Stop timeout watchdog.
          self._processing_goal = False 
          self._is_stalled = False
          result.error_code = result.SUCCESSFUL          
          result.error_string = "Object detected/grasped" 
          goal_handle.succeed()  
          return result
        # Check if current trajectory point was reached 
        if error < GOAL_DETECTION_THRESHOLD :      
          break
        
      # Entire trajectory was followed/reached
      watchdog.shutdown() 
     
      self.get_logger().debug("Goal reached")
      result.error_code = result.SUCCESSFUL          
      result.error_string = "Goal reached" 
      goal_handle.succeed()  

      self._processing_goal = False 
      self._is_stalled = False 

      return result

    def _execution_timeout(self, event):
        self.get_logger().error("%s: Achieving goal is taking too long, dropping current goal")
        self._is_stalled = True
        self._processing_goal = False

def main():

    rclpy.init()

   
    # Start action server 
    robotiq_action_server = CommandGripperActionServer()

    rclpy.spin(robotiq_action_server)
    
    
if __name__ == "__main__":
    main()