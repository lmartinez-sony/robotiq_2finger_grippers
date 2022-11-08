import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from robotiq_2f_gripper_msgs.action import CommandRobotiqGripper


class RobotiqActionClient(Node):

    def __init__(self):
        super().__init__('robotiq_action_client')
        self._action_client = ActionClient(self, CommandRobotiqGripper, '/robotiq_gripper_1/command')

        self._action_client.wait_for_server()

    def send_goal(self, position, speed, force):
        goal_msg = CommandRobotiqGripper.Goal()
        goal_msg.position = position
        goal_msg.speed = speed
        goal_msg.force = force

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = RobotiqActionClient()

    future = action_client.send_goal(0.05, 0.1, 0.0)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()