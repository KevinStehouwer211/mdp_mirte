import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose
from action_tutorials_interfaces.action import Nav


class NavActionClient(Node):

    def __init__(self):
        super().__init__('nav_client')
        self._action_client = ActionClient(self, Nav, 'nav')

    def send_goal(self, goal_pose: Pose):
        goal_msg = Nav.Goal()
        goal_msg.goal_pose = goal_pose

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = NavActionClient()

    goal_pose = Pose()
    goal_pose.position.x = 1.0
    goal_pose.position.y = 0.0
    goal_pose.position.z = 0.0
    goal_pose.orientation.x = 0.0
    goal_pose.orientation.y = 0.0
    goal_pose.orientation.z = 0.0
    goal_pose.orientation.w = 1.0

    action_client.send_goal(goal_pose)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()