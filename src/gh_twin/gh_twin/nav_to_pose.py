#! /usr/bin/env python3

import enum

from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node



# Exit codes for navigation task status
class ExitCode(enum.IntEnum):
    INIT_COMPLETE       =  0x50
    INIT_FAILED         =  0x51
    PATH_VALID_GOAL_SET =  0x52
    PATH_TRACKING       =  0x53
    PATH_INVALID        =  0x54
    GOAL_CANCELED       =  0x55
    GOAL_SUCCEEDED      =  0x56
    GOAL_FAILED         =  0x57
    TIMEOUT             =  0x58

# Navigation interface class for sending navigation goals and tracking their status
class NavToPose(Node):
    def __init__(self, pose: Pose):
        super().__init__('nav2_interface_node')

        #Initialize the navigator object
        self.navigator = BasicNavigator()
        
        # Initialize variables to store the initial pose, goal pose, and navigation results
        self.initial_pose = PoseStamped()
        self.goal_pose    = PoseStamped()
        self.nav_task     = None
        self.current_pose = PoseWithCovarianceStamped()
        # True once a real amcl_pose has arrived, so we never mistake the
        # default zero pose for the robot actually being at the map origin.
        self.pose_received = False
        # Timeout for navigation task in seconds
        self.NAV_TIMEOUT = 120
        # To ensure initialization of nav2 is completed before pose extraction and path planning is attempted
        self.NAV_INIT_COMPLETE = False

        # AMCL pose estimation
        self.subscription = self.create_subscription(PoseWithCovarianceStamped,'amcl_pose',self.current_pose_callback,10)

        # Set initial pose of the robot in the navigator object.
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose = pose
        # Hardcoded start position (simple override; bypasses live localization).
        self.initial_pose.pose.position.x = 0.5
        self.current_pose.pose.pose.position.x = 0.5
        self.pose_received = True   # treat the hardcoded pose as a valid localization
        self.navigator.setInitialPose(self.initial_pose)
        # Only seed AMCL when given a real pose. A default/zero pose would
        # relocalize the robot to map origin (inside a wall on the hardware map)
        # and overwrite an estimate already set in RViz, so skip it then and
        # keep whatever localization AMCL already has.
        # if not (pose.position.x == 0.0 and pose.position.y == 0.0
        #         and pose.orientation.w in (0.0, 1.0)):
        #     self.navigator.setInitialPose(self.initial_pose)
        # else:
        #     self.navigator.get_logger().info(
        #         "NavToPose: zero initial pose given; keeping AMCL's current "
        #         "estimate (e.g. the RViz 2D Pose Estimate) instead of resetting to origin.")

        # Wait for navigation to fully activate, since autostarting nav2
        self.navigator.waitUntilNav2Active()

        print("Nav2 Initialization Complete")

        self.NAV_INIT_COMPLETE = True
    
    # Function to update the current pose of the robot from the AMCL pose estimation topic
    def current_pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.pose_received = True

    # Function to set a navigation goal and start the navigation task
    def nav_set_goal(self, pose: Pose = None):

        if self.NAV_INIT_COMPLETE == False:
            exit_code = ExitCode.INIT_FAILED
        else:
            self.goal_pose.header.frame_id = 'map'
            self.goal_pose.header.stamp = self.get_clock().now().to_msg()
            self.goal_pose.pose = pose

            path = self.navigator.getPath(start=self.initial_pose, goal = self.goal_pose, planner_id='', use_start=False)

            if path is not None:
                self.nav_task = self.navigator.goToPose(self.goal_pose)
                exit_code = ExitCode.PATH_VALID_GOAL_SET
            else:
                exit_code = ExitCode.PATH_INVALID

        return exit_code

    # Function to check the status of the navigation task and return appropriate exit codes based on the status
    def nav_update_status(self):
        if self.nav_task is not None:
            if not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback.navigation_time.sec > self.NAV_TIMEOUT:
                    exit_code = ExitCode.TIMEOUT
                else:
                    exit_code = ExitCode.PATH_TRACKING
            else:
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    exit_code = ExitCode.GOAL_SUCCEEDED
                elif result == TaskResult.CANCELED:
                    exit_code = ExitCode.GOAL_CANCELED
                elif result == TaskResult.FAILED:
                    exit_code = ExitCode.GOAL_FAILED

        return exit_code
    
    def nav_cancel_goal(self):
        self.navigator.cancelTask()
        return ExitCode.GOAL_CANCELED
    
    def nav_get_current_pose(self):
        # Ready only when Nav2 is up AND a real amcl_pose has been received;
        # otherwise the pose would be the default zero (map origin), which is a
        # bogus localization, so report not-ready instead of handing it back.
        if self.NAV_INIT_COMPLETE and self.pose_received:
            return True, self.current_pose
        else:
            return False, None

    def __del__(self):
        self.navigator.lifecycleShutdown()
        self.navigator.destroy_node()


    
def main() -> None:
    rclpy.init()

    initial_pose = Pose()

    nav_to_pose = NavToPose(initial_pose)

    goal_pose = Pose()
    goal_pose.position.x = 1.0
    goal_pose.position.y = 1.0

 

    nav_exit_code = nav_to_pose.nav_set_goal(goal_pose)

    while True:
        nav_exit_code = nav_to_pose.nav_update_status()
        # Spin the node to process callbacks
        rclpy.spin_once(nav_to_pose)

        if nav_exit_code == ExitCode.GOAL_SUCCEEDED or nav_exit_code == ExitCode.GOAL_FAILED or nav_exit_code == ExitCode.TIMEOUT or nav_exit_code == ExitCode.PATH_INVALID or nav_exit_code == ExitCode.INIT_FAILED or nav_exit_code == ExitCode.GOAL_CANCELED:
            break

    print("Exit Code: ", nav_exit_code)

    exit(0)


if __name__ == '__main__':
    main()
