import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node


class SimpleCommander(Node):
    def __init__(self):
        super().__init__("simple_commander")
        # Initialise ROS 2 and the navigator
        # rclpy.init()
        self.nav = BasicNavigator()

        # You can expose parameters here if you want to set poses via launch files
        # Example:
        # self.declare_parameter('init_x', 0.0)
        # ...

    def run(self):
        # ------------------------------------------------------------------
        # Insert the original script logic here, adapted to the class context
        # ------------------------------------------------------------------

        # Example placeholder – replace with your real init/goal poses
        # init_pose = ...  # geometry_msgs.msg.PoseStamped
        # goal_pose = ...  # geometry_msgs.msg.PoseStamped

        self.nav.setInitialPose(init_pose)
        self.nav.waitUntilNav2Active()  # or lifecycleStartup()

        path = self.nav.getPath(init_pose, goal_pose)
        smoothed_path = self.nav.smoothPath(path)

        self.nav.goToPose(goal_pose)
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            if feedback.navigation_duration > 600:
                self.nav.cancelTask()

        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
        elif result == TaskResult.CANCELED:
            self.get_logger().info("Goal was canceled!")
        elif result == TaskResult.FAILED:
            self.get_logger().info("Goal failed!")


def main(args=None):
    rclpy.init(args=args)
    commander = SimpleCommander()
    commander.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
