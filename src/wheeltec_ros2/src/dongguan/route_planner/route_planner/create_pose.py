#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.time import Time
from tf_transformations import quaternion_from_euler  # pip install tf-transformations


def make_pose(x: float, y: float, yaw_deg: float) -> PoseStamped:
    """
    Helper that builds a PoseStamped in the 'map' frame.
    Parameters
    ----------
    x, y : float
        Position in meters.
    yaw_deg : float
        Heading in degrees (counter‑clockwise from the +X axis).
    Returns
    -------
    geometry_msgs.msg.PoseStamped
    """
    pose = PoseStamped()
    # Header – we use the global map frame; timestamp can be 0 (means “now”)
    pose.header.frame_id = "map"
    pose.header.stamp = rclpy.time.Time().to_msg()  # or Time(seconds=0).to_msg()

    # Position
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0  # ground robot → z = 0

    # Orientation – convert yaw (deg) → quaternion
    yaw_rad = yaw_deg * 3.14159265 / 180.0
    q = quaternion_from_euler(0.0, 0.0, yaw_rad)  # roll, pitch, yaw
    pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    return pose


def main(args=None):
    rclpy.init(args=args)

    # ----------------------------------------------------------------------
    # 1️⃣  Define the initial pose (where the robot starts)
    # ----------------------------------------------------------------------
    # Example: robot starts at (1.2 m, 0.5 m) facing 45°
    init_pose = make_pose(x=1.2, y=0.5, yaw_deg=45.0)

    # ----------------------------------------------------------------------
    # 2️⃣  Define the goal pose (where we want the robot to go)
    # ----------------------------------------------------------------------
    # Example: target location at (5.8 m, 3.4 m) facing 0° (pointing along +X)
    goal_pose = make_pose(x=5.8, y=3.4, yaw_deg=0.0)

    # ----------------------------------------------------------------------
    # 3️⃣  Run the navigator
    # ----------------------------------------------------------------------
    nav = BasicNavigator()

    nav.setInitialPose(init_pose)
    nav.waitUntilNav2Active()  # make sure Nav2 is ready

    # Optional: visualize the planned path
    path = nav.getPath(init_pose, goal_pose)
    smoothed_path = nav.smoothPath(path)

    # Send the robot to the goal
    nav.goToPose(goal_pose)

    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        # Example timeout: cancel if navigation takes > 10 min
        if fb.navigation_duration > 600.0:
            nav.cancelTask()
            break

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print("✅ Goal succeeded!")
    elif result == TaskResult.CANCELED:
        print("⚠️ Goal was canceled.")
    else:
        print("❌ Goal failed.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
