# Filename: unified_waypoint_executor.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import time

from ur_asu.custom_libraries.ik_solver import inverse_kinematics  # Your IK solver

WAYPOINTS = [
    {"position": [0.245, -0.451, 0.112], "rpy": [180, 0, -180], "gripper": "open", "conveyor": [300], "duration": [5]},
    {"position": [0.245, -0.741, 0.112], "rpy": [180, 0, -180], "gripper": "close", "conveyor": [300], "duration": [3]},
    {"position": [0.245, -0.741, 0.112], "rpy": [180, 0, -90],  "gripper": "close", "conveyor": [300], "duration": [3]},
    {"position": [0.245, -0.741, 0.136], "rpy": [180, 0, -90],  "gripper": "close", "conveyor": [300], "duration": [3]},
    {"position": [0.245, -0.831, 0.136], "rpy": [180, 0, -180], "gripper": "open", "conveyor": [300], "duration": [3]},
    {"position": [0.245, -0.831, 0.136], "rpy": [180, 0, -180], "gripper": "open", "conveyor": [300], "duration": [3]},
    {"position": [0.245, -0.831, 0.150], "rpy": [180, 0, -180], "gripper": "open", "conveyor": [300], "duration": [3]},
]

class UnifiedExecutor(Node):
    def __init__(self):
        super().__init__('unified_executor')
        self.gripper_pub = self.create_publisher(String, '/gripper_command', 10)
        self.vention_pub = self.create_publisher(Float64, '/vention/axis1_position_cmd', 10)

        self.ur5_action_client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self.get_logger().info("Waiting for UR5 action server...")
        self.ur5_action_client.wait_for_server()

        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]

        self.run_sequence()

    def run_sequence(self):
        for wp in WAYPOINTS:
            joint_angles = inverse_kinematics(wp["position"], wp["rpy"])
            if joint_angles is None:
                self.get_logger().error("IK failed for waypoint.")
                continue

            self.send_ur5_goal(joint_angles, wp["duration"][0])
            self.send_gripper_command(wp["gripper"])
            self.send_vention_command(wp["conveyor"][0])
            time.sleep(wp["duration"][0])  # Allow time for execution

        self.get_logger().info("Finished all waypoints.")

    def send_ur5_goal(self, joint_angles, duration_sec):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start = Duration(sec=duration_sec)
        goal_msg.trajectory.points = [point]

        future = self.ur5_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("UR5 goal was rejected.")
        else:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            self.get_logger().info("UR5 movement complete.")

    def send_gripper_command(self, command):
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)
        self.get_logger().info(f"Gripper command: {command}")

    def send_vention_command(self, value):
        msg = Float64()
        msg.data = float(value)
        self.vention_pub.publish(msg)
        self.get_logger().info(f"Vention target: {value} mm")

def main(args=None):
    rclpy.init(args=args)
    node = UnifiedExecutor()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
