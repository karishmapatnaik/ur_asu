import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from scipy.spatial.transform import Rotation as R
import numpy as np

from ur_asu.custom_libraries.actionlibrariesmax import hover_over  # <-- your function here

# GRIPPER_COMMANDS = {
#     "traj3": "close",  # grip block
#     "traj7": "open",   # release block
# }

class JTCClient(Node):
    def __init__(self):
        super().__init__("trajectory_executor")
        # Parameter Management
        self.declare_parameter("controller_name", "scaled_joint_trajectory_controller")
        self.declare_parameter("joints", [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"])
        self.declare_parameter("target_marker_id", 6)

        controller_name = self.get_parameter("controller_name").value + "/follow_joint_trajectory"
        self.joints = self.get_parameter("joints").value
        self.target_marker_id = self.get_parameter("target_marker_id").value


        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        # self._gripper_pub = self.create_publisher(String, "/gripper_command", 10)

        self.subscription = self.create_subscription(
            PoseStamped,
            f"/marker_poses/marker_{self.target_marker_id}",  # Listening to a specific marker
            self.marker_pose_callback,
            10)

        self.get_logger().info(f"Waiting for action server on {controller_name}")
        self._action_client.wait_for_server()

        self.latest_target_pose = None
        self.last_sent_pose = None
        # self.pose_update_time = time.time()

        # Timer to check for new poses periodically
        self.timer = self.create_timer(1.0, self.check_for_new_pose)  # every 1s
        

        self.active_goal_handle = None
        self.executing = False
        self.last_pose_sent_time = 0
        self.i = 0

    def parse_trajectories(self):
        goals = {}
        for traj_name, points in self.trajectories.items():
            traj = JointTrajectory()
            traj.joint_names = self.joints
            for pt in points:
                point = JointTrajectoryPoint()
                point.positions = pt["positions"]
                point.velocities = pt["velocities"]
                point.time_from_start = pt["time_from_start"]
                traj.points.append(point)
            goals[traj_name] = traj
        return goals

    # def publish_gripper(self, cmd):
    #     msg = String()
    #     msg.data = cmd
    #     self.get_logger().info(f"Sending gripper command: {cmd}")
    #     self._gripper_pub.publish(msg)

    def execute_next_trajectory(self):
        # if self.i >= len(self.goals):
        #     self.get_logger().info("Done with all trajectories")
        #     raise SystemExit

        traj_name = list(self.goals)[self.i]
        # self.i += 1

        # if traj_name in GRIPPER_COMMANDS:
        #     # Close before sending traj3, open after sending traj7
        #     if traj_name == "traj3":
        #         self.publish_gripper("close")

        self.execute_trajectory(traj_name)

    def execute_trajectory(self, traj_name):
        self.get_logger().info(f"▶ Executing trajectory {traj_name}")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self.goals[traj_name]
        goal.goal_time_tolerance = Duration(sec=0, nanosec=500_000_000)
        goal.goal_tolerance = [JointTolerance(position=0.01, velocity=0.01, name=name) for name in self.joints]

        self.executing = True  # Flag it
        self._send_goal_future = self._action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(lambda f: self.goal_response_callback(f, traj_name))


    def marker_pose_callback(self, msg: PoseStamped):
        # Convert quaternion to rpy
        quat = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        rpy = R.from_quat(quat).as_euler('xyz', degrees=True)
        xyz = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        self.latest_target_pose = (xyz, rpy)
        # self.pose_update_time = time.time()

    def check_for_new_pose(self):
        if self.latest_target_pose is None:
            return

        # Only replan if we're not running something
        if self.executing:
            return

        if self.last_sent_pose is None or self._pose_changed_enough():
            # (x, y, z), (r, p, yw) = self.latest_target_pose
            self.get_logger().info(f"New target detected!")
            self.send_trajectory(self.latest_target_pose)
            self.last_sent_pose = self.latest_target_pose
            # self.last_pose_sent_time = now

    def _pose_changed_enough(self, lin_threshold=0.01, ang_threshold=10):
        if self.last_sent_pose is None:
            return True
        old_pos, old_rot = self.last_sent_pose
        new_pos, new_rot = self.latest_target_pose
        lin_dist = np.linalg.norm(np.array(old_pos) - np.array(new_pos))
        rot_dist = np.linalg.norm(np.array(old_rot) - np.array(new_rot))
        return lin_dist > lin_threshold or rot_dist > ang_threshold

    def send_trajectory(self, target_pose):
        (x, y, z), (r, p, yw) = target_pose
        self.get_logger().info(f"Received pose for marker {self.target_marker_id}: <{x:.3f}, {y:.3f}, {z:.3f}> @ angle [{r:.1f}, {p:.1f}, {yw:.1f}]")
        self.trajectories = hover_over(target_pose, 0.30)
        self.goals = self.parse_trajectories()
        self.execute_next_trajectory()

    def goal_response_callback(self, future, traj_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            raise RuntimeError("Goal rejected :(")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(lambda f: self.get_result_callback(f, traj_name))

    def get_result_callback(self, future, traj_name):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f"✔ Trajectory {traj_name} completed with status: {self.status_to_str(status)}")

        self.executing = False  # <-- Reset flag

        # if traj_name == "traj7":
        #     self.publish_gripper("open")

        if status == GoalStatus.STATUS_SUCCEEDED:
            time.sleep(0)
            # self.execute_next_trajectory()
        else:
            raise RuntimeError("Trajectory failed: " + str(result.error_string))

    @staticmethod
    def status_to_str(status):
        return {
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
            GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
            GoalStatus.STATUS_EXECUTING: "EXECUTING",
            GoalStatus.STATUS_CANCELING: "CANCELING",
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }.get(status, "?")

def main(args=None):
    rclpy.init(args=args)
    node = JTCClient()
    try:
        rclpy.spin(node)
    except (RuntimeError, SystemExit):
        node.get_logger().info("Shutting down")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
