from builtin_interfaces.msg import Duration
from ur_asu.custom_libraries.ik_solver import compute_ik

HOME_POSE = [0.065, -0.385, 0.481, 0, 180, 0]  # XYZRPY

def make_point(joint_positions, seconds):
    return {
        "positions": [float(x) for x in joint_positions],  # ensures all are float
        "velocities": [0.0] * 6,
        "time_from_start": Duration(sec=int(seconds)),
    }

def home():
    joint_angles = compute_ik(HOME_POSE[0:3], HOME_POSE[3:6])
    if joint_angles is not None:
        return [make_point(joint_angles, 4)]
    return []

def move(position, rpy, seconds):
    if len(position) != 3:
        raise ValueError(f"Expected 3D position, got {position}")
    joint_angles = compute_ik(position, rpy)
    # (j1, j2, j3, j4, j5, j6) = joint_angles
    # print(f"{j1:.3f}, {j2:.3f}, {j3:.3f}, {j4:.3f}, {j5:.3f}, {j6:.3f}")
    if joint_angles is not None:
        return [make_point(joint_angles, seconds)]
    return []

def moveZ(position, rpy, seconds):
    if len(position) != 3:
        raise ValueError(f"Expected 3D position, got {position}")
    joint_angles = compute_ik(position, rpy)
    if joint_angles is not None:
        return [make_point(joint_angles, seconds)]
    return []

def moveXY(position, rpy, seconds):
    if len(position) != 3:
        raise ValueError(f"Expected 3D position, got {position}")
    joint_angles = compute_ik(position, rpy)
    if joint_angles is not None:
        return [make_point(joint_angles, seconds)]
    return []

def pick_and_place(block_pose, slot_pose):
    """
    block_pose and slot_pose are each (position, rpy), where position = [x, y, z]
    and EE orienttaion is [r, p, y]
    """
    block_hover = block_pose[0].copy() ## copying positions
    block_hover[2] += 0.1  # hover 10cm above block

    slot_hover = slot_pose[0].copy()
    slot_hover[2] += 0.1  # hover 10cm above slot

    segment_duration = 6 # specify segment_duration

    return {
        "traj0": home(),
        "traj1": move(block_hover,block_pose[1],segment_duration), # hovers on block 
        "traj2": moveZ(block_pose[0],block_pose[1],segment_duration), # descends to grip position, 
        "traj3": moveZ(block_pose[0],block_pose[1],segment_duration), # gripper close
        "traj4": moveZ(block_hover,block_pose[1],segment_duration), # holds block and hovers 
        "traj5": moveXY(slot_hover,slot_pose[1],segment_duration), # holds block and moves in 2D to hover on slot
        "traj6": moveZ(slot_pose[0],slot_pose[1],segment_duration), # holds block and descends into slot,
        "traj7": moveZ(slot_pose[0],slot_pose[1],segment_duration), # gripper open
        "traj8": home() # homing
    }

def spin_around(target_pose, height):
    """
    target_pose is (position, rpy), where position = [x, y, z] and only x, y are considered
    """
    target_position = target_pose[0].copy() # copying positions
    target_position[2] = height  # Set height to given value
    yaws = range(0, 360, 45)
    segment_duration = 3 # specify segment_duration
    return {
        "traj0": move(target_position, [0, 180, yaws[0]], segment_duration),
        "traj1": move(target_position, [0, 180, yaws[1]], segment_duration),
        "traj2": move(target_position, [0, 180, yaws[2]], segment_duration),
        "traj3": move(target_position, [0, 180, yaws[3]], segment_duration),
        "traj4": move(target_position, [0, 180, yaws[4]], segment_duration),
        "traj5": move(target_position, [0, 180, yaws[5]], segment_duration),
        "traj6": move(target_position, [0, 180, yaws[6]], segment_duration),
        "traj7": move(target_position, [0, 180, yaws[7]], segment_duration),
    }

def hover_over(target_pose, height):
    """
    target_pose is (position, rpy), where position = [x, y, z] and only x, y are considered
    """
    target_position = target_pose[0].copy() # copying positions
    target_position[2] = height  # Set height to given value
    fixed_roll = 0
    fixed_pitch = 180
    yaw = target_pose[1][2] # in degrees
    # null_rot = [0, 180, 0]
    target_rot = [fixed_roll, fixed_pitch, yaw]
    segment_duration = 3 # specify segment_duration
    # print(block_hover, target_rot)
    (x, y, z) = target_position
    print(f"Made target pose of <{x:.3f}, {y:.3f}, {z:.3f}> @ rpy [{fixed_roll:.1f}, {fixed_pitch:.1f}, {yaw:.1f}]")

    return {
        # "traj0": move(target_position,null_rot,segment_duration), # hovers over target 
        "traj1": move(target_position,target_rot,segment_duration), # hovers over target, matching angle
    }