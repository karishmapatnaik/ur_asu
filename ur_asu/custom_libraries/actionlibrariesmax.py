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
