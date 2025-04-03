#!/usr/bin/env python3
import numpy as np
from urdfpy import URDF

# Load the URDF file
urdf_path = "/home/kpatnaik/Desktop/robots/UR5e_RG2/ur5e_rg2_no_articulation_export.urdf"
robot = URDF.load(urdf_path)

# Define UR5e Joint Order
ur5_joint_order = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
]

# Extract joint parameters
dh_table = []
for joint_name in ur5_joint_order:
    joint = next((j for j in robot.joints if j.name == joint_name), None)
    
    if joint is None:
        print(f"❌ Joint {joint_name} not found in URDF!")
        continue

    # Extract xyz (d_i) and rpy (alpha_i)
    xyz = joint.origin[:3, 3]  # Extract d_i
    rpy = joint.origin[:3, :3]  # Extract alpha_i from rotation matrix

    # Convert rotation to RPY
    roll, pitch, yaw = np.arctan2(rpy[2, 1], rpy[2, 2]), np.arctan2(-rpy[2, 0], np.sqrt(rpy[2, 1]**2 + rpy[2, 2]**2)), np.arctan2(rpy[1, 0], rpy[0, 0])

    # Extract a_i (x translation)
    a_i = xyz[0]
    d_i = xyz[2]  # z translation as d_i

    # Append to DH table
    dh_table.append([joint_name, d_i, a_i, roll])

# Print DH Table
print("\n🔹 **UR5e Extracted DH Parameters from URDF**")
print(f"{'Joint':<20} {'d_i (m)':<10} {'a_i (m)':<10} {'alpha_i (rad)':<15}")
print("=" * 60)
for row in dh_table:
    print(f"{row[0]:<20} {row[1]:<10.5f} {row[2]:<10.5f} {row[3]:<15.5f}")
