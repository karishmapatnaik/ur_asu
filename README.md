# 🤖 UR5e Custom IK Control (ROS 2)

This **ROS 2 package** provides custom inverse kinematics (IK) control for the **UR5e** robot, eliminating the need for MoveIt. It enables direct joint control and has been tested on **ROS 2 Humble**.

## ✨ Features

- Custom inverse kinematics implementation
- Direct joint control of the UR5e robot
- Compatible with ROS 2 Humble
- Support for tool communication (e.g., grippers)

## 📦 Dependencies

Ensure the **UR ROS 2 driver** is installed:

```bash
sudo apt-get install ros-humble-ur
```
## 📂 Pakage Structure 

```ur_asu/
└── ur_asu/
    ├── custom_libraries/
    │   └── inverse_kinematics.py
    ├── scripts/
    │   └── <all scripts go here>
    ├── resource/
    ├── test/
    ├── setup.cfg
    ├── setup.py
    └── package.xml
```

## 🛠️ Setup
1. Connect the UR5e robot to your network. 

2. In one terminal **source ROS Humble**. Launch the UR ROS 2 driver with tool communication enabled:

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.1.111 \
  use_tool_communication:=true \
  tool_baud_rate:=1000000 \
  use_mock_hardware:=false \
  tool_voltage:=24 \
  tool_parity:=2 \
  tool_stop_bits:=1 \
  tool_rx_idle_chars:=1.5 \
  tool_tx_idle_chars:=3.5
```
## 🔧 Testing

To test the package:

1. In a second terminal, **source Humble, overlay the local workspace** and navigate to the package directory:
  ```
   cd <your_ros2_ws>/src/ur_asu/ur_asu/scripts
```

2. Run the test script:
```
  python3 testcartesian.py
```
