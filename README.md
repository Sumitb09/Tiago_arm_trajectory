# TIAGo Arm Trajectory Publisher

This ROS 2 package publishes joint trajectories for the TIAGo robot's arm in simulation. It is designed to work with the [TIAGo simulation](https://github.com/pal-robotics/tiago_simulation) from PAL Robotics. The node generates a smooth circular trajectory and sends it to the `/arm_controller/joint_trajectory` topic.

## Features  

- Publishes a predefined trajectory for the TIAGo arm.  
- Generates a circular motion for the first two joints.  
- Runs on a timer, publishing a new trajectory every 2 seconds.  
- Compatible with `tiago_simulation`.  

## Installation  

Ensure you have ROS 2 (Jazzy or Humble) and `tiago_simulation` installed. Then, clone this repository inside your ROS 2 workspace:  

```sh
cd ~/ros2_ws/src
git clone https://github.com/Sumitb09/Tiago_arm_trajectory.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Running the Simulation  

First, launch the TIAGo simulation in Gazebo:  

```sh
ros2 launch tiago_gazebo tiago_gazebo.launch.py
```

Then, run the trajectory publisher node:  

```sh
ros2 run Tiago_arm_trajectory tiago_trajectory_publisher
```

## Output  
 
[click here to see the output](output/Screencast from 03-24-2025 02_54_16 PM (online-video-cutter.com).mp4)


## Node Details  

### **Published Topic:**  
- `/arm_controller/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`)  

### **Trajectory Description:**  
- **Joint Names:** `arm_1_joint`, `arm_2_joint`, ..., `arm_7_joint`  
- **Motion:** Circular trajectory for `arm_1_joint` and `arm_2_joint`, fixed values for the others.  
- **Number of waypoints:** 20  
- **Time per step:** 0.5 seconds  

## Dependencies  

- `rclcpp` (ROS 2 C++ client library)  
- `trajectory_msgs` (for joint trajectory messages)  
- `tiago_simulation` (for running the TIAGo robot in Gazebo)  

## License  

This project is licensed under the MIT License.

