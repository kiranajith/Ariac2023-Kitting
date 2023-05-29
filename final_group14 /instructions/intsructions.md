# ROS2 ARIAC Competition Execution Guide

This guide provides instructions on how to run the ARIAC competition using ROS2. Follow the steps below to set up and run the competition nodes.

## Step 1: Source the Workspaces

Before running the competition nodes, ensure you have sourced the workspaces correctly.

```bash
source /opt/ros/galactic/setup.bash 
source ~/<your_ws>/install/setup.bash
```
if you have a zsh shell run the following commands

```bash
source /opt/ros/galactic/setup.zsh 
source ~/<your_ws>/install/setup.zsh
```


## Step 2: Run the Nodes

### Terminal 1: Launch the ARIAC Environment 
```bash
ros2 launch ariac_gazebo ariac.launch.py trial_name:=final
```

### Terminal 2: Launch the Robot Commander which contains the MOVEit package   
```bash
ros2 launch robot_commander robot_commander.launch.py
```

### Terminal 3: Launch final_group14 Node
```bash 
ros2 launch final_group14 comp_launch.py
```

After executing these commands, the ARIAC competition will start.
