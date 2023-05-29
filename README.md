# Ariac2023-Kitting
ENPM809E Final Project
# ARIAC Kitting Task

This repository contains the implementation of the ARIAC Kitting Task project. The project focuses on automating the kitting task in the ARIAC (Agile Robotics for Industrial Automation Competition) competition.

## Project Overview

The ARIAC Kitting Task project aims to develop a robotic system capable of performing kitting operations efficiently and accurately. The system utilizes ROS (Robot Operating System) and leverages various packages and libraries for communication, control, and perception.

The main components of the project include:

- RobotCommanderInterface: A ROS node that acts as the main controller for the kitting task. It receives orders, coordinates robot actions, and communicates with other nodes and services.
- OrderInfo: A class that represents order information, including order ID, AGV number, tray ID, and kitting task details.
- Tray: A class that represents a tray used in the kitting task. It stores tray-related information such as ID, world pose, and sensor pose.
- Part: A class that represents a part used in the kitting task. It contains information about the part's color, type, and pose.

## Getting Started

### Prerequisites

- ROS: Make sure you have ROS installed on your system. This project was developed using ROS Galactic.
- Dependencies: Install any required dependencies by following the instructions provided in the `requirements.txt` file.

### Installation

1. Clone the repository:

```bash
git clone https://github.com/your-username/ariac-kitting-task.git
```
2. Build the Package
```bash
cd ~/your_workspace
colcon build
```
3. Launch the Environment 
```bash
source install/setup.bash
ros2 launch ariac_gazebo ariac.launch.py trial_name:=final
```
4. Launch the MoveIt package 
```bash
ros2 launch robot_commander robot_commander.launch.py       
```
5. Launch the node that simluates the kitting process 
```bash 
ros2 launch final_group14 comp_launch.py                  
```
