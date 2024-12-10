# Experimental Robotics Project

This repository contains a set of ROS packages and nodes developed as part of an experimental robotics project. The main focus is on integrating a robotic arm with a camera system for marker detection, environment mapping, and executing motion commands using MoveIt. The code demonstrates how to set up a simulation environment, detect visual markers, and control a robot arm to approach and align with those markers.

## Overview

The project involves:

- A Gazebo simulation environment with a ground plane, multiple marker objects placed at various positions, and a robotic arm.
- Nodes dedicated to:
  - Loading and interpreting the robot model using MoveIt.
  - Planning and executing trajectories for the robotic arm.
  - Processing camera images to detect and track markers.
  - Dynamically rotating a camera or moving the arm to align with detected markers.
  
The main functionality includes:
- Collecting marker IDs and their positions.
- Dynamically rotating the camera or the end-effector mount to locate and align with each detected marker.
- Using a service interface to trigger motion planning and arm movements to predefined poses.
- Executing inverse kinematics to find suitable joint configurations that achieve a given target pose.

## Packages and Nodes

1. **Robot Model and MoveIt Integration**:  
   - Loads the robot’s URDF model and SRDF configuration.
   - Uses MoveIt to handle path planning and inverse kinematics.
   - Provides a service to move the arm to high or low predefined poses based on external requests.

2. **Marker Detection and Camera Control**:  
   - Subscribes to camera topics for compressed images.
   - Processes images using OpenCV to detect markers and compute their 2D positions.
   - Records marker IDs and their coordinates for reference.
   - Rotates the camera holder or the robot’s joints to align the camera view with each marker.
   - Once aligned, a marker is highlighted and its data recorded, after which the system moves on to the next marker.

3. **Service Interfaces**:  
   - A ROS service that, when called, triggers the robot arm to move to one of two predefined poses.
   - Another node that, on receiving a request, performs IK computation, plans a path, and executes the motion.

## Getting Started

### Prerequisites

- ROS (tested with ROS Noetic or Melodic)
- MoveIt
- Gazebo
- OpenCV
- Python 3 and related libraries


