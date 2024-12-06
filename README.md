# Two-Link Robotic Arm Simulation

This repository contains a simulation of a two-link robotic manipulator tracking a circular trajectory. The simulation demonstrates the use of inverse kinematics for joint angle calculations and forward kinematics for visualizing the motion.

## Features

- **Trajectory Tracking**: The robotic arm follows a predefined circular path.
- **Kinematics**: Inverse kinematics for calculating joint angles, forward kinematics for visualizing movement.
- **Visualization**: Real-time animation of the robotic arm using `matplotlib`.

## How It Works

1. **Trajectory Definition**: A circular trajectory is defined as the desired end-effector path.
2. **Kinematics**:
   - **Inverse Kinematics**: Calculates joint angles required to achieve a given end-effector position.
   - **Forward Kinematics**: Computes the position of the end-effector and links based on joint angles.
3. **Real-Time Simulation**: Animates the manipulator as it follows the trajectory.

 ![Simulation Preview](assets/simulation.gif)

## Requirements

- Python 3.7 or later
- NumPy
- Matplotlib

Install the dependencies using pip:
```bash
pip install numpy matplotlib
