# CS885 Reinforcement Learning

This repository contains the project content for the course CS885-Reinforcement Learning (taught in University of Waterloo), focusing on designing an actor-critic based controller and comparing its performance with the virtual energy regulator [[Link](https://ieeexplore.ieee.org/document/9492025)] and a PID controller 

## Prerequisites
Before you begin, ensure you have the following installed:
- MATLAB (R2020a or later)
- Simulink
- Control System Toolbox
- Reinforcement Learning Toolbox

The repository contains:
- `report.pdf`: Detailed project report and theoretical background.
- `panda.slx`: Pre-built Simulink model for Franka Emika Panda arm.
- `main_manipulator.m`: the main code which needs to be run.
- `Functions/`: Matlab functions that are used in `main_manipulator.m` are stored in this folder.
- `trajectory/`: Joint trajectories of the robot for the cyclic motion are stored in this folder.
- `TD3Agent/` and `DDPGAgent/`: saved agents are stored in these folders.
