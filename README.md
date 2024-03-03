# CS885 Reinforcement Learning

This repository contains the project content for the course CS885-Reinforcement Learning (taught in University of Waterloo), focusing on designing an actor-critic based controller and comparing its performance with the virtual energy regulator [[Link](https://ieeexplore.ieee.org/document/9492025)] and a PID controller 

## Prerequisites
Before you begin, ensure you have the following installed:
- MATLAB (R2021a or later)
- Simulink
- Control System Toolbox
- Quanser Hardware Support Package (for real-time implementation)
- Quanser linear flexible joint [[https://www.quanser.com/products/linear-flexible-joint/](https://www.quanser.com/products/linear-flexible-joint/)]

## Lab Content
The repository is structured as follows:
- `Lab1/`: Introduction to Simulink for control system design
  - System identification through step response measurement.
  - Proportional (P-type) position control.
  - Closed-loop system identification using Bode plot.
- `Lab2/`: Controller design and stability analysis:
  - Proportional-Derivative (PD) controller.
  - Proportional-Integral-Derivative (PID) controller and stability analysis.
  - Disturbance and steady-state error analysis.
- `Lab3/`: Identification and P-control of cascaded systems:
  - Collocated vs. non-collocated control.
  - Step response measurement.
  - Frequency response measurement.
- `Lab4/`: Frequency domain loop shaping control of cascaded systems
  - Gain selection
  - Notch filter design
  - Lead compensator design
  - Lag compensator design
  - Feedforward controller design

Each lab folder contains:
- `Instructions.pdf`: Detailed lab instructions and theoretical background.
- `SimulinkModels/`: Pre-built Simulink models for the lab exercises.
