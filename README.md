# A Novel Approach for Integrated Vehicle Dynamics Management

This repository contains the implementation of a novel approach for autonomous vehicle path tracking and integrated vehicle dynamics management, as presented in our recent paper. This approach leverages a Nonlinear Model Predictive Control (NMPC) algorithm combined with an enhanced Random Projection Neural Network (RPNN) to address unknown nonlinearities in vehicle dynamics, achieving accurate path tracking with reduced computational burden.

## Overview

The primary goal of this project is to develop an effective control system for autonomous vehicles that can ensure high tracking accuracy while managing unknown nonlinear uncertainties in lateral tire forces. We adopted an NMPC-based controller, integrated with an improved RPNN, to enhance adaptability and minimize input parameters for precise path tracking.

### Key Features
- **Nonlinear Model Predictive Control (NMPC):** Used for vehicle path tracking while addressing the nonlinearity in vehicle dynamics. NMPC is enhanced with a cost function designed to minimize input parameters.
- **Improved Random Projection Neural Network (RPNN):** Utilizes a novel weight adaptation method, employing gradient descent from the input to the hidden layer, and an adaptive law based on Lyapunov stability theory from the hidden to output layer.
- **Real-time Simulation Results:** The integrated NMPC and RPNN approach demonstrates superior path-tracking accuracy compared to traditional methods, reducing mean lateral error by over 48%.

## System Model

The project utilizes a three-degree-of-freedom (3-DOF) vehicle dynamics model, representing the longitudinal, lateral, and yaw motions of the vehicle. This model, combined with the NMPC controller, forms the basis for ensuring stable and precise path tracking.

The proposed approach employs:
- **NMPC for higher-level control**, managing the vehicle's path tracking while minimizing steering angle inputs.
- **RPNN for uncertainty compensation**, specifically addressing the nonlinear uncertainties in lateral tire forces, ensuring system stability through adaptive learning based on Lyapunov stability theory.

## Prerequisites
To run the simulations and implement the control system, you will need:
- **MATLAB/Simulink**: The main environment used for modeling and simulation.
- **CarSim**: Used for vehicle dynamics simulation, allowing for realistic co-simulations with Simulink.

## Results and Analysis

Our experimental results indicate that the proposed RPNN+NMPC controller:

- Achieves more accurate path-tracking performance with only a small dataset, thanks to the high adaptability of RPNN.
- Reduces mean lateral error by 48% compared to NMPC alone, highlighting the advantages of integrating neural network-based compensation.
- Demonstrates reduced computational burden, validated through real-time testing on an Intel i7 CPU, with a computational time ratio lower than traditional approaches.

![Path Tracking Trajectory](RPNN_NMPC/images/images/path_tracking_trajectory.png)

The above figure shows the path tracking trajectory achieved by the proposed control system during the double lane change (DLC) scenario.

### Citation
@inproceedings{Guo2024VTC,
  author = {Fangrui Guo and Quan Zhang},
  title = {A Novel Approach for Integrated Vehicle Dynamics Management Using Improved Adaptive Neural Network},
  booktitle = {Proceedings of the 2024 Vehicular Technology Conference (VTC)},
  year = {2024},
  organization = {IEEE}
}
