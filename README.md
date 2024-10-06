### Running Simulations
The project includes several Simulink models and MATLAB scripts:

A Novel Approach for Integrated Vehicle Dynamics Management

This repository contains the implementation of a novel approach for autonomous vehicle path tracking and integrated vehicle dynamics management, as presented in our recent paper. This approach leverages a Nonlinear Model Predictive Control (NMPC) algorithm combined with an enhanced Random Projection Neural Network (RPNN) to address unknown nonlinearities in vehicle dynamics, achieving accurate path tracking with reduced computational burden.

Overview

The primary goal of this project is to develop an effective control system for autonomous vehicles that can ensure high tracking accuracy while managing unknown nonlinear uncertainties in lateral tire forces. We adopted an NMPC-based controller, integrated with an improved RPNN, to enhance adaptability and minimize input parameters for precise path tracking.

Key Features

Nonlinear Model Predictive Control (NMPC): Used for vehicle path tracking while addressing the nonlinearity in vehicle dynamics. NMPC is enhanced with a cost function designed to minimize input parameters.

Improved Random Projection Neural Network (RPNN): Utilizes a novel weight adaptation method, employing gradient descent from the input to the hidden layer, and an adaptive law based on Lyapunov stability theory from the hidden to output layer.

Real-time Simulation Results: The integrated NMPC and RPNN approach demonstrates superior path-tracking accuracy compared to traditional methods, reducing mean lateral error by over 48%.

System Model

The project utilizes a three-degree-of-freedom (3-DOF) vehicle dynamics model, representing the longitudinal, lateral, and yaw motions of the vehicle. This model, combined with the NMPC controller, forms the basis for ensuring stable and precise path tracking.

The proposed approach employs:

NMPC for higher-level control, managing the vehicle's path tracking while minimizing steering angle inputs.

RPNN for uncertainty compensation, specifically addressing the nonlinear uncertainties in lateral tire forces, ensuring system stability through adaptive learning based on Lyapunov stability theory.

Getting Started

Prerequisites

To run the simulations and implement the control system, you will need:

MATLAB/Simulink: The main environment used for modeling and simulation.

CarSim: Used for vehicle dynamics simulation, allowing for realistic co-simulations with Simulink. (2020 Version)

Installation

Clone the repository:

Install required software and ensure compatibility between CarSim, Simulink, and MATLAB.

Running Simulations

The project includes several Simulink models:

NMPC_casadi_yhn.m: The MATLAB script for the NMPC implementation using CasADi.

Nmpc_pathctl_casadi.slx: The Simulink model for the path tracking control using CasADi.

adapt_fy.m: The MATLAB script for adapting lateral forces.

data.mat and data_com.mat: Data files required for simulations.

plotfig.m: Script for plotting the simulation results.

Run the co-simulation in MATLAB/Simulink to test the control strategy under different road conditions and evaluate path-tracking performance.

Results and Analysis

Our experimental results indicate that the proposed RPNN+NMPC controller:

Achieves more accurate path-tracking performance with only a small dataset, thanks to the high adaptability of RPNN.

Reduces mean lateral error by 48% compared to NMPC alone, highlighting the advantages of integrating neural network-based compensation.

Demonstrates reduced computational burden, validated through real-time testing on an Intel i7 CPU, with a computational time ratio lower than traditional approaches.

Citation

If you use this repository or find our work helpful, please cite our paper:

License

This project is licensed under the MIT License. See the LICENSE file for details.


.
├── NMPC_casadi_yhn.m                # MATLAB script for NMPC implementation using CasADi
├── Nmpc_pathctl_casadi.slx          # Simulink model for path tracking control using CasADi
├── adapt_fy.m                       # MATLAB script for adapting lateral forces
├── data.mat                         # Data file required for simulations
├── data_com.mat                     # Additional data file for simulations
├── plotfig.m                        # Script for plotting simulation results
├── casadi-windows-matlabR2016a-v3.5.5/  # CasADi toolbox for MATLAB R2016a
└── slprj/                           # Simulink project-related files
