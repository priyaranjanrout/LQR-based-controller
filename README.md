# Eyantra Summer internship 2025 Project 13 

Overview
A comprehensive implementation of Linear Quadratic Regulator (LQR) controller for quadcopter control with suspended payload
This repository contains advanced control algorithms for quadcopter position control.
LQR Position Control: Optimal state-feedback controller for 3D positioning.

Analysis Tools
Scripts for logging and plotting trajectory data, velocity profiles, and control inputs.
Tools for visualizing error convergence, motor response, and overall system behavior.

Experimental Results
Tracking Error: ±3–5 cm for position under moderate conditions
Overshoot: <10% with appropriately tuned Q and R matrices
Responsiveness: Smooth control with minimal oscillation when tuned well

Troubleshooting
Drift or steady-state offset: Re-tune position and velocity weights in the Q matrix
Oscillations: Reduce Q values or increase R values to penalize aggressive control
Poor trajectory tracking: Check linearization accuracy and model fidelity
Unstable response: Ensure correct sign conventions in mixer matrix and input mapping


