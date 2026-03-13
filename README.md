# Linearized Pitch-Axis Attitude Control of a Rocket Using State Feedback and Observer Design

**Final Project — ECE 6200: Linear Systems Analysis**
Tennessee Technological University | Collaborator: Dakota Moye (djmoye43@tntech.edu)

## Course Description
State space analysis of multiple-input/multiple-output continuous and discrete-time systems; linear spaces; time-varying systems, controllability, observability, and stability.

## Abstract
This project examines pitch-axis attitude control for launch vehicles using a linear state-space model and thrust-vector control under small-angle assumptions. A two-state model (pitch angle, pitch rate) is derived from rigid-body dynamics, linearized at an operating point, and expressed in standard state-space form. Structural analysis covers eigenstructure, open-loop stability, and controllability/observability properties. Control designs include pole placement and LQR state feedback, a Luenberger observer to handle limited sensor measurements via the separation principle, and integral action for zero steady-state error. MATLAB/Simulink simulations compare open- and closed-loop behavior under step commands, torque disturbances, parameter variations, and actuator saturation. Results demonstrate stabilization, improved damping, reduced settling time, and bounded control effort within the linearized operating envelope. The project concludes with a discussion of trade-offs between pole placement and LQR, observer bandwidth versus noise sensitivity, and presents a reproducible modeling-to-validation pipeline for launch-vehicle attitude control.

## Deliverables
- IEEE-style conference paper
- Presentation slides
