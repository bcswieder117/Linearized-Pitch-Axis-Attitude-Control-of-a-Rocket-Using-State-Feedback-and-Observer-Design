# Linearized Pitch-Axis Attitude Control of a Rocket Using State Feedback and Observer Design

This will be a project of some sort dealing with rockets and/or satellites. The goal is to discuss the physical setup, system parameters, design criteria, and system equations. Will have a presentation and a short IEEE-styled paper. This was 25% of my course average, and I worked on this project with Dakota Moye (djmoye43@tntech.edu), but the project idea was my own. While I do not have the precise grade average for the project, I made an A in ECE 6200 so I assume that it was an A on the Final Project.

## Final Project for ECE 6200 @ TnTech


**Course Description**: State space analysis of multiple-input/multiple-output continuous and discrete-time systems; linear spaces; time-varying systems, controllability, observability, and stability.

**Abstract**: In this project, we study pitch-axis control for launch vehicles via a linear state-space model and thrust-vector control under small-angle derivations. We derive a two-state model (pitch angle, pitch rate) from rigid-body dynamics, linearized at an operating point, and expressed in standard form. Our Structural Analysis examines eigenstructure, open-loop stability, and controllability/observability. We design state feedback by pole placement and LQR, add a Luenberger observer to address limited measurements (separation principle), and include integral action when zero steady-state error is required. We also utilize MATLAB/simulink to simulate and compare open- and closed-loop behavior by using step commands, torque disturbances, parameter variations, and actuator limits. The controllers stabilize the dynamics, increase damping, reduce settling time, and bound control effort within the linearized envelope. We summarize trade-offs between pole placement and LQR and note observer bandwidth versus noise sensitivity, which gives a clear reproducible pipeline from modeling to validation for launch-vehicle attitude control.  
