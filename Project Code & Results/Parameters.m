% Blaine Swieder, Dakota Moye
% December 3rd, 2025
% Filename: Parameters.m
% The purpose of this file is show common parameters and plant definition
% for our rocket pitch control project.
% -----------------------------------------------------------------------

% -----------------------------------------------------------------------

% Our Physical / Model Parameters 

a = 2.0; % This is the stiffness parameter.
b = 1.0; % Damping Parameter
g = 1.0; % Thrust-Vector Control (TVC) Gain (Torque per gimbal command)
Iy = 1.0; % Pitch Inertia (normalized)

% ----------------------------------------------------------------------
% Actuators / Reference Settings

u_max = 5 * pi/180; % Actuator Limit[rad] (±5 deg)
r_step_nom = 2 * pi/180; % Nominal Reference Step [rad] (2 deg) 
r_step_big = 8 * pi/180; % Larger Step [rad] for saturation test (8 deg)

% ----------------------------------------------------------------------
% Disturbance Settings 

d_mag = 0.05; % Disturbance Torque Magnitude (Normalized) 

% ----------------------------------------------------------------------
% Simulation Time Paramters

tspan = [0 10];  % Simulates 0-10 s

% ----------------------------------------------------------------------
% Augmented State-Space Model (theta, q, xi)

% x_a = [theta; q; xi], where xi is an integral of (r - theta)

Aa = [0 1 0; -a -b 0; -1 0 0];
Ba = [0; g; 0];
Ca = [1 0 0];
Da = 0; 


  
 

