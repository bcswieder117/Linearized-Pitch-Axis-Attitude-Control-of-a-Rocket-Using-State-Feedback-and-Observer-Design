% Blaine Swieder, Dakota Moye 
% December 3rd, 2025
% Filename: design_controllers.m
% Purpose: Will design pole-placement and LQR gains on the augmented
% 3-state system 
% -------------------------------------------------------------------

% Load in our parameters and augmented matrix

Parameters; 

% -------------------------------------------------------------------
% Pole-Placement Design on Augmented System 

zeta_cl = 0.8; 
wn_cl = 2.0; 

% The Desired Closed-Loop poles (complex pair + real pole for integrator)

p1 = -zeta_cl*wn_cl + 1i*wn_cl*sqrt(1 - zeta_cl^2); 
p2 = conj(p1); 
p3 = -4; 

% State-Feedback Gain for [theta; q; xi]

K_pp = place(Aa, Ba, [p1 p2 p3]); 

% --------------------------------------------------------------------
% LQR Design on Augmented System

% State Weighting: Penalize theta, q, and Integral State
Q = diag([10, 1, 5]);
R = 0.1;

% LQR State-Feedback Gain for [theta; q; xi] 

K_lqr = lqr(Aa, Ba, Q, R);


% --------------------------------------------------------------------

