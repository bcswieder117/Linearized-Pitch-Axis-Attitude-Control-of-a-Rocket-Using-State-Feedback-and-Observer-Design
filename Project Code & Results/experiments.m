% Name(s): Blaine Swieder; Dakota Moye
% Date: December 3rd, 2025
% Filename: experiments.m
% Purpose: Run four separate experiments for the rocket pitch control project: 
% (1) Step Tracking (nominal) 
% (2) Actuator Saturation (large step) 
% (3) Disturbance Rejection
% (4) +/- 10% Parameter Variation (robustness) 
% -----------------------------------------------------------------------

clear; clc; close all; 

% We need to load our parameters and design controllers

Parameters; 
design_controllers; % Gives K_pp and K_lqr 

% Intitial Augmented State [theta; q; xi]

x0 = [0; 0; 0]; 

%------------------------------------------------------------------------
% Experiment I: Step Tracking (Nominal Parameters) 
% -----------------------------------------------------------------------

r_step = r_step_nom; 
use_dist = false;

% Pole-Placement Controller

[t_pp, x_pp] = ode45(@(t,x) rocket_cl_ode(t, x, a, b, g, Iy, K_pp,  u_max, r_step, d_mag, use_dist), ...
                     tspan, x0);

% LQR Controller

[t_lqr, x_lqr] = ode45(@(t,x) rocket_cl_ode(t, x, a, b, g, Iy, K_lqr, u_max, r_step, d_mag, use_dist), ...
                       tspan, x0);

theta_pp = x_pp(:,1); 
theta_lqr = x_lqr(:,1);

% Figure 1 MATLAB Code

figure; 
plot(t_pp,  theta_pp*180/pi, ...
     t_lqr, theta_lqr*180/pi, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('\theta [deg]');
title('Experiment 1: Step Tracking (Nominal)');
legend('Pole-Placement', 'LQR', 'Location', 'Best'); 
grid on; 


% -----------------------------------------------------------------------



% -----------------------------------------------------------------------
% Experiment 2: Actuator Saturation (larger step)
% -----------------------------------------------------------------------

r_step = r_step_big; % Larger Step to induce saturation
use_dist = false; 

[t_pp2, x_pp2] = ode45(@(t,x) rocket_cl_ode(t, x, a, b, g, Iy, K_pp,  u_max, r_step, d_mag, use_dist), ...
                       tspan, x0);
[t_lqr2, x_lqr2] = ode45(@(t,x) rocket_cl_ode(t, x, a, b, g, Iy, K_lqr, u_max, r_step, d_mag, use_dist), ...
                         tspan, x0);

theta_pp2 = x_pp2(:,1); 
theta_lqr2 = x_lqr2(:,1);

% Recalculate control histories for plotting u(t) 

u_pp2 = zeros(size(t_pp2));
u_lqr2 = zeros(size(t_lqr2));
for k = 1:length(t_pp2)
    xk = x_pp2(k,:)';
    u_lqr2(k) = max(min(-K_pp * xk, u_max), -u_max);
end 

for k = 1:length(t_lqr2)
    xk = x_lqr2(k,:)';
    u_lqr2(k) = max(min(-K_lqr * xk, u_max), -u_max);
end

% Graph Experiment 2

figure; 
subplot(2,1,1);
plot(t_pp2,  theta_pp2*180/pi, ...
     t_lqr2, theta_lqr2*180/pi, 'LineWidth', 1.5);
ylabel('\theta [deg]');
title('Experiment 2: Actuator Saturation (Larger Step)');
legend('Pole-Placement', 'LQR', 'Location', 'Best'); 
grid on; 

subplot(2,1,2); 
plot(t_pp2,  theta_pp2*180/pi, ...
     t_lqr2, theta_lqr2*180/pi, 'LineWidth', 1.5);
xlabel('Time [s]'); 
ylabel('u [deg]');
grid on;

% -----------------------------------------------------------------------



% -----------------------------------------------------------------------
% Experiment 3: Disturbance Rejection
% -----------------------------------------------------------------------

r_step = 0; 
use_dist = true; 

[t_pp3, x_pp3] = ode45(@(t,x) rocket_cl_ode(t, x, a, b, g, Iy, K_pp,  u_max, r_step, d_mag, use_dist), ...
                       tspan, x0);
[t_lqr3, x_lqr3] = ode45(@(t,x) rocket_cl_ode(t, x, a, b, g, Iy, K_lqr, u_max, r_step, d_mag, use_dist), ...
                         tspan, x0);

theta_pp3 = x_pp3(:,1); 
theta_lqr3 = x_lqr3(:,1);

% Graph Experiment 3

figure; 
subplot(2,1,1);
plot(t_pp3,  theta_pp3*180/pi, ...
     t_lqr3, theta_lqr3*180/pi, 'LineWidth', 1.5);
ylabel('\theta [deg]');
title('Experiment 3: Disturbance Rejection');
legend('Pole-Placement', 'LQR', 'Location', 'Best'); 
grid on; 

% -----------------------------------------------------------------------



% -----------------------------------------------------------------------
% Experiment 4: +/- 10% Parameter Variation (robustness)
% -----------------------------------------------------------------------

% Use an LQR Controller as an example

r_step = r_step_nom; 
use_dist = false; 

% Nominal Run 

[t_nom, x_nom] = ode45(@(t,x) rocket_cl_ode(t, x, a, b, g, Iy, K_lqr, u_max, r_step, d_mag, use_dist), ...
                       tspan, x0);

theta_nom = x_nom(:,1); 

% Peturbed Parameters (+ 10% a, -10% b)

a_pert = 1.1 * a; 
b_pert = 0.9 * b; 
g_pert = g; % Can also perturb g if desired.

% Perturbed Run with modified parameters
[t_pert, x_pert] = ode45(@(t,x) rocket_cl_ode(t, x, a_pert, b_pert, g_pert, Iy, K_lqr, u_max, r_step, d_mag, use_dist), ...
                         tspan, x0);
theta_pert = x_pert(:,1);

% Graph Experiment 4

% Graph nominal and perturbed responses
figure; 
subplot(2,1,1);
plot(t_nom,  theta_nom*180/pi, ...
     t_pert, theta_pert*180/pi, 'LineWidth', 1.5);
ylabel('\theta [deg]');
title('Experiment 4: \pm10% Parameter Variation (LQR Controller)');
legend('Nominal', 'Perturbed', 'Location', 'Best'); 
grid on; 

% -----------------------------------------------------------------------