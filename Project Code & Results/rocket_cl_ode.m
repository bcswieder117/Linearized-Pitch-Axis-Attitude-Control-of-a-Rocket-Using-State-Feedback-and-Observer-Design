% Name(s): Blaine Swieder; Dakota Moye
% Date: December 3rd, 2025
% Filename: rocket_cl_ode.m
% Purpose: Closed-Loop Augmented Dynamics for Rocket Pitch Control

function dx = rocket_cl_ode(t, x, a, b, g, Iy, K, u_max, r_step, d_mag, use_dist)
% rocket_cl_ode  Closed-loop augmented dynamics for rocket pitch control.
%
%   x = [theta; q; xi]
%     theta : pitch angle [rad]
%     q     : pitch rate [rad/s]
%     xi    : integral of tracking error (r - theta)
%
%   u = -K * x, with saturation |u| <= u_max
%   Disturbance torque d(t) applied to pitch equation (optional).

    % Unpack states
    theta = x(1);
    q     = x(2);
    xi    = x(3); %#ok<NASGU> % (not used explicitly here, but part of x)

    % Reference: constant step in angle
    r = r_step;

    % Tracking error
    e = r - theta;

    % State-feedback control (unsaturated)
    u_unsat = -K * x;

    % Apply symmetric saturation
    u = max(min(u_unsat, u_max), -u_max);

    % Disturbance torque (optional)
    d = 0;
    if use_dist
        if t >= 5.0     % apply step disturbance after 5 seconds
            d = d_mag;
        end
    end

    % Dynamics of augmented system
    dx = zeros(3,1);
    dx(1) = q;
    dx(2) = -a*theta - b*q + g*u + d / Iy;
    dx(3) = e;         % xi_dot = r - theta

end

% ----------------------------------------------------------------------