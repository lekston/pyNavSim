% TODO Prepare a model that accounts for the dynamics of rolling motion
%       Reflect logitudinal and lateral crosscouplings and make use of them, e.g.:
%           - allow for a tight climbing turn
%           - use rudder to adjust pitch when banked (indirectly affecting airspeed)
%
% XXX  Does L1 account for roll dynamics?

figure(1);
% Prolate cycloids
alpha = linspace(0,4*3.1415,400);
a = 3;  % windspeed
b = 5;  % airspeed (1st case)
x = a*alpha - b*sin(alpha);     % a * 2*pi
y = a - b*cos(alpha);

x2 = a*alpha - 2*b*sin(alpha);
y2 = a - 2*b*cos(alpha);
plot(x,y,'b',x2,y2,'g')
% Curves depict same rate of turn for different speeds (hence bank angles are different)
% in pure Prolate cycloids the circle does not slip. However, if the turn rate of the aircraft
% is not perfectly adjusted to the a/b ratio, then we get a sliping cycloid...

% x = [ V_wind / ksi_dot ] * (ksi_dot * dt) - [ V_plane / ksi_dot ] * sin(ksi_dot * dt)

figure(2);
% XXX 2nd derivative of yaw -> what does it correspond to?
% 
% a_n = yaw_dot^2 * R -> effective turn rate at max. bank angle (in level flight)
%
% a_n = g * tan (phi)
% from the above we compute the jerk
% a_n_dot = g * ( phi_dot/(cos(phi)^2) )
% by flight dynamic notations: phi_dot = p (roll rate)
% a_n_dot = g * ( p / (cos(phi)^2 )
% XXX assume p is a sigmoid
%
% a_n_ddot = g * ( phi_ddot * cos(phi)^2 - phi_dot * 2 * cos(phi) * (-sin(phi)) ) / (cos(phi)^4)
% a_n_ddot = g * ( phi_ddot * cos(phi) + 2 * phi_dot * sin(phi) ) / (cos(phi)^3) 
% by flight dynamic notations: phi_ddot = p_dot (derivative of roll rate)
% a_n_ddot = g * ( p_dot * cos(phi) + 2 * p * sin(phi) ) / (cos(phi)^3)

x = linspace(0,3.1415, 1400);

% Calculate actual track when turning with variable bank
%   The main idea here is to depict turn entry
phi_max = pi/9;         % pi/9  rad = 20deg
phi = phi_max * sin(x); % pi/12 rad = 15deg
g = 9.81;
a_n = g * tan (phi);
a_n_ref = g * tan(phi_max);

v_ref = 10;

%v_x_circle = v_ref*sin(x);
%v_y_circle = v_ref*cos(x);

radius = v_ref^2 / a_n_ref;

x_circle = radius-radius*cos(x);
y_circle = radius*sin(x);

idx_max = length(x);
dt = 0.01;
T_max = idx_max*dt;

v_x_sim = zeros(idx_max);
v_y_sim = zeros(idx_max);

v_x_sim(1) = 0;     % m/s
v_y_sim(1) = v_ref; % m/s
v_x_sim(2) = 0;     % m/s
v_y_sim(2) = v_ref; % m/s

x_sim = zeros(idx_max);
y_sim = zeros(idx_max);

% use integration method from the Earth-Moon sim

for i = 3:idx_max;
    n_x = v_y_sim(i-1); % (positive to the right of track) 
    n_y = -v_x_sim(i-1);

    n_length = sqrt(n_x^2 + n_y^2);

    % normalize
    n_x /= n_length;
    n_y /= n_length;

    v_x_sim(i) = v_x_sim(i-1) + a_n(i) * n_x * dt;
    v_y_sim(i) = v_y_sim(i-1) + a_n(i) * n_y * dt;

    recalc_v = true;
    if (recalc_v) % recalculate the speed estimate using updated normal vector

        n_x = v_y_sim(i); % (positive to the right of track) 
        n_y = -v_x_sim(i);

        n_length = sqrt(n_x^2 + n_y^2);

        % normalize
        n_x /= n_length;
        n_y /= n_length;

        v_x_sim(i) = v_x_sim(i-1) + a_n(i) * n_x * dt;
        v_y_sim(i) = v_y_sim(i-1) + a_n(i) * n_y * dt;
    endif

    x_sim(i-1) = x_sim(i-2) + 0.5*(v_x_sim(i) + v_x_sim(i-1)) * dt;
    y_sim(i-1) = y_sim(i-2) + 0.5*(v_y_sim(i) + v_y_sim(i-1)) * dt;
endfor

x_sim(idx_max) = x_sim(idx_max-1);
y_sim(idx_max) = y_sim(idx_max-1);

% TODO: Assume the following roll dynamics model:
%   - constant angular acceleration phase: p_dot up to p_max
%   - constant angular rotation phase: p_max
%   - constant angular decceleration phase: -p_dot down to 0
%
% Time to roll during accel/deccel phases:
% phi_err / 2 = p_dot * t^2 / 2;
% p_dot * t_xover = p_max;
%
% phi_err = phi_err_xover + phi_err_cont
%
% When angle error value surpasses the crossover value, the phi_err(t) relation becomes linear.
% Hence, relation becomes linear whenever phi_err > phi_err_xover
% phi_err_xover = p_dot * (t_xover)^2 = ??? p_max * t_xover;
% phi_err_cont = p_max * t_cont

if (false)
    p_dot = p_max/t_xover;

    if (phi_err < phi_err_xover)
        % non-linear:
        t = sqrt(phi_err/p_dot);
    else
        % linear:
        phi_err_cont = phi_err - phi_err_xover;
        t = t_xover + phi_err_cont / p_max;
    endif
endif

plot(x_sim,y_sim,'b',x_circle,y_circle,'r')
