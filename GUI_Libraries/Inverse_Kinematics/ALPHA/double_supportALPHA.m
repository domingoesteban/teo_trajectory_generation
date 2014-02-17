function [q_right, q_dot_right, p_right, R_right, q_left, q_dot_left, p_left, R_left] = double_supportALPHA (T, Ts, q0_right_r, q0_left_r, delta_right, Rd0_right, delta_left, Rd0_left)
% inverse kinematics settings
Kp=2e2*eye(3);
Ko=5e0*eye(3);

%***********************
% supporting right leg *
%***********************
delta_x_right = delta_right(1);
delta_y_right = delta_right(2);
delta_z_right = delta_right(3);

%pass from robot reference to matlab reference
[Rr_m2r, Rl_m2r] = rotate_matrix();
q0_right=Rr_m2r'*q0_right_r;

% initial conditions
p0_right = evaluate_position_right (q0_right);              %initial position of the right supporting leg

% determine the desired trajectory for the supporting right leg 
[xd_right, xd_dot_right] = trajectory_double_soporte (T, Ts, p0_right, delta_x_right, delta_y_right, delta_z_right);

% determine the desired orientations for the supporting right leg 
global Rd_right wd_right
[Rd_right, wd_right] = desired_orientations (T, Ts, Rd0_right);

% inverse kinematics
[q_right_m, q_dot_right_m, p_right, R_right, error_p, error_o] = inverse_kinematics_rightALPHA (Ts, T, q0_right, Kp, Ko, xd_right, xd_dot_right, Rd_right, wd_right);

q_right = Rr_m2r*q_right_m;
q_dot_right = Rr_m2r*q_dot_right_m;

%********************
% floating left leg *
%********************
delta_x_left = delta_left(1);
delta_y_left = delta_left(2);
delta_z_left = delta_left(3);

%pass from robot reference to matlab reference
q0_left=Rl_m2r'*q0_left_r;

% initial conditions
R0_left = evaluate_orientation_left (q0_left);
p0_left = evaluate_position_left (q0_left);              %initial position and orientation of the left floating leg

% determine the desired trajectory for the floating left leg 
[xd_left, xd_dot_left] = trajectory_double_soporte (T, Ts, p0_left, delta_x_left, delta_y_left, delta_z_left);

% determine the desired orientations for the supporting right leg 
[Rd_left, wd_left] = desired_orientations (T, Ts, Rd0_left);

% inverse kinematics
[q_left_m, q_dot_left_m, p_left, R_left, error_p, error_o] = inverse_kinematics_leftALPHA (Ts, T, q0_left, Kp, Ko, xd_left, xd_dot_left, Rd_left, wd_left);

q_left = Rl_m2r*q_left_m;
q_dot_left = Rl_m2r*q_dot_left_m;

%%%%%%%%%%%%%
% FUNCTIONS %
%%%%%%%%%%%%%

%**********************************
% trajectory double support phase *
%**********************************
function [xd, xd_dot] = trajectory_double_soporte (T, Ts, p0, delta_x, delta_y, delta_z)
time=0:Ts:T-Ts;
% frontal plane
% x axis
xd(1,:) = interpolation (T, time, p0(1), p0(1)+delta_x, 0, 0);

% sagital plane
% y axis
xd(2,:) = interpolation (T, time, p0(2), p0(2)+delta_y, 0, 0);

% z axis
xd(3,:) = interpolation (T, time, p0(3), p0(3)+delta_z, 0, 0);

xd_dot = [zeros(3,1),diff(xd,1,2)/Ts];


function [R_right, R_left] = rotate_matrix()
R_right =   [   0, 0, 0, 0, 0, 1;
                0, 0, 0, 0,-1, 0;
                0, 0, 0, 1, 0, 0;
                0, 0,-1, 0, 0, 0;
                0,-1, 0, 0, 0, 0;
                1, 0, 0, 0, 0, 0];

R_left =    [   1, 0, 0, 0, 0, 0;
                0, 1, 0, 0, 0, 0;
                0, 0, 1, 0, 0, 0;
                0, 0, 0,-1, 0, 0;
                0, 0, 0, 0,-1, 0;
                0, 0, 0, 0, 0,-1];