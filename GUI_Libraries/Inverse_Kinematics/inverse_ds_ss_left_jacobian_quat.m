function [q, dq, ddq] = inverse_ds_ss_left_jacobian_quat(q0, trajectory, h)

%INVERSE_RIGHT_DS_SS_TEO Inverse Differential Kinematics using Jacobian with unit quaternion algorithm for a humanoid with
%left foot as support leg
%   [Q, DQ, DDQ] = 
%   INVERSE_LEFT_DS_SS_JACOBIAN_QUAT(Q0, TRAJECTORY, H)
%   returns the joint trajectory, joint velocity and joint acceleration for
%   the selected trajectory.
%
%   INPUT:
%       Q0 = Initial configuration
%       TRAJECTORY = Trajectory for the humanoid parts
%       H = humanoids equations
%
%   NOTE: This algorithm doesn't allow to control velocities neither accelerations
%   TODO: Return accelerations (ddq) considering velocities (dq) diff.
%   TODO2: It doesn't consider arms and CoM movement yet
%   See also INVERSE_RIGHT_DS_SS_JACOBIAN_QUAT.

%   Author: Domingo Esteban
%   References from: P. Pierro
%   RoboticsLab - Universidad Carlos III de Madrid
%   $Revision: 1.0 $  $Date: 2013/11/05 $
% *************************************************************************


%        ______      _           _   _            _           _     
%        | ___ \    | |         | | (_)          | |         | |    
%        | |_/ /___ | |__   ___ | |_ _  ___ ___  | |     __ _| |__  
%        |    // _ \| '_ \ / _ \| __| |/ __/ __| | |    / _` | '_ \ 
%        | |\ \ (_) | |_) | (_) | |_| | (__\__ \ | |___| (_| | |_) |
%        \_| \_\___/|_.__/ \___/ \__|_|\___|___/ \_____/\__,_|_.__/ 

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%     INVERSE_RIGHT_DS_SS_JACOBIAN_QUAT    %%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            
% Inverse Kinematics for Right Leg Support in Simple Support (Using
% Jacobian with unit quaternions)

% Parameters and Variables needed
    L = length(trajectory.time);
    q  = zeros(size(q0,1), L);
    dq = zeros(size(q0,1), L);
    ddq = zeros(size(q0,1), L); %TODO: Return accelerations (ddq) considering velocities (dq) diff.
    Ts = trajectory.Ts;

% Position and Orientation Gain
    Kp = 5*eye(3);
    Ko = 5*eye(3);
 
% Initial configuration
    q(:,1) = q0;

% Initial poses of the feet
    pose_SF_RF = trajectory.RF(:,1);
    pose_SF_LF = trajectory.LF(:,1);

% Errors Preallocation
    e_p_RF = zeros(3,L-1);
    e_o_RF = zeros(3,L-1);
    e_p_LF = zeros(3,L-1);
    e_o_LF = zeros(3,L-1);
    e_p_RH = zeros(3,L-1);
    e_o_RH = zeros(3,L-1);
    e_p_LH = zeros(3,L-1);
    e_o_LH = zeros(3,L-1);

for jj=1:L-1
    % Inverse differential kinematics for Legs
    switch trajectory.SF(jj)
        case 0      % double support
            % Errors signals
            [e_p_LF(:,jj) e_o_LF(:,jj)] = determine_error (trajectory.CoM(:,jj)-pose_quat2rpy(h.w_T_CoM(q(:,jj))),...
                   pose_SF_LF+pose_quat2rpy(h.LF_T_w(q(:,jj))));              
            [e_p_RF(:,jj) e_o_RF(:,jj)] = determine_error (trajectory.CoM(:,jj)-pose_quat2rpy(h.w_T_CoM(q(:,jj))),...
                   pose_SF_RF+pose_quat2rpy(h.RF_T_w(q(:,jj))));
            
            % Control signals
            u_L = [Kp*e_p_LF(:,jj); Ko*e_o_LF(:,jj)];
            u_R = [Kp*e_p_RF(:,jj); Ko*e_o_RF(:,jj)];
            
            % Joints velocities Output
            dq(1:6, jj+1) = invert_kinematics_standard (q(:,jj), h.RF_J_w, u_R, 1:6, 1:6);
            dq(7:12, jj+1) = invert_kinematics_standard (q(:,jj), h.LF_J_w, u_L, 1:6, 1:6);  
            
        case 1     % left foot support
            % Errors signals for support foot
            [e_p_LF(:,jj) e_o_LF(:,jj)] = determine_error (trajectory.CoM(:,jj)-pose_quat2rpy(h.w_T_CoM(q(:,jj))),...
                   pose_SF_LF+pose_quat2rpy(h.LF_T_w(q(:,jj))));

            % Control signals for support foot
            u_L = [Kp*e_p_LF(:,jj); Ko*e_o_LF(:,jj)];

            % Joints velocities Output for support leg
            dq(7:12, jj+1) = invert_kinematics_standard (q(:,jj), h.LF_J_w, u_L, 1:6, 1:6);
    end
    
    
    % Inverse differential kinematics other parts
%     TODO2: It doesn't consider arms and CoM movement yet
    
	% Errors signals for arms
    [e_p_RH(:,jj) e_o_RH(:,jj)] = determine_error (zeros(6,1), zeros(6,1));
    [e_p_LH(:,jj) e_o_LH(:,jj)] = determine_error (zeros(6,1), zeros(6,1));
    
    % Control signals for arms
    u_RH = [Kp*e_p_RH(:,jj); Ko*e_o_RH(:,jj)];
    u_LH = [Kp*e_p_LH(:,jj); Ko*e_o_LH(:,jj)];
    
    % Joints velocities Output for arms               
    dq(15:20, jj+1) = invert_kinematics_standard (q(:,jj), h.CoM_J_RH, u_RH, 1:3, 1:6);
    dq(21:26, jj+1) = invert_kinematics_standard (q(:,jj), h.CoM_J_LH, u_LH, 1:3, 1:6);
    
    
    
    % Joints positions Output for support legs and arms
    q(:, jj+1)   = integrate_vector (q(:, jj), dq(:, jj), Ts);
    
    
    
    % Algorithm for the floating leg
    if (trajectory.SF(jj)== 1) % right foot floating
        % Errors signals for floating foot
        [e_p_RF(:,jj) e_o_RF(:,jj)] = determine_error (trajectory.RF(:,jj),...
                (pose_SF_LF+pose_quat2rpy(h.LF_T_w(q(:,jj))))+pose_quat2rpy(h.w_T_RF(q(:,jj))));
            
        % Control signals for floating foot
        u_R = [Kp*e_p_RF(:,jj); Ko*e_o_RF(:,jj)];
        
        % Joints velocities and positions output for floating foot
        dq(1:6, jj+1) = invert_kinematics_standard (q(:,jj), h.w_J_RF, u_R, 1:6, 1:6);
        q(1:6, jj+1)   = integrate_vector (q(1:6, jj), dq(1:6, jj), Ts);
    end

end
end


function ddq = invert_kinematics_standard (q_act, J, e, m, n)
J1 = J(q_act);
ddq = J1(m, n)\e(m); %A\B matrix division of A into B. Same as INV(A)*B
end

function x_next = integrate_vector (x, dx, Ts)
x_next = x + dx * Ts;
end

function [error_p error_o] = determine_error (pd, p)
% Return position and orientation errors
desired = pose_rpy2quat(pd);
real = pose_rpy2quat(p);
eta = real(4);
eps = real(5:end);
eta_d = desired(4);
eps_d = desired(5:end);

error_p= desired(1:3) - real(1:3);
error_o = eta*eps_d - eta_d*eps - matrix_S(eps_d)*eps;
end

function S = matrix_S (w)
% Return skew vector
S = [       0,  -w(3),   w(2);
         w(3),      0,  -w(1);
        -w(2),   w(1),     0];
end