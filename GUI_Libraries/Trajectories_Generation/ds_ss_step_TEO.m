function [q, dq, ddq, trajectory, d_trajectory, dd_trajectory] = ds_ss_step_TEO(delta, data, leg)
                      
%DS_SS_STEP_TEO Double Support Step for the robot TEO
%   [Q, DQ, DDQ, TRAJECTORY, D_TRAJECTORY, DD_TRAJECTORY] =
%   DS_SS_STEP_TEP(DELTA, DATA, LEG)
%   returns the joint trajectory, joint velocity, joint acceleration,
%   operational space trajectory, operational space velocity and 
%   acceleration for a double support and simple support step.
%
%   INPUT:
%       DELTA = 
%       DATA = 
%       LEG = 
%
%   See also MOVE_DOUBLE_SUPPORT, MOVE_SIMPLE_SUPPORT.

%   Author: Domingo Esteban
%   References from: P. Pierro and D. García-Cano
%   RoboticsLab - Universidad Carlos III de Madrid
%   $Revision: 1.0 $  $Date: 2013/07/28 $
% *************************************************************************

                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%% DS_SS_STEP_TEO FUNCTION %%%
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global h TEO pvia 

% INPUTS
    % Steps Data
    L = data.L;     % Step Length
    H = data.H;     % Step Height
    q0 = data.q0;   % Initial pose
    
    % Time Parameters
    Ts = data.TS;   % Sampling time
    t0 = data.t0;   % Initial time
    T = data.T;     % Total time. Time of the step

    % Variation of CoM and FF in Simple Support Phase
    delta_ss_com = [delta.delta_CoM_SS1 delta.delta_CoM_SS2];
    delta_ss_ff = [delta.delta_FF_SS1 delta.delta_FF_SS2];

    % Variation of Right Hand and Left Hand in Simple Support Phase
    delta_RH = delta.delta_RH;
    delta_LH = delta.delta_LH;

    
% CREATE TRAJECTORY
    % Fields corresponding to the operational space of TEO
    TEO_fields = humanoid_operational_fields(); 

    % Creates the trajectories structures
    trajectory = create_trajectory_template (TEO_fields, Ts);
    d_trajectory = create_trajectory_template (TEO_fields, Ts);
    dd_trajectory = create_trajectory_template (TEO_fields, Ts);


switch data.DS_or_SS
    case 'Double and Simple' % Double Support followed by Simple Support movement
        % Calculate the time of step's phases
        alpha_ds = data.alpha_ds;   % Percentage of time for double support
        alpha_sf = data.alpha_sf;   % Percentage of time for SF ????
        T_ds = round_to_Ts(alpha_ds * T, Ts);       % Total time for Double Support
        T_ss = round_to_Ts((1-alpha_ds) * T, Ts);   % Total time for Simple Support

        % Variation of CoM in Double Support phase
        delta_ds_com = delta.delta_CoM_DS;

        switch leg %Select the support leg for the Simple Support phase
            case 'Right Leg' % Right Leg
                
                % Insert Initial operational space positions in the
                % trajectory
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(zeros(6,1), Ts, t0), 'RF');%(pose_quat2rpy(h.CoM_T_RF(q0)), Ts, t0), 'RF');
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(zeros(6,1), Ts, t0), 'LF');%(pose_quat2rpy(h.CoM_T_LF(q0)), Ts, t0), 'LF');
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(zeros(6,1), Ts, t0), 'RH');%(pose_quat2rpy(h.CoM_T_RH(q0)), Ts, t0), 'RH');
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(zeros(6,1), Ts, t0), 'LH');%(pose_quat2rpy(h.CoM_T_LH(q0)), Ts, t0), 'LH');

                % Generate trajectory for the double support phase
                [trajectory, d_trajectory, dd_trajectory] = move_double_support (delta_ds_com, Ts, [t0;T_ds], trajectory, d_trajectory, dd_trajectory,delta.interpola_CoM_DS);
                % Generate trajectory for the simple support phase
                [trajectory, d_trajectory, dd_trajectory] = move_simple_support (delta_ss_com, delta_ss_ff, Ts, [T_ds;T_ds+T_ss/2;T], trajectory, d_trajectory, dd_trajectory, 'Simple Support RF',delta.interpola_CoM_SS,delta.interpola_FF_SS); % Support on Right foot
%                 
%                 % Next STEP
%                 delta_ds_com2=-delta_ds_com;
%                 [trajectory, d_trajectory, dd_trajectory] = move_double_support_next (delta_ds_com2, Ts, [T;T+T_ds], trajectory, d_trajectory, dd_trajectory,delta.interpola_CoM_DS);
                               
                % Generate trajectory for the arms
                %[trajectory, d_trajectory, dd_trajectory] = moving_arm ('RH',delta_RH,zeros(6,1),zeros(6,1),Ts, [t0 T], trajectory, d_trajectory, dd_trajectory,delta.interpola_RH);
                %[trajectory, d_trajectory, dd_trajectory] = moving_arm ('LH',delta_LH,zeros(6,1),zeros(6,1),Ts, [t0 T], trajectory, d_trajectory, dd_trajectory,delta.interpola_LH);

                % Inverse Differential Kinematics Algorithm with right foot
                % as support
                [q, dq, ddq] = inverse_right_ds_ss_TEO(q0, trajectory, d_trajectory, dd_trajectory, h);

            case 'Left Leg' % Left Leg
                
                % Insert Initial operational space positions in the
                % trajectory
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(pose_quat2rpy(h.CoM_T_RF(q0)), Ts, t0), 'RF');
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(pose_quat2rpy(h.CoM_T_LF(q0)), Ts, t0), 'LF');
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(pose_quat2rpy(h.CoM_T_RH(q0)), Ts, t0), 'RH');
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(pose_quat2rpy(h.CoM_T_LH(q0)), Ts, t0), 'LH');

                % Generate trajectory for the double support phase
                [trajectory, d_trajectory, dd_trajectory] = move_double_support (delta_ds_com, Ts, [t0;T_ds], trajectory, d_trajectory, dd_trajectory,delta.interpola_CoM_DS);

                % Generate trajectory for the simple support phase
                [trajectory, d_trajectory, dd_trajectory] = move_simple_support (delta_ss_com, delta_ss_ff, Ts, [T_ds;T_ds+T_ss/2;T], trajectory, d_trajectory, dd_trajectory,'Simple Support LF',delta.interpola_CoM_SS,delta.interpola_FF_SS); 

                % Generate trajectory for the arms
%                 [trajectory, d_trajectory, dd_trajectory] = moving_arm ('RH',delta_RH,zeros(6,1),zeros(6,1),Ts, [t0 T], trajectory, d_trajectory, dd_trajectory,delta.interpola_RH);
%                 [trajectory, d_trajectory, dd_trajectory] = moving_arm ('LH',delta_LH,zeros(6,1),zeros(6,1),Ts, [t0 T], trajectory, d_trajectory, dd_trajectory,delta.interpola_LH);

                % Inverse Differential Kinematics Algorithm with left foot
                % as support
                [q, dq, ddq] = inverse_left_ds_ss_TEO(q0, trajectory, d_trajectory, dd_trajectory, h);
        end
        
    case 'Simple' % Only Simple Support movement
        
        alpha_ds = 0;
        alpha_sf = data.alpha_sf;
        T_ds = 0;
        T_ss = T;

        delta_ds_com = zeros(6,1);
        
        switch leg % In Simple Support

            case 'Right Leg' % Right Leg

                % Initial positions
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(pose_quat2rpy(h.CoM_T_RF(q0)), Ts, t0), 'RF');
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(pose_quat2rpy(h.CoM_T_LF(q0)), Ts, 0), 'LF');
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(pose_quat2rpy(h.CoM_T_RH(q0)), Ts, t0), 'RH');
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(pose_quat2rpy(h.CoM_T_LH(q0)), Ts, t0), 'LH');

                [trajectory, d_trajectory, dd_trajectory] = move_simple_support (delta_ss_com, delta_ss_ff, Ts, [T_ds;T_ds+T_ss/2;T], trajectory, d_trajectory, dd_trajectory, 'Simple Support RF',delta.interpola_CoM_SS,delta.interpola_FF_SS); % Support on Right foot
                [trajectory, d_trajectory, dd_trajectory] = moving_arm ('RH',delta_RH,zeros(6,1),zeros(6,1),Ts, [t0 T], trajectory, d_trajectory, dd_trajectory,delta.interpola_RH);
                [trajectory, d_trajectory, dd_trajectory] = moving_arm ('LH',delta_LH,zeros(6,1),zeros(6,1),Ts, [t0 T], trajectory, d_trajectory, dd_trajectory,delta.interpola_LH);

                [q, dq, ddq] = inverse_right_ss_TEO(q0, trajectory, d_trajectory, dd_trajectory, h);

            case 'Left Leg' % Left Leg

                % Initial positions

                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(pose_quat2rpy(h.CoM_T_LF(q0)), Ts, t0), 'LF');% esto pa ver una cosa
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(pose_quat2rpy(h.CoM_T_RH(q0)), Ts, t0), 'RH');
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(pose_quat2rpy(h.CoM_T_LH(q0)), Ts, t0), 'LH');

                [trajectory, d_trajectory, dd_trajectory] = move_simple_support (delta_ss_com, delta_ss_ff, Ts, [T_ds;T_ds+T_ss/2;T], trajectory, d_trajectory, dd_trajectory,'Simple Support LF',delta.interpola_CoM_SS,delta.interpola_FF_SS); 

                [trajectory, d_trajectory, dd_trajectory] = moving_arm ('RH',delta_RH,zeros(6,1),zeros(6,1),Ts, [t0 T], trajectory, d_trajectory, dd_trajectory,delta.interpola_RH);
                [trajectory, d_trajectory, dd_trajectory] = moving_arm ('LH',delta_LH,zeros(6,1),zeros(6,1),Ts, [t0 T], trajectory, d_trajectory, dd_trajectory,delta.interpola_LH);

                [q, dq, ddq] = inverse_left_ss_TEO(q0, trajectory, d_trajectory, dd_trajectory, h);


        end
        
end

