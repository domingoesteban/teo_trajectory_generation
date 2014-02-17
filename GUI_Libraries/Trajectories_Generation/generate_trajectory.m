function [traj d_traj dd_traj]=generate_trajectory(traj, d_traj, dd_traj, trajectory_points, total_points, SETTINGS_TEO)

%Author: Domingo Esteban
ZERO_POS = zeros(3,1);
ZERO_ORIENT = zeros(3,1);

global ppos dppos ddppos position0 position1 porient dporient ddporient orientation0 orientation1 traj_stretch d_traj_stretch dd_traj_stretch
    for jj=1:(length(SETTINGS_TEO.humanoid_fields)-1)
        for ii=1:total_points
            position0 = set_trajectory_condition(trajectory_points.t0_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii),trajectory_points.initial_point.(SETTINGS_TEO.humanoid_fields(jj).name)(1:3,ii),ZERO_POS, ZERO_POS);
            orientation0 = set_trajectory_condition(trajectory_points.t0_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii),trajectory_points.initial_point.(SETTINGS_TEO.humanoid_fields(jj).name)(4:6,ii),ZERO_ORIENT, ZERO_ORIENT);
            if strcmp(trajectory_points.time_diff.(SETTINGS_TEO.humanoid_fields(jj).name){ii},'Abs')
                T1 = trajectory_points.T_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii);
            elseif strcmp(trajectory_points.time_diff.(SETTINGS_TEO.humanoid_fields(jj).name){ii},'Diff')
                T1 = trajectory_points.T_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii)+trajectory_points.t0_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii);
            else
                disp('Wrong diff/abs time variation')
            end
            if strcmp(trajectory_points.pos_diff.(SETTINGS_TEO.humanoid_fields(jj).name){ii},'Abs')
                P1 = trajectory_points.final_point.(SETTINGS_TEO.humanoid_fields(jj).name)(1:3,ii);
            elseif strcmp(trajectory_points.pos_diff.(SETTINGS_TEO.humanoid_fields(jj).name){ii},'Diff')
                P1 = trajectory_points.final_point.(SETTINGS_TEO.humanoid_fields(jj).name)(1:3,ii)+trajectory_points.initial_point.(SETTINGS_TEO.humanoid_fields(jj).name)(1:3,ii);
            else
                disp('Wrong diff/abs position variation')
            end
            if strcmp(trajectory_points.orient_diff.(SETTINGS_TEO.humanoid_fields(jj).name){ii},'Abs')
                O1 = trajectory_points.final_point.(SETTINGS_TEO.humanoid_fields(jj).name)(4:6,ii);
            elseif strcmp(trajectory_points.orient_diff.(SETTINGS_TEO.humanoid_fields(jj).name){ii},'Diff')
                O1 = trajectory_points.final_point.(SETTINGS_TEO.humanoid_fields(jj).name)(4:6,ii)+trajectory_points.initial_point.(SETTINGS_TEO.humanoid_fields(jj).name)(4:6,ii);
            else
                disp('Wrong diff/abs orientation variation')
            end
            position1 = set_trajectory_condition(T1,P1,ZERO_POS,ZERO_POS);
            orientation1 = set_trajectory_condition(T1,O1,ZERO_ORIENT,ZERO_ORIENT);
            switch trajectory_points.interpola_pos.(SETTINGS_TEO.humanoid_fields(jj).name){ii}
                case 'Linear'
                    [ppos, dppos, ddppos] = lineartrajectory(position0, position1, trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii));
                case 'Spline'
                    [ppos, dppos, ddppos] = spline_interpolation (position0, position1, trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii));   
                case 'Polynomial3'
                    [ppos, dppos, ddppos] = poly3_trajectory (position0, position1, trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii));   
                case 'Cubic Spline'
%                     [ppos, dppos, ddppos,tt] = cubic_spline_trajectory (position0, position1, trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii), [1 1 1]);
%                     % Define trajectory structures
%                     ppos    = create_trajectory_structure(ppos,   trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii), tt);
% %                     dppos   = create_trajectory_structure(dppos,  trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii), tt);
% %                     ddppos  = create_trajectory_structure(ddppos, trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii), tt);                       
                case 'Polynomial5'
                    [ppos, dppos, ddppos] = poly5_trajectory (position0, position1, trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii));   
                case 'Polynomial7'
                    [ppos, dppos, ddppos] = poly7_trajectory (position0, position1, trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii));   
            end
            switch trajectory_points.interpola_orient.(SETTINGS_TEO.humanoid_fields(jj).name){ii}
                case 'Linear'
                    [porient, dporient, ddporient] = lineartrajectory(orientation0, orientation1, trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii));
                case 'Spline'
                    [porient, dporient, ddporient] = spline_interpolation(orientation0, orientation1, trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii));
                case 'Polynomial3'
                    [porient, dporient, ddporient] = poly3_trajectory(orientation0, orientation1, trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii));
                case 'Cubic Spline'
%                     [porient, dporient, ddporient] = cubic_spline_trajectory(orientation0, orientation1, trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii), [1 1 1]);
%                     % Define trajectory structures
%                     porient    = create_trajectory_structure(porient,   trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii), tt);
% %                     dporient   = create_trajectory_structure(dporient,  trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii), tt);
% %                     ddporient  = create_trajectory_structure(ddporient, trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii), tt);
                case 'Polynomial5'
                    [porient, dporient, ddporient] = poly5_trajectory(orientation0, orientation1, trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii));
                case 'Polynomial7'
                    [porient, dporient, ddporient] = poly7_trajectory(orientation0, orientation1, trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii));
            end

            traj_stretch = combine_structures(ppos,porient);
            d_traj_stretch =  combine_structures(dppos,dporient);
            dd_traj_stretch = combine_structures(ddppos,ddporient);
            
            traj   = insert_trajectory(traj,   SETTINGS_TEO.humanoid_fields, traj_stretch,  SETTINGS_TEO.humanoid_fields(jj).name);
            d_traj = insert_trajectory(d_traj, SETTINGS_TEO.humanoid_fields, d_traj_stretch, SETTINGS_TEO.humanoid_fields(jj).name);
            dd_traj = insert_trajectory(dd_traj, SETTINGS_TEO.humanoid_fields, dd_traj_stretch, SETTINGS_TEO.humanoid_fields(jj).name);
            
            %Create support foot values with CoM values, and insert them
            %into traj, d_traj and dd_traj structures
            if jj==1
                values=ones(1,size(traj_stretch.time,2))*trajectory_points.support_foot.(SETTINGS_TEO.humanoid_fields(jj).name)(ii);
                support_foot_stretch = create_trajectory_structure(values,   trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii), traj_stretch.time);
                traj   = insert_trajectory(traj,   SETTINGS_TEO.humanoid_fields, support_foot_stretch ,  'SF');
                d_traj = insert_trajectory(d_traj, SETTINGS_TEO.humanoid_fields, support_foot_stretch , 'SF');
                dd_traj = insert_trajectory(dd_traj, SETTINGS_TEO.humanoid_fields, support_foot_stretch , 'SF');
            end
            
        end
        
%         trajectory_points.initial_point.(SETTINGS_TEO.humanoid_fields(jj).name)(:,end) = trajectory_points.final_point.(SETTINGS_TEO.humanoid_fields(jj).name)(:,end);
%         trajectory_points.final_point.(SETTINGS_TEO.humanoid_fields(jj).name)(:,end) = zeros(SETTINGS_TEO.humanoid_fields(jj).size, 1);
%         trajectory_points.t0_val.(SETTINGS_TEO.humanoid_fields(jj).name)(end+1);
%         trajectory_points.T_val.(SETTINGS_TEO.humanoid_fields(jj).name)(end+1);
%         trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name)(end+1) = SETTINGS_TEO.parameters.Ts;
%         trajectory_points.pos_diff.(SETTINGS_TEO.humanoid_fields(jj).name){end+1} = 'Diff';
%         trajectory_points.orient_diff.(SETTINGS_TEO.humanoid_fields(jj).name){end+1} = 'Diff';
%         trajectory_points.time_diff.(SETTINGS_TEO.humanoid_fields(jj).name){end+1} = 'Diff';
%         trajectory_points.interpola_pos.(SETTINGS_TEO.humanoid_fields(jj).name){end+1} = 'Linear';
%         trajectory_points.interpola_orient.(SETTINGS_TEO.humanoid_fields(jj).name){end+1} = 'Linear';    
    end

end

function structure = combine_structures(pos_struc,orient_struct)

if (length(pos_struc.data)~=length(orient_struct.data))
    error('CREATE_TRAJECTORY_STRUCTURE:argChk', 'Orientation and position do not have same length data')
end

structure.data = [pos_struc.data;orient_struct.data];
structure.Ts   = pos_struc.Ts;
structure.time = pos_struc.time;
structure.T    = pos_struc.T;


end
