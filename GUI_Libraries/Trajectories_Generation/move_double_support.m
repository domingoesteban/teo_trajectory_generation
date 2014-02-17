function [trajectory, d_trajectory, dd_trajectory] = move_double_support (delta_p, Ts, T, trajectory, d_trajectory, dd_trajectory,interpola)
ZERO_CON = zeros(6,1);
humanoid_fields = humanoid_operational_fields ();
% L = round(T(1)/Ts)+1;

L = size(trajectory.CoM,2);
% Final Positions
p0_com = trajectory.CoM(:, L); %DOMINGO: ver si es mejor --> p0_com = trajectory.CoM(:, end);
p1_com = p0_com + delta_p;

% Generating foot trajectory
% P = SET_TRAJECTORY_CONDITION(T, X, DX, DDX)
% creates the structure for a point to be used in some interpolating function. 
% Depending on the number of input arguments, it sets the conditions for time ‘t’ also for the first and second derivative.
P0 = set_trajectory_condition(T(1), p0_com, ZERO_CON, ZERO_CON); 
P1 = set_trajectory_condition(T(2), p1_com, ZERO_CON, ZERO_CON);



% Ahora seleccionamos la trayectoria a seguir


    %%% DOMINGOOOO
  
    global pp_com1
    %%% FIN
    
switch interpola
    
    case 'Linear'
        %%%DOMINGOOOOOOOOO:
        % Antes era: 
        % [pp_com1, dpp_com1, ddpp_com1] = lineartrajectory(P0, P1, Ts);
        [pp_com, dpp_com, ddpp_com] = lineartrajectory(P0, P1, Ts);
        
        % ANTES:
        % pp_com  = create_trajectory_structure(pp_com1,  Ts, T);
        % dpp_com = create_trajectory_structure(dpp_com1, Ts, T);
        % ddpp_com = create_trajectory_structure(ddpp_com1, Ts, T);
        
%         pp_com  = create_trajectory_structure(pp_com1,  Ts, tt);
%         dpp_com = create_trajectory_structure(dpp_com1, Ts, tt);
%         ddpp_com = create_trajectory_structure(ddpp_com1, Ts, tt);
        %%%
       
    case 'Spline'
        % Spline
        [pp_com, dpp_com, ddpp_com] = spline_interpolation (P0, P1, Ts);
        
    case 'Cubic'
        [pp_com, dpp_com, ddpp_com] = cubic_spline_trajectory (P0, P1, Ts);
        
    case 'Polynomial3'
        % Polynomial 3
        [pp_com, dpp_com, ddpp_com] = poly3_trajectory (P0, P1, Ts);
        
    case 'Polynomial5'
        % Polynomial 5
        [pp_com, dpp_com, ddpp_com] = poly5_trajectory (P0, P1, Ts);
        
    case 'Polynomial7'
        % Polynomial 7
        [pp_com, dpp_com, ddpp_com] = poly7_trajectory (P0, P1, Ts);
        
end
% Aquí cambio el tiempo que le metemos a la función create_trajectory_structure
% ya que es tiempo inicial y tiempo final

trajectory   = insert_trajectory(trajectory,   humanoid_fields, pp_com,  'CoM');
d_trajectory = insert_trajectory(d_trajectory, humanoid_fields, dpp_com, 'CoM');
dd_trajectory = insert_trajectory(dd_trajectory, humanoid_fields, ddpp_com, 'CoM');
end