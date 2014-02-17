function [q, q_dot, p, R, error_p, error_o] = inverse_kinematics_rightALPHA (Ts, T, q0, Kp, Ko,  xd, xd_dot, Rd, wd)
%Domingo: OJO, q0 corresponde a las 3 primeras
%initialization
iterations = T/Ts;
error_p = zeros(3,iterations);
error_o = zeros(3,iterations);
q = zeros(6,iterations);
q_dot = zeros(6,iterations);
R = zeros(3,3,iterations);
p = zeros(3,iterations);
q(:,1) = q0;


for kk = 1:iterations-1
    %desired quaternion
    [eta_d, eps_d] = rot2quat (Rd(:,:,kk));
    %present position
    R(:,:,kk) = evaluate_orientation_right (q(:,kk));
    p(:,kk) = evaluate_position_right (q(:,kk));
    %present quaternion
    [eta, eps] = rot2quat (R(:,:,kk));
    %errors
    error_p(:,kk) = xd(:,kk) - p(:,kk);
    error_o(:,kk) = eta*eps_d - eta_d*eps - matrix_S(wd)*eps;
    %present velocity
    q_dot(:,kk) = evaluate_jacobian_right(q(:,kk))\[xd_dot(:,kk) + Kp*error_p(:,kk); wd(:,kk) + Ko*error_o(:,kk)];
    %next q
    q(:,kk+1) = q(:,kk) + q_dot(:,kk)*Ts;
end
R(:,:,iterations) = evaluate_orientation_right (q(:,iterations));
p(:,iterations) = evaluate_position_right (q(:,iterations));
[eta, eps] = rot2quat (R(:,:,iterations));
error_p(:,iterations) = xd(:,iterations) - p(:,iterations);
error_o(:,iterations) = eta*eps_d - eta_d*eps - matrix_S(wd)*eps;


%****
function S = matrix_S (w)
S = [       0,  -w(3),   w(2);
         w(3),      0,  -w(1);
        -w(2),   w(1),     0];
    
%****
function [eta, eps] = rot2quat (R)
eta=real(1/2*sqrt(R(1,1)+R(2,2)+R(3,3)+1));
eps=real(1/2*[   sign(R(3,2)-R(2,3))*sqrt(R(1,1)-R(2,2)-R(3,3)+1);
            sign(R(1,3)-R(3,1))*sqrt(R(2,2)-R(1,1)-R(3,3)+1);
            sign(R(2,1)-R(1,2))*sqrt(R(3,3)-R(1,1)-R(2,2)+1)]);
        