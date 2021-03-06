function h = RR_kinematics_library ()
h.a_T_d = @a_T_d;
h.a_J_d = @a_J_d;
h.a_H_d = @a_H_d;
end
function M = a_T_d(in1)
%A_T_D
%    M = A_T_D(IN1)

%    This function was generated by the Symbolic Math Toolbox version 5.6.
%    26-Jul-2011 17:26:16

theta_b = in1(1,:);
theta_c = in1(2,:);
t2 = sin(theta_b);
t3 = cos(theta_b);
t4 = cos(theta_c);
t5 = sin(theta_c);
M = [t2.*(-2.0./5.0)-t2.*t4.*(1.0./5.0)-t3.*t5.*(1.0./5.0);t3.*(-2.0./5.0)+t2.*t5.*(1.0./5.0)-t3.*t4.*(1.0./5.0)-1.0./5.0];

end
function M = a_J_d(in1)
%A_J_D
%    M = A_J_D(IN1)

%    This function was generated by the Symbolic Math Toolbox version 5.6.
%    26-Jul-2011 17:26:17

theta_b = in1(1,:);
theta_c = in1(2,:);
t7 = cos(theta_b);
t8 = cos(theta_c);
t9 = sin(theta_b);
t10 = sin(theta_c);
t11 = t10.*t9.*(1.0./5.0);
t12 = t10.*t7.*(1.0./5.0);
t13 = t8.*t9.*(1.0./5.0);
M = reshape([t11-t7.*(2.0./5.0)-t7.*t8.*(1.0./5.0),t12+t13+t9.*(2.0./5.0),t11-t7.*t8.*(1.0./5.0),t12+t13],[2, 2]);

end
function M = a_H_d(in1)
%A_H_D
%    M = A_H_D(IN1)

%    This function was generated by the Symbolic Math Toolbox version 5.6.
%    26-Jul-2011 17:26:17

theta_b = in1(1,:);
theta_c = in1(2,:);
t15 = sin(theta_b);
t16 = cos(theta_b);
t17 = sin(theta_c);
t18 = t16.*t17.*(1.0./5.0);
t19 = cos(theta_c);
t20 = t15.*t19.*(1.0./5.0);
t21 = t18+t20;
t22 = t16.*t19.*(1.0./5.0);
t24 = t15.*t17.*(1.0./5.0);
t23 = t22-t24;
M = reshape([t15.*(2.0./5.0)+t18+t20,t16.*(2.0./5.0)+t22-t15.*t17.*(1.0./5.0),t21,t23,t21,t23,t21,t23],[2, 2, 2]);

end