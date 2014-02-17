function RPY = Quat2RPY(Q)
%   QUAT2RPY Convert quaternion to roll, pitch and yaw angles.
%   RPY = QUAT2RPY(Q) calculates the RPY angles for a given quaternion. The
%   input should be a 4-by-M vector and the result will be a 3-by-M vector,
%   representing the corresponding RPYs.
%
%   Example:
%       RPY = [0; 0; pi/2];
%       Q = RPY2Quat(RPY);
%       RPY_ = Quat2RPY(Q);
%
%   See also RPY2QUAT, ROT_MAT2QUAT, QUAT2ROT_MAT, RPY2ROT_MAT,
%   ROT_MAT2RPY.

%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/03/03 $

t2 = 2*Q(2,:).*Q(3,:);
t3 = Q(1,:).^2;
t4 = 2*t3;
t5 = 2*Q(2,:).*Q(4,:);
t6 = 2*Q(3,:).*Q(1,:);
t7 = 2*Q(3,:).*Q(4,:);
t8 = t7 + 2*Q(2,:).*Q(1,:);
t9 = 2*Q(4,:).^2 + t4 - 1;
RPY = [ atan2(t8,t9);
        atan2(t6 - t5,sqrt(t8.^2+t9.^2));
        atan2(t2 + 2*Q(4,:).*Q(1,:),2*Q(2,:).^2 + t4 - 1)];
end