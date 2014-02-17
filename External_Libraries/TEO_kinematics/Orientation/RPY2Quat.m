function Q = RPY2Quat(RPY)
%   RPY2QUAT Convert roll, pitch and yaw angles to quaternion.
%   Q = RPY2ROT_MAT(RPY) calculates the quaternion for a given set of RPY
%   angles. The input should be a 3-by-M vector and the result will be a
%   4-by-M vector, representing the corresponding quaternions.
%
%   Example:
%       RPY = [0; 0; pi/2];
%       Q = RPY2Quat(RPY);
%
%   See also QUAT2RPY, ROT_MAT2QUAT, QUAT2ROT_MAT, RPY2ROT_MAT,
%   ROT_MAT2RPY.

%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/03/03 $

t2 = cos(RPY(3,:));
t3 = sin(RPY(1,:));
t4 = sin(RPY(3,:));
t5 = cos(RPY(1,:));
t6 = sin(RPY(2,:));
t7 = cos(RPY(2,:));

Q(1,:)   = (t2.*t5 + t2.*t7 + t5.*t7 + t3.*t4.*t6 + 1).^(1/2)./2;
Q(2:4,:) = 1/2 .*[sign(t2.*t3 + t3.*t7 - t4.*t5.*t6).*sqrt(t2.*t7 - t2.*t5 - t5.*t7 - t3.*t4.*t6 + 1);
            sign(t6 + t3.*t4 + t2.*t5.*t6).*sqrt(t2.*t5 - t2.*t7 - t5.*t7 + t3.*t4.*t6 + 1);
            sign(t4.*t5 + t4.*t7 - t2.*t3.*t6).*sqrt(t5.*t7 - t2.*t7 - t2.*t5 - t3.*t4.*t6 + 1)];
end