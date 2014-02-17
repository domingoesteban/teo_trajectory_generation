function R = RPY2Rot_Mat (q)
%   RPY2ROT_MAT Convert roll, pitch and yaw angles to rotation matrix.
%   R = RPY2ROT_MAT(Q) calculates the rotation matrix for a given set of
%   RPY angles. The input should be a 3-by-M vector and the result will be
%   a 3-by-3-by-M vector, representing the corresponding rotation matrices.
%
%   Example:
%       RPY = [0; 0; pi/2];
%       R = RPY2Rot_Mat(RPY);
%
%   See also ROT_MAT2RPY, RPY2QUAT, QUAT2RPY, ROT_MAT2QUAT, QUAT2ROT_MAT.

%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/03/03 $

c1 = cos(q(1,:));
s1 = sin(q(1,:));
c2 = cos(q(2,:));
s2 = sin(q(2,:));
c3 = cos(q(3,:));
s3 = sin(q(3,:));

R1 = [c3.*c2; c3.*s1.*s2 - c1.*s3; s1.*s3 + c1.*c3.*s2];
R2 = [c2.*s3; c1.*c3 + s1.*s3.*s2; c1.*s3.*s2 - c3.*s1];
R3 = [-s2; c2.*s1; c1.*c2];
 
R(1,1:3,:)=R1;
R(2,1:3,:)=R2;
R(3,1:3,:)=R3;
end