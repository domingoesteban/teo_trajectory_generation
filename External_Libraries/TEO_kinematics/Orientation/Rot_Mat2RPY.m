function q = Rot_Mat2RPY (R)
%   ROT_MAT2RPY Convert rotation matrix to roll, pitch and yaw angles.
%   Q = ROT_MAT2RPY(R) calculates the set of RPY angles for a given
%   rotation matrix. The input could be a 3-by-3-by-M vector resulting in a
%   3-by-M vector representing the corresponding RPY angles.
%
%   Example:
%       RPY = [0; 0; pi/2];
%       R = RPY2Rot_Mat(RPY);
%       RPY_ = Rot_Mat2RPY(R);
%
%   See also RPY2ROT_MAT, RPY2QUAT, QUAT2RPY, ROT_MAT2QUAT, QUAT2ROT_MAT.

%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/03/03 $

q = [ atan2(R(3,2,:),R(3,3,:));
      atan2(-R(3,1,:),sqrt(R(3,2,:).^2+R(3,3,:).^2));
      atan2(R(2,1,:),R(1,1,:))];
end