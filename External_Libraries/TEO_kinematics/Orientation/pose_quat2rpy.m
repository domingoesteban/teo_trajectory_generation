function p_rpy = pose_quat2rpy (p_quat)
%   POSE_QUAT2RPY Convert a pose orientation from quaternion to roll, pitch and yaw angles.
%   P_RPY = POSE_QUAT2RPY(P_QUAT) calculates the pose with orientation in
%   RPY angles for the input pose with orientation expressed in quaternion.
%   The input should be a 7-by-M vector and the result will be a 6-by-M
%   vector, representing the corresponding position and orientation in RPY.
%
%   See also POSE_QUAT2RPY, QUAT2RPY, RPY2QUAT.

%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/04/08 $

p_rpy(1:3,:) = p_quat(1:3,:);
p_rpy(4:6,:) = Quat2RPY(real(p_quat(4:7,:)));
end