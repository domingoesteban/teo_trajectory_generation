function p_quat = pose_rpy2quat (p_rpy)
%   POSE_QUAT2RPY Convert a pose orientation from roll, pitch and yaw angles to quaternion.
%   P_QUAT = POSE_RPY2QUAT(P_RPY) calculates the pose with orientation in
%   quaternion for the input pose whose orientation is expressed in  RPY
%   angles. The input should be a 6-by-M vector and the result will be a
%   7-by-M vector, representing the corresponding position and orientation
%   in RPYs.
%
%   See also POSE_QUAT2RPY, QUAT2RPY, RPY2QUAT.

%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/04/08 $

p_quat(1:3,:) = p_rpy(1:3,:);
p_quat(4:7,:) = RPY2Quat(p_rpy(4:6,:));
end