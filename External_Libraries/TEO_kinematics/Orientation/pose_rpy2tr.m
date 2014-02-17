function p_tr = pose_rpy2tr (p_rpy)
%   POSE_RPY2TR Convert
%
%   See also ROT_MAT2RPY, RPY2QUAT, QUAT2RPY, ROT_MAT2QUAT, QUAT2ROT_MAT.

%   Author: Domingo Esteban
%   $Revision: 1.0 $  $Date: 2011/03/03 $

p_tr = rt2tr(RPY2Rot_Mat(p_rpy(4:6,:)), p_rpy(1:3,:));

end