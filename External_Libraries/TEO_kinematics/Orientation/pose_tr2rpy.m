function q = pose_tr2rpy (R)
%   TR2RPY Convert homogeneus transformation matrix pose to a pose with RPY orientation.
%
%   See also RPY2ROT_MAT, RPY2QUAT, QUAT2RPY, ROT_MAT2QUAT, QUAT2ROT_MAT.

%   Author: Domingo Esteban
%   $Revision: 1.0 $  $Date: 2013/09/01 $

q = [ R(1,4,:) ;
      R(2,4,:) ;
      R(3,4,:) ;
      atan2(R(3,2,:),R(3,3,:));
      atan2(-R(3,1,:),sqrt(R(3,2,:).^2+R(3,3,:).^2));
      atan2(R(2,1,:),R(1,1,:))];
  
  
  
end