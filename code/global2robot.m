function[xyR] = global2robot(pose,xyG)
% GLOBAL2ROBOT: transform a 2D point in global coordinates into robot
% coordinates (assumes planar world).
% 
%   XYR = GLOBAL2ROBOT(POSE,XYG) returns the 2D point in robot coordinates
%   corresponding to a 2D point in global coordinates.
% 
%   INPUTS
%       pose    robot's current pose [x y theta]  (1-by-3)
%       xyG     2D point in global coordinates (1-by-2)
% 
%   OUTPUTS
%       xyR     2D point in robot coordinates (1-by-2)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   hong, donghao 

x = pose(1);

y = pose(2);

theta = pose(3);

c = cos(theta);

s = sin(theta);

T = [c s -c*x-s*y;
     -s c s*x-c*y;
     0 0 1];

poseR = T * [xyG(1);xyG(2);1];

xyR = poseR(1:2)';

end
