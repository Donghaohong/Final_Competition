function [cmdV, cmdW] = feedbackLin(cmdVx,cmdVy,theta,epsilon)
% FEEDBACKLIN Transforms Vx and Vy commands into V and omega commands using
% feedback linearization techniques
% Inputs:
%  cmdVx: input velocity in x direction wrt inertial frame
%  cmdVy: input velocity in y direction wrt inertial frame
%  theta: orientation of the robot
%  epsilon: turn radius
% Outputs:
%  cmdV: fwd velocity
%  cmdW: angular velocity
%
% hong, donghao

c = cos(theta);
s = sin(theta);
R = [c, s;
     -s, c];

v = [cmdVx; cmdVy];

u = [1 0; 0 1/epsilon] * R * v;

cmdV = u(1);
cmdW = u(2);

end
