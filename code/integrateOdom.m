function [finalPose] = integrateOdom(initPose,d,phi)
% integrateOdom: Calculate the robot pose in the initial frame based on the
% odometry
% 
% [finalPose] = integrateOdom(initPose,dis,phi) returns a 3-by-N matrix of the
% robot pose in the initial frame, consisting of x, y, and theta.

%   INPUTS
%       initPose    robot's initial pose [x y theta]  (3-by-1)
%       d     distance vectors returned by DistanceSensorRoomba (1-by-N)
%       phi     angle vectors returned by AngleSensorRoomba (1-by-N)

% 
%   OUTPUTS
%       finalPose     The final pose of the robot in the initial frame
%       (3-by-N)

%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework #2
N = length(d);

finalPose = zeros(3, N);

x = initPose(1);
y = initPose(2);
th = initPose(3);

for k = 1:N
    dk = d(k);
    pk = phi(k);
    if abs(pk) == 0
        dx = dk * cos(th);
        dy = dk * sin(th);
        dth = 0;
    else
        R = dk / pk;
        dx = R * (sin(th + pk) - sin(th));
        dy = R * (-cos(th + pk) + cos(th));
        dth = pk;

    end

    x  = x  + dx;
    y  = y  + dy;
    th = th + dth;

    finalPose(:, k) = [x; y; th];
    end
end