function odom = getInitializationOdometry(Robot, ~)
% getInitializationOdometry Wrapper around course odometry APIs.
%
% Assumption:
%   DistanceSensorRoomba returns translation increment in meters.
%   AngleSensorRoomba returns heading increment in radians.
% TODO:
%   Verify these units on the competition workstation if they differ.

odom = struct('d', 0, 'phi', 0, 'valid', false);

try
    CreatePort = getCreateSensorPort(Robot);
    odom.d = DistanceSensorRoomba(CreatePort);
    odom.phi = AngleSensorRoomba(CreatePort);
    if ~isfinite(odom.d), odom.d = 0; end
    if ~isfinite(odom.phi), odom.phi = 0; end
    odom.valid = true;
catch
    % Keep the zero increment and let the caller continue safely.
end
end
