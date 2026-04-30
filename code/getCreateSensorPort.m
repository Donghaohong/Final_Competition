function CreatePort = getCreateSensorPort(Robot)
% getCreateSensorPort Return the Create base port for odometry/bump APIs.
%
% On the physical robot, course Create sensor APIs such as
% DistanceSensorRoomba, AngleSensorRoomba, and BumpsWheelDropsSensorsRoomba
% use Robot.CreatePort. The simulator object exposes the same APIs directly
% on Robot, so this helper keeps both cases working.

if isstruct(Robot) && isfield(Robot, 'CreatePort')
    CreatePort = Robot.CreatePort;
else
    try
        CreatePort = Robot.CreatePort;
    catch
        CreatePort = Robot;
    end
end
end
