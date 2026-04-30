function stopRobotSafe(Robot)
% stopRobotSafe Best-effort zero-velocity command.

try
    SetFwdVelAngVelCreate(Robot, 0, 0);
catch
    % Swallow the error so cleanup never throws.
end
end
