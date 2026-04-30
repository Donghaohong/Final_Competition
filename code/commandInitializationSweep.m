function cmd = commandInitializationSweep(Robot, params, ~, cumulativeAbsTurn)
% commandInitializationSweep Conservative in-place sweep motion command.

if nargin < 4
    cumulativeAbsTurn = 0;
end

cmd = struct('v', params.sweepFwdVel, 'w', params.sweepAngVel, 'issued', false);

% Optional sweep reversal after every completed 360 deg to reduce cable twist.
direction = 1;
completedTurns = floor(cumulativeAbsTurn / (2 * pi));
if mod(completedTurns, 2) == 1
    direction = -1;
end
cmd.w = direction * params.sweepAngVel;

try
    SetFwdVelAngVelCreate(Robot, cmd.v, cmd.w);
    cmd.issued = true;
catch
    cmd.v = 0;
    cmd.w = 0;
    cmd.issued = false;
end
end
