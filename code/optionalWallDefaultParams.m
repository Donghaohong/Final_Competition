function params = optionalWallDefaultParams(override)
% optionalWallDefaultParams Defaults for active optional-wall verification.

params = struct();

% Observation-point generation.
params.observeDistance = 0.80;
params.observeDistances = [0.45 0.60 0.75 0.90 1.10];
params.observeTangentOffsets = [-0.40 -0.20 0 0.20 0.40];
params.observeClearance = 0.30;
params.minWallViewDistance = 0.25;
params.maxWallViewDistance = 2.00;
params.observeDistancePenalty = 0.25;
params.observeAlignmentPenalty = 0.20;

% Motion to observation point.
params.followerMaxRunTime = 140.0;
params.stopIfNavigationFails = false;
% PF navigation to optional-wall observe/probe points should not use depth by
% default: the candidate wall is not in the map yet, so depth can pull PF to a
% wrong known-wall explanation before classification.
params.pfFollowerUseDepth = false;
params.pfFollowerDepthWeight = 0.0;
params.pfFollowerUseBeacons = true;

% Turn-to-wall behavior.
params.turnMaxTime = 8.0;
params.turnControlDt = 0.10;
params.turnGain = 1.2;
params.turnMaxAngVel = 0.60;
params.turnHeadingTol = deg2rad(8);

% Depth collection and classification.
params.numDepthFrames = 8;
params.depthFramePause = 0.08;
params.depthFovDeg = 54;
params.dropFirstDepthSample = true;
params.minDepthRange = 0.05;
params.maxDepthRange = 3.50;
params.maxRaysForWall = 9;
params.wallBearingWindow = deg2rad(18);
params.minModelSeparation = 0.12;
params.minRaysToClassify = 2;
params.residualMargin = 0.08;
params.maxMeanResidualForDecision = 0.35;
params.confidenceScale = 0.25;
params.unexpectedObstacleMargin = 0.18;
params.minUnexpectedObstacleFrac = 0.35;
params.minMeanUnexpectedObstacle = 0.20;
params.minStrongUnexpectedObstacle = 0.45;

% Low-speed contact probe. This is more reliable than depth for deciding
% whether a candidate wall physically blocks the path.
params.enableBumpProbe = true;
params.bumpProbeMode = 'always'; % 'always' or 'depthFallback'
params.bumpProbeSpeed = 0.045;
params.bumpProbeControlDt = 0.08;
params.bumpProbeMaxTime = 25.0;
params.bumpProbeBeyondWall = 0.28;
params.bumpProbeMaxTravel = 0.95;
params.bumpProbeRobotRadius = 0.17;
params.bumpProbeMinTravelForWall = 0.06;
params.bumpProbeBackupDistance = 0.60;
params.bumpProbeBackupSpeed = -0.06;

% If the normal depth observation point is unreachable, try a direct low-speed
% bump probe from a nearby reachable standoff point instead of giving up.
params.enableBumpProbeFallback = true;
params.bumpFallbackDistances = [0.42 0.55 0.70 0.90];
params.bumpFallbackTangentOffsets = [-0.45 -0.25 0 0.25 0.45];
params.bumpFallbackClearance = 0.22;
params.bumpFallbackMaxRunTime = 90.0;

params.debugPrint = true;
params.wallIdxLabels = [];
params.followerMode = 'ekf';

if nargin < 1 || isempty(override)
    return;
end

if ~isstruct(override)
    error('override must be a struct.');
end

fn = fieldnames(override);
for i = 1:numel(fn)
    params.(fn{i}) = override.(fn{i});
end
end
