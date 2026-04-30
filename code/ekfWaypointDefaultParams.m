function params = ekfWaypointDefaultParams(override)
% ekfWaypointDefaultParams Defaults for EKF localization + waypoint following.

params = struct();

% Runtime and loop timing.
params.maxRunTime = 60.0;
params.controlDt = 0.10;
params.debugPrint = true;
params.debugPrintEvery = 10;
params.enableLivePlot = false;
params.livePlotUpdatePeriod = 0.35;

% Waypoint controller.
params.closeEnough = 0.12;
params.kPosition = 0.8;
params.vxyMax = 0.08;
params.epsilon = 0.45;
params.maxWheelSpeed = 0.10;
params.wheel2Center = 0.13;
params.maxFwdVel = 0.10;
params.maxAngVel = 1.00;
params.turnInPlaceHeadingThresh = deg2rad(55);
params.turnInPlaceGain = 1.4;

% EKF covariance protection and process noise.
params.minInitStd = [0.06; 0.06; deg2rad(8)];
params.processXYStdBase = 0.010;
params.processXYStdPerMeter = 0.080;
params.processThetaStdBase = deg2rad(1.0);
params.processThetaStdPerRad = 0.080;
params.noOdomThetaStd = deg2rad(0.2);

% Beacon update.
params.useBeacons = true;
params.beaconSigmaX = 0.14;
params.beaconSigmaY = 0.14;
params.beaconResidualGateXY = 0.60;
params.beaconMinPredictedX = 0.02;
params.beaconMaxUpdatesPerStep = 4;
params.jacobianEps = [1e-4; 1e-4; 1e-4];

% Depth update.
params.useDepth = true;
params.dropFirstDepthSample = true;
params.depthFovDeg = 54;
params.minDepthRange = 0.05;
params.maxDepthRange = 3.50;
params.depthSigma = 0.16;
params.depthResidualGate = 0.45;
params.depthMaxBeams = 9;
params.depthMinBeamsForUpdate = 2;
params.wallThickness = 0.10;

% Safety.
params.stopOnBump = true;
params.stopIfNoGoals = true;
params.enableBumpRecovery = true;
params.maxBumpRecoveries = 3;
params.recoveryBackDistance = 0.22;
params.recoveryTurnAngle = deg2rad(35);
params.recoveryBackVel = -0.08;
params.recoveryTurnVel = 0.45;
params.recoveryControlDt = 0.10;
params.recoveryMaxBackTime = 5.0;
params.recoveryMaxTurnTime = 5.0;
params.recoveryEscapeDistances = [0.35 0.45 0.60 0.75];
params.recoveryEscapeAngleOffsets = deg2rad([0 25 -25 50 -50 80 -80 120 -120 180]);
params.recoveryEscapeMinWallClearance = 0.35;
params.recoveryEscapeBoundsMargin = 0.08;

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
