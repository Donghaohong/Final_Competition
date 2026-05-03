function params = pfWaypointDefaultParams(override)
% pfWaypointDefaultParams Defaults for PF localization + waypoint following.

ekf = ekfWaypointDefaultParams();
init = initDefaultParams();

params = struct();

% Runtime and loop timing.
params.maxRunTime = ekf.maxRunTime;
params.controlDt = ekf.controlDt;
params.debugPrint = ekf.debugPrint;
params.debugPrintEvery = ekf.debugPrintEvery;
params.enableLivePlot = false;
params.livePlotUpdatePeriod = ekf.livePlotUpdatePeriod;
params.disableBeep = ekf.disableBeep;

% Waypoint controller.
params.closeEnough = ekf.closeEnough;
params.kPosition = ekf.kPosition;
params.vxyMax = 0.12;
params.epsilon = ekf.epsilon;
params.maxWheelSpeed = 0.12;
params.wheel2Center = ekf.wheel2Center;
params.maxFwdVel = 0.12;
params.maxAngVel = ekf.maxAngVel;
params.turnInPlaceHeadingThresh = ekf.turnInPlaceHeadingThresh;
params.turnInPlaceGain = ekf.turnInPlaceGain;

% Particle set.
params.numParticles = 500;
params.localInitStd = [0.08; 0.08; deg2rad(12)];
params.minPoseCovDiag = [0.01; 0.01; deg2rad(3)] .^ 2;

% PF motion model.
params.odomSigmaDBase = init.odomSigmaDBase;
params.odomSigmaDPerMeter = init.odomSigmaDPerMeter;
params.odomSigmaPhiBase = init.odomSigmaPhiBase;
params.odomSigmaPhiPerRad = init.odomSigmaPhiPerRad;
params.processXYJitter = init.processXYJitter;
params.processThetaJitter = init.processThetaJitter;

% Beacon model.
params.useBeacons = init.useBeacons;
params.beaconSigmaX = init.beaconSigmaX;
params.beaconSigmaY = init.beaconSigmaY;
params.beaconSigmaRange = init.beaconSigmaRange;
params.beaconSigmaBearing = init.beaconSigmaBearing;
params.beaconResidualClipXY = init.beaconResidualClipXY;
params.beaconResidualClipRange = init.beaconResidualClipRange;
params.beaconResidualClipBearing = init.beaconResidualClipBearing;
params.beaconInvisibleLogPenalty = init.beaconInvisibleLogPenalty;
params.unknownTagLogPenalty = init.unknownTagLogPenalty;
params.beaconWeight = init.beaconWeight;
params.minVisibleCameraX = init.minVisibleCameraX;

% Depth model.
params.useDepth = init.useDepth;
params.dropFirstDepthSample = init.dropFirstDepthSample;
params.depthFovDeg = init.depthFovDeg;
params.minDepthRange = init.minDepthRange;
params.maxDepthRange = init.maxDepthRange;
params.depthMaxBeams = init.depthMaxBeams;
params.depthSigma = init.depthSigma;
params.depthResidualClip = init.depthResidualClip;
params.depthWeight = init.depthWeight;
params.wallThickness = init.wallThickness;

% Weight management.
params.weightFloor = init.weightFloor;
params.uniformMix = init.uniformMix;
params.resampleESSFrac = init.resampleESSFrac;
params.rougheningXYStd = 0.006;
params.rougheningThetaStd = deg2rad(0.60);

% Safety and temporary recovery.
params.stopOnBump = ekf.stopOnBump;
params.stopIfNoGoals = ekf.stopIfNoGoals;
params.enableBumpRecovery = ekf.enableBumpRecovery;
params.maxBumpRecoveries = ekf.maxBumpRecoveries;
params.recoveryBackDistance = ekf.recoveryBackDistance;
params.recoveryTurnAngle = ekf.recoveryTurnAngle;
params.recoveryBackVel = ekf.recoveryBackVel;
params.recoveryTurnVel = ekf.recoveryTurnVel;
params.recoveryControlDt = ekf.recoveryControlDt;
params.recoveryMaxBackTime = ekf.recoveryMaxBackTime;
params.recoveryMaxTurnTime = ekf.recoveryMaxTurnTime;
params.recoveryEscapeDistances = ekf.recoveryEscapeDistances;
params.recoveryEscapeAngleOffsets = ekf.recoveryEscapeAngleOffsets;
params.recoveryEscapeMinWallClearance = ekf.recoveryEscapeMinWallClearance;
params.recoveryEscapeBoundsMargin = ekf.recoveryEscapeBoundsMargin;

% Runtime-filled fields.
params.cameraOffset = [0 0];
params.sensorOffset = [0 0];
params.mapGeom = [];

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
