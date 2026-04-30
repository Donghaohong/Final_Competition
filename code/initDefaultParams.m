function params = initDefaultParams(initParams)
% initDefaultParams Default parameters for PF-based initialization.

params = struct();

% Particle prior over start hypotheses.
params.numParticles = 192;
params.numHeadingBins = 16;
params.startPosStd = 0.035;
params.startHeadingJitter = deg2rad(6);

% Sweep behavior and overall initialization budget.
params.minInitTime = 8.0;
params.maxInitTime = 25.0;
params.maxSweeps = 2.0;
params.minSweepTurnForConvergence = 2 * pi;
params.minSweepTurnForResampling = inf;
params.controlDt = 0.20;
params.sweepFwdVel = 0.00;
params.sweepAngVel = 0.35;

% Motion noise used inside the particle filter.
params.odomSigmaDBase = 0.002;
params.odomSigmaDPerMeter = 0.08;
params.odomSigmaPhiBase = deg2rad(0.5);
params.odomSigmaPhiPerRad = 0.10;
params.processXYJitter = 0.003;
params.processThetaJitter = deg2rad(0.25);

% Depth model.
params.useDepth = true;
params.dropFirstDepthSample = true;
params.depthFovDeg = 54;
params.minDepthRange = 0.05;
params.maxDepthRange = 3.50;
params.depthMaxBeams = 9;
params.depthSigma = 0.12;
params.depthResidualClip = 0.35;
params.depthWeight = 1.0;
params.wallThickness = 0.10;

% Beacon model.
params.useBeacons = true;
params.beaconSigmaX = 0.12;
params.beaconSigmaY = 0.12;
params.beaconSigmaRange = 0.16;
params.beaconSigmaBearing = deg2rad(10);
params.beaconResidualClipXY = 0.40;
params.beaconResidualClipRange = 0.45;
params.beaconResidualClipBearing = deg2rad(35);
params.beaconInvisibleLogPenalty = -4.0;
params.unknownTagLogPenalty = -1.5;
params.beaconWeight = 4.0;
params.minVisibleCameraX = 0.02;

% Weight management.
params.weightFloor = 1e-12;
params.uniformMix = 0.02;
params.resampleESSFrac = 0.55;

% Convergence logic.
params.minConvergenceUpdates = 5;
params.stabilityWindow = 4;
params.dominantWaypointThresh = 0.75;
params.maxPosStdForConvergence = 0.18;
params.maxHeadingStdForConvergence = deg2rad(18);
params.stablePosThreshold = 0.08;
params.stableHeadingThreshold = deg2rad(9);
params.remainingHypothesisFrac = 0.25;
params.maxUnconvergedConfidence = 0.55;

% Miscellaneous.
params.debugPrint = false;
params.cameraOffset = [0 0];
params.sensorOffset = [0 0];
params.mapGeom = [];
params.numWaypoints = [];

if nargin < 1 || isempty(initParams)
    return;
end

if ~isstruct(initParams)
    error('initParams must be a struct when provided.');
end

fn = fieldnames(initParams);
for i = 1:numel(fn)
    params.(fn{i}) = initParams.(fn{i});
end
end
