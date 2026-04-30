function [initState, dataStore] = initializeLocalization(Robot, map, waypoints, beaconLoc, offset_x, offset_y, initParams, dataStore)
% initializeLocalization PF-based start localization using known walls only.

if nargin < 7 || isempty(initParams)
    initParams = struct();
end
if nargin < 8 || isempty(dataStore)
    dataStore = struct();
end

if size(map, 2) ~= 4
    error('map must be an N x 4 known-wall matrix.');
end
if size(waypoints, 2) ~= 2 || isempty(waypoints)
    error('waypoints must be a nonempty K x 2 matrix.');
end
if ~isempty(beaconLoc) && size(beaconLoc, 2) ~= 3
    error('beaconLoc must be a B x 3 matrix [tagNum x y].');
end

params = initDefaultParams(initParams);
params.cameraOffset = [offset_x, offset_y];
params.sensorOffset = [offset_x, offset_y];
params.numWaypoints = size(waypoints, 1);
params.numParticles = max(params.numParticles, params.numHeadingBins * params.numWaypoints);
params.mapGeom = precomputeKnownMapGeometry(map, params.wallThickness);

dataStore = ensureDataStoreFields(dataStore);
dataStore.init.params = params;
dataStore.init.cameraOffset = [offset_x, offset_y];

[particles, weights, waypointIds, priorDebug] = initializeParticlesFromWaypoints(waypoints, params);

pfState = struct();
pfState.particles = particles;
pfState.weights = weights;
pfState.waypointIds = waypointIds;
pfState.priorDebug = priorDebug;

[pfState, dataStore] = runInitializationSweep( ...
    Robot, map, beaconLoc, offset_x, offset_y, params, pfState, dataStore);

if isfield(pfState, 'latestSummary') && ~isempty(pfState.latestSummary)
    summary = pfState.latestSummary;
else
    waypointProb = computeWaypointPosterior(pfState.weights, pfState.waypointIds, size(waypoints, 1));
    est = estimatePoseFromParticles(pfState.particles, pfState.weights, pfState.waypointIds, waypointProb);
    summary = struct();
    summary.pose = est.pose;
    summary.posCov = est.posCov;
    summary.headingStd = est.headingStd;
    summary.waypointProb = waypointProb;
    summary.dominantWaypointIdx = est.dominantWaypointIdx;
    summary.confidence = 0.0;
    summary.converged = false;
end

remainingHypotheses = find(summary.waypointProb >= max(summary.waypointProb) * params.remainingHypothesisFrac);

initState = struct();
initState.pose = summary.pose(:);
initState.poseCov = summary.posCov;
initState.posCov = summary.posCov;
initState.startWaypointIdx = summary.dominantWaypointIdx;
initState.converged = logical(summary.converged);
initState.confidence = summary.confidence;
initState.particles = pfState.particles;
initState.weights = pfState.weights;
initState.waypointProb = summary.waypointProb(:);
initState.headingStd = summary.headingStd;
initState.initTimeSec = pfState.elapsedSec;
initState.numSweeps = pfState.numSweeps;
initState.numUpdates = pfState.numUpdates;
initState.remainingHypotheses = remainingHypotheses(:);
initState.debug = struct( ...
    'prior', pfState.priorDebug, ...
    'history', pfState.debug.history, ...
    'cameraOffset', [offset_x, offset_y], ...
    'mapUsedForInit', 'known map only (optWalls excluded)');

dataStore.init.finalState = initState;
end

function dataStore = ensureDataStoreFields(dataStore)
requiredNumeric = {'truthPose', 'odometry', 'rsdepth', 'bump', 'beacon'};
for i = 1:numel(requiredNumeric)
    fieldName = requiredNumeric{i};
    if ~isfield(dataStore, fieldName) || isempty(dataStore.(fieldName))
        dataStore.(fieldName) = [];
    end
end

if ~isfield(dataStore, 'init') || isempty(dataStore.init)
    dataStore.init = struct();
end

initFieldsNumeric = {'command', 'poseEstimate', 'confidence', 'headingStd', 'ess', ...
                     'dominantWaypoint', 'waypointProb', 'posTrace', ...
                     'depthBeamCount', 'beaconCount', 'converged', ...
                     'scoreStats', 'resampleTime'};
for i = 1:numel(initFieldsNumeric)
    fieldName = initFieldsNumeric{i};
    if ~isfield(dataStore.init, fieldName) || isempty(dataStore.init.(fieldName))
        dataStore.init.(fieldName) = [];
    end
end

initFieldsCell = {'depthRaw', 'beaconObs', 'rawTagMatrix', 'bumpObs', 'debug'};
for i = 1:numel(initFieldsCell)
    fieldName = initFieldsCell{i};
    if ~isfield(dataStore.init, fieldName) || isempty(dataStore.init.(fieldName))
        dataStore.init.(fieldName) = cell(0, 1);
    end
end
end
