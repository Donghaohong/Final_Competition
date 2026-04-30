function [pfState, dataStore] = runInitializationSweep(Robot, map, beaconLoc, offset_x, offset_y, params, pfState, dataStore)
% runInitializationSweep Execute the init sweep and PF updates on real hardware.

if ~isfield(params, 'mapGeom') || isempty(params.mapGeom)
    params.mapGeom = precomputeKnownMapGeometry(map, params.wallThickness);
end

initTimer = tic;
stopRobotSafe(Robot);

cumulativeAbsTurn = 0.0;
numUpdates = 0;

history = struct();
history.times = zeros(0, 1);
history.poses = zeros(0, 3);
history.dominantWaypointIdx = zeros(0, 1);

dataStore.init.command = [dataStore.init.command; 0 0 0];

% Initial observation before motion starts.
obs = getInitializationObservations(Robot, offset_x, offset_y, params);
dataStore = appendSensorLogs(dataStore, 0.0, struct('d', 0, 'phi', 0, 'valid', false), obs);
numUpdates = numUpdates + 1;
[pfState, summary, history, dataStore] = measurementUpdate( ...
    numUpdates, 0.0, obs, pfState, history, dataStore, beaconLoc, offset_x, offset_y, params, false);
printStatus(summary, params, 'initial');

while toc(initTimer) < params.maxInitTime
    if obs.bump.valid && obs.bump.any
        break;
    end

    tCmd = toc(initTimer);
    cmd = commandInitializationSweep(Robot, params, tCmd, cumulativeAbsTurn);
    dataStore.init.command = [dataStore.init.command; tCmd cmd.v cmd.w];

    pause(params.controlDt);
    tNow = toc(initTimer);

    odom = getInitializationOdometry(Robot, params);
    if odom.valid
        pfState.particles = propagateParticles(pfState.particles, odom, params);
        cumulativeAbsTurn = cumulativeAbsTurn + abs(odom.phi);
    end

    obs = getInitializationObservations(Robot, offset_x, offset_y, params);
    dataStore = appendSensorLogs(dataStore, tNow, odom, obs);

    numUpdates = numUpdates + 1;
    allowResample = cumulativeAbsTurn >= params.minSweepTurnForResampling;
    [pfState, summary, history, dataStore] = measurementUpdate( ...
        numUpdates, tNow, obs, pfState, history, dataStore, beaconLoc, offset_x, offset_y, params, allowResample);
    printStatus(summary, params, 'loop');

    if obs.bump.valid && obs.bump.any
        break;
    end

    if summary.converged && ...
            tNow >= params.minInitTime && ...
            cumulativeAbsTurn >= params.minSweepTurnForConvergence
        break;
    end

    if (cumulativeAbsTurn / (2 * pi)) >= params.maxSweeps && tNow >= params.minInitTime
        break;
    end
end

stopRobotSafe(Robot);

if cumulativeAbsTurn < params.minSweepTurnForConvergence
    summary.converged = false;
    summary.confidence = min(summary.confidence, params.maxUnconvergedConfidence);
end

pfState.elapsedSec = toc(initTimer);
pfState.numSweeps = cumulativeAbsTurn / (2 * pi);
pfState.numUpdates = numUpdates;
pfState.converged = summary.converged;
pfState.confidence = summary.confidence;
pfState.latestSummary = summary;
pfState.debug.history = history;
end

function [pfStateOut, summaryOut, historyOut, dataStoreOut] = measurementUpdate(updateIdx, tNow, obsNow, pfStateIn, historyIn, dataStoreIn, beaconLoc, offset_x, offset_y, params, allowResample)
beaconLogLike = zeros(size(pfStateIn.weights));
depthLogLike = zeros(size(pfStateIn.weights));

if params.useBeacons
    [beaconLogLike, beaconDebug] = scoreParticlesWithBeacons( ...
        pfStateIn.particles, obsNow, beaconLoc, offset_x, offset_y, params);
else
    beaconDebug = struct('numDetections', 0, 'usedCartesian', false);
end

if params.useDepth
    [depthLogLike, depthDebug] = scoreParticlesWithDepth( ...
        pfStateIn.particles, obsNow, params.mapGeom, params);
else
    depthDebug = struct('numBeamsUsed', 0, 'beamIndices', []);
end

logW = combineParticleLogWeights(pfStateIn.weights, beaconLogLike, depthLogLike, params);
[weightsNew, normDebug] = normalizeLogWeights(logW, params);

waypointProb = computeWaypointPosterior(weightsNew, pfStateIn.waypointIds, params.numWaypoints);
est = estimatePoseFromParticles(pfStateIn.particles, weightsNew, pfStateIn.waypointIds, waypointProb);

ess = 1 / max(sum(weightsNew .^ 2), eps);

summaryOut = struct();
summaryOut.pose = est.pose;
summaryOut.posCov = est.posCov;
summaryOut.headingStd = est.headingStd;
summaryOut.waypointProb = waypointProb;
summaryOut.dominantWaypointIdx = est.dominantWaypointIdx;
summaryOut.ess = ess;
summaryOut.numUpdates = updateIdx;

historyOut = historyIn;
historyOut.times(end+1, 1) = tNow;
historyOut.poses(end+1, :) = est.pose(:).';
historyOut.dominantWaypointIdx(end+1, 1) = est.dominantWaypointIdx;

[summaryOut.converged, summaryOut.confidence, convDebug] = ...
    checkInitializationConvergence(summaryOut, historyOut, params);

pfStateOut = pfStateIn;
pfStateOut.weights = weightsNew;

if allowResample && ess < params.resampleESSFrac * numel(weightsNew)
    idx = systematicResample(weightsNew);
    pfStateOut.particles = pfStateIn.particles(idx, :);
    pfStateOut.waypointIds = pfStateIn.waypointIds(idx);
    pfStateOut.weights = ones(numel(weightsNew), 1) / numel(weightsNew);
    dataStoreIn.init.resampleTime(end+1, 1) = tNow;
end

dataStoreOut = appendInitLogs(dataStoreIn, tNow, summaryOut, obsNow, ...
                              beaconDebug, depthDebug, normDebug, convDebug);
end

function [weights, debug] = normalizeLogWeights(logW, params)
shifted = logW - max(logW);
w = exp(shifted);

debug = struct();
debug.recoveredAllZero = false;
debug.maxLogWeight = max(logW);

if ~all(isfinite(w)) || sum(w) <= 0
    w = ones(size(logW));
    debug.recoveredAllZero = true;
end

w = w / max(sum(w), eps);
w = (1 - params.uniformMix) * w + params.uniformMix / numel(w);
w = w / max(sum(w), eps);

weights = max(w, params.weightFloor);
weights = weights / max(sum(weights), eps);
end

function dataStore = appendSensorLogs(dataStore, tNow, odom, obs)
if odom.valid
    dataStore.odometry = [dataStore.odometry; tNow odom.d odom.phi];
end

if ~isempty(obs.depthRaw)
    dataStore.rsdepth = [dataStore.rsdepth; tNow obs.depthRaw(:).'];
end

if obs.bump.valid
    dataStore.bump = [dataStore.bump; ...
        tNow ...
        obs.bump.right obs.bump.left ...
        obs.bump.dropRight obs.bump.dropLeft obs.bump.dropCaster ...
        obs.bump.front];
end

if ~isempty(obs.rawTagMatrix)
    stamp = repmat(tNow, size(obs.rawTagMatrix, 1), 1);
    dataStore.beacon = [dataStore.beacon; stamp obs.rawTagMatrix];
end
end

function dataStore = appendInitLogs(dataStore, tNow, summary, obs, beaconDebug, depthDebug, normDebug, convDebug)
dataStore.init.poseEstimate = [dataStore.init.poseEstimate; tNow summary.pose(:).'];
dataStore.init.confidence = [dataStore.init.confidence; tNow summary.confidence];
dataStore.init.headingStd = [dataStore.init.headingStd; tNow summary.headingStd];
dataStore.init.ess = [dataStore.init.ess; tNow summary.ess];
dataStore.init.dominantWaypoint = [dataStore.init.dominantWaypoint; ...
    tNow summary.dominantWaypointIdx max(summary.waypointProb)];
dataStore.init.waypointProb = [dataStore.init.waypointProb; tNow summary.waypointProb(:).'];
dataStore.init.posTrace = [dataStore.init.posTrace; tNow trace(summary.posCov(1:2, 1:2))];
dataStore.init.depthBeamCount = [dataStore.init.depthBeamCount; tNow depthDebug.numBeamsUsed];
dataStore.init.beaconCount = [dataStore.init.beaconCount; tNow beaconDebug.numDetections];
dataStore.init.converged = [dataStore.init.converged; tNow double(summary.converged)];
dataStore.init.scoreStats = [dataStore.init.scoreStats; ...
    tNow beaconDebug.numDetections depthDebug.numBeamsUsed normDebug.recoveredAllZero];

dataStore.init.depthRaw{end+1, 1} = obs.depthRaw;
dataStore.init.beaconObs{end+1, 1} = obs.beacons;
dataStore.init.rawTagMatrix{end+1, 1} = obs.rawTagMatrix;
dataStore.init.bumpObs{end+1, 1} = obs.bump;
dataStore.init.debug{end+1, 1} = struct( ...
    'conv', convDebug, ...
    'beamIndices', depthDebug.beamIndices, ...
    'norm', normDebug);
end

function printStatus(summary, params, tag)
if ~params.debugPrint
    return;
end

fprintf('[initializeLocalization:%s] wp=%d prob=%.3f conf=%.3f headingStd=%.2f deg posStd=%.3f m conv=%d\n', ...
    tag, ...
    summary.dominantWaypointIdx, ...
    max(summary.waypointProb), ...
    summary.confidence, ...
    (180 / pi) * summary.headingStd, ...
    sqrt(max(0, trace(summary.posCov(1:2, 1:2)))), ...
    summary.converged);
end
