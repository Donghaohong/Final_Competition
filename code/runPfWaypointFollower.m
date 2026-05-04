function [pfState, dataStore, navState] = runPfWaypointFollower( ...
    Robot, map, beaconLoc, goalWaypoints, offset_x, offset_y, initState, pfParams, dataStore)
% runPfWaypointFollower Particle-filter localization with waypoint following.

if nargin < 8 || isempty(pfParams)
    pfParams = struct();
end
if nargin < 9 || isempty(dataStore)
    dataStore = struct();
end

if size(map, 2) ~= 4
    error('map must be an N x 4 known-wall matrix.');
end
if ~isempty(beaconLoc) && size(beaconLoc, 2) ~= 3
    error('beaconLoc must be a B x 3 matrix [tagNum x y].');
end
if isempty(goalWaypoints)
    goalWaypoints = zeros(0, 2);
end
if size(goalWaypoints, 2) ~= 2
    error('goalWaypoints must be an N x 2 matrix.');
end
if ~isfield(initState, 'pose') || numel(initState.pose) ~= 3
    error('initState.pose must be a 3x1 pose.');
end

params = pfWaypointDefaultParams(pfParams);
params.sensorOffset = [offset_x, offset_y];
params.cameraOffset = [offset_x, offset_y];
params.mapGeom = precomputeKnownMapGeometry(map, params.wallThickness);

dataStore = ensurePfDataStoreFields(dataStore);
[particles, weights] = initializeTrackingParticles(initState, params);
[mu, sigma] = estimatePfPoseAndCov(particles, weights, params);

tBase = getPfDataStoreTimeBase(dataStore);
tStart = tBase;
dataStore.pfMu = [dataStore.pfMu; tStart mu.'];
dataStore.pfParticles{end + 1, 1} = particles;
dataStore.pfWeights{end + 1, 1} = weights;

navState = struct();
navState.goalWaypoints = goalWaypoints;
navState.currentGoalIdx = 1;
navState.reachedGoals = false(size(goalWaypoints, 1), 1);
navState.beepMask = normalizePfBeepMask(params, size(goalWaypoints, 1));
navState.finalDistanceToGoal = NaN;
navState.stopReason = '';
navState.bumpRecoveryCount = 0;

stopReason = 'maxTime';
reachedAllGoals = isempty(goalWaypoints);
numUpdates = 0;
livePlot = initializePfLivePlot(params, map, beaconLoc, goalWaypoints, particles, mu, sigma);
lastLivePlotUpdate = -inf;

if reachedAllGoals && params.stopIfNoGoals
    stopRobotSafe(Robot);
    stopReason = 'noGoals';
else
    stopRobotSafe(Robot);
    runTimer = tic;

    while toc(runTimer) < params.maxRunTime
        tNow = tBase + toc(runTimer);
        obs = readPfSensors(Robot, params);
        dataStore = appendPfSensorLogs(dataStore, tNow, obs);

        if params.stopOnBump && obs.bump.valid && obs.bump.any
            if params.enableBumpRecovery && navState.bumpRecoveryCount < params.maxBumpRecoveries
                if obs.odom.valid
                    particles = propagateParticles(particles, obs.odom, params);
                end
                navState.bumpRecoveryCount = navState.bumpRecoveryCount + 1;
                [particles, weights, mu, sigma, goalWaypoints, navState, dataStore, recoveryOk] = ...
                    runPfTemporaryBumpRecovery( ...
                        Robot, particles, weights, goalWaypoints, navState, map, obs.bump, params, dataStore, tNow);
                if ~recoveryOk
                    stopReason = 'bumpRecoveryFailed';
                    break;
                end
                continue;
            else
                stopReason = 'bumpRecoveryFailed';
                break;
            end
        end

        [particles, weights, mu, sigma, updateStats] = pfLocalizationStep( ...
            particles, weights, obs, beaconLoc, offset_x, offset_y, params);
        numUpdates = numUpdates + 1;

        prevReachedGoals = navState.reachedGoals;
        [cmdV, cmdW, navState, reachedAllGoals, finalDist] = ...
            computePfWaypointCommand(mu, goalWaypoints, navState, params);
        navState.finalDistanceToGoal = finalDist;
        newlyReachedGoals = navState.reachedGoals & ~prevReachedGoals;
        navState.beepMask = normalizePfMaskLength(navState.beepMask, numel(navState.reachedGoals), false);
        if any(newlyReachedGoals & navState.beepMask)
            beepPfRobotSafe(Robot);
        end

        dataStore.pfMu = [dataStore.pfMu; tNow mu.'];
        dataStore.pfParticles{end + 1, 1} = particles;
        dataStore.pfWeights{end + 1, 1} = weights;
        dataStore.pfUpdateStats = [dataStore.pfUpdateStats; ...
            tNow updateStats.beaconDetections updateStats.beaconsUsed ...
            updateStats.depthCandidateBeams updateStats.depthBeamsUsed ...
            updateStats.ess double(updateStats.resampled)];

        if params.enableLivePlot && (tNow - lastLivePlotUpdate >= params.livePlotUpdatePeriod || reachedAllGoals)
            livePlot = updatePfLivePlot(livePlot, dataStore, goalWaypoints, navState, particles, mu, sigma);
            lastLivePlotUpdate = tNow;
        end

        if reachedAllGoals
            stopReason = 'reachedAllGoals';
            dataStore.pfCommand = [dataStore.pfCommand; tNow 0 0];
            SetFwdVelAngVelCreate(Robot, 0, 0);
            break;
        end

        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        dataStore.pfCommand = [dataStore.pfCommand; tNow cmdV cmdW];
        dataStore.pfGoal = [dataStore.pfGoal; ...
            tNow navState.currentGoalIdx ...
            goalWaypoints(navState.currentGoalIdx, :) finalDist];

        if params.debugPrint && mod(numUpdates, params.debugPrintEvery) == 0
            fprintf('[runPfWaypointFollower] goal=%d dist=%.3f pose=[%.2f %.2f %.1fdeg] beacons=%d depth=%d ess=%.1f cmd=[%.2f %.2f]\n', ...
                navState.currentGoalIdx, finalDist, mu(1), mu(2), mu(3) * 180 / pi, ...
                updateStats.beaconsUsed, updateStats.depthBeamsUsed, updateStats.ess, cmdV, cmdW);
        end

        pause(params.controlDt);
    end
end

stopRobotSafe(Robot);

navState.stopReason = stopReason;
navState.reachedAllGoals = reachedAllGoals;

pfState = struct();
pfState.pose = mu;
pfState.poseCov = sigma;
pfState.reachedAllGoals = reachedAllGoals;
pfState.stopReason = stopReason;
pfState.numUpdates = numUpdates;
pfState.runTimeSec = max(0, getPfDataStoreTimeBase(dataStore) - tBase);
pfState.particles = particles;
pfState.weights = weights;
pfState.params = params;

dataStore.pfFinalState = pfState;
dataStore.navState = navState;
end

function dataStore = ensurePfDataStoreFields(dataStore)
numericFields = {'truthPose', 'odometry', 'rsdepth', 'bump', 'beacon', ...
                 'pfMu', 'pfCommand', 'pfGoal', 'pfUpdateStats', ...
                 'pfRecovery', 'pfRecoveryEscape'};
for i = 1:numel(numericFields)
    name = numericFields{i};
    if ~isfield(dataStore, name) || isempty(dataStore.(name))
        dataStore.(name) = [];
    end
end

if ~isfield(dataStore, 'pfParticles') || isempty(dataStore.pfParticles)
    dataStore.pfParticles = cell(0, 1);
end
if ~isfield(dataStore, 'pfWeights') || isempty(dataStore.pfWeights)
    dataStore.pfWeights = cell(0, 1);
end
if ~isfield(dataStore, 'pfRawDepthCell') || isempty(dataStore.pfRawDepthCell)
    dataStore.pfRawDepthCell = cell(0, 1);
end
if ~isfield(dataStore, 'pfRawBeaconCell') || isempty(dataStore.pfRawBeaconCell)
    dataStore.pfRawBeaconCell = cell(0, 1);
end
end

function [particles, weights] = initializeTrackingParticles(initState, params)
particleIdx = [];
if isfield(initState, 'particles') && size(initState.particles, 2) == 3 && ~isempty(initState.particles)
    particles = initState.particles;
    if size(particles, 1) > params.numParticles
        particleIdx = round(linspace(1, size(particles, 1), params.numParticles));
        particles = particles(particleIdx, :);
    end
else
    pose = initState.pose(:).';
    N = params.numParticles;
    particles = repmat(pose, N, 1);
    particles(:, 1) = particles(:, 1) + params.localInitStd(1) * randn(N, 1);
    particles(:, 2) = particles(:, 2) + params.localInitStd(2) * randn(N, 1);
    particles(:, 3) = wrapToPiLocal(particles(:, 3) + params.localInitStd(3) * randn(N, 1));
end

N = size(particles, 1);
if ~isempty(particleIdx) && isfield(initState, 'weights') && numel(initState.weights) >= max(particleIdx)
    weights = initState.weights(particleIdx);
    weights = weights(:);
elseif isfield(initState, 'weights') && numel(initState.weights) == N
    weights = initState.weights(:);
elseif isfield(initState, 'weights') && numel(initState.weights) >= N
    weights = initState.weights(1:N);
    weights = weights(:);
else
    weights = ones(N, 1) / N;
end
weights = max(weights, params.weightFloor);
weights = weights / max(sum(weights), eps);
end

function [particles, weights, mu, sigma, stats] = pfLocalizationStep( ...
    particles, weights, obs, beaconLoc, offset_x, offset_y, params)
stats = struct( ...
    'beaconDetections', numel(obs.beacons), ...
    'beaconsUsed', 0, ...
    'depthCandidateBeams', 0, ...
    'depthBeamsUsed', 0, ...
    'ess', 1 / max(sum(weights .^ 2), eps), ...
    'resampled', false);

if obs.odom.valid
    particles = propagateParticles(particles, obs.odom, params);
end

beaconLogLike = zeros(size(weights));
depthLogLike = zeros(size(weights));

if params.useBeacons
    [beaconLogLike, beaconDebug] = scoreParticlesWithBeacons( ...
        particles, obs, beaconLoc, offset_x, offset_y, params);
    stats.beaconsUsed = beaconDebug.numDetections;
end

if params.useDepth
    [depthLogLike, depthDebug] = scoreParticlesWithDepth( ...
        particles, obs, params.mapGeom, params);
    stats.depthCandidateBeams = depthDebug.numBeamsUsed;
    stats.depthBeamsUsed = depthDebug.numBeamsUsed;
end

logW = combineParticleLogWeights(weights, beaconLogLike, depthLogLike, params);
weights = normalizePfLogWeights(logW, params);
stats.ess = 1 / max(sum(weights .^ 2), eps);

if stats.ess < params.resampleESSFrac * numel(weights)
    idx = systematicResample(weights);
    particles = particles(idx, :);
    particles = roughenParticles(particles, params);
    weights = ones(numel(weights), 1) / numel(weights);
    stats.resampled = true;
end

[mu, sigma] = estimatePfPoseAndCov(particles, weights, params);
end

function weights = normalizePfLogWeights(logW, params)
if isempty(logW) || any(~isfinite(logW))
    weights = ones(numel(logW), 1) / max(numel(logW), 1);
    return;
end
w = exp(logW - max(logW));
if ~any(isfinite(w)) || sum(w) <= eps
    weights = ones(numel(logW), 1) / max(numel(logW), 1);
    return;
end
weights = w / sum(w);
if params.uniformMix > 0
    weights = (1 - params.uniformMix) * weights + params.uniformMix / numel(weights);
end
weights = max(weights, params.weightFloor);
weights = weights / max(sum(weights), eps);
end

function particles = roughenParticles(particles, params)
particles(:, 1) = particles(:, 1) + params.rougheningXYStd * randn(size(particles, 1), 1);
particles(:, 2) = particles(:, 2) + params.rougheningXYStd * randn(size(particles, 1), 1);
particles(:, 3) = wrapToPiLocal(particles(:, 3) + params.rougheningThetaStd * randn(size(particles, 1), 1));
end

function [mu, sigma] = estimatePfPoseAndCov(particles, weights, params)
weights = weights(:);
weights = weights / max(sum(weights), eps);

xHat = sum(weights .* particles(:, 1));
yHat = sum(weights .* particles(:, 2));
thetaHat = atan2(sum(weights .* sin(particles(:, 3))), ...
                 sum(weights .* cos(particles(:, 3))));
mu = [xHat; yHat; thetaHat];

dx = particles(:, 1) - xHat;
dy = particles(:, 2) - yHat;
dtheta = wrapToPiLocal(particles(:, 3) - thetaHat);
X = [dx, dy, dtheta];
sigma = X' * bsxfun(@times, X, weights);
sigma = sanitizePfCovariance(sigma, params.minPoseCovDiag);
end

function obs = readPfSensors(Robot, params)
obs = struct();
obs.odom = struct('d', 0, 'phi', 0, 'valid', false);
obs.depthRaw = [];
obs.depthRanges = [];
obs.depthAngles = [];
obs.validDepthMask = [];
obs.rawTagMatrix = [];
obs.beacons = struct('tagNum', {}, 'latency', {}, 'xCam', {}, 'yCam', {}, ...
                     'thetaCam', {}, 'range', {}, 'bearing', {}, 'confidence', {});
obs.bump = struct('right', 0, 'left', 0, 'dropRight', 0, 'dropLeft', 0, ...
                  'dropCaster', 0, 'front', 0, 'any', false, 'valid', false);

try
    CreatePort = getCreateSensorPort(Robot);
    obs.odom.d = DistanceSensorRoomba(CreatePort);
    obs.odom.phi = AngleSensorRoomba(CreatePort);
    if ~isfinite(obs.odom.d), obs.odom.d = 0; end
    if ~isfinite(obs.odom.phi), obs.odom.phi = 0; end
    obs.odom.valid = true;
catch
end

try
    depthArray = RealSenseDist(Robot);
    depthRaw = depthArray(:).';
    obs.depthRaw = depthRaw;
    if params.dropFirstDepthSample && numel(depthRaw) > 1
        depthRanges = depthRaw(2:end);
    else
        depthRanges = depthRaw;
    end
    numDepth = numel(depthRanges);
    fovRad = deg2rad(params.depthFovDeg);
    obs.depthRanges = depthRanges(:);
    obs.depthAngles = linspace(0.5 * fovRad, -0.5 * fovRad, numDepth).';
    obs.validDepthMask = isfinite(obs.depthRanges) & ...
                         obs.depthRanges >= params.minDepthRange & ...
                         obs.depthRanges <= params.maxDepthRange;
catch
end

try
    CreatePort = getCreateSensorPort(Robot);
    [BumpRight, BumpLeft, DropRight, DropLeft, DropCaster, BumpFront] = ...
        BumpsWheelDropsSensorsRoomba(CreatePort);
    obs.bump = struct( ...
        'right', BumpRight, ...
        'left', BumpLeft, ...
        'dropRight', DropRight, ...
        'dropLeft', DropLeft, ...
        'dropCaster', DropCaster, ...
        'front', BumpFront, ...
        'any', logical(BumpRight || BumpLeft || BumpFront), ...
        'valid', true);
catch
end

try
    tagMatrix = RealSenseTag(Robot);
    if ~isempty(tagMatrix)
        obs.rawTagMatrix = tagMatrix;
        obs.beacons = parsePfRawTagDetections(tagMatrix);
    end
catch
end
end

function dataStore = appendPfSensorLogs(dataStore, tNow, obs)
if obs.odom.valid
    dataStore.odometry = [dataStore.odometry; tNow obs.odom.d obs.odom.phi];
end

if ~isempty(obs.depthRaw)
    [dataStore, appended] = appendPfNumericRows(dataStore, 'rsdepth', [tNow obs.depthRaw(:).']);
    if ~appended
        dataStore.pfRawDepthCell{end + 1, 1} = [tNow obs.depthRaw(:).'];
    end
end

if obs.bump.valid
    dataStore.bump = [dataStore.bump; ...
        tNow ...
        obs.bump.right obs.bump.left ...
        obs.bump.dropRight obs.bump.dropLeft obs.bump.dropCaster ...
        obs.bump.front];
end

if ~isempty(obs.rawTagMatrix)
    rows = [repmat(tNow, size(obs.rawTagMatrix, 1), 1) obs.rawTagMatrix];
    [dataStore, appended] = appendPfNumericRows(dataStore, 'beacon', rows);
    if ~appended
        dataStore.pfRawBeaconCell{end + 1, 1} = rows;
    end
end
end

function [dataStore, appended] = appendPfNumericRows(dataStore, fieldName, rows)
appended = false;
if ~isfield(dataStore, fieldName) || isempty(dataStore.(fieldName))
    dataStore.(fieldName) = rows;
    appended = true;
elseif size(dataStore.(fieldName), 2) == size(rows, 2)
    dataStore.(fieldName) = [dataStore.(fieldName); rows];
    appended = true;
end
end

function [cmdV, cmdW, navState, reachedAllGoals, finalDist] = ...
    computePfWaypointCommand(mu, goalWaypoints, navState, params)
numGoals = size(goalWaypoints, 1);
cmdV = 0;
cmdW = 0;
reachedAllGoals = false;
finalDist = NaN;

while navState.currentGoalIdx <= numGoals
    goal = goalWaypoints(navState.currentGoalIdx, :).';
    delta = goal - mu(1:2);
    finalDist = norm(delta);
    if finalDist > params.closeEnough
        break;
    end
    navState.reachedGoals(navState.currentGoalIdx) = true;
    navState.currentGoalIdx = navState.currentGoalIdx + 1;
end

if navState.currentGoalIdx > numGoals
    reachedAllGoals = true;
    return;
end

goal = goalWaypoints(navState.currentGoalIdx, :).';
delta = goal - mu(1:2);
finalDist = norm(delta);

desiredHeading = atan2(delta(2), delta(1));
headingErr = wrapToPiLocal(desiredHeading - mu(3));
if abs(headingErr) > params.turnInPlaceHeadingThresh
    cmdV = 0;
    cmdW = params.turnInPlaceGain * headingErr;
    cmdW = max(-params.maxAngVel, min(params.maxAngVel, cmdW));
    [cmdV, cmdW] = limitCmds(cmdV, cmdW, params.maxWheelSpeed, params.wheel2Center);
    return;
end

cmdVx = params.kPosition * delta(1);
cmdVy = params.kPosition * delta(2);
vxyNorm = hypot(cmdVx, cmdVy);
if vxyNorm > params.vxyMax
    cmdVx = cmdVx / vxyNorm * params.vxyMax;
    cmdVy = cmdVy / vxyNorm * params.vxyMax;
end

[cmdV, cmdW] = feedbackLin(cmdVx, cmdVy, mu(3), params.epsilon);
cmdV = max(-params.maxFwdVel, min(params.maxFwdVel, cmdV));
cmdW = max(-params.maxAngVel, min(params.maxAngVel, cmdW));
[cmdV, cmdW] = limitCmds(cmdV, cmdW, params.maxWheelSpeed, params.wheel2Center);
end

function [particles, weights, mu, sigma, goalWaypoints, navState, dataStore, recoveryOk] = ...
    runPfTemporaryBumpRecovery(Robot, particles, weights, goalWaypoints, navState, ~, bump, params, dataStore, tNow)
stopRobotSafe(Robot);
pause(0.05);

[particles, weights, backedDistance] = executePfRecoveryBackUp(Robot, particles, weights, params);
stopRobotSafe(Robot);
[mu, sigma] = estimatePfPoseAndCov(particles, weights, params);

recoveryOk = true;
dataStore = appendPfRecoveryLog(dataStore, tNow, navState.bumpRecoveryCount, bump, backedDistance, 0, recoveryOk);
end

function [particles, weights, backedDistance] = executePfRecoveryBackUp(Robot, particles, weights, params)
backedDistance = 0;
timerObj = tic;
while backedDistance < params.recoveryBackDistance && toc(timerObj) < params.recoveryMaxBackTime
    SetFwdVelAngVelCreate(Robot, params.recoveryBackVel, 0);
    pause(params.recoveryControlDt);
    odom = readPfRecoveryOdom(Robot);
    if odom.valid
        particles = propagateParticles(particles, odom, params);
        backedDistance = backedDistance + abs(odom.d);
    else
        backedDistance = backedDistance + abs(params.recoveryBackVel) * params.recoveryControlDt;
    end
end
stopRobotSafe(Robot);
end

function odom = readPfRecoveryOdom(Robot)
odom = struct('d', 0, 'phi', 0, 'valid', false);
try
    CreatePort = getCreateSensorPort(Robot);
    odom.d = DistanceSensorRoomba(CreatePort);
    odom.phi = AngleSensorRoomba(CreatePort);
    if ~isfinite(odom.d), odom.d = 0; end
    if ~isfinite(odom.phi), odom.phi = 0; end
    odom.valid = true;
catch
end
end

function livePlot = initializePfLivePlot(params, map, beaconLoc, goalWaypoints, particles, mu, sigma)
livePlot = struct('enabled', false);
if ~isfield(params, 'enableLivePlot') || ~params.enableLivePlot
    return;
end
fig = figure('Name', 'PF Waypoint Live', 'Color', 'w');
ax = axes(fig);
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');
xlabel(ax, 'x (m)');
ylabel(ax, 'y (m)');
title(ax, 'PF Waypoint Live');
for i = 1:size(map, 1)
    if i == 1
        plot(ax, map(i, [1 3]), map(i, [2 4]), 'k-', 'LineWidth', 1.8, 'DisplayName', 'Known map');
    else
        plot(ax, map(i, [1 3]), map(i, [2 4]), 'k-', 'LineWidth', 1.8, 'HandleVisibility', 'off');
    end
end
if ~isempty(beaconLoc)
    scatter(ax, beaconLoc(:, 2), beaconLoc(:, 3), 45, 'g', 'filled', ...
        'MarkerEdgeColor', [0 0.25 0], 'DisplayName', 'Beacons');
end
if ~isempty(goalWaypoints)
    plot(ax, goalWaypoints(:, 1), goalWaypoints(:, 2), 'r--o', ...
        'LineWidth', 1.1, 'MarkerFaceColor', 'r', 'DisplayName', 'Planned goals');
end
particleScatter = scatter(ax, particles(:, 1), particles(:, 2), 12, [0.7 0.7 0.7], ...
    'filled', 'MarkerFaceAlpha', 0.35, 'DisplayName', 'Particles');
pathLine = animatedline(ax, 'Color', 'b', 'LineWidth', 2.0, 'DisplayName', 'PF path');
addpoints(pathLine, mu(1), mu(2));
poseMarker = plot(ax, mu(1), mu(2), 'bo', 'MarkerFaceColor', 'b', ...
    'MarkerSize', 8, 'DisplayName', 'Current PF pose');
headingArrow = quiver(ax, mu(1), mu(2), 0.30 * cos(mu(3)), 0.30 * sin(mu(3)), ...
    0, 'b', 'LineWidth', 1.8, 'MaxHeadSize', 2.0, 'DisplayName', 'Heading');
goalMarker = plot(ax, NaN, NaN, 'mo', 'MarkerFaceColor', 'm', ...
    'MarkerSize', 9, 'DisplayName', 'Current target');
covLine = plot(ax, NaN, NaN, 'm-', 'LineWidth', 1.2, 'DisplayName', '2-sigma covariance');
setPfLivePlotLimits(ax, map, beaconLoc, goalWaypoints, mu);
legend(ax, 'Location', 'bestoutside');
drawnow limitrate;
livePlot = struct('enabled', true, 'fig', fig, 'ax', ax, ...
    'particleScatter', particleScatter, 'pathLine', pathLine, ...
    'poseMarker', poseMarker, 'headingArrow', headingArrow, ...
    'goalMarker', goalMarker, 'covLine', covLine);
livePlot = updatePfLivePlot(livePlot, struct('pfMu', [0 mu.']), goalWaypoints, ...
    struct('currentGoalIdx', 1), particles, mu, sigma);
end

function livePlot = updatePfLivePlot(livePlot, dataStore, goalWaypoints, navState, particles, mu, sigma)
if ~isstruct(livePlot) || ~isfield(livePlot, 'enabled') || ~livePlot.enabled
    return;
end
if ~ishandle(livePlot.fig) || ~ishandle(livePlot.ax)
    livePlot.enabled = false;
    return;
end
set(livePlot.particleScatter, 'XData', particles(:, 1), 'YData', particles(:, 2));
if isfield(dataStore, 'pfMu') && ~isempty(dataStore.pfMu)
    clearpoints(livePlot.pathLine);
    addpoints(livePlot.pathLine, dataStore.pfMu(:, 2), dataStore.pfMu(:, 3));
end
set(livePlot.poseMarker, 'XData', mu(1), 'YData', mu(2));
set(livePlot.headingArrow, ...
    'XData', mu(1), 'YData', mu(2), ...
    'UData', 0.30 * cos(mu(3)), 'VData', 0.30 * sin(mu(3)));
if isfield(navState, 'currentGoalIdx') && navState.currentGoalIdx <= size(goalWaypoints, 1)
    goal = goalWaypoints(navState.currentGoalIdx, :);
    set(livePlot.goalMarker, 'XData', goal(1), 'YData', goal(2));
else
    set(livePlot.goalMarker, 'XData', NaN, 'YData', NaN);
end
[covX, covY] = covariancePfEllipsePoints(mu, sigma);
set(livePlot.covLine, 'XData', covX, 'YData', covY);
drawnow limitrate;
end

function setPfLivePlotLimits(ax, map, beaconLoc, goalWaypoints, mu)
xy = [map(:, 1:2); map(:, 3:4); mu(1:2).'];
if ~isempty(beaconLoc)
    xy = [xy; beaconLoc(:, 2:3)];
end
if ~isempty(goalWaypoints)
    xy = [xy; goalWaypoints(:, 1:2)];
end
xy = xy(all(isfinite(xy), 2), :);
padding = 0.55;
xlim(ax, [min(xy(:, 1)) - padding, max(xy(:, 1)) + padding]);
ylim(ax, [min(xy(:, 2)) - padding, max(xy(:, 2)) + padding]);
end

function [x, y] = covariancePfEllipsePoints(mu, sigma)
x = NaN;
y = NaN;
if ~isequal(size(sigma), [3 3])
    return;
end
P = sigma(1:2, 1:2);
if any(~isfinite(P(:))) || any(~isfinite(mu(1:2)))
    return;
end
P = (P + P.') / 2;
[V, D] = eig(P);
evals = max(diag(D), 0);
if all(evals <= eps)
    return;
end
ang = linspace(0, 2 * pi, 60);
ellipse = mu(1:2) + 2.0 * V * diag(sqrt(evals)) * [cos(ang); sin(ang)];
x = ellipse(1, :);
y = ellipse(2, :);
end

function tBase = getPfDataStoreTimeBase(dataStore)
tBase = 0;
fields = {'odometry', 'rsdepth', 'bump', 'beacon', 'pfMu', 'pfCommand'};
for i = 1:numel(fields)
    name = fields{i};
    if isfield(dataStore, name) && ~isempty(dataStore.(name)) && size(dataStore.(name), 2) >= 1
        tBase = max(tBase, dataStore.(name)(end, 1));
    end
end
if isfield(dataStore, 'init') && isfield(dataStore.init, 'command') && ~isempty(dataStore.init.command)
    tBase = max(tBase, dataStore.init.command(end, 1));
end
end

function beepMask = normalizePfBeepMask(params, numGoals)
% Beep only for explicitly marked scoring goals. Planned intermediate nodes
% and helper navigation targets must stay quiet unless the caller opts in.
beepMask = false(numGoals, 1);
if isfield(params, 'beepMask') && ~isempty(params.beepMask)
    beepMask = normalizePfMaskLength(params.beepMask, numGoals, false);
end
end

function mask = normalizePfMaskLength(mask, targetLen, defaultValue)
if nargin < 3
    defaultValue = false;
end
mask = logical(mask(:));
if numel(mask) < targetLen
    mask(end + 1:targetLen, 1) = logical(defaultValue);
elseif numel(mask) > targetLen
    mask = mask(1:targetLen);
end
end

function dataStore = appendPfRecoveryLog(dataStore, tNow, recoveryIdx, bump, backedDistance, turnedAngle, success)
if ~isfield(dataStore, 'pfRecovery') || isempty(dataStore.pfRecovery)
    dataStore.pfRecovery = [];
end
dataStore.pfRecovery = [dataStore.pfRecovery; ...
    tNow recoveryIdx getPfBumpField(bump, 'right') getPfBumpField(bump, 'left') ...
    getPfBumpField(bump, 'front') backedDistance turnedAngle double(success)];
end

function value = getPfBumpField(bump, fieldName)
if isstruct(bump) && isfield(bump, fieldName)
    value = double(bump.(fieldName));
else
    value = 0;
end
end

function sigma = sanitizePfCovariance(sigma, minDiag)
if ~isequal(size(sigma), [3 3]) || any(~isfinite(sigma(:)))
    sigma = diag(minDiag);
    return;
end
sigma = (sigma + sigma.') / 2;
for i = 1:3
    if sigma(i, i) < minDiag(i)
        sigma(i, i) = minDiag(i);
    end
end
end

function beepPfRobotSafe(Robot)
try
    BeepCreate(Robot);
catch
end
end

function beacons = parsePfRawTagDetections(tagMatrix)
tagMatrix = double(tagMatrix);
numRows = size(tagMatrix, 1);
numCols = size(tagMatrix, 2);
template = struct('tagNum', NaN, 'latency', NaN, 'xCam', NaN, 'yCam', NaN, ...
                  'thetaCam', NaN, 'range', NaN, 'bearing', NaN, 'confidence', 1.0);
beacons = repmat(template, numRows, 1);
for i = 1:numRows
    row = tagMatrix(i, :);
    if numCols >= 5
        latency = row(1); tagNum = row(2); xCam = row(3); yCam = row(4); thetaCam = row(5);
    elseif numCols == 4
        latency = NaN; tagNum = row(1); xCam = row(2); yCam = row(3); thetaCam = row(4);
    elseif numCols == 3
        latency = NaN; tagNum = row(1); xCam = row(2); yCam = row(3); thetaCam = NaN;
    else
        continue;
    end
    beacons(i).tagNum = tagNum;
    beacons(i).latency = latency;
    beacons(i).xCam = xCam;
    beacons(i).yCam = yCam;
    beacons(i).thetaCam = thetaCam;
    beacons(i).range = hypot(xCam, yCam);
    beacons(i).bearing = atan2(yCam, xCam);
    beacons(i).confidence = 1.0;
end
validRows = arrayfun(@(b) isfinite(b.tagNum) && isfinite(b.xCam) && isfinite(b.yCam), beacons);
beacons = beacons(validRows);
end
