function [ekfState, dataStore, navState] = runEkfWaypointFollower( ...
    Robot, map, beaconLoc, goalWaypoints, offset_x, offset_y, initState, ekfParams, dataStore)
% runEkfWaypointFollower EKF localization with a simple waypoint follower.
%
% This module assumes initialization already produced initState.pose and
% initState.poseCov. It does not use overhead localization.

if nargin < 8 || isempty(ekfParams)
    ekfParams = struct();
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

params = ekfWaypointDefaultParams(ekfParams);
params.sensorOffset = [offset_x, offset_y];
params.cameraOffset = [offset_x, offset_y];
params.mapGeom = precomputeKnownMapGeometry(map, params.wallThickness);

dataStore = ensureEkfDataStoreFields(dataStore);

mu = initState.pose(:);
mu(3) = wrapToPiLocal(mu(3));
if isfield(initState, 'poseCov') && isequal(size(initState.poseCov), [3 3])
    sigma = initState.poseCov;
else
    sigma = diag(params.minInitStd .^ 2);
end
sigma = sanitizeCovariance(sigma, params.minInitStd .^ 2);

tBase = getDataStoreTimeBase(dataStore);
tStart = tBase;
dataStore.ekfMu = [dataStore.ekfMu; tStart mu.'];
dataStore.ekfSigma(:, :, end + 1) = sigma;

navState = struct();
navState.goalWaypoints = goalWaypoints;
navState.currentGoalIdx = 1;
navState.reachedGoals = false(size(goalWaypoints, 1), 1);
navState.beepMask = normalizeBeepMask(params, size(goalWaypoints, 1));
navState.finalDistanceToGoal = NaN;
navState.stopReason = '';
navState.bumpRecoveryCount = 0;

stopReason = 'maxTime';
reachedAllGoals = isempty(goalWaypoints);
numUpdates = 0;
livePlot = initializeEkfLivePlot(params, map, beaconLoc, goalWaypoints, mu, sigma);
lastLivePlotUpdate = -inf;

if reachedAllGoals && params.stopIfNoGoals
    stopRobotSafe(Robot);
    stopReason = 'noGoals';
else
    stopRobotSafe(Robot);

    runTimer = tic;
    while toc(runTimer) < params.maxRunTime
        tNow = tBase + toc(runTimer);
        obs = readEkfSensors(Robot, params);
        dataStore = appendEkfSensorLogs(dataStore, tNow, obs);

        if params.stopOnBump && obs.bump.valid && obs.bump.any
            if params.enableBumpRecovery && navState.bumpRecoveryCount < params.maxBumpRecoveries
                [mu, sigma] = ekfPredict(mu, sigma, obs.odom, params);
                navState.bumpRecoveryCount = navState.bumpRecoveryCount + 1;
                [mu, sigma, goalWaypoints, navState, dataStore, recoveryOk] = ...
                    runTemporaryBumpRecovery( ...
                        Robot, mu, sigma, goalWaypoints, navState, map, obs.bump, params, dataStore, tNow);
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

        [mu, sigma, updateStats] = ekfLocalizationStep( ...
            mu, sigma, obs, beaconLoc, params);
        numUpdates = numUpdates + 1;

        prevReachedGoals = navState.reachedGoals;
        [cmdV, cmdW, navState, reachedAllGoals, finalDist] = ...
            computeWaypointCommand(mu, goalWaypoints, navState, params);
        navState.finalDistanceToGoal = finalDist;
        newlyReachedGoals = navState.reachedGoals & ~prevReachedGoals;
        navState.beepMask = normalizeMaskLength(navState.beepMask, numel(navState.reachedGoals), false);
        if any(newlyReachedGoals & navState.beepMask)
            beepRobotSafe(Robot);
        end

        dataStore.ekfMu = [dataStore.ekfMu; tNow mu.'];
        dataStore.ekfSigma(:, :, end + 1) = sigma;
        dataStore.ekfUpdateStats = [dataStore.ekfUpdateStats; ...
            tNow updateStats.beaconDetections updateStats.beaconsUsed ...
            updateStats.depthCandidateBeams updateStats.depthBeamsUsed ...
            double(updateStats.depthSkipped) double(updateStats.beaconSkipped)];
        if params.enableLivePlot && (tNow - lastLivePlotUpdate >= params.livePlotUpdatePeriod || reachedAllGoals)
            livePlot = updateEkfLivePlot(livePlot, dataStore, goalWaypoints, navState, mu, sigma);
            lastLivePlotUpdate = tNow;
        end

        if reachedAllGoals
            stopReason = 'reachedAllGoals';
            dataStore.ekfCommand = [dataStore.ekfCommand; tNow 0 0];
            SetFwdVelAngVelCreate(Robot, 0, 0);
            break;
        end

        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        dataStore.ekfCommand = [dataStore.ekfCommand; tNow cmdV cmdW];
        dataStore.ekfGoal = [dataStore.ekfGoal; ...
            tNow navState.currentGoalIdx ...
            goalWaypoints(navState.currentGoalIdx, :) finalDist];

        if params.debugPrint && mod(numUpdates, params.debugPrintEvery) == 0
            fprintf('[runEkfWaypointFollower] goal=%d dist=%.3f pose=[%.2f %.2f %.1fdeg] beacons=%d depth=%d cmd=[%.2f %.2f]\n', ...
                navState.currentGoalIdx, finalDist, mu(1), mu(2), mu(3) * 180 / pi, ...
                updateStats.beaconsUsed, updateStats.depthBeamsUsed, cmdV, cmdW);
        end

        pause(params.controlDt);
    end
end

stopRobotSafe(Robot);

navState.stopReason = stopReason;
navState.reachedAllGoals = reachedAllGoals;

ekfState = struct();
ekfState.pose = mu;
ekfState.poseCov = sigma;
ekfState.reachedAllGoals = reachedAllGoals;
ekfState.stopReason = stopReason;
ekfState.numUpdates = numUpdates;
ekfState.runTimeSec = max(0, getDataStoreTimeBase(dataStore) - tBase);
ekfState.params = params;

dataStore.ekfFinalState = ekfState;
dataStore.navState = navState;
end

function dataStore = ensureEkfDataStoreFields(dataStore)
numericFields = {'truthPose', 'odometry', 'rsdepth', 'bump', 'beacon', ...
                 'ekfMu', 'ekfCommand', 'ekfGoal', 'ekfUpdateStats', ...
                 'ekfRecovery', 'ekfRecoveryEscape'};
for i = 1:numel(numericFields)
    name = numericFields{i};
    if ~isfield(dataStore, name) || isempty(dataStore.(name))
        dataStore.(name) = [];
    end
end

if ~isfield(dataStore, 'ekfSigma') || isempty(dataStore.ekfSigma)
    dataStore.ekfSigma = zeros(3, 3, 0);
end
if ~isfield(dataStore, 'ekfRawDepthCell') || isempty(dataStore.ekfRawDepthCell)
    dataStore.ekfRawDepthCell = cell(0, 1);
end
if ~isfield(dataStore, 'ekfRawBeaconCell') || isempty(dataStore.ekfRawBeaconCell)
    dataStore.ekfRawBeaconCell = cell(0, 1);
end
end

function tBase = getDataStoreTimeBase(dataStore)
tBase = 0;
fields = {'odometry', 'rsdepth', 'bump', 'beacon', 'ekfMu', 'ekfCommand'};
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

function beepMask = normalizeBeepMask(params, numGoals)
% Beep only for explicitly marked scoring goals. Planned intermediate nodes
% and helper navigation targets must stay quiet unless the caller opts in.
beepMask = false(numGoals, 1);
if isfield(params, 'beepMask') && ~isempty(params.beepMask)
    beepMask = normalizeMaskLength(params.beepMask, numGoals, false);
end
end

function mask = normalizeMaskLength(mask, targetLen, defaultValue)
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

function livePlot = initializeEkfLivePlot(params, map, beaconLoc, goalWaypoints, mu, sigma)
livePlot = struct('enabled', false);
if ~isfield(params, 'enableLivePlot') || ~params.enableLivePlot
    return;
end

fig = figure('Name', 'EKF Waypoint Live', 'Color', 'w');
ax = axes(fig);
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');
xlabel(ax, 'x (m)');
ylabel(ax, 'y (m)');
title(ax, 'EKF Waypoint Live');

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

pathLine = animatedline(ax, 'Color', 'b', 'LineWidth', 2.0, 'DisplayName', 'EKF path');
addpoints(pathLine, mu(1), mu(2));
poseMarker = plot(ax, mu(1), mu(2), 'bo', 'MarkerFaceColor', 'b', ...
    'MarkerSize', 8, 'DisplayName', 'Current EKF pose');
headingArrow = quiver(ax, mu(1), mu(2), 0.30 * cos(mu(3)), 0.30 * sin(mu(3)), ...
    0, 'b', 'LineWidth', 1.8, 'MaxHeadSize', 2.0, 'DisplayName', 'Heading');
goalMarker = plot(ax, NaN, NaN, 'mo', 'MarkerFaceColor', 'm', ...
    'MarkerSize', 9, 'DisplayName', 'Current target');
covLine = plot(ax, NaN, NaN, 'm-', 'LineWidth', 1.2, 'DisplayName', '2-sigma covariance');

setLivePlotLimits(ax, map, beaconLoc, goalWaypoints, mu);
legend(ax, 'Location', 'bestoutside');
drawnow limitrate;

livePlot = struct( ...
    'enabled', true, ...
    'fig', fig, ...
    'ax', ax, ...
    'pathLine', pathLine, ...
    'poseMarker', poseMarker, ...
    'headingArrow', headingArrow, ...
    'goalMarker', goalMarker, ...
    'covLine', covLine);
livePlot = updateEkfLivePlot(livePlot, struct('ekfMu', [0 mu.'], 'ekfSigma', sigma), goalWaypoints, ...
    struct('currentGoalIdx', 1), mu, sigma);
end

function livePlot = updateEkfLivePlot(livePlot, dataStore, goalWaypoints, navState, mu, sigma)
if ~isstruct(livePlot) || ~isfield(livePlot, 'enabled') || ~livePlot.enabled
    return;
end
if ~ishandle(livePlot.fig) || ~ishandle(livePlot.ax)
    livePlot.enabled = false;
    return;
end

if isfield(dataStore, 'ekfMu') && ~isempty(dataStore.ekfMu)
    clearpoints(livePlot.pathLine);
    addpoints(livePlot.pathLine, dataStore.ekfMu(:, 2), dataStore.ekfMu(:, 3));
end

set(livePlot.poseMarker, 'XData', mu(1), 'YData', mu(2));
set(livePlot.headingArrow, ...
    'XData', mu(1), ...
    'YData', mu(2), ...
    'UData', 0.30 * cos(mu(3)), ...
    'VData', 0.30 * sin(mu(3)));

if isfield(navState, 'currentGoalIdx') && navState.currentGoalIdx <= size(goalWaypoints, 1)
    goal = goalWaypoints(navState.currentGoalIdx, :);
    set(livePlot.goalMarker, 'XData', goal(1), 'YData', goal(2));
else
    set(livePlot.goalMarker, 'XData', NaN, 'YData', NaN);
end

[covX, covY] = covarianceEllipsePoints(mu, sigma);
set(livePlot.covLine, 'XData', covX, 'YData', covY);
drawnow limitrate;
end

function setLivePlotLimits(ax, map, beaconLoc, goalWaypoints, mu)
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

function [x, y] = covarianceEllipsePoints(mu, sigma)
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

function obs = readEkfSensors(Robot, params)
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
        obs.beacons = parseRawTagDetections(tagMatrix);
    end
catch
end
end

function dataStore = appendEkfSensorLogs(dataStore, tNow, obs)
if obs.odom.valid
    dataStore.odometry = [dataStore.odometry; tNow obs.odom.d obs.odom.phi];
end

if ~isempty(obs.depthRaw)
    [dataStore, appended] = appendNumericRow(dataStore, 'rsdepth', [tNow obs.depthRaw(:).']);
    if ~appended
        dataStore.ekfRawDepthCell{end + 1, 1} = [tNow obs.depthRaw(:).'];
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
    [dataStore, appended] = appendNumericRows(dataStore, 'beacon', rows);
    if ~appended
        dataStore.ekfRawBeaconCell{end + 1, 1} = rows;
    end
end
end

function [dataStore, appended] = appendNumericRow(dataStore, fieldName, row)
[dataStore, appended] = appendNumericRows(dataStore, fieldName, row);
end

function [dataStore, appended] = appendNumericRows(dataStore, fieldName, rows)
appended = false;
if ~isfield(dataStore, fieldName) || isempty(dataStore.(fieldName))
    dataStore.(fieldName) = rows;
    appended = true;
elseif size(dataStore.(fieldName), 2) == size(rows, 2)
    dataStore.(fieldName) = [dataStore.(fieldName); rows];
    appended = true;
end
end

function [mu, sigma, stats] = ekfLocalizationStep(mu, sigma, obs, beaconLoc, params)
stats = struct( ...
    'beaconDetections', numel(obs.beacons), ...
    'beaconsUsed', 0, ...
    'depthCandidateBeams', 0, ...
    'depthBeamsUsed', 0, ...
    'depthSkipped', false, ...
    'beaconSkipped', false);

[mu, sigma] = ekfPredict(mu, sigma, obs.odom, params);

if params.useBeacons
    [mu, sigma, usedBeacons, beaconSkipped] = ekfBeaconUpdate(mu, sigma, obs, beaconLoc, params);
    stats.beaconsUsed = usedBeacons;
    stats.beaconSkipped = beaconSkipped;
end

if params.useDepth
    [mu, sigma, depthStats] = ekfDepthUpdate(mu, sigma, obs, params.mapGeom, params);
    stats.depthCandidateBeams = depthStats.candidateBeams;
    stats.depthBeamsUsed = depthStats.usedBeams;
    stats.depthSkipped = depthStats.skipped;
end

sigma = sanitizeCovariance(sigma, [1e-8; 1e-8; 1e-8]);
end

function [muBar, sigmaBar] = ekfPredict(mu, sigma, odom, params)
if odom.valid
    u = [odom.d; odom.phi];
    muBar = integrateOdom(mu, u(1), u(2));
    muBar = muBar(:, end);
    muBar(3) = wrapToPiLocal(muBar(3));

    G = GjacDiffDrive(mu, u);
    R = processNoiseFromOdom(u, params);
    sigmaBar = G * sigma * G' + R;
else
    muBar = mu;
    R = diag([0, 0, params.noOdomThetaStd .^ 2]);
    sigmaBar = sigma + R;
end
sigmaBar = (sigmaBar + sigmaBar.') / 2;
end

function R = processNoiseFromOdom(u, params)
d = abs(u(1));
phi = abs(u(2));
xyStd = params.processXYStdBase + params.processXYStdPerMeter * d;
thetaStd = params.processThetaStdBase + params.processThetaStdPerRad * phi;
R = diag([xyStd .^ 2, xyStd .^ 2, thetaStd .^ 2]);
end

function [mu, sigma, usedCount, skipped] = ekfBeaconUpdate(mu, sigma, obs, beaconLoc, params)
usedCount = 0;
skipped = false;
if isempty(obs.beacons) || isempty(beaconLoc)
    skipped = true;
    return;
end

for i = 1:numel(obs.beacons)
    if usedCount >= params.beaconMaxUpdatesPerStep
        break;
    end

    b = obs.beacons(i);
    if ~isfinite(b.tagNum) || ~isfinite(b.xCam) || ~isfinite(b.yCam)
        skipped = true;
        continue;
    end

    zHat = beaconMeasurementModel(mu, beaconLoc, params.cameraOffset, b.tagNum);
    if any(~isfinite(zHat)) || zHat(1) < params.beaconMinPredictedX
        skipped = true;
        continue;
    end

    z = [b.xCam; b.yCam];
    innov = z - zHat;
    if norm(innov) > params.beaconResidualGateXY
        skipped = true;
        continue;
    end

    H = numericalJacobian(@(x) beaconMeasurementModel(x, beaconLoc, params.cameraOffset, b.tagNum), ...
                          mu, params.jacobianEps);
    Q = diag([params.beaconSigmaX .^ 2, params.beaconSigmaY .^ 2]);
    [mu, sigma, ok] = ekfCorrect(mu, sigma, innov, H, Q);
    if ok
        usedCount = usedCount + 1;
    else
        skipped = true;
    end
end
end

function [mu, sigma, depthStats] = ekfDepthUpdate(mu, sigma, obs, mapGeom, params)
depthStats = struct('candidateBeams', 0, 'usedBeams', 0, 'skipped', false);
if isempty(obs.depthRanges) || isempty(obs.depthAngles) || isempty(obs.validDepthMask)
    depthStats.skipped = true;
    return;
end

validIdx = find(obs.validDepthMask(:) & isfinite(obs.depthRanges(:)));
if isempty(validIdx)
    depthStats.skipped = true;
    return;
end

if numel(validIdx) > params.depthMaxBeams
    samplePos = unique(round(linspace(1, numel(validIdx), params.depthMaxBeams)), 'stable');
    validIdx = validIdx(samplePos);
end

zCandidate = obs.depthRanges(validIdx);
angleCandidate = obs.depthAngles(validIdx);
zHatCandidate = raycastKnownMap(mu, mapGeom, angleCandidate, params.sensorOffset, params);
residual = zCandidate - zHatCandidate;

good = isfinite(zCandidate) & isfinite(zHatCandidate) & ...
       abs(residual) <= params.depthResidualGate;
depthStats.candidateBeams = numel(validIdx);

if nnz(good) < params.depthMinBeamsForUpdate
    depthStats.skipped = true;
    return;
end

z = zCandidate(good);
zHat = zHatCandidate(good);
beamAngles = angleCandidate(good);
innov = z - zHat;

H = numericalJacobian(@(x) raycastKnownMap(x, mapGeom, beamAngles, params.sensorOffset, params), ...
                      mu, params.jacobianEps);
finiteRows = all(isfinite(H), 2) & isfinite(innov);
if nnz(finiteRows) < params.depthMinBeamsForUpdate
    depthStats.skipped = true;
    return;
end

innov = innov(finiteRows);
H = H(finiteRows, :);
Q = params.depthSigma .^ 2 * eye(numel(innov));

[mu, sigma, ok] = ekfCorrect(mu, sigma, innov, H, Q);
if ok
    depthStats.usedBeams = numel(innov);
else
    depthStats.skipped = true;
end
end

function [mu, sigma, ok] = ekfCorrect(mu, sigma, innovation, H, Q)
ok = false;
if isempty(innovation) || isempty(H) || any(~isfinite(innovation)) || any(~isfinite(H(:)))
    return;
end

S = H * sigma * H' + Q;
if rcond(S) < 1e-12
    return;
end

K = sigma * H' / S;
mu = mu + K * innovation;
mu(3) = wrapToPiLocal(mu(3));

I = eye(3);
sigma = (I - K * H) * sigma * (I - K * H)' + K * Q * K';
sigma = (sigma + sigma.') / 2;
ok = true;
end

function zHat = beaconMeasurementModel(mu, beaconLoc, cameraOffset, tagNum)
idx = find(beaconLoc(:, 1) == tagNum, 1, 'first');
if isempty(idx)
    zHat = [NaN; NaN];
    return;
end

xyRobot = global2robot(mu(:).', beaconLoc(idx, 2:3));
zHat = [xyRobot(1) - cameraOffset(1); xyRobot(2) - cameraOffset(2)];
end

function H = numericalJacobian(measureFun, x, epsVec)
x = x(:);
y0 = measureFun(x);
H = zeros(numel(y0), numel(x));

for j = 1:numel(x)
    epsj = epsVec(min(j, numel(epsVec)));
    xp = x;
    xm = x;
    xp(j) = xp(j) + epsj;
    xm(j) = xm(j) - epsj;
    if j == 3
        xp(j) = wrapToPiLocal(xp(j));
        xm(j) = wrapToPiLocal(xm(j));
    end

    yp = measureFun(xp);
    ym = measureFun(xm);
    if numel(yp) == numel(y0) && numel(ym) == numel(y0) && ...
            all(isfinite(yp)) && all(isfinite(ym))
        H(:, j) = (yp(:) - ym(:)) / (2 * epsj);
    end
end
end

function [cmdV, cmdW, navState, reachedAllGoals, finalDist] = ...
    computeWaypointCommand(mu, goalWaypoints, navState, params)
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

function [mu, sigma, goalWaypoints, navState, dataStore, recoveryOk] = ...
    runTemporaryBumpRecovery(Robot, mu, sigma, goalWaypoints, navState, map, bump, params, dataStore, tNow)
% runTemporaryBumpRecovery Back up, turn away, and replan to current goal.

recoveryOk = false;
stopRobotSafe(Robot);
pause(0.05);

[mu, sigma, backedDistance] = executeRecoveryBackUp(Robot, mu, sigma, params);
turnDir = chooseRecoveryTurnDirection(bump);
[mu, sigma, turnedAngle] = executeRecoveryTurn(Robot, mu, sigma, turnDir, params);
stopRobotSafe(Robot);

[mu, sigma, forwardDistance] = executeRecoveryForwardNudge(Robot, mu, sigma, params);

stopRobotSafe(Robot);
if navState.currentGoalIdx > size(goalWaypoints, 1)
    recoveryOk = true;
    dataStore = appendRecoveryLog(dataStore, tNow, navState.bumpRecoveryCount, bump, backedDistance, turnedAngle, recoveryOk);
    return;
end

currentGoal = goalWaypoints(navState.currentGoalIdx, :);
plannerParams = knownMapPlannerDefaultParams();
[replannedPath, plannerInfo] = planPathKnownMap(mu(1:2).', currentGoal, map, plannerParams);

if isempty(replannedPath) || ~plannerInfo.success
    [replannedPath, escapeInfo] = planRecoveryEscapePath(mu, currentGoal, map, plannerParams, params);
    dataStore = appendRecoveryEscapeLog(dataStore, tNow, navState.bumpRecoveryCount, escapeInfo);
    if isempty(replannedPath) || ~escapeInfo.success
        dataStore = appendRecoveryLog(dataStore, tNow, navState.bumpRecoveryCount, bump, backedDistance, turnedAngle, false);
        return;
    end
    plannerInfo = escapeInfo;
end

remainingOriginalGoals = goalWaypoints(navState.currentGoalIdx + 1:end, :);
currentGoalShouldBeep = false;
if isfield(navState, 'beepMask') && navState.currentGoalIdx <= numel(navState.beepMask)
    currentGoalShouldBeep = navState.beepMask(navState.currentGoalIdx);
end
remainingBeepMask = false(size(remainingOriginalGoals, 1), 1);
if isfield(navState, 'beepMask') && navState.currentGoalIdx < numel(navState.beepMask)
    remainingBeepMask = navState.beepMask(navState.currentGoalIdx + 1:end);
    remainingBeepMask = normalizeMaskLength(remainingBeepMask, size(remainingOriginalGoals, 1), false);
end
replannedBeepMask = [false(max(0, size(replannedPath, 1) - 1), 1); currentGoalShouldBeep];
goalWaypoints = [replannedPath; remainingOriginalGoals];
newBeepMask = [replannedBeepMask; remainingBeepMask];
[goalWaypoints, newBeepMask] = removeConsecutiveDuplicateGoalsWithMask(goalWaypoints, newBeepMask, 0.03);

navState.goalWaypoints = goalWaypoints;
navState.currentGoalIdx = 1;
navState.reachedGoals = false(size(goalWaypoints, 1), 1);
navState.beepMask = normalizeMaskLength(newBeepMask, size(goalWaypoints, 1), false);
navState.finalDistanceToGoal = NaN;
navState.lastRecoveryPlanInfo = plannerInfo;

recoveryOk = true;
dataStore = appendRecoveryLog(dataStore, tNow, navState.bumpRecoveryCount, bump, backedDistance, turnedAngle, recoveryOk);
end

function [escapePath, escapeInfo] = planRecoveryEscapePath(mu, currentGoal, map, plannerParams, params)
% planRecoveryEscapePath Adds one local waypoint away from the nearest wall,
% then plans normally from that escape point to the original current goal.

robotXY = mu(1:2).';
currentGoal = currentGoal(:).';
escapePath = zeros(0, 2);
escapeInfo = struct();
escapeInfo.success = false;
escapeInfo.reason = 'noEscapeCandidate';
escapeInfo.escapePoint = [NaN NaN];
escapeInfo.escapeClearance = NaN;
escapeInfo.plannerInfo = [];

[~, closestPoint] = nearestWallPointLocal(robotXY, map);
baseDir = robotXY - closestPoint;
if norm(baseDir) < 1e-6
    baseDir = [cos(mu(3)), sin(mu(3))];
end
baseDir = baseDir / norm(baseDir);

bounds = inferMapBoundsLocal(map);
bestScore = inf;
distances = params.recoveryEscapeDistances(:).';
angleOffsets = params.recoveryEscapeAngleOffsets(:).';

for dist = distances
    for angleOffset = angleOffsets
        dir = rotateVector2d(baseDir, angleOffset);
        candidate = robotXY + dist * dir;

        if ~pointInsideBoundsLocal(candidate, bounds, params.recoveryEscapeBoundsMargin)
            continue;
        end
        if segmentIntersectsAnyWallLocal(robotXY, candidate, map)
            continue;
        end

        clearance = pointWallClearanceLocal(candidate, map);
        if clearance < params.recoveryEscapeMinWallClearance
            continue;
        end

        [pathFromEscape, pathInfo] = planPathKnownMap(candidate, currentGoal, map, plannerParams);
        if isempty(pathFromEscape) || ~pathInfo.success
            continue;
        end

        candidatePath = [candidate; pathFromEscape(2:end, :)];
        candidatePath = removeConsecutiveDuplicateGoals(candidatePath, 0.03);
        score = pathLengthLocal(candidatePath) - 0.25 * clearance;
        if score < bestScore
            bestScore = score;
            escapePath = candidatePath;
            escapeInfo.success = true;
            escapeInfo.reason = 'escapeThenPlan';
            escapeInfo.escapePoint = candidate;
            escapeInfo.escapeClearance = clearance;
            escapeInfo.plannerInfo = pathInfo;
            escapeInfo.score = score;
        end
    end
end
end

function [mu, sigma, backedDistance] = executeRecoveryBackUp(Robot, mu, sigma, params)
backedDistance = 0;
timerObj = tic;

while backedDistance < params.recoveryBackDistance && toc(timerObj) < params.recoveryMaxBackTime
    SetFwdVelAngVelCreate(Robot, params.recoveryBackVel, 0);
    pause(params.recoveryControlDt);
    odom = readRecoveryOdom(Robot);
    if odom.valid
        [mu, sigma] = predictRecoveryState(mu, sigma, odom, params);
        backedDistance = backedDistance + abs(odom.d);
    else
        backedDistance = backedDistance + abs(params.recoveryBackVel) * params.recoveryControlDt;
    end
end

stopRobotSafe(Robot);
end

function [mu, sigma, turnedAngle] = executeRecoveryTurn(Robot, mu, sigma, turnDir, params)
turnedAngle = 0;
timerObj = tic;
targetTurn = abs(params.recoveryTurnAngle);

while abs(turnedAngle) < targetTurn && toc(timerObj) < params.recoveryMaxTurnTime
    SetFwdVelAngVelCreate(Robot, 0, turnDir * abs(params.recoveryTurnVel));
    pause(params.recoveryControlDt);
    odom = readRecoveryOdom(Robot);
    if odom.valid
        [mu, sigma] = predictRecoveryState(mu, sigma, odom, params);
        turnedAngle = turnedAngle + odom.phi;
    else
        turnedAngle = turnedAngle + turnDir * abs(params.recoveryTurnVel) * params.recoveryControlDt;
    end
end

stopRobotSafe(Robot);
end


function [mu, sigma, forwardDistance] = executeRecoveryForwardNudge(Robot, mu, sigma, params)

forwardDistance = 0;
timerObj = tic;

while forwardDistance < params.recoveryForwardDistance && ...
        toc(timerObj) < params.recoveryMaxForwardTime

    SetFwdVelAngVelCreate(Robot, params.recoveryForwardVel, 0);
    pause(params.controlDt);

    odom = readRecoveryOdom(Robot);

    if odom.valid
        [mu, sigma] = predictRecoveryState(mu, sigma, odom, params);
        forwardDistance = forwardDistance + odom.d;
    else
        forwardDistance = forwardDistance + params.recoveryForwardVel * params.controlDt;
    end
end

stopRobotSafe(Robot);

end

function turnDir = chooseRecoveryTurnDirection(bump)
% Positive angular velocity is left turn in the course convention.
if isfield(bump, 'right') && bump.right && ~(isfield(bump, 'left') && bump.left)
    turnDir = 1;
elseif isfield(bump, 'left') && bump.left && ~(isfield(bump, 'right') && bump.right)
    turnDir = -1;
else
    turnDir = 1;
end
end

function odom = readRecoveryOdom(Robot)
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

function [mu, sigma] = predictRecoveryState(mu, sigma, odom, params)
u = [odom.d; odom.phi];
muPrev = mu;
mu = integrateOdom(muPrev, u(1), u(2));
mu = mu(:, end);
mu(3) = wrapToPiLocal(mu(3));

G = GjacDiffDrive(muPrev, u);
R = processNoiseFromOdom(u, params);
sigma = G * sigma * G' + R;
sigma = (sigma + sigma.') / 2;
end

function goals = removeConsecutiveDuplicateGoals(goals, tol)
if isempty(goals)
    return;
end
keep = true(size(goals, 1), 1);
for i = 2:size(goals, 1)
    if norm(goals(i, :) - goals(i - 1, :)) < tol
        keep(i) = false;
    end
end
goals = goals(keep, :);
end

function [goals, mask] = removeConsecutiveDuplicateGoalsWithMask(goals, mask, tol)
if isempty(goals)
    mask = false(0, 1);
    return;
end
mask = normalizeMaskLength(mask, size(goals, 1), false);
keep = true(size(goals, 1), 1);
for i = 2:size(goals, 1)
    if norm(goals(i, :) - goals(i - 1, :)) < tol
        keep(i) = false;
        mask(i - 1) = mask(i - 1) || mask(i);
    end
end
goals = goals(keep, :);
mask = mask(keep);
end

function dataStore = appendRecoveryLog(dataStore, tNow, recoveryIdx, bump, backedDistance, turnedAngle, success)
if ~isfield(dataStore, 'ekfRecovery') || isempty(dataStore.ekfRecovery)
    dataStore.ekfRecovery = [];
end

bumpRight = getBumpField(bump, 'right');
bumpLeft = getBumpField(bump, 'left');
bumpFront = getBumpField(bump, 'front');
dataStore.ekfRecovery = [dataStore.ekfRecovery; ...
    tNow recoveryIdx bumpRight bumpLeft bumpFront backedDistance turnedAngle double(success)];
end

function dataStore = appendRecoveryEscapeLog(dataStore, tNow, recoveryIdx, escapeInfo)
if ~isfield(dataStore, 'ekfRecoveryEscape') || isempty(dataStore.ekfRecoveryEscape)
    dataStore.ekfRecoveryEscape = [];
end

escapePoint = [NaN NaN];
escapeClearance = NaN;
success = false;
if isstruct(escapeInfo)
    if isfield(escapeInfo, 'escapePoint')
        escapePoint = escapeInfo.escapePoint;
    end
    if isfield(escapeInfo, 'escapeClearance')
        escapeClearance = escapeInfo.escapeClearance;
    end
    if isfield(escapeInfo, 'success')
        success = escapeInfo.success;
    end
end

dataStore.ekfRecoveryEscape = [dataStore.ekfRecoveryEscape; ...
    tNow recoveryIdx escapePoint(1) escapePoint(2) escapeClearance double(success)];
end

function value = getBumpField(bump, fieldName)
if isstruct(bump) && isfield(bump, fieldName)
    value = double(bump.(fieldName));
else
    value = 0;
end
end

function [dMin, closestPoint] = nearestWallPointLocal(p, map)
dMin = inf;
closestPoint = p;
for i = 1:size(map, 1)
    [d, proj] = pointSegmentDistanceLocal(p, map(i, 1:2), map(i, 3:4));
    if d < dMin
        dMin = d;
        closestPoint = proj;
    end
end
end

function dMin = pointWallClearanceLocal(p, map)
dMin = inf;
for i = 1:size(map, 1)
    d = pointSegmentDistanceLocal(p, map(i, 1:2), map(i, 3:4));
    dMin = min(dMin, d);
end
end

function [d, proj] = pointSegmentDistanceLocal(p, a, b)
ab = b - a;
den = dot(ab, ab);
if den < eps
    proj = a;
    d = norm(p - a);
    return;
end
t = dot(p - a, ab) / den;
t = max(0, min(1, t));
proj = a + t * ab;
d = norm(p - proj);
end

function bounds = inferMapBoundsLocal(map)
xs = [map(:, 1); map(:, 3)];
ys = [map(:, 2); map(:, 4)];
bounds = [min(xs), max(xs), min(ys), max(ys)];
end

function tf = pointInsideBoundsLocal(p, bounds, margin)
tf = p(1) >= bounds(1) + margin && ...
     p(1) <= bounds(2) - margin && ...
     p(2) >= bounds(3) + margin && ...
     p(2) <= bounds(4) - margin;
end

function vRot = rotateVector2d(v, angleRad)
c = cos(angleRad);
s = sin(angleRad);
vRot = [c * v(1) - s * v(2), s * v(1) + c * v(2)];
end

function tf = segmentIntersectsAnyWallLocal(p1, p2, map)
tf = false;
for i = 1:size(map, 1)
    if segmentsIntersectLocal(p1, p2, map(i, 1:2), map(i, 3:4))
        tf = true;
        return;
    end
end
end

function tf = segmentsIntersectLocal(a, b, c, d)
tol = 1e-9;
o1 = orientationLocal(a, b, c);
o2 = orientationLocal(a, b, d);
o3 = orientationLocal(c, d, a);
o4 = orientationLocal(c, d, b);

tf = false;
if o1 * o2 < -tol && o3 * o4 < -tol
    tf = true;
    return;
end
if abs(o1) <= tol && onSegmentLocal(a, c, b), tf = true; return; end
if abs(o2) <= tol && onSegmentLocal(a, d, b), tf = true; return; end
if abs(o3) <= tol && onSegmentLocal(c, a, d), tf = true; return; end
if abs(o4) <= tol && onSegmentLocal(c, b, d), tf = true; return; end
end

function val = orientationLocal(a, b, c)
val = (b(1) - a(1)) * (c(2) - a(2)) - (b(2) - a(2)) * (c(1) - a(1));
end

function tf = onSegmentLocal(a, b, c)
tol = 1e-9;
tf = b(1) >= min(a(1), c(1)) - tol && b(1) <= max(a(1), c(1)) + tol && ...
     b(2) >= min(a(2), c(2)) - tol && b(2) <= max(a(2), c(2)) + tol;
end

function len = pathLengthLocal(path)
if size(path, 1) < 2
    len = 0;
    return;
end
diffs = diff(path, 1, 1);
len = sum(sqrt(sum(diffs .^ 2, 2)));
end

function beepRobotSafe(Robot)
try
    BeepCreate(Robot);
catch
    % Beep is diagnostic only; never interrupt navigation for it.
end
end

function sigma = sanitizeCovariance(sigma, minDiag)
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

function beacons = parseRawTagDetections(tagMatrix)
tagMatrix = double(tagMatrix);
numRows = size(tagMatrix, 1);
numCols = size(tagMatrix, 2);

template = struct('tagNum', NaN, 'latency', NaN, 'xCam', NaN, 'yCam', NaN, ...
                  'thetaCam', NaN, 'range', NaN, 'bearing', NaN, 'confidence', 1.0);
beacons = repmat(template, numRows, 1);

for i = 1:numRows
    row = tagMatrix(i, :);
    if numCols >= 5
        latency = row(1);
        tagNum = row(2);
        xCam = row(3);
        yCam = row(4);
        thetaCam = row(5);
    elseif numCols == 4
        latency = NaN;
        tagNum = row(1);
        xCam = row(2);
        yCam = row(3);
        thetaCam = row(4);
    elseif numCols == 3
        latency = NaN;
        tagNum = row(1);
        xCam = row(2);
        yCam = row(3);
        thetaCam = NaN;
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
