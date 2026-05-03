function [wallStatus, dataStore, verifiedMap] = verifyOptionalWalls( ...
    Robot, map, optWalls, beaconLoc, offset_x, offset_y, initState, ekfParams, plannerParams, verifyParams, dataStore)
% verifyOptionalWalls Actively visit and classify optional walls.
%
% Each optional wall is verified by driving to a generated observation point,
% turning toward the wall midpoint, collecting depth, and comparing known-map
% raycast residuals with known+candidate-wall residuals.

if nargin < 8 || isempty(ekfParams)
    ekfParams = struct();
end
if nargin < 9 || isempty(plannerParams)
    plannerParams = struct();
end
if nargin < 10 || isempty(verifyParams)
    verifyParams = struct();
end
if nargin < 11 || isempty(dataStore)
    dataStore = struct();
end

if isempty(optWalls)
    wallStatus = repmat(emptyWallStatus(), 0, 1);
    verifiedMap = map;
    return;
end

params = optionalWallDefaultParams(verifyParams);
trackingParams = normalizeOptionalWallFollowerParams(ekfParams, params);
plannerParams = knownMapPlannerDefaultParams(plannerParams);
plannerParams = applyOptionalWallNavigationClearance(plannerParams, params);

currentState = initState;
confirmedOptionalWalls = zeros(0, 4);
wallStatus = repmat(emptyWallStatus(), size(optWalls, 1), 1);

for i = 1:size(optWalls, 1)
    currentMap = [map; confirmedOptionalWalls];
    wall = optWalls(i, :);
    displayWallIdx = getOptionalWallDisplayIdx(i, params);

    [observePoint, observeInfo] = chooseOptionalWallObservePoint( ...
        currentState.pose(1:2).', wall, currentMap, plannerParams, params);

    wallStatus(i).wallIdx = displayWallIdx;
    wallStatus(i).wall = wall;
    wallStatus(i).observePoint = observePoint;
    wallStatus(i).observeInfo = observeInfo;

    if isempty(observePoint)
        [currentState, dataStore, fallbackStatus] = runOptionalWallBumpFallback( ...
            Robot, currentState, wall, currentMap, beaconLoc, offset_x, offset_y, ...
            trackingParams, plannerParams, params, dataStore, i);
        wallStatus(i) = mergeFallbackWallStatus(wallStatus(i), fallbackStatus);
        if strcmp(wallStatus(i).status, 'exists')
            confirmedOptionalWalls = [confirmedOptionalWalls; wall]; %#ok<AGROW>
        end
        continue;
    end

    [pathToObserve, planInfo] = planPathKnownMap( ...
        currentState.pose(1:2).', observePoint, currentMap, plannerParams);
    wallStatus(i).planInfo = planInfo;

    if isempty(pathToObserve) || ~planInfo.success
        wallStatus(i).status = 'unknown';
        wallStatus(i).reason = ['planFailed_' planInfo.reason];
        if params.stopIfNavigationFails
            break;
        end
        continue;
    end

    followerParams = trackingParams;
    followerParams.maxRunTime = params.followerMaxRunTime;
    [trackingState, dataStore, navState] = runOptionalWallFollower( ...
        Robot, currentMap, beaconLoc, pathToObserve, offset_x, offset_y, ...
        currentState, followerParams, params, dataStore);

    wallStatus(i).pathToObserve = pathToObserve;
    wallStatus(i).navState = navState;
    wallStatus(i) = storeOptionalWallTrackingState(wallStatus(i), trackingState, 'Observe', params);

    currentState = updateOptionalWallCurrentState(currentState, trackingState);

    if ~trackingState.reachedAllGoals
        wallStatus(i).status = 'unknown';
        wallStatus(i).reason = ['navigationFailed_' trackingState.stopReason];
        if params.stopIfNavigationFails
            break;
        end
        continue;
    end

    wallMid = wallMidpoint(wall);
    [currentState, dataStore, turnInfo] = turnToFacePointWithEkf( ...
        Robot, currentState, wallMid, params, dataStore);
    wallStatus(i).turnInfo = turnInfo;
    wallStatus(i) = storeOptionalWallCurrentState(wallStatus(i), currentState, 'Classify', params);

    depthObs = collectOptionalWallDepth(Robot, params);
    dataStore = appendOptionalWallDepthLog(dataStore, i, depthObs);

    classification = classifyOptionalWallFromDepth( ...
        currentState.pose, depthObs, currentMap, wall, [offset_x offset_y], params);

    bumpProbe = struct();
    if shouldRunBumpProbe(classification, params)
        [currentState, dataStore, bumpProbe] = runOptionalWallBumpProbe( ...
            Robot, currentState, wall, params, dataStore, i);
        classification = fuseDepthAndBumpProbe(classification, bumpProbe);
    end

    wallStatus(i).status = classification.status;
    wallStatus(i).confidence = classification.confidence;
    wallStatus(i).reason = classification.reason;
    wallStatus(i).numFrames = depthObs.numFrames;
    wallStatus(i).numRaysUsed = classification.numRaysUsed;
    wallStatus(i).errPresent = classification.errPresent;
    wallStatus(i).errAbsent = classification.errAbsent;
    wallStatus(i).classification = classification;
    wallStatus(i).bumpProbe = bumpProbe;

    if strcmp(classification.status, 'exists')
        confirmedOptionalWalls = [confirmedOptionalWalls; wall]; %#ok<AGROW>
    end

    if params.debugPrint
        fprintf('[verifyOptionalWalls] wall %d status=%s conf=%.2f rays=%d absentErr=%.3f presentErr=%.3f\n', ...
            displayWallIdx, wallStatus(i).status, wallStatus(i).confidence, wallStatus(i).numRaysUsed, ...
            wallStatus(i).errAbsent, wallStatus(i).errPresent);
    end
end

verifiedMap = [map; confirmedOptionalWalls];
dataStore.optionalWallStatus = wallStatus;
dataStore.verifiedMap = verifiedMap;
end

function plannerParams = applyOptionalWallNavigationClearance(plannerParams, verifyParams)
% Keep optional-wall approach paths slightly farther from known walls than the
% normal waypoint planner without changing the global planner defaults.
if isfield(verifyParams, 'navigationEdgeClearance') && isfinite(verifyParams.navigationEdgeClearance)
    plannerParams.edgeClearance = max(plannerParams.edgeClearance, verifyParams.navigationEdgeClearance);
end
if isfield(verifyParams, 'navigationNodeClearance') && isfinite(verifyParams.navigationNodeClearance)
    plannerParams.nodeClearance = max(plannerParams.nodeClearance, verifyParams.navigationNodeClearance);
else
    plannerParams.nodeClearance = max(plannerParams.nodeClearance, plannerParams.edgeClearance);
end
if isfield(verifyParams, 'navigationCornerOffsetRadius') && isfinite(verifyParams.navigationCornerOffsetRadius)
    plannerParams.cornerOffsetRadius = max(plannerParams.cornerOffsetRadius, verifyParams.navigationCornerOffsetRadius);
else
    plannerParams.cornerOffsetRadius = max(plannerParams.cornerOffsetRadius, plannerParams.edgeClearance + 0.12);
end
if isfield(verifyParams, 'navigationStartGoalClearance') && isfinite(verifyParams.navigationStartGoalClearance)
    plannerParams.startGoalClearance = max(plannerParams.startGoalClearance, verifyParams.navigationStartGoalClearance);
end
end

function displayIdx = getOptionalWallDisplayIdx(localIdx, params)
displayIdx = localIdx;
if isfield(params, 'wallIdxLabels') && numel(params.wallIdxLabels) >= localIdx && ...
        isfinite(params.wallIdxLabels(localIdx))
    displayIdx = params.wallIdxLabels(localIdx);
end
end

function trackingParams = normalizeOptionalWallFollowerParams(paramsIn, verifyParams)
mode = getOptionalWallFollowerMode(verifyParams);
if strcmp(mode, 'pf')
    trackingParams = pfWaypointDefaultParams(paramsIn);
    if isfield(verifyParams, 'pfFollowerUseDepth')
        trackingParams.useDepth = verifyParams.pfFollowerUseDepth;
    end
    if isfield(verifyParams, 'pfFollowerDepthWeight')
        trackingParams.depthWeight = verifyParams.pfFollowerDepthWeight;
    end
    if isfield(verifyParams, 'pfFollowerUseBeacons')
        trackingParams.useBeacons = verifyParams.pfFollowerUseBeacons;
    end
else
    trackingParams = ekfWaypointDefaultParams(paramsIn);
    if isfield(verifyParams, 'ekfFollowerUseDepth')
        trackingParams.useDepth = verifyParams.ekfFollowerUseDepth;
    end
    if isfield(verifyParams, 'ekfFollowerDepthSigma')
        trackingParams.depthSigma = verifyParams.ekfFollowerDepthSigma;
    end
    if isfield(verifyParams, 'ekfFollowerDepthResidualGate')
        trackingParams.depthResidualGate = verifyParams.ekfFollowerDepthResidualGate;
    end
    if isfield(verifyParams, 'ekfFollowerDepthMinBeamsForUpdate')
        trackingParams.depthMinBeamsForUpdate = verifyParams.ekfFollowerDepthMinBeamsForUpdate;
    end
end
end

function mode = getOptionalWallFollowerMode(params)
mode = 'ekf';
if isfield(params, 'followerMode') && ~isempty(params.followerMode)
    mode = lower(string(params.followerMode));
    mode = char(mode);
end
if ~strcmp(mode, 'pf')
    mode = 'ekf';
end
end

function [trackingState, dataStore, navState] = runOptionalWallFollower( ...
    Robot, map, beaconLoc, goalWaypoints, offset_x, offset_y, currentState, followerParams, verifyParams, dataStore)
% Optional-wall observation/probe navigation points are not scoring goals.
followerParams.beepMask = false(size(goalWaypoints, 1), 1);
followerParams.disableBeep = true;
if strcmp(getOptionalWallFollowerMode(verifyParams), 'pf')
    [trackingState, dataStore, navState] = runPfWaypointFollower( ...
        Robot, map, beaconLoc, goalWaypoints, offset_x, offset_y, ...
        currentState, followerParams, dataStore);
else
    [trackingState, dataStore, navState] = runEkfWaypointFollower( ...
        Robot, map, beaconLoc, goalWaypoints, offset_x, offset_y, ...
        currentState, followerParams, dataStore);
end
end

function currentState = updateOptionalWallCurrentState(currentState, trackingState)
currentState.pose = trackingState.pose;
currentState.poseCov = trackingState.poseCov;
if isfield(trackingState, 'particles')
    currentState.particles = trackingState.particles;
end
if isfield(trackingState, 'weights')
    currentState.weights = trackingState.weights;
end
end

function currentState = refreshOptionalWallPfParticles(currentState, params)
if ~strcmp(getOptionalWallFollowerMode(params), 'pf')
    return;
end

if isfield(currentState, 'particles') && ~isempty(currentState.particles)
    numParticles = size(currentState.particles, 1);
else
    numParticles = 192;
end

pose = currentState.pose(:).';
stdXY = 0.025;
stdTheta = deg2rad(4);
currentState.particles = repmat(pose, numParticles, 1);
currentState.particles(:, 1) = currentState.particles(:, 1) + stdXY * randn(numParticles, 1);
currentState.particles(:, 2) = currentState.particles(:, 2) + stdXY * randn(numParticles, 1);
currentState.particles(:, 3) = wrapToPiLocal(currentState.particles(:, 3) + stdTheta * randn(numParticles, 1));
currentState.weights = ones(numParticles, 1) / numParticles;
end

function status = storeOptionalWallTrackingState(status, trackingState, suffix, params)
if strcmp(getOptionalWallFollowerMode(params), 'pf')
    status.(['pfPoseAt' suffix]) = trackingState.pose;
    status.(['pfCovAt' suffix]) = trackingState.poseCov;
    if isfield(trackingState, 'particles')
        status.(['pfParticlesAt' suffix]) = trackingState.particles;
    end
    if isfield(trackingState, 'weights')
        status.(['pfWeightsAt' suffix]) = trackingState.weights;
    end
else
    status.(['ekfPoseAt' suffix]) = trackingState.pose;
    status.(['ekfCovAt' suffix]) = trackingState.poseCov;
end
end

function status = storeOptionalWallCurrentState(status, currentState, suffix, params)
if strcmp(getOptionalWallFollowerMode(params), 'pf')
    status.(['pfPoseAt' suffix]) = currentState.pose;
    status.(['pfCovAt' suffix]) = currentState.poseCov;
    if isfield(currentState, 'particles')
        status.(['pfParticlesAt' suffix]) = currentState.particles;
    end
    if isfield(currentState, 'weights')
        status.(['pfWeightsAt' suffix]) = currentState.weights;
    end
else
    status.(['ekfPoseAt' suffix]) = currentState.pose;
    status.(['ekfCovAt' suffix]) = currentState.poseCov;
end
end

function status = emptyWallStatus()
status = struct( ...
    'wallIdx', NaN, ...
    'wall', NaN(1, 4), ...
    'status', 'unknown', ...
    'confidence', 0, ...
    'reason', '', ...
    'observePoint', [], ...
    'observeInfo', struct(), ...
    'pathToObserve', [], ...
    'planInfo', struct(), ...
    'navState', struct(), ...
    'ekfPoseAtObserve', [], ...
    'ekfCovAtObserve', [], ...
    'ekfPoseAtClassify', [], ...
    'ekfCovAtClassify', [], ...
    'pfPoseAtObserve', [], ...
    'pfCovAtObserve', [], ...
    'pfParticlesAtObserve', [], ...
    'pfWeightsAtObserve', [], ...
    'pfPoseAtClassify', [], ...
    'pfCovAtClassify', [], ...
    'pfParticlesAtClassify', [], ...
    'pfWeightsAtClassify', [], ...
    'turnInfo', struct(), ...
    'numFrames', 0, ...
    'numRaysUsed', 0, ...
    'errPresent', inf, ...
    'errAbsent', inf, ...
    'classification', struct(), ...
    'bumpProbe', struct());
end

function [observePoint, info] = chooseOptionalWallObservePoint(startXY, wall, map, plannerParams, verifyParams)
mid = wallMidpoint(wall);
wallVec = wall(3:4) - wall(1:2);
wallLen = norm(wallVec);
info = struct('reason', '', 'candidates', [], 'candidateScores', []);
observePoint = [];

if wallLen < eps
    info.reason = 'zeroLengthWall';
    return;
end

tangent = wallVec / wallLen;
normal = [-tangent(2), tangent(1)];

distances = verifyParams.observeDistance;
if isfield(verifyParams, 'observeDistances') && ~isempty(verifyParams.observeDistances)
    distances = verifyParams.observeDistances(:).';
end
offsets = 0;
if isfield(verifyParams, 'observeTangentOffsets') && ~isempty(verifyParams.observeTangentOffsets)
    offsets = verifyParams.observeTangentOffsets(:).';
end

candidates = zeros(0, 2);
candidateMeta = zeros(0, 3); % [distance tangentOffset side]
for d = distances
    for offset = offsets
        wallPoint = mid + offset * tangent;
        for side = [-1 1]
            candidates(end + 1, :) = wallPoint + side * d * normal; %#ok<AGROW>
            candidateMeta(end + 1, :) = [d offset side]; %#ok<AGROW>
        end
    end
end

candidates = uniqueRowsToleranceLocal(candidates, 1e-6);
info.candidates = candidates;
info.candidateScores = inf(size(candidates, 1), 1);
info.candidateMeta = candidateMeta;

bestScore = inf;
bestPoint = [];
bestPlanInfo = struct();
for i = 1:size(candidates, 1)
    p = candidates(i, :);
    if ~candidateObservePointIsValid(p, mid, map, plannerParams, verifyParams)
        continue;
    end
    [path, planInfo] = planPathKnownMap(startXY, p, map, plannerParams);
    if isempty(path) || ~planInfo.success
        continue;
    end
    wallRange = norm(p - mid);
    approachVec = mid - p;
    if norm(approachVec) > eps
        alignmentPenalty = abs(dot(approachVec / norm(approachVec), tangent));
    else
        alignmentPenalty = 1;
    end
    score = planInfo.pathCost + ...
            verifyParams.observeDistancePenalty * wallRange + ...
            verifyParams.observeAlignmentPenalty * alignmentPenalty;
    info.candidateScores(i, 1) = score;
    if score < bestScore
        bestScore = score;
        bestPoint = p;
        bestPlanInfo = planInfo;
    end
end

if isempty(bestPoint)
    info.reason = 'noReachableObservePoint';
    return;
end

observePoint = bestPoint;
info.reason = 'ok';
info.bestScore = bestScore;
info.bestPlanInfo = bestPlanInfo;
end

function tf = candidateObservePointIsValid(p, wallMid, map, plannerParams, verifyParams)
tf = false;
bounds = inferBoundsLocal(map);
if p(1) < bounds(1) + verifyParams.observeClearance || ...
   p(1) > bounds(2) - verifyParams.observeClearance || ...
   p(2) < bounds(3) + verifyParams.observeClearance || ...
   p(2) > bounds(4) - verifyParams.observeClearance
    return;
end

if minPointWallDistanceLocal(p, map) < verifyParams.observeClearance
    return;
end

tmpParams = plannerParams;
tmpParams.edgeClearance = min(tmpParams.edgeClearance, 0.08);
tmpParams.startGoalClearance = min(tmpParams.startGoalClearance, 0.04);
if ~edgeClearKnownOnlyLocal(p, wallMid, map, tmpParams)
    return;
end

tf = true;
end

function [currentState, dataStore, turnInfo] = turnToFacePointWithEkf(Robot, currentState, targetXY, params, dataStore)
turnInfo = struct('converged', false, 'finalHeadingErr', NaN, 'numSteps', 0);
mu = currentState.pose(:);
sigma = currentState.poseCov;

timerObj = tic;
while toc(timerObj) < params.turnMaxTime
    desiredHeading = atan2(targetXY(2) - mu(2), targetXY(1) - mu(1));
    headingErr = wrapToPiLocal(desiredHeading - mu(3));
    turnInfo.finalHeadingErr = headingErr;
    if abs(headingErr) <= params.turnHeadingTol
        turnInfo.converged = true;
        break;
    end

    cmdW = max(-params.turnMaxAngVel, min(params.turnMaxAngVel, params.turnGain * headingErr));
    SetFwdVelAngVelCreate(Robot, 0, cmdW);
    pause(params.turnControlDt);

    odom = readTurnOdom(Robot);
    if odom.valid
        [mu, sigma] = predictTurnState(mu, sigma, odom);
    end
    turnInfo.numSteps = turnInfo.numSteps + 1;
end

stopRobotSafe(Robot);
currentState.pose = mu;
currentState.poseCov = sigma;
currentState = refreshOptionalWallPfParticles(currentState, params);

if ~isfield(dataStore, 'optionalWallTurn') || isempty(dataStore.optionalWallTurn)
    dataStore.optionalWallTurn = [];
end
dataStore.optionalWallTurn = [dataStore.optionalWallTurn; ...
    getOptionalWallTime(dataStore) turnInfo.numSteps double(turnInfo.converged) turnInfo.finalHeadingErr];
end

function odom = readTurnOdom(Robot)
odom = struct('d', 0, 'phi', 0, 'valid', false);
try
    sensorPort = getCreateSensorPort(Robot);
    odom.d = DistanceSensorRoomba(sensorPort);
    odom.phi = AngleSensorRoomba(sensorPort);
    if ~isfinite(odom.d), odom.d = 0; end
    if ~isfinite(odom.phi), odom.phi = 0; end
    odom.valid = true;
catch
end
end

function [mu, sigma] = predictTurnState(mu, sigma, odom)
u = [odom.d; odom.phi];
mu = integrateOdom(mu, u(1), u(2));
mu = mu(:, end);
mu(3) = wrapToPiLocal(mu(3));
G = GjacDiffDrive(mu, u);
R = diag([0.004 ^ 2, 0.004 ^ 2, deg2rad(0.8) ^ 2]);
sigma = G * sigma * G' + R;
sigma = (sigma + sigma.') / 2;
end

function depthObs = collectOptionalWallDepth(Robot, params)
frames = cell(params.numDepthFrames, 1);
angles = [];
validMaskFrames = cell(params.numDepthFrames, 1);
numFrames = 0;

for k = 1:params.numDepthFrames
    try
        raw = RealSenseDist(Robot);
        raw = raw(:).';
        if params.dropFirstDepthSample && numel(raw) > 1
            ranges = raw(2:end);
        else
            ranges = raw;
        end
        fov = deg2rad(params.depthFovDeg);
        angles = linspace(0.5 * fov, -0.5 * fov, numel(ranges)).';
        ranges = ranges(:);
        valid = isfinite(ranges) & ranges >= params.minDepthRange & ranges <= params.maxDepthRange;
        numFrames = numFrames + 1;
        frames{numFrames} = ranges;
        validMaskFrames{numFrames} = valid;
    catch
    end
    pause(params.depthFramePause);
end

frames = frames(1:numFrames);
validMaskFrames = validMaskFrames(1:numFrames);

depthObs = struct();
depthObs.frames = frames;
depthObs.validMaskFrames = validMaskFrames;
depthObs.angles = angles;
depthObs.numFrames = numFrames;
end

function dataStore = appendOptionalWallDepthLog(dataStore, wallIdx, depthObs)
if ~isfield(dataStore, 'optionalWallDepth') || isempty(dataStore.optionalWallDepth)
    dataStore.optionalWallDepth = cell(0, 1);
end
entry = struct('wallIdx', wallIdx, 'depthObs', depthObs);
dataStore.optionalWallDepth{end + 1, 1} = entry;
end

function tf = shouldRunBumpProbe(classification, params)
tf = false;
if ~isfield(params, 'enableBumpProbe') || ~params.enableBumpProbe
    return;
end
mode = 'always';
if isfield(params, 'bumpProbeMode') && ~isempty(params.bumpProbeMode)
    mode = params.bumpProbeMode;
end
switch mode
    case 'always'
        tf = true;
    case 'depthFallback'
        tf = ~strcmp(classification.status, 'exists');
    otherwise
        tf = true;
end
end

function [currentState, dataStore, probe] = runOptionalWallBumpProbe( ...
    Robot, currentState, wall, params, dataStore, wallIdx)
probe = struct( ...
    'ran', true, ...
    'bumped', false, ...
    'status', 'absent', ...
    'reason', 'noBumpPastCandidate', ...
    'travel', 0, ...
    'maxTravel', 0, ...
    'expectedCenterTravel', 0, ...
    'bump', struct(), ...
    'startPose', currentState.pose, ...
    'endPose', currentState.pose, ...
    'backupTravel', 0);

mu = currentState.pose(:);
sigma = currentState.poseCov;
wallMid = wallMidpoint(wall);
centerRange = norm(wallMid(:) - mu(1:2));
expectedCenterTravel = max(0, centerRange - params.bumpProbeRobotRadius);
maxTravel = min(params.bumpProbeMaxTravel, expectedCenterTravel + params.bumpProbeBeyondWall);
maxTravel = max(maxTravel, params.bumpProbeMinTravelForWall);
probe.expectedCenterTravel = expectedCenterTravel;
probe.maxTravel = maxTravel;

tStart = tic;
travel = 0;
effectiveMaxTime = max(params.bumpProbeMaxTime, maxTravel / max(abs(params.bumpProbeSpeed), eps) + 2.0);
while toc(tStart) < effectiveMaxTime && travel < maxTravel
    SetFwdVelAngVelCreate(Robot, params.bumpProbeSpeed, 0);
    pause(params.bumpProbeControlDt);

    odom = readTurnOdom(Robot);
    if odom.valid
        [mu, sigma] = predictTurnState(mu, sigma, odom);
        travel = travel + max(0, odom.d);
    else
        travel = travel + params.bumpProbeSpeed * params.bumpProbeControlDt;
        mu(1) = mu(1) + params.bumpProbeSpeed * params.bumpProbeControlDt * cos(mu(3));
        mu(2) = mu(2) + params.bumpProbeSpeed * params.bumpProbeControlDt * sin(mu(3));
    end

    bump = readOptionalWallBump(Robot);
    if bump.valid && bump.any
        probe.bumped = true;
        probe.status = 'exists';
        probe.reason = 'bumpContact';
        probe.bump = bump;
        break;
    end
end

stopRobotSafe(Robot);
probe.travel = travel;

if probe.bumped
    [mu, sigma, backupTravel] = backAwayAfterOptionalWallBump(Robot, mu, sigma, params);
    probe.backupTravel = backupTravel;
elseif travel < maxTravel
    probe.status = 'unknown';
    probe.reason = 'probeTimeout';
end

probe.endPose = mu;
currentState.pose = mu;
currentState.poseCov = sigma;
currentState = refreshOptionalWallPfParticles(currentState, params);

if ~isfield(dataStore, 'optionalWallBumpProbe') || isempty(dataStore.optionalWallBumpProbe)
    dataStore.optionalWallBumpProbe = [];
end
dataStore.optionalWallBumpProbe = [dataStore.optionalWallBumpProbe; ...
    getOptionalWallTime(dataStore) wallIdx double(probe.bumped) ...
    probe.travel probe.maxTravel double(strcmp(probe.status, 'exists'))];
end

function [currentState, dataStore, fallbackStatus] = runOptionalWallBumpFallback( ...
    Robot, currentState, wall, currentMap, beaconLoc, offset_x, offset_y, ...
    trackingParams, plannerParams, verifyParams, dataStore, wallIdx)
fallbackStatus = emptyWallStatus();
fallbackStatus.wallIdx = wallIdx;
fallbackStatus.wall = wall;
fallbackStatus.reason = 'noReachableObservePoint';

if ~isfield(verifyParams, 'enableBumpProbeFallback') || ~verifyParams.enableBumpProbeFallback
    return;
end

[probeStart, probeInfo] = chooseOptionalWallProbeStartPoint( ...
    currentState.pose(1:2).', wall, currentMap, plannerParams, verifyParams);
fallbackStatus.observePoint = probeStart;
fallbackStatus.observeInfo = probeInfo;

if isempty(probeStart)
    fallbackStatus.reason = ['bumpFallback_' probeInfo.reason];
    return;
end

[pathToProbe, planInfo] = planPathKnownMap( ...
    currentState.pose(1:2).', probeStart, currentMap, plannerParams);
fallbackStatus.pathToObserve = pathToProbe;
fallbackStatus.planInfo = planInfo;
if isempty(pathToProbe) || ~planInfo.success
    fallbackStatus.reason = ['bumpFallbackPlanFailed_' planInfo.reason];
    return;
end

followerParams = trackingParams;
followerParams.maxRunTime = verifyParams.bumpFallbackMaxRunTime;
[trackingState, dataStore, navState] = runOptionalWallFollower( ...
    Robot, currentMap, beaconLoc, pathToProbe, offset_x, offset_y, ...
    currentState, followerParams, verifyParams, dataStore);

fallbackStatus.navState = navState;
fallbackStatus = storeOptionalWallTrackingState(fallbackStatus, trackingState, 'Observe', verifyParams);
currentState = updateOptionalWallCurrentState(currentState, trackingState);

if ~trackingState.reachedAllGoals
    fallbackStatus.reason = ['bumpFallbackNavigationFailed_' trackingState.stopReason];
    return;
end

wallMid = wallMidpoint(wall);
[currentState, dataStore, turnInfo] = turnToFacePointWithEkf( ...
    Robot, currentState, wallMid, verifyParams, dataStore);
fallbackStatus.turnInfo = turnInfo;
fallbackStatus = storeOptionalWallCurrentState(fallbackStatus, currentState, 'Classify', verifyParams);

[currentState, dataStore, bumpProbe] = runOptionalWallBumpProbe( ...
    Robot, currentState, wall, verifyParams, dataStore, wallIdx);
fallbackStatus.bumpProbe = bumpProbe;
fallbackStatus.classification = struct('status', bumpProbe.status, 'reason', bumpProbe.reason);
fallbackStatus.numFrames = 0;
fallbackStatus.numRaysUsed = 0;
fallbackStatus.errPresent = inf;
fallbackStatus.errAbsent = inf;

switch bumpProbe.status
    case 'exists'
        fallbackStatus.status = 'exists';
        fallbackStatus.confidence = 1;
        fallbackStatus.reason = 'bumpFallbackContact';
    case 'absent'
        fallbackStatus.status = 'absent';
        fallbackStatus.confidence = 1;
        fallbackStatus.reason = 'bumpFallbackNoContact';
    otherwise
        fallbackStatus.status = 'unknown';
        fallbackStatus.confidence = 0;
        fallbackStatus.reason = ['bumpFallback_' bumpProbe.reason];
end
end

function [probeStart, info] = chooseOptionalWallProbeStartPoint(startXY, wall, map, plannerParams, verifyParams)
mid = wallMidpoint(wall);
wallVec = wall(3:4) - wall(1:2);
wallLen = norm(wallVec);
info = struct('reason', '', 'candidates', [], 'candidateScores', []);
probeStart = [];

if wallLen < eps
    info.reason = 'zeroLengthWall';
    return;
end

tangent = wallVec / wallLen;
normal = [-tangent(2), tangent(1)];
distances = verifyParams.bumpFallbackDistances(:).';
offsets = verifyParams.bumpFallbackTangentOffsets(:).';

candidates = zeros(0, 2);
for d = distances
    for offset = offsets
        wallPoint = mid + offset * tangent;
        for side = [-1 1]
            candidates(end + 1, :) = wallPoint + side * d * normal; %#ok<AGROW>
        end
    end
end
candidates = uniqueRowsToleranceLocal(candidates, 1e-6);
info.candidates = candidates;
info.candidateScores = inf(size(candidates, 1), 1);

bestScore = inf;
bestPoint = [];
bestPlanInfo = struct();
for i = 1:size(candidates, 1)
    p = candidates(i, :);
    if ~probeStartPointIsValid(p, map, verifyParams)
        continue;
    end

    [path, planInfo] = planPathKnownMap(startXY, p, map, plannerParams);
    if isempty(path) || ~planInfo.success
        continue;
    end

    headingPenalty = abs(dot((mid - p) / max(norm(mid - p), eps), tangent));
    score = planInfo.pathCost + 0.15 * norm(p - mid) + 0.15 * headingPenalty;
    info.candidateScores(i) = score;
    if score < bestScore
        bestScore = score;
        bestPoint = p;
        bestPlanInfo = planInfo;
    end
end

if isempty(bestPoint)
    info.reason = 'noReachableProbeStart';
    return;
end

probeStart = bestPoint;
info.reason = 'ok';
info.bestScore = bestScore;
info.bestPlanInfo = bestPlanInfo;
end

function tf = probeStartPointIsValid(p, map, verifyParams)
tf = false;
bounds = inferBoundsLocal(map);
clearance = verifyParams.bumpFallbackClearance;
if p(1) < bounds(1) + clearance || p(1) > bounds(2) - clearance || ...
   p(2) < bounds(3) + clearance || p(2) > bounds(4) - clearance
    return;
end
if minPointWallDistanceLocal(p, map) < clearance
    return;
end
tf = true;
end

function statusOut = mergeFallbackWallStatus(statusIn, fallbackStatus)
statusOut = statusIn;
fn = fieldnames(fallbackStatus);
for i = 1:numel(fn)
    statusOut.(fn{i}) = fallbackStatus.(fn{i});
end
end

function [mu, sigma, backupTravel] = backAwayAfterOptionalWallBump(Robot, mu, sigma, params)
backupTravel = 0;
while backupTravel < params.bumpProbeBackupDistance
    SetFwdVelAngVelCreate(Robot, params.bumpProbeBackupSpeed, 0);
    pause(params.bumpProbeControlDt);
    odom = readTurnOdom(Robot);
    if odom.valid
        [mu, sigma] = predictTurnState(mu, sigma, odom);
        backupTravel = backupTravel + abs(odom.d);
    else
        step = abs(params.bumpProbeBackupSpeed) * params.bumpProbeControlDt;
        backupTravel = backupTravel + step;
        mu(1) = mu(1) - step * cos(mu(3));
        mu(2) = mu(2) - step * sin(mu(3));
    end
end
stopRobotSafe(Robot);
end

function bump = readOptionalWallBump(Robot)
bump = struct('right', 0, 'left', 0, 'dropRight', 0, 'dropLeft', 0, ...
              'dropCaster', 0, 'front', 0, 'any', false, 'valid', false);
try
    sensorPort = getCreateSensorPort(Robot);
    [BumpRight, BumpLeft, DropRight, DropLeft, DropCaster, BumpFront] = ...
        BumpsWheelDropsSensorsRoomba(sensorPort);
    bump = struct( ...
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
end

function classification = fuseDepthAndBumpProbe(classification, bumpProbe)
if ~isstruct(bumpProbe) || ~isfield(bumpProbe, 'ran') || ~bumpProbe.ran
    return;
end
classification.bumpProbe = bumpProbe;
switch bumpProbe.status
    case 'exists'
        classification.status = 'exists';
        classification.confidence = 1;
        classification.reason = 'bumpProbeContact';
    case 'absent'
        classification.status = 'absent';
        classification.confidence = 1;
        classification.reason = 'bumpProbeNoContact';
    otherwise
        if strcmp(classification.status, 'unknown')
            classification.confidence = 0;
            classification.reason = ['bumpProbe_' bumpProbe.reason];
        end
end
end

function classification = classifyOptionalWallFromDepth(pose, depthObs, knownMap, optWall, sensorOffset, params)
classification = struct( ...
    'status', 'unknown', ...
    'confidence', 0, ...
    'reason', '', ...
    'numRaysUsed', 0, ...
    'errPresent', inf, ...
    'errAbsent', inf, ...
    'rayIndices', []);

if depthObs.numFrames == 0 || isempty(depthObs.angles)
    classification.reason = 'noDepthFrames';
    return;
end

angles = depthObs.angles(:);
wallMidRobot = global2robot(pose(:).', wallMidpoint(optWall));
wallMidCam = [wallMidRobot(1) - sensorOffset(1), wallMidRobot(2) - sensorOffset(2)];
wallBearing = atan2(wallMidCam(2), wallMidCam(1));
wallRange = hypot(wallMidCam(1), wallMidCam(2));

if wallRange < params.minWallViewDistance || wallRange > params.maxWallViewDistance
    classification.reason = 'wallRangeOutOfBounds';
    return;
end

bearingErr = abs(wrapToPiLocal(angles - wallBearing));
knownGeom = precomputeKnownMapGeometry(knownMap, 0.10);
presentGeom = precomputeKnownMapGeometry([knownMap; optWall], 0.10);
zAbsentAll = raycastKnownMap(pose, knownGeom, angles, sensorOffset, makeRaycastParams(params));
zPresentAll = raycastKnownMap(pose, presentGeom, angles, sensorOffset, makeRaycastParams(params));

bearingIdx = find(bearingErr <= params.wallBearingWindow);
informativeIdx = find(isfinite(zAbsentAll) & isfinite(zPresentAll) & ...
                      (zAbsentAll - zPresentAll) >= params.minModelSeparation);
candidateIdx = unique([bearingIdx(:); informativeIdx(:)]);

if isempty(candidateIdx)
    [~, order] = sort(bearingErr);
    candidateIdx = order(1:min(params.maxRaysForWall, numel(order)));
elseif numel(candidateIdx) > params.maxRaysForWall
    modelGap = abs(zAbsentAll(candidateIdx) - zPresentAll(candidateIdx));
    combinedScore = bearingErr(candidateIdx) - 0.5 * modelGap;
    [~, order] = sort(combinedScore);
    candidateIdx = candidateIdx(order(1:params.maxRaysForWall));
end
candidateIdx = sort(candidateIdx(:));

zAbsent = zAbsentAll(candidateIdx);
zPresent = zPresentAll(candidateIdx);

obsMatrix = nan(numel(candidateIdx), depthObs.numFrames);
for f = 1:depthObs.numFrames
    ranges = depthObs.frames{f};
    valid = depthObs.validMaskFrames{f};
    usable = candidateIdx(candidateIdx <= numel(ranges));
    localIdx = ismember(candidateIdx, usable);
    frameVals = nan(numel(candidateIdx), 1);
    for j = 1:numel(candidateIdx)
        idx = candidateIdx(j);
        if idx <= numel(ranges) && idx <= numel(valid) && valid(idx)
            frameVals(j) = ranges(idx);
        end
    end
    obsMatrix(localIdx, f) = frameVals(localIdx);
end

zObs = median(obsMatrix, 2, 'omitnan');
validRows = isfinite(zObs) & isfinite(zAbsent) & isfinite(zPresent);
if nnz(validRows) < params.minRaysToClassify
    classification.reason = 'notEnoughValidRays';
    return;
end

zObs = zObs(validRows);
zAbsent = zAbsent(validRows);
zPresent = zPresent(validRows);
usedIdx = candidateIdx(validRows);

errAbsent = mean(abs(zObs - zAbsent));
errPresent = mean(abs(zObs - zPresent));
unexpectedClose = zAbsent - zObs;
unexpectedClose = unexpectedClose(isfinite(unexpectedClose));
if isempty(unexpectedClose)
    unexpectedFrac = 0;
    meanUnexpectedClose = 0;
    maxUnexpectedClose = 0;
else
    unexpectedFrac = mean(unexpectedClose > params.unexpectedObstacleMargin);
    meanUnexpectedClose = mean(max(0, unexpectedClose));
    maxUnexpectedClose = max(unexpectedClose);
end
classification.errAbsent = errAbsent;
classification.errPresent = errPresent;
classification.numRaysUsed = numel(zObs);
classification.rayIndices = usedIdx;
classification.zObs = zObs;
classification.zAbsent = zAbsent;
classification.zPresent = zPresent;
classification.rayAngles = angles(usedIdx);
classification.unexpectedObstacleFrac = unexpectedFrac;
classification.meanUnexpectedObstacle = meanUnexpectedClose;
classification.maxUnexpectedObstacle = maxUnexpectedClose;

if errPresent + params.residualMargin < errAbsent && errPresent < params.maxMeanResidualForDecision
    classification.status = 'exists';
    classification.reason = 'presentModelCloser';
elseif unexpectedFrac >= params.minUnexpectedObstacleFrac && ...
       meanUnexpectedClose >= params.minMeanUnexpectedObstacle && ...
       errAbsent >= params.maxMeanResidualForDecision
    classification.status = 'exists';
    classification.reason = 'unexpectedObstacleNearCandidate';
elseif maxUnexpectedClose >= params.minStrongUnexpectedObstacle
    classification.status = 'exists';
    classification.reason = 'strongUnexpectedObstacleNearCandidate';
elseif errAbsent + params.residualMargin < errPresent && errAbsent < params.maxMeanResidualForDecision
    classification.status = 'absent';
    classification.reason = 'absentModelCloser';
else
    classification.status = 'unknown';
    classification.reason = 'ambiguousResiduals';
end

gap = abs(errAbsent - errPresent);
classification.confidence = min(1, gap / max(params.confidenceScale, eps));
end

function rayParams = makeRaycastParams(params)
rayParams = struct();
rayParams.maxDepthRange = params.maxDepthRange;
rayParams.minDepthRange = params.minDepthRange;
rayParams.wallThickness = 0.10;
end

function mid = wallMidpoint(wall)
mid = 0.5 * [wall(1) + wall(3), wall(2) + wall(4)];
end

function bounds = inferBoundsLocal(map)
xs = [map(:, 1); map(:, 3)];
ys = [map(:, 2); map(:, 4)];
bounds = [min(xs), max(xs), min(ys), max(ys)];
end

function rowsOut = uniqueRowsToleranceLocal(rowsIn, tol)
if isempty(rowsIn)
    rowsOut = rowsIn;
    return;
end

rowsOut = zeros(0, size(rowsIn, 2));
for i = 1:size(rowsIn, 1)
    row = rowsIn(i, :);
    if isempty(rowsOut) || all(vecnorm(rowsOut - row, 2, 2) > tol)
        rowsOut(end + 1, :) = row; %#ok<AGROW>
    end
end
end

function dMin = minPointWallDistanceLocal(p, map)
dMin = inf;
for i = 1:size(map, 1)
    dMin = min(dMin, pointSegmentDistanceLocal(p, map(i, 1:2), map(i, 3:4)));
end
end

function d = pointSegmentDistanceLocal(p, a, b)
ab = b - a;
den = dot(ab, ab);
if den < eps
    d = norm(p - a);
    return;
end
t = dot(p - a, ab) / den;
t = max(0, min(1, t));
proj = a + t * ab;
d = norm(p - proj);
end

function tf = edgeClearKnownOnlyLocal(p1, p2, map, plannerParams)
bounds = inferBoundsLocal(map);
if p1(1) < bounds(1) || p1(1) > bounds(2) || p1(2) < bounds(3) || p1(2) > bounds(4) || ...
   p2(1) < bounds(1) || p2(1) > bounds(2) || p2(2) < bounds(3) || p2(2) > bounds(4)
    tf = false;
    return;
end

tf = true;
for i = 1:size(map, 1)
    d = segmentSegmentDistanceLocal(p1, p2, map(i, 1:2), map(i, 3:4));
    if d < plannerParams.edgeClearance
        tf = false;
        return;
    end
end
end

function d = segmentSegmentDistanceLocal(a, b, c, dpt)
if segmentsIntersectLocal(a, b, c, dpt)
    d = 0;
    return;
end
d = min([ ...
    pointSegmentDistanceLocal(a, c, dpt), ...
    pointSegmentDistanceLocal(b, c, dpt), ...
    pointSegmentDistanceLocal(c, a, b), ...
    pointSegmentDistanceLocal(dpt, a, b)]);
end

function tf = segmentsIntersectLocal(a, b, c, d)
tol = 1e-10;
o1 = orientLocal(a, b, c);
o2 = orientLocal(a, b, d);
o3 = orientLocal(c, d, a);
o4 = orientLocal(c, d, b);
tf = (o1 * o2 < -tol) && (o3 * o4 < -tol);
end

function val = orientLocal(a, b, c)
val = (b(1) - a(1)) * (c(2) - a(2)) - ...
      (b(2) - a(2)) * (c(1) - a(1));
end

function t = getOptionalWallTime(dataStore)
t = 0;
if isfield(dataStore, 'ekfMu') && ~isempty(dataStore.ekfMu)
    t = dataStore.ekfMu(end, 1);
elseif isfield(dataStore, 'odometry') && ~isempty(dataStore.odometry)
    t = dataStore.odometry(end, 1);
end
end
