function dataStore = testPfWaypointEcOptionalPracticeMap(Robot, goalWaypoints, ecGoalWaypoints)
% testPfWaypointEcOptionalPracticeMap PF normal waypoints -> optional walls -> EC.
%
% SimulatorGUI Autonomous Start usage:
%   testPfWaypointEcOptionalPracticeMap
%
% Optional command-line style usage:
%   dataStore = testPfWaypointEcOptionalPracticeMap(Robot, normalGoals, ecGoals)

codeDir = fileparts(mfilename('fullpath'));
S = load(fullfile(codeDir, 'PracticeMap2026.mat'), ...
    'map', 'optWalls', 'waypoints', 'ECwaypoints', 'beaconLoc');

offset_x = 0.0;
offset_y = 0.08;

initParams = initDefaultParams();
initParams.maxInitTime = 25.0;
initParams.minInitTime = 8.0;
initParams.maxDepthRange = 10.0;
initParams.minDepthRange = 0.175;
initParams.debugPrint = true;

[initState, dataStore] = initializeLocalization( ...
    Robot, S.map, S.waypoints, S.beaconLoc, offset_x, offset_y, initParams, struct());

if nargin < 2 || isempty(goalWaypoints)
    normalGoals = defaultPracticeGoals(S.waypoints, initState.startWaypointIdx);
else
    normalGoals = goalWaypoints;
end

if nargin < 3 || isempty(ecGoalWaypoints)
    ecGoals = S.ECwaypoints;
else
    ecGoals = ecGoalWaypoints;
end

plannerParams = knownMapPlannerDefaultParams();
pfParams = pfWaypointDefaultParams();
pfParams.maxRunTime = 500.0;
pfParams.maxDepthRange = 10.0;
pfParams.minDepthRange = 0.175;
pfParams.debugPrint = true;
pfParams.enableLivePlot = true;

postWallEscapeParams = struct();
postWallEscapeParams.maxAttempts = 3;
postWallEscapeParams.backDistance = 0.35;
postWallEscapeParams.backVel = -0.06;
postWallEscapeParams.turnAngle = deg2rad(30);
postWallEscapeParams.turnVel = 0.35;
postWallEscapeParams.controlDt = 0.10;

currentState = initState;
stageOrder = {'normalWaypoints', 'optionalWalls', 'ecWaypoints'};
normalPfState = struct();
normalNavState = struct();
normalPlannedPath = [];
normalPlannerInfo = struct('success', true, 'reason', 'noNormalGoals');
normalSnapInfo = struct('snapped', false, 'target', [], 'source', 'notRun');

if ~isempty(normalGoals)
    [normalPlannedPath, normalPlannerInfo] = planWaypointSequenceKnownMap( ...
        currentState.pose(1:2).', normalGoals, S.map, plannerParams);

    if isempty(normalPlannedPath) || ~normalPlannerInfo.success
        stopRobotSafe(Robot);
        dataStore.normalGoals = normalGoals;
        dataStore.normalPlannedPath = normalPlannedPath;
        dataStore.normalPlannerInfo = normalPlannerInfo;
        savePartialRun(codeDir, initState, dataStore, S, normalGoals, ecGoals);
        error('Known-map normal waypoint planner failed: %s', normalPlannerInfo.reason);
    end

    normalFollowerParams = pfParams;
    if isfield(normalPlannerInfo, 'beepMask')
        normalFollowerParams.beepMask = normalPlannerInfo.beepMask;
    end

    [normalPfState, dataStore, normalNavState] = runPfWaypointFollower( ...
        Robot, S.map, S.beaconLoc, normalPlannedPath, offset_x, offset_y, ...
        currentState, normalFollowerParams, dataStore);
    currentState = updateStateFromPf(currentState, normalPfState);
    [currentState, normalSnapInfo] = snapPlanningStateToLastReachedGoal( ...
        currentState, normalNavState, normalPlannedPath, normalFollowerParams.beepMask);
end

postNormalSafeState = currentState;

verifyParams = optionalWallDefaultParams();
verifyParams.followerMode = 'pf';
verifyParams.maxDepthRange = 10.0;
verifyParams.minDepthRange = 0.175;
verifyParams.debugPrint = true;

pfVerifyParams = pfParams;
pfVerifyParams.beepMask = [];

[wallStatus, dataStore, verifiedMap] = verifyOptionalWalls( ...
    Robot, S.map, S.optWalls, S.beaconLoc, offset_x, offset_y, ...
    currentState, pfVerifyParams, plannerParams, verifyParams, dataStore);
currentState = updateStateFromWallStatus(currentState, wallStatus);
[ecPlanningStartState, postOptionalWallPlanningStartInfo] = selectPostOptionalWallPlanningState( ...
    currentState, wallStatus, postNormalSafeState, ecGoals, verifiedMap, plannerParams);

[ecVisitOrder, ecTailPath, ecPlannerInfo, skippedECwaypoints] = ...
    planEcWaypointsNearestNeighbor(ecPlanningStartState.pose(1:2).', ecGoals, verifiedMap, plannerParams);
[ecPlannedPath, ecPlannerInfo] = prependBridgePathToEcPlan( ...
    currentState.pose(1:2).', ecPlanningStartState.pose(1:2).', ...
    ecTailPath, ecPlannerInfo, verifiedMap, plannerParams);
if ~ecPlannerInfo.bridgeSuccess
    [currentState, ecPlanningStartState, postOptionalWallPlanningStartInfo, ...
        ecVisitOrder, ~, ecPlannerInfo, skippedECwaypoints, ...
        ecPlannedPath, dataStore] = runPostOptionalWallEscapeAndReplan( ...
            Robot, currentState, wallStatus, postNormalSafeState, ecGoals, ...
            verifiedMap, plannerParams, postWallEscapeParams, dataStore);
end

ecPfState = struct();
ecNavState = struct();
if ~isempty(ecPlannedPath)
    ecFollowerParams = pfParams;
    ecFollowerParams.beepMask = ecPlannerInfo.beepMask;
    [ecPfState, dataStore, ecNavState] = runPfWaypointFollower( ...
        Robot, verifiedMap, S.beaconLoc, ecPlannedPath, offset_x, offset_y, ...
        currentState, ecFollowerParams, dataStore);
    currentState = updateStateFromPf(currentState, ecPfState);
end

dataStore.stageOrder = stageOrder;
dataStore.initState = initState;
dataStore.normalGoals = normalGoals;
dataStore.normalPlannedPath = normalPlannedPath;
dataStore.normalPlannerInfo = normalPlannerInfo;
dataStore.normalSnapInfo = normalSnapInfo;
dataStore.normalPfState = normalPfState;
dataStore.normalNavState = normalNavState;
dataStore.postNormalSafeState = postNormalSafeState;
dataStore.ecPlanningStartState = ecPlanningStartState;
dataStore.postOptionalWallPlanningStartInfo = postOptionalWallPlanningStartInfo;
dataStore.ecGoals = ecGoals;
dataStore.ecVisitOrder = ecVisitOrder;
dataStore.ecPlannedPath = ecPlannedPath;
dataStore.ecPlannerInfo = ecPlannerInfo;
dataStore.ecPfState = ecPfState;
dataStore.ecNavState = ecNavState;
dataStore.skippedECwaypoints = skippedECwaypoints;
dataStore.wallStatus = wallStatus;
dataStore.verifiedMap = verifiedMap;
dataStore.finalCurrentState = currentState;

save(fullfile(codeDir, 'latestPfWaypointEcOptionalPracticeRun.mat'), ...
    'initState', 'normalPfState', 'normalNavState', ...
    'ecPfState', 'ecNavState', 'wallStatus', 'verifiedMap', ...
    'dataStore', 'S', 'normalGoals', 'ecGoals', ...
    'normalPlannedPath', 'normalPlannerInfo', ...
    'ecVisitOrder', 'ecPlannedPath', 'ecPlannerInfo', ...
    'skippedECwaypoints', 'stageOrder', 'normalSnapInfo', ...
    'postOptionalWallPlanningStartInfo');

disp('Initialization result:');
disp(initState);
disp('Normal waypoint planned path:');
disp(normalPlannedPath);
disp('Normal PF waypoint follower result:');
disp(normalPfState);
disp('Normal planning snap info:');
disp(normalSnapInfo);
disp('PF optional wall verification result:');
for i = 1:numel(wallStatus)
    fprintf('  optWall %d: status=%s confidence=%.2f reason=%s\n', ...
        i, wallStatus(i).status, wallStatus(i).confidence, wallStatus(i).reason);
end
fprintf('Post optional-wall EC planning start: source=%s reachableEC=%d pose=[%.2f %.2f %.1fdeg]\n', ...
    postOptionalWallPlanningStartInfo.selectedSource, ...
    postOptionalWallPlanningStartInfo.numReachableEC, ...
    postOptionalWallPlanningStartInfo.selectedPose(1), ...
    postOptionalWallPlanningStartInfo.selectedPose(2), ...
    postOptionalWallPlanningStartInfo.selectedPose(3) * 180 / pi);
fprintf('Post optional-wall bridge: success=%d reason=%s\n', ...
    ecPlannerInfo.bridgeSuccess, ecPlannerInfo.bridgeReason);
disp('EC visit order:');
disp(ecVisitOrder);
disp('EC planned path:');
disp(ecPlannedPath);
disp('EC PF waypoint follower result:');
disp(ecPfState);
end

function goals = defaultPracticeGoals(waypoints, startWaypointIdx)
goals = waypoints;
if isfinite(startWaypointIdx) && startWaypointIdx >= 1 && startWaypointIdx <= size(goals, 1)
    goals(startWaypointIdx, :) = [];
end
end

function currentState = updateStateFromPf(currentState, pfState)
if isfield(pfState, 'pose') && numel(pfState.pose) == 3
    currentState.pose = pfState.pose;
end
if isfield(pfState, 'poseCov') && isequal(size(pfState.poseCov), [3 3])
    currentState.poseCov = pfState.poseCov;
end
if isfield(pfState, 'particles')
    currentState.particles = pfState.particles;
end
if isfield(pfState, 'weights')
    currentState.weights = pfState.weights;
end
end

function [currentState, snapInfo] = snapPlanningStateToLastReachedGoal( ...
    currentState, navState, goalWaypoints, beepMask)
snapInfo = struct('snapped', false, 'target', [], 'source', 'notReachedAllGoals');

if isempty(goalWaypoints) || ~isfield(navState, 'reachedAllGoals') || ~navState.reachedAllGoals
    return;
end

if isfield(navState, 'goalWaypoints') && ~isempty(navState.goalWaypoints)
    goalWaypoints = navState.goalWaypoints;
end
if isfield(navState, 'beepMask') && ~isempty(navState.beepMask)
    beepMask = navState.beepMask;
end

numGoals = size(goalWaypoints, 1);
mask = normalizeLocalMask(beepMask, numGoals, false);
reachedMask = normalizeLocalMask(navState.reachedGoals, numGoals, false);
idx = find(mask & reachedMask, 1, 'last');
source = 'lastBeepGoal';
if isempty(idx)
    idx = numGoals;
    source = 'lastGoalFallback';
end

target = goalWaypoints(idx, :);
currentState.pose(1:2) = target(:);
currentState = resetPfParticlesAroundPose(currentState);
snapInfo = struct('snapped', true, 'target', target, 'source', source);
end

function mask = normalizeLocalMask(maskIn, n, defaultValue)
if nargin < 3
    defaultValue = false;
end
mask = repmat(defaultValue, n, 1);
if isempty(maskIn)
    return;
end
maskIn = logical(maskIn(:));
nCopy = min(n, numel(maskIn));
mask(1:nCopy) = maskIn(1:nCopy);
end

function [selectedState, info] = selectPostOptionalWallPlanningState( ...
    currentState, wallStatus, fallbackState, ecGoals, map, plannerParams)
info = struct( ...
    'selectedSource', 'none', ...
    'selectedPose', currentState.pose(:).', ...
    'numReachableEC', 0, ...
    'candidateReasons', {{}});

candidates = buildPostOptionalWallStartCandidates(currentState, wallStatus, fallbackState);
selectedState = currentState;
candidateReasons = cell(numel(candidates), 1);

for i = 1:numel(candidates)
    candidate = candidates(i);
    [numReachable, reasons] = countReachableEcGoals( ...
        candidate.state.pose(1:2).', ecGoals, map, plannerParams);
    candidateReasons{i} = struct( ...
        'source', candidate.source, ...
        'pose', candidate.state.pose(:).', ...
        'numReachableEC', numReachable, ...
        'reasons', {reasons});
    if numReachable > 0
        selectedState = candidate.state;
        info.selectedSource = candidate.source;
        info.selectedPose = selectedState.pose(:).';
        info.numReachableEC = numReachable;
        info.candidateReasons = candidateReasons;
        return;
    end
end

info.selectedSource = 'noReachableECFromSafeStarts';
info.selectedPose = selectedState.pose(:).';
info.candidateReasons = candidateReasons;
end

function candidates = buildPostOptionalWallStartCandidates(currentState, wallStatus, fallbackState)
candidates = struct('source', {}, 'state', {});
candidates(end + 1) = struct('source', 'postNormalSafeState', 'state', fallbackState);

for i = numel(wallStatus):-1:1
    status = wallStatus(i);
    if isfield(status, 'pfPoseAtObserve') && numel(status.pfPoseAtObserve) == 3
        state = stateFromOptionalWallStatus(fallbackState, status, 'Observe');
        candidates(end + 1) = struct('source', sprintf('wall%d_pfPoseAtObserve', i), 'state', state); %#ok<AGROW>
    end
end

for i = numel(wallStatus):-1:1
    status = wallStatus(i);
    if isfield(status, 'pfPoseAtClassify') && numel(status.pfPoseAtClassify) == 3
        state = stateFromOptionalWallStatus(fallbackState, status, 'Classify');
        candidates(end + 1) = struct('source', sprintf('wall%d_pfPoseAtClassify', i), 'state', state); %#ok<AGROW>
    end
end

for i = numel(wallStatus):-1:1
    status = wallStatus(i);
    if isfield(status, 'bumpProbe') && isstruct(status.bumpProbe) && ...
            isfield(status.bumpProbe, 'endPose') && numel(status.bumpProbe.endPose) == 3
        state = fallbackState;
        state.pose = status.bumpProbe.endPose(:);
        state = copyPfBeliefFromStatus(state, status, 'Classify');
        candidates(end + 1) = struct('source', sprintf('wall%d_bumpProbeEnd', i), 'state', state); %#ok<AGROW>
    end
end

candidates(end + 1) = struct('source', 'postOptionalWallCurrentState', 'state', currentState);
end

function state = stateFromOptionalWallStatus(baseState, status, suffix)
state = baseState;
poseField = ['pfPoseAt' suffix];
state.pose = status.(poseField)(:);
state = copyPfBeliefFromStatus(state, status, suffix);
end

function [numReachable, reasons] = countReachableEcGoals(startXY, ecGoals, map, plannerParams)
numReachable = 0;
reasons = cell(size(ecGoals, 1), 1);
for i = 1:size(ecGoals, 1)
    [path, pathInfo] = planPathKnownMap(startXY, ecGoals(i, :), map, plannerParams);
    reasons{i} = pathInfo.reason;
    if ~isempty(path) && pathInfo.success
        numReachable = numReachable + 1;
    end
end
end

function [plannedPath, sequenceInfo] = prependBridgePathToEcPlan( ...
    currentXY, selectedXY, ecTailPath, sequenceInfo, map, plannerParams)
sequenceInfo.bridgePath = currentXY;
sequenceInfo.bridgeSuccess = true;
sequenceInfo.bridgeReason = 'sameStart';

if isempty(ecTailPath)
    plannedPath = zeros(0, 2);
    sequenceInfo.bridgeSuccess = false;
    sequenceInfo.bridgeReason = 'emptyEcTailPath';
    return;
end

if norm(currentXY(:).' - selectedXY(:).') < 0.05
    plannedPath = ecTailPath;
    return;
end

[bridgePath, bridgeInfo] = planPathKnownMap(currentXY, selectedXY, map, plannerParams);
sequenceInfo.bridgePath = bridgePath;
sequenceInfo.bridgeInfo = bridgeInfo;
sequenceInfo.bridgeSuccess = ~isempty(bridgePath) && bridgeInfo.success;
sequenceInfo.bridgeReason = bridgeInfo.reason;

if ~sequenceInfo.bridgeSuccess
    plannedPath = currentXY;
    sequenceInfo.success = false;
    sequenceInfo.reason = ['bridgeFailed_' bridgeInfo.reason];
    sequenceInfo.beepMask = false(1, 1);
    return;
end

plannedPath = [bridgePath; ecTailPath(2:end, :)];
bridgeBeepMask = false(size(bridgePath, 1), 1);
tailMask = sequenceInfo.beepMask(:);
if numel(tailMask) >= 2
    plannedMask = [bridgeBeepMask; tailMask(2:end)];
else
    plannedMask = bridgeBeepMask;
end
sequenceInfo.beepMask = plannedMask;
sequenceInfo.plannedPath = plannedPath;
sequenceInfo.pathLength = pathLengthLocal(plannedPath);
end

function [currentState, ecPlanningStartState, planningStartInfo, ...
    ecVisitOrder, ecTailPath, ecPlannerInfo, skippedECwaypoints, ...
    ecPlannedPath, dataStore] = runPostOptionalWallEscapeAndReplan( ...
        Robot, currentState, wallStatus, postNormalSafeState, ecGoals, ...
        verifiedMap, plannerParams, escapeParams, dataStore)

if ~isfield(dataStore, 'postOptionalWallEscape') || isempty(dataStore.postOptionalWallEscape)
    dataStore.postOptionalWallEscape = [];
end

ecPlanningStartState = currentState;
planningStartInfo = struct( ...
    'selectedSource', 'bridgeFailedBeforeEscape', ...
    'selectedPose', currentState.pose(:).', ...
    'numReachableEC', 0, ...
    'candidateReasons', {{}});
ecVisitOrder = [];
ecTailPath = currentState.pose(1:2).';
ecPlannerInfo = struct('success', false, 'reason', 'bridgeFailedBeforeEscape', ...
    'beepMask', false(1, 1), 'escapeAttempts', 0);
skippedECwaypoints = zeros(0, 3);
ecPlannedPath = currentState.pose(1:2).';

for attempt = 1:escapeParams.maxAttempts
    turnSign = 1;
    if mod(attempt, 2) == 0
        turnSign = -1;
    end
    [currentState, backDistance, turnAngle] = runPostOptionalWallEscapeMotion( ...
        Robot, currentState, escapeParams, turnSign);
    currentState = resetPfParticlesAroundPose(currentState);

    [ecPlanningStartState, planningStartInfo] = selectPostOptionalWallPlanningState( ...
        currentState, wallStatus, postNormalSafeState, ecGoals, verifiedMap, plannerParams);
    [ecVisitOrder, ecTailPath, ecPlannerInfo, skippedECwaypoints] = ...
        planEcWaypointsNearestNeighbor(ecPlanningStartState.pose(1:2).', ecGoals, verifiedMap, plannerParams);
    [ecPlannedPath, ecPlannerInfo] = prependBridgePathToEcPlan( ...
        currentState.pose(1:2).', ecPlanningStartState.pose(1:2).', ...
        ecTailPath, ecPlannerInfo, verifiedMap, plannerParams);
    ecPlannerInfo.escapeAttempts = attempt;

    dataStore.postOptionalWallEscape = [dataStore.postOptionalWallEscape; ...
        attempt backDistance turnAngle double(ecPlannerInfo.bridgeSuccess)];
    fprintf('Post optional-wall escape attempt %d: bridgeSuccess=%d reason=%s\n', ...
        attempt, ecPlannerInfo.bridgeSuccess, ecPlannerInfo.bridgeReason);

    if ecPlannerInfo.bridgeSuccess
        return;
    end
end
end

function [currentState, backDistance, turnAngle] = runPostOptionalWallEscapeMotion( ...
    Robot, currentState, escapeParams, turnSign)
stopRobotSafe(Robot);

backDistance = 0;
while backDistance < escapeParams.backDistance
    SetFwdVelAngVelCreate(Robot, escapeParams.backVel, 0);
    pause(escapeParams.controlDt);
    odom = readPostWallEscapeOdom(Robot);
    if odom.valid
        currentState.pose = integratePostWallEscapePose(currentState.pose, odom.d, odom.phi);
        backDistance = backDistance + abs(odom.d);
    else
        step = abs(escapeParams.backVel) * escapeParams.controlDt;
        currentState.pose(1) = currentState.pose(1) - step * cos(currentState.pose(3));
        currentState.pose(2) = currentState.pose(2) - step * sin(currentState.pose(3));
        backDistance = backDistance + step;
    end
end

turnTarget = abs(escapeParams.turnAngle);
turnAngle = 0;
while abs(turnAngle) < turnTarget
    SetFwdVelAngVelCreate(Robot, 0, turnSign * abs(escapeParams.turnVel));
    pause(escapeParams.controlDt);
    odom = readPostWallEscapeOdom(Robot);
    if odom.valid
        currentState.pose = integratePostWallEscapePose(currentState.pose, odom.d, odom.phi);
        turnAngle = turnAngle + odom.phi;
    else
        step = turnSign * abs(escapeParams.turnVel) * escapeParams.controlDt;
        currentState.pose(3) = wrapToPiLocal(currentState.pose(3) + step);
        turnAngle = turnAngle + step;
    end
end

stopRobotSafe(Robot);
end

function odom = readPostWallEscapeOdom(Robot)
odom = struct('d', 0, 'phi', 0, 'valid', false);
try
    sensorPort = getCreateSensorPort(Robot);
    odom.d = DistanceSensorRoomba(sensorPort);
    odom.phi = AngleSensorRoomba(sensorPort);
    odom.valid = isfinite(odom.d) && isfinite(odom.phi);
catch
end
end

function pose = integratePostWallEscapePose(pose, d, phi)
pose = pose(:);
thetaMid = pose(3) + 0.5 * phi;
pose(1) = pose(1) + d * cos(thetaMid);
pose(2) = pose(2) + d * sin(thetaMid);
pose(3) = wrapToPiLocal(pose(3) + phi);
end

function currentState = updateStateFromWallStatus(currentState, wallStatus)
for i = 1:numel(wallStatus)
    status = wallStatus(i);
    if isfield(status, 'bumpProbe') && isstruct(status.bumpProbe) && ...
            isfield(status.bumpProbe, 'endPose') && numel(status.bumpProbe.endPose) == 3
        currentState.pose = status.bumpProbe.endPose(:);
        currentState = copyPfBeliefFromStatus(currentState, status, 'Classify');
        currentState = resetPfParticlesAroundPose(currentState);
    elseif isfield(status, 'pfPoseAtClassify') && numel(status.pfPoseAtClassify) == 3
        currentState.pose = status.pfPoseAtClassify(:);
        currentState = copyPfBeliefFromStatus(currentState, status, 'Classify');
    elseif isfield(status, 'pfPoseAtObserve') && numel(status.pfPoseAtObserve) == 3
        currentState.pose = status.pfPoseAtObserve(:);
        currentState = copyPfBeliefFromStatus(currentState, status, 'Observe');
    end
end
end

function currentState = copyPfBeliefFromStatus(currentState, status, suffix)
covField = ['pfCovAt' suffix];
particlesField = ['pfParticlesAt' suffix];
weightsField = ['pfWeightsAt' suffix];
if isfield(status, covField) && isequal(size(status.(covField)), [3 3])
    currentState.poseCov = status.(covField);
end
if isfield(status, particlesField) && ~isempty(status.(particlesField))
    currentState.particles = status.(particlesField);
end
if isfield(status, weightsField) && ~isempty(status.(weightsField))
    currentState.weights = status.(weightsField);
end
end

function currentState = resetPfParticlesAroundPose(currentState)
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

function [visitOrder, plannedPath, sequenceInfo, skipped] = ...
    planEcWaypointsNearestNeighbor(startXY, ecGoals, map, plannerParams)
visitOrder = [];
skipped = zeros(0, 3);
sequenceInfo = struct();
sequenceInfo.success = true;
sequenceInfo.reason = 'ok';
sequenceInfo.segmentInfo = {};
sequenceInfo.beepMask = false(0, 1);
sequenceInfo.highLevelGoals = ecGoals;

if isempty(ecGoals)
    plannedPath = zeros(0, 2);
    sequenceInfo.reason = 'noECGoals';
    return;
end

currentXY = startXY(:).';
plannedPath = currentXY;
beepMask = false(1, 1);
segmentInfo = {};

for goalIdx = 1:size(ecGoals, 1)
    [segmentPath, segmentInfoOne] = planPathKnownMap( ...
        currentXY, ecGoals(goalIdx, :), map, plannerParams);
    if isempty(segmentPath) || ~segmentInfoOne.success
        skipped = [skipped; [goalIdx, ecGoals(goalIdx, :)]]; %#ok<AGROW>
        continue;
    end

    visitOrder(end + 1, 1) = goalIdx; %#ok<AGROW>
    segmentInfo{end + 1, 1} = segmentInfoOne; %#ok<AGROW>
    if size(segmentPath, 1) >= 2
        plannedPath = [plannedPath; segmentPath(2:end, :)]; %#ok<AGROW>
        beepMask = [beepMask; false(max(0, size(segmentPath, 1) - 2), 1); true]; %#ok<AGROW>
    else
        beepMask(end) = true;
    end

    currentXY = ecGoals(goalIdx, :);
end

sequenceInfo.segmentInfo = segmentInfo;
sequenceInfo.beepMask = beepMask(:);
sequenceInfo.visitOrder = visitOrder;
sequenceInfo.plannedPath = plannedPath;
sequenceInfo.skippedECwaypoints = skipped;
sequenceInfo.pathLength = pathLengthLocal(plannedPath);
if ~isempty(skipped)
    sequenceInfo.reason = 'someECGoalsSkipped';
end
end

function len = pathLengthLocal(path)
if size(path, 1) < 2
    len = 0;
    return;
end
steps = diff(path, 1, 1);
len = sum(sqrt(sum(steps .^ 2, 2)));
end

function savePartialRun(codeDir, initState, dataStore, S, normalGoals, ecGoals)
save(fullfile(codeDir, 'latestPfWaypointEcOptionalPracticeRun.mat'), ...
    'initState', 'dataStore', 'S', 'normalGoals', 'ecGoals');
end
