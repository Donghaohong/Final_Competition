function dataStore = testEkfWaypointEcOptionalPracticeMap(Robot, goalWaypoints, ecGoalWaypoints)
% testEkfWaypointEcOptionalPracticeMap EKF normal waypoints -> optional walls -> EC.
%
% SimulatorGUI Autonomous Start usage:
%   testEkfWaypointEcOptionalPracticeMap
%
% Optional command-line style usage:
%   dataStore = testEkfWaypointEcOptionalPracticeMap(Robot, normalGoals, ecGoals)

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
ekfParams = ekfWaypointDefaultParams();
ekfParams.maxRunTime = 500.0;
ekfParams.maxDepthRange = 10.0;
ekfParams.minDepthRange = 0.175;
ekfParams.debugPrint = true;
ekfParams.enableLivePlot = true;

postWallEscapeParams = struct();
postWallEscapeParams.maxAttempts = 3;
postWallEscapeParams.backDistance = 0.35;
postWallEscapeParams.backVel = -0.06;
postWallEscapeParams.turnAngle = deg2rad(30);
postWallEscapeParams.turnVel = 0.35;
postWallEscapeParams.controlDt = 0.10;

currentState = initState;
stageOrder = {'normalWaypoints', 'optionalWalls', 'ecWaypoints'};

normalEkfState = struct();
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

    normalFollowerParams = ekfParams;
    if isfield(normalPlannerInfo, 'beepMask')
        normalFollowerParams.beepMask = normalPlannerInfo.beepMask;
    end

    [normalEkfState, dataStore, normalNavState] = runEkfWaypointFollower( ...
        Robot, S.map, S.beaconLoc, normalPlannedPath, offset_x, offset_y, ...
        currentState, normalFollowerParams, dataStore);
    currentState = updateStateFromEkf(currentState, normalEkfState);
    [currentState, normalSnapInfo] = snapPlanningStateToLastReachedGoal( ...
        currentState, normalNavState, normalPlannedPath, normalFollowerParams.beepMask);
end

postNormalSafeState = currentState;

verifyParams = optionalWallDefaultParams();
verifyParams.followerMode = 'ekf';
verifyParams.maxDepthRange = 10.0;
verifyParams.minDepthRange = 0.175;
verifyParams.debugPrint = true;

ekfVerifyParams = ekfParams;
ekfVerifyParams.beepMask = [];

[wallStatus, dataStore, verifiedMap] = verifyOptionalWalls( ...
    Robot, S.map, S.optWalls, S.beaconLoc, offset_x, offset_y, ...
    currentState, ekfVerifyParams, plannerParams, verifyParams, dataStore);
currentState = updateStateFromWallStatus(currentState, wallStatus);

[ecPlanningStartState, postOptionalWallPlanningStartInfo] = ...
    selectPostOptionalWallPlanningState( ...
        currentState, wallStatus, postNormalSafeState, ecGoals, verifiedMap, plannerParams);

[ecPlannedPath, ecPlannerInfo] = planWaypointSequenceKnownMap( ...
    ecPlanningStartState.pose(1:2).', ecGoals, verifiedMap, plannerParams);
[ecPlannedPath, ecPlannerInfo] = prependBridgePathToEcPlan( ...
    currentState.pose(1:2).', ecPlanningStartState.pose(1:2).', ...
    ecPlannedPath, ecPlannerInfo, verifiedMap, plannerParams);
if ~ecPlannerInfo.bridgeSuccess
    [currentState, ecPlanningStartState, postOptionalWallPlanningStartInfo, ...
        ecPlannerInfo, ecPlannedPath, dataStore] = runPostOptionalWallEscapeAndReplan( ...
            Robot, currentState, wallStatus, postNormalSafeState, ecGoals, ...
            verifiedMap, plannerParams, postWallEscapeParams, dataStore);
end

ecEkfState = struct();
ecNavState = struct();
if ~isempty(ecPlannedPath) && isfield(ecPlannerInfo, 'success') && ecPlannerInfo.success
    ecFollowerParams = ekfParams;
    if isfield(ecPlannerInfo, 'beepMask')
        ecFollowerParams.beepMask = ecPlannerInfo.beepMask;
    end
    [ecEkfState, dataStore, ecNavState] = runEkfWaypointFollower( ...
        Robot, verifiedMap, S.beaconLoc, ecPlannedPath, offset_x, offset_y, ...
        currentState, ecFollowerParams, dataStore);
    currentState = updateStateFromEkf(currentState, ecEkfState);
else
    stopRobotSafe(Robot);
end

dataStore.stageOrder = stageOrder;
dataStore.initState = initState;
dataStore.normalGoals = normalGoals;
dataStore.normalPlannedPath = normalPlannedPath;
dataStore.normalPlannerInfo = normalPlannerInfo;
dataStore.normalSnapInfo = normalSnapInfo;
dataStore.normalEkfState = normalEkfState;
dataStore.normalNavState = normalNavState;
dataStore.postNormalSafeState = postNormalSafeState;
dataStore.wallStatus = wallStatus;
dataStore.verifiedMap = verifiedMap;
dataStore.ecPlanningStartState = ecPlanningStartState;
dataStore.postOptionalWallPlanningStartInfo = postOptionalWallPlanningStartInfo;
dataStore.ecGoals = ecGoals;
dataStore.ecPlannedPath = ecPlannedPath;
dataStore.ecPlannerInfo = ecPlannerInfo;
dataStore.ecEkfState = ecEkfState;
dataStore.ecNavState = ecNavState;
dataStore.finalCurrentState = currentState;

save(fullfile(codeDir, 'latestEkfWaypointEcOptionalPracticeRun.mat'), ...
    'initState', 'normalEkfState', 'normalNavState', ...
    'ecEkfState', 'ecNavState', 'wallStatus', 'verifiedMap', ...
    'dataStore', 'S', 'normalGoals', 'ecGoals', ...
    'normalPlannedPath', 'normalPlannerInfo', ...
    'ecPlannedPath', 'ecPlannerInfo', 'stageOrder', ...
    'normalSnapInfo', 'postOptionalWallPlanningStartInfo');

disp('Initialization result:');
disp(initState);
disp('Normal waypoint planned path:');
disp(normalPlannedPath);
disp('Normal EKF waypoint follower result:');
disp(normalEkfState);
disp('Normal planning snap info:');
disp(normalSnapInfo);
disp('EKF optional wall verification result:');
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
disp('EC planned path:');
disp(ecPlannedPath);
disp('EC EKF waypoint follower result:');
disp(ecEkfState);
end

function goals = defaultPracticeGoals(waypoints, startWaypointIdx)
goals = waypoints;
if isfinite(startWaypointIdx) && startWaypointIdx >= 1 && startWaypointIdx <= size(goals, 1)
    goals(startWaypointIdx, :) = [];
end
end

function currentState = updateStateFromEkf(currentState, ekfState)
if isfield(ekfState, 'pose') && numel(ekfState.pose) == 3
    currentState.pose = ekfState.pose;
end
if isfield(ekfState, 'poseCov') && isequal(size(ekfState.poseCov), [3 3])
    currentState.poseCov = ekfState.poseCov;
end
end

function [currentState, snapInfo] = snapPlanningStateToLastReachedGoal( ...
    currentState, navState, goalWaypoints, beepMask)
snapInfo = struct('snapped', false, 'target', [], 'source', 'notReachedAllGoals');

if isempty(goalWaypoints) || ~isfield(navState, 'reachedAllGoals') || ~navState.reachedAllGoals
    return;
end

numGoals = size(goalWaypoints, 1);
mask = normalizeLocalMask(beepMask, numGoals, false);
lastIdx = find(mask & navState.reachedGoals(:), 1, 'last');
if isempty(lastIdx)
    lastIdx = numGoals;
    snapInfo.source = 'lastPathGoal';
else
    snapInfo.source = 'lastBeepGoal';
end

target = goalWaypoints(lastIdx, :);
currentState.pose(1:2) = target(:);
snapInfo.snapped = true;
snapInfo.target = target;
end

function mask = normalizeLocalMask(mask, targetLen, defaultValue)
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

function currentState = updateStateFromWallStatus(currentState, wallStatus)
for i = 1:numel(wallStatus)
    status = wallStatus(i);
    if isfield(status, 'bumpProbe') && isstruct(status.bumpProbe) && ...
            isfield(status.bumpProbe, 'endPose') && numel(status.bumpProbe.endPose) == 3
        currentState.pose = status.bumpProbe.endPose(:);
    elseif isfield(status, 'ekfPoseAtClassify') && numel(status.ekfPoseAtClassify) == 3
        currentState.pose = status.ekfPoseAtClassify(:);
    elseif isfield(status, 'ekfPoseAtObserve') && numel(status.ekfPoseAtObserve) == 3
        currentState.pose = status.ekfPoseAtObserve(:);
    end

    if isfield(status, 'ekfCovAtClassify') && isequal(size(status.ekfCovAtClassify), [3 3])
        currentState.poseCov = status.ekfCovAtClassify;
    elseif isfield(status, 'ekfCovAtObserve') && isequal(size(status.ekfCovAtObserve), [3 3])
        currentState.poseCov = status.ekfCovAtObserve;
    end
end
end

function [selectedState, info] = selectPostOptionalWallPlanningState( ...
    currentState, wallStatus, postNormalSafeState, ecGoals, map, plannerParams)
candidates = {};
candidates{end + 1} = struct('source', 'postNormalSafeState', 'state', postNormalSafeState);
candidates{end + 1} = struct('source', 'currentState', 'state', currentState);

for i = numel(wallStatus):-1:1
    status = wallStatus(i);
    candidateState = currentState;
    if isfield(status, 'ekfPoseAtObserve') && numel(status.ekfPoseAtObserve) == 3
        candidateState.pose = status.ekfPoseAtObserve(:);
        if isfield(status, 'ekfCovAtObserve') && isequal(size(status.ekfCovAtObserve), [3 3])
            candidateState.poseCov = status.ekfCovAtObserve;
        end
        candidates{end + 1} = struct('source', sprintf('wall%dObserve', i), 'state', candidateState); %#ok<AGROW>
    end
    if isfield(status, 'ekfPoseAtClassify') && numel(status.ekfPoseAtClassify) == 3
        candidateState.pose = status.ekfPoseAtClassify(:);
        if isfield(status, 'ekfCovAtClassify') && isequal(size(status.ekfCovAtClassify), [3 3])
            candidateState.poseCov = status.ekfCovAtClassify;
        end
        candidates{end + 1} = struct('source', sprintf('wall%dClassify', i), 'state', candidateState); %#ok<AGROW>
    end
end

info = struct('selectedSource', 'none', 'selectedPose', currentState.pose(:).', ...
    'numReachableEC', 0, 'candidateReasons', {cell(0, 3)});
selectedState = currentState;

for i = 1:numel(candidates)
    candidate = candidates{i};
    [numReachable, reason] = countReachableGoals( ...
        candidate.state.pose(1:2).', ecGoals, map, plannerParams);
    info.candidateReasons(end + 1, :) = {candidate.source, numReachable, reason};
    if numReachable > 0
        selectedState = candidate.state;
        info.selectedSource = candidate.source;
        info.selectedPose = selectedState.pose(:).';
        info.numReachableEC = numReachable;
        return;
    end
end
end

function [numReachable, reason] = countReachableGoals(startXY, goals, map, plannerParams)
numReachable = 0;
reason = 'noGoals';
for i = 1:size(goals, 1)
    [path, planInfo] = planPathKnownMap(startXY, goals(i, :), map, plannerParams);
    if ~isempty(path) && planInfo.success
        numReachable = numReachable + 1;
        reason = 'ok';
    elseif numReachable == 0
        reason = planInfo.reason;
    end
end
end

function [plannedPath, sequenceInfo] = prependBridgePathToEcPlan( ...
    currentXY, selectedXY, ecTailPath, sequenceInfo, map, plannerParams)
currentXY = currentXY(:).';
selectedXY = selectedXY(:).';
sequenceInfo.ecTailPath = ecTailPath;
sequenceInfo.bridgePath = currentXY;
sequenceInfo.bridgeSuccess = true;
sequenceInfo.bridgeReason = 'sameStart';

if isempty(ecTailPath)
    plannedPath = zeros(0, 2);
    sequenceInfo.success = false;
    sequenceInfo.bridgeSuccess = false;
    sequenceInfo.bridgeReason = 'emptyEcTailPath';
    sequenceInfo.beepMask = false(0, 1);
    return;
end

if norm(currentXY - selectedXY) < 0.05
    plannedPath = ecTailPath;
    sequenceInfo.bridgeSuccess = isfield(sequenceInfo, 'success') && sequenceInfo.success;
    sequenceInfo.bridgeReason = 'sameStart';
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
    sequenceInfo.beepMask = [bridgeBeepMask; tailMask(2:end)];
else
    sequenceInfo.beepMask = bridgeBeepMask;
end
sequenceInfo.plannedPath = plannedPath;
sequenceInfo.pathLength = pathLengthLocal(plannedPath);
end

function [currentState, ecPlanningStartState, planningStartInfo, ...
    ecPlannerInfo, ecPlannedPath, dataStore] = runPostOptionalWallEscapeAndReplan( ...
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
ecPlannerInfo = struct('success', false, 'reason', 'bridgeFailedBeforeEscape', ...
    'beepMask', false(1, 1), 'escapeAttempts', 0, ...
    'bridgeSuccess', false, 'bridgeReason', 'bridgeFailedBeforeEscape');
ecPlannedPath = currentState.pose(1:2).';

for attempt = 1:escapeParams.maxAttempts
    turnSign = 1;
    if mod(attempt, 2) == 0
        turnSign = -1;
    end

    [currentState, backDistance, turnAngle] = runPostOptionalWallEscapeMotion( ...
        Robot, currentState, escapeParams, turnSign);

    [ecPlanningStartState, planningStartInfo] = selectPostOptionalWallPlanningState( ...
        currentState, wallStatus, postNormalSafeState, ecGoals, verifiedMap, plannerParams);
    [ecTailPath, ecPlannerInfo] = planWaypointSequenceKnownMap( ...
        ecPlanningStartState.pose(1:2).', ecGoals, verifiedMap, plannerParams);
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
        currentState.poseCov = inflatePostWallEscapeCov(currentState.poseCov, odom);
        backDistance = backDistance + abs(odom.d);
    else
        step = abs(escapeParams.backVel) * escapeParams.controlDt;
        currentState.pose(1) = currentState.pose(1) - step * cos(currentState.pose(3));
        currentState.pose(2) = currentState.pose(2) - step * sin(currentState.pose(3));
        currentState.poseCov = inflatePostWallEscapeCov(currentState.poseCov, struct('d', step, 'phi', 0));
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
        currentState.poseCov = inflatePostWallEscapeCov(currentState.poseCov, odom);
        turnAngle = turnAngle + odom.phi;
    else
        step = turnSign * abs(escapeParams.turnVel) * escapeParams.controlDt;
        currentState.pose(3) = wrapToPiLocal(currentState.pose(3) + step);
        currentState.poseCov = inflatePostWallEscapeCov(currentState.poseCov, struct('d', 0, 'phi', step));
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

function poseCov = inflatePostWallEscapeCov(poseCov, odom)
if ~isequal(size(poseCov), [3 3]) || any(~isfinite(poseCov(:)))
    poseCov = diag([0.08 0.08 deg2rad(12)] .^ 2);
end
poseCov = (poseCov + poseCov.') / 2;
motionScale = abs(odom.d) + abs(odom.phi);
extra = diag([0.015 + 0.04 * motionScale, ...
              0.015 + 0.04 * motionScale, ...
              deg2rad(2.0) + 0.03 * abs(odom.phi)] .^ 2);
poseCov = poseCov + extra;
poseCov = (poseCov + poseCov.') / 2;
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
save(fullfile(codeDir, 'latestEkfWaypointEcOptionalPracticeRun.mat'), ...
    'initState', 'dataStore', 'S', 'normalGoals', 'ecGoals');
end
