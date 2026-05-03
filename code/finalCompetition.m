function [dataStoreOut] = finalCompetition(Robot, maxTime, offset_x, offset_y)
% finalCompetition Main 4-credit competition entry.
%
% Required signature:
%   dataStore = finalCompetition(Robot, maxTime, offset_x, offset_y)
%
% Data is also mirrored to global dataStore so a Ctrl-C interruption still
% leaves report data in the MATLAB workspace.

global dataStore; %#ok<GVMIS>

if nargin < 2 || isempty(maxTime)
    maxTime = 420.0;
end
if nargin < 3 || isempty(offset_x)
    offset_x = 0.0;
end
if nargin < 4 || isempty(offset_y)
    offset_y = 0.08;
end

codeDir = fileparts(mfilename('fullpath'));
runTimer = tic;
dataStore = struct();
dataStore.final = struct();
dataStore.final.status = 'running';
dataStore.final.stopReason = '';
dataStore.final.maxTime = maxTime;
dataStore.final.offset = [offset_x offset_y];
dataStore.final.startTime = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
dataStore.final.saveFile = fullfile(codeDir, 'latestFinalCompetitionRun.mat');
dataStore.final.figureFile = fullfile(codeDir, 'latestFinalCompetitionResult.fig');
dataStore.final.stageOrder = {'normalWaypoints', 'optionalWalls', 'ecWaypoints'};
dataStore = syncFinalGlobal(dataStore, 'start');

cleanupObj = onCleanup(@()finalCompetitionCleanup(Robot, codeDir));

try
    S = loadCompetitionMap(codeDir);
    dataStore.S = S;
    dataStore.compMap = S;
    dataStore = syncFinalGlobal(dataStore, 'loadedMap');

    initParams = initDefaultParams();
    initParams.maxInitTime = min(25.0, max(8.0, remainingTime(maxTime, runTimer) - 5.0));
    initParams.minInitTime = min(8.0, initParams.maxInitTime);
    initParams.maxDepthRange = 10.0;
    initParams.minDepthRange = 0.175;
    initParams.debugPrint = true;

    [initState, dataStore] = initializeLocalization( ...
        Robot, S.map, S.waypoints, S.beaconLoc, offset_x, offset_y, initParams, dataStore);
    dataStore.initState = initState;
    dataStore = syncFinalGlobal(dataStore, 'initialized');

    plannerParams = knownMapPlannerDefaultParams();
    originalNormalGoals = defaultCompetitionGoals(S.waypoints, initState.startWaypointIdx);
    [normalGoals, normalGoalOrder, normalGoalOrderInfo] = planGoalVisitOrderNearestNeighbor( ...
        initState.pose(1:2).', originalNormalGoals, S.map, plannerParams);
    ecGoals = getFieldOrDefault(S, 'ECwaypoints', zeros(0, 2));

    ekfParams = ekfWaypointDefaultParams();
    ekfParams.maxDepthRange = 10.0;
    ekfParams.minDepthRange = 0.175;
    ekfParams.debugPrint = true;
    ekfParams.enableLivePlot = true;
    ekfParams.livePlotUpdatePeriod = 0.35;

    postWallEscapeParams = struct();
    postWallEscapeParams.maxAttempts = 3;
    postWallEscapeParams.backDistance = 0.35;
    postWallEscapeParams.backVel = -0.06;
    postWallEscapeParams.turnAngle = deg2rad(30);
    postWallEscapeParams.turnVel = 0.35;
    postWallEscapeParams.controlDt = 0.10;

    currentState = initState;
    normalEkfState = struct();
    normalNavState = struct();
    normalPlannedPath = [];
    normalPlannerInfo = struct('success', true, 'reason', 'noNormalGoals');
    normalSnapInfo = struct('snapped', false, 'target', [], 'source', 'notRun');

    if ~isempty(normalGoals) && remainingTime(maxTime, runTimer) > 5.0
        [normalPlannedPath, normalPlannerInfo] = planWaypointSequenceKnownMap( ...
            currentState.pose(1:2).', normalGoals, S.map, plannerParams);
        if isempty(normalPlannedPath) || ~normalPlannerInfo.success
            dataStore.final.stopReason = ['normalPlanFailed_' normalPlannerInfo.reason];
        else
            normalFollowerParams = ekfParams;
            normalFollowerParams.maxRunTime = max(1.0, remainingTime(maxTime, runTimer) - 8.0);
            if isfield(normalPlannerInfo, 'beepMask')
                normalFollowerParams.beepMask = normalPlannerInfo.beepMask;
            end
            [normalEkfState, dataStore, normalNavState] = runEkfWaypointFollower( ...
                Robot, S.map, S.beaconLoc, normalPlannedPath, offset_x, offset_y, ...
                currentState, normalFollowerParams, dataStore);
            currentState = updateStateFromEkf(currentState, normalEkfState);
            [currentState, normalSnapInfo] = snapPlanningStateToLastReachedGoal( ...
                currentState, normalNavState, normalPlannedPath, normalFollowerParams.beepMask);
            dataStore = syncFinalGlobal(dataStore, 'normalWaypointsDone');
        end
    end

    postNormalSafeState = currentState;

    verifyParams = optionalWallDefaultParams();
    verifyParams.followerMode = 'ekf';
    verifyParams.maxDepthRange = 10.0;
    verifyParams.minDepthRange = 0.175;
    verifyParams.debugPrint = true;
    verifyParams.followerMaxRunTime = min(verifyParams.followerMaxRunTime, ...
        max(20.0, remainingTime(maxTime, runTimer) / max(1, size(S.optWalls, 1))));

    ekfVerifyParams = ekfParams;
    ekfVerifyParams.beepMask = [];
    wallStatus = repmat(emptyFinalWallStatus(), size(S.optWalls, 1), 1);
    verifiedMap = S.map;

    if isempty(dataStore.final.stopReason) && ~isempty(S.optWalls) && remainingTime(maxTime, runTimer) > 8.0
        [wallStatus, dataStore, verifiedMap] = verifyOptionalWalls( ...
            Robot, S.map, S.optWalls, S.beaconLoc, offset_x, offset_y, ...
            currentState, ekfVerifyParams, plannerParams, verifyParams, dataStore);
        currentState = updateStateFromWallStatus(currentState, wallStatus);
        dataStore = syncFinalGlobal(dataStore, 'optionalWallsDone');
    end

    ecPlanningStartState = currentState;
    postOptionalWallPlanningStartInfo = struct('selectedSource', 'notRun', ...
        'selectedPose', currentState.pose(:).', 'numReachableEC', 0, 'candidateReasons', {{}});
    ecPlannedPath = [];
    ecPlannerInfo = struct('success', true, 'reason', 'noECGoals', ...
        'bridgeSuccess', true, 'bridgeReason', 'noECGoals', 'beepMask', false(0, 1));
    ecEkfState = struct();
    ecNavState = struct();

    originalECGoals = ecGoals;
    ecGoalOrder = [];
    ecGoalOrderInfo = struct('reason', 'notRun');

    if isempty(dataStore.final.stopReason) && ~isempty(ecGoals) && remainingTime(maxTime, runTimer) > 8.0
        [ecPlanningStartState, postOptionalWallPlanningStartInfo] = ...
            selectPostOptionalWallPlanningState( ...
                currentState, wallStatus, postNormalSafeState, ecGoals, verifiedMap, plannerParams);
        [ecGoals, ecGoalOrder, ecGoalOrderInfo] = planGoalVisitOrderNearestNeighbor( ...
            ecPlanningStartState.pose(1:2).', originalECGoals, verifiedMap, plannerParams);
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

        if ~isempty(ecPlannedPath) && isfield(ecPlannerInfo, 'success') && ecPlannerInfo.success
            ecFollowerParams = ekfParams;
            ecFollowerParams.maxRunTime = max(1.0, remainingTime(maxTime, runTimer) - 2.0);
            if isfield(ecPlannerInfo, 'beepMask')
                ecFollowerParams.beepMask = ecPlannerInfo.beepMask;
            end
            [ecEkfState, dataStore, ecNavState] = runEkfWaypointFollower( ...
                Robot, verifiedMap, S.beaconLoc, ecPlannedPath, offset_x, offset_y, ...
                currentState, ecFollowerParams, dataStore);
            currentState = updateStateFromEkf(currentState, ecEkfState);
        else
            dataStore.final.ecSkippedReason = getFieldOrDefault(ecPlannerInfo, 'reason', 'ecPlanFailed');
        end
        dataStore = syncFinalGlobal(dataStore, 'ecWaypointsDone');
    end

    dataStore.originalNormalGoals = originalNormalGoals;
    dataStore.normalGoals = normalGoals;
    dataStore.normalGoalOrder = normalGoalOrder;
    dataStore.normalGoalOrderInfo = normalGoalOrderInfo;
    dataStore.normalPlannedPath = normalPlannedPath;
    dataStore.normalPlannerInfo = normalPlannerInfo;
    dataStore.normalSnapInfo = normalSnapInfo;
    dataStore.normalEkfState = normalEkfState;
    dataStore.normalNavState = normalNavState;
    dataStore.postNormalSafeState = postNormalSafeState;
    dataStore.wallStatus = wallStatus;
    dataStore.verifiedMap = verifiedMap;
    dataStore.originalECGoals = originalECGoals;
    dataStore.ecGoals = ecGoals;
    dataStore.ecGoalOrder = ecGoalOrder;
    dataStore.ecGoalOrderInfo = ecGoalOrderInfo;
    dataStore.ecPlanningStartState = ecPlanningStartState;
    dataStore.postOptionalWallPlanningStartInfo = postOptionalWallPlanningStartInfo;
    dataStore.ecPlannedPath = ecPlannedPath;
    dataStore.ecPlannerInfo = ecPlannerInfo;
    dataStore.ecEkfState = ecEkfState;
    dataStore.ecNavState = ecNavState;
    dataStore.finalCurrentState = currentState;
    dataStore.final.status = 'completed';
    if isempty(dataStore.final.stopReason)
        dataStore.final.stopReason = 'completedMainFlow';
    end
    dataStore.final.runTimeSec = toc(runTimer);
    dataStore = syncFinalGlobal(dataStore, 'completed');

catch ME
    stopRobotSafe(Robot);
    dataStore.final.status = 'error';
    dataStore.final.stopReason = ['error_' ME.identifier];
    dataStore.final.errorMessage = ME.message;
    dataStore.final.errorReport = getReport(ME, 'extended', 'hyperlinks', 'off');
    dataStore.final.runTimeSec = toc(runTimer);
    dataStore = syncFinalGlobal(dataStore, 'error');
    fprintf(2, 'finalCompetition error: %s\n', ME.message);
end

stopRobotSafe(Robot);
finalCompetitionSaveAndPlot(codeDir);
dataStoreOut = dataStore;
end

function S = loadCompetitionMap(codeDir)
candidateFiles = { ...
    fullfile(pwd, 'PracticeMap2026.mat'), ...
    fullfile(codeDir, 'PracticeMap2026.mat'), ...
    fullfile(pwd, 'compMap.mat'), ...
    fullfile(codeDir, 'compMap.mat')};
mapFile = '';
for i = 1:numel(candidateFiles)
    if exist(candidateFiles{i}, 'file')
        mapFile = candidateFiles{i};
        break;
    end
end
if isempty(mapFile)
    error('finalCompetition:missingMap', 'Could not find PracticeMap2026.mat or compMap.mat.');
end
S = load(mapFile);
required = {'map', 'optWalls', 'waypoints', 'beaconLoc'};
for i = 1:numel(required)
    if ~isfield(S, required{i})
        error('finalCompetition:badMap', 'Map file missing field %s.', required{i});
    end
end
if ~isfield(S, 'ECwaypoints')
    S.ECwaypoints = zeros(0, 2);
end
S.mapFile = mapFile;
end

function dataStoreOut = syncFinalGlobal(dataStoreIn, stage)
global dataStore; %#ok<GVMIS>
dataStoreIn.final.globalStage = stage;
dataStoreIn.final.globalSyncTime = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss.SSS'));
dataStore = dataStoreIn;
dataStoreOut = dataStoreIn;
end

function finalCompetitionCleanup(Robot, codeDir)
global dataStore; %#ok<GVMIS>
try
    stopRobotSafe(Robot);
catch
end
try
    if isstruct(dataStore) && isfield(dataStore, 'final') && ...
            isfield(dataStore.final, 'saveCompleted') && dataStore.final.saveCompleted
        return;
    end
    finalCompetitionSaveAndPlot(codeDir);
catch ME
    fprintf(2, 'finalCompetition cleanup save/plot failed: %s\n', ME.message);
end
end

function finalCompetitionSaveAndPlot(codeDir)
global dataStore; %#ok<GVMIS>
if isempty(dataStore) || ~isstruct(dataStore)
    return;
end
if ~isfield(dataStore, 'final')
    dataStore.final = struct();
end
dataStore.final.lastSaveTime = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss.SSS'));
if ~isfield(dataStore.final, 'saveFile') || isempty(dataStore.final.saveFile)
    dataStore.final.saveFile = fullfile(codeDir, 'latestFinalCompetitionRun.mat');
end
save(dataStore.final.saveFile, 'dataStore');
try
    fig = plotFinalCompetitionResult(dataStore);
    if isfield(dataStore.final, 'figureFile') && ~isempty(dataStore.final.figureFile)
        savefig(fig, dataStore.final.figureFile);
    end
    dataStore.final.saveCompleted = true;
    save(dataStore.final.saveFile, 'dataStore');
catch ME
    dataStore.final.plotError = ME.message;
    dataStore.final.saveCompleted = true;
    save(dataStore.final.saveFile, 'dataStore');
end
end

function t = remainingTime(maxTime, runTimer)
t = max(0, maxTime - toc(runTimer));
end

function goals = defaultCompetitionGoals(waypoints, startWaypointIdx)
goals = waypoints;
if isfinite(startWaypointIdx) && startWaypointIdx >= 1 && startWaypointIdx <= size(goals, 1)
    goals(startWaypointIdx, :) = [];
end
end

function [orderedGoals, visitOrder, info] = planGoalVisitOrderNearestNeighbor( ...
    startXY, goals, map, plannerParams)
% Order scoring goals greedily by planned path length on the provided map.
orderedGoals = zeros(0, 2);
visitOrder = zeros(0, 1);
info = struct('reason', 'ok', 'skipped', zeros(0, 3), ...
    'segmentLengths', zeros(0, 1));

if isempty(goals)
    info.reason = 'noGoals';
    return;
end

remainingIdx = (1:size(goals, 1)).';
currentXY = startXY(:).';

while ~isempty(remainingIdx)
    bestLocal = NaN;
    bestLength = inf;
    bestReason = '';

    for k = 1:numel(remainingIdx)
        goalIdx = remainingIdx(k);
        [path, planInfo] = planPathKnownMap(currentXY, goals(goalIdx, :), map, plannerParams);
        if isempty(path) || ~planInfo.success
            if isempty(bestReason)
                bestReason = planInfo.reason;
            end
            continue;
        end
        candidateLength = pathLengthLocal(path);
        if candidateLength < bestLength
            bestLength = candidateLength;
            bestLocal = k;
        end
    end

    if isnan(bestLocal)
        for k = 1:numel(remainingIdx)
            goalIdx = remainingIdx(k);
            info.skipped(end + 1, :) = [goalIdx goals(goalIdx, :)];
        end
        if isempty(orderedGoals)
            info.reason = ['noReachableGoals_' bestReason];
        else
            info.reason = ['someGoalsUnreachable_' bestReason];
        end
        return;
    end

    selectedIdx = remainingIdx(bestLocal);
    orderedGoals(end + 1, :) = goals(selectedIdx, :); %#ok<AGROW>
    visitOrder(end + 1, 1) = selectedIdx; %#ok<AGROW>
    info.segmentLengths(end + 1, 1) = bestLength;
    currentXY = goals(selectedIdx, :);
    remainingIdx(bestLocal) = [];
end
end

function value = getFieldOrDefault(S, fieldName, defaultValue)
if isstruct(S) && isfield(S, fieldName)
    value = S.(fieldName);
else
    value = defaultValue;
end
end

function status = emptyFinalWallStatus()
status = struct('wallIdx', NaN, 'wall', NaN(1, 4), 'status', 'unknown', ...
    'confidence', 0, 'reason', '');
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
if isempty(idx)
    idx = numGoals;
    snapInfo.source = 'lastPathGoal';
else
    snapInfo.source = 'lastBeepGoal';
end
target = goalWaypoints(idx, :);
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
candidates{end + 1} = struct('source', 'postNormalSafeState', 'state', postNormalSafeState);
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
planningStartInfo = struct('selectedSource', 'bridgeFailedBeforeEscape', ...
    'selectedPose', currentState.pose(:).', 'numReachableEC', 0, 'candidateReasons', {{}});
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
    dataStore = syncFinalGlobal(dataStore, sprintf('postWallEscape%d', attempt));
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
    odom.d = DistanceSensorRoomba(Robot);
    odom.phi = AngleSensorRoomba(Robot);
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
