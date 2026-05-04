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

    normalGoals = defaultCompetitionGoals(S.waypoints, initState.startWaypointIdx);
    ecGoals = getFieldOrDefault(S, 'ECwaypoints', zeros(0, 2));
    ecGoalVisitOrder = [];

    plannerParams = knownMapPlannerDefaultParams();
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
    [normalGoals, normalGoalVisitOrder] = orderNearestPoints( ...
        currentState.pose(1:2).', normalGoals);
    normalPlanningMap = [S.map; S.optWalls];
    normalEkfState = struct();
    normalNavState = struct();
    normalPlannedPath = [];
    normalPlannerInfo = struct('success', true, 'reason', 'noNormalGoals');
    normalSnapInfo = struct('snapped', false, 'target', [], 'source', 'notRun');

    if ~isempty(normalGoals) && remainingTime(maxTime, runTimer) > 5.0
        [normalPlannedPath, normalPlannerInfo] = planWaypointSequenceKnownMap( ...
            currentState.pose(1:2).', normalGoals, normalPlanningMap, plannerParams);
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
            dataStore.normalGoals = normalGoals;
            dataStore.normalGoalVisitOrder = normalGoalVisitOrder;
            dataStore.normalPlanningMap = normalPlanningMap;
            dataStore.normalPlannedPath = normalPlannedPath;
            dataStore.normalPlannerInfo = normalPlannerInfo;
            dataStore.normalSnapInfo = normalSnapInfo;
            dataStore.normalEkfState = normalEkfState;
            dataStore.normalNavState = normalNavState;
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
    for wallIdx = 1:size(S.optWalls, 1)
        wallStatus(wallIdx).wallIdx = wallIdx;
        wallStatus(wallIdx).wall = S.optWalls(wallIdx, :);
    end
    wallStatusByVisit = [];
    optionalWallVisitOrder = [];
    verifiedMap = S.map;

    if isempty(dataStore.final.stopReason) && ~isempty(S.optWalls) && remainingTime(maxTime, runTimer) > 8.0
        remainingWallIdx = 1:size(S.optWalls, 1);
        while ~isempty(remainingWallIdx) && remainingTime(maxTime, runTimer) > 8.0
            nextWallIdx = nearestOptionalWallIndex( ...
                currentState.pose(1:2).', S.optWalls, remainingWallIdx);
            optionalWallVisitOrder(end + 1, 1) = nextWallIdx; %#ok<AGROW>

            [localWallStatus, dataStore, verifiedMap] = verifyOptionalWallWithRelaxation( ...
                Robot, verifiedMap, S.optWalls(nextWallIdx, :), nextWallIdx, ...
                S.beaconLoc, offset_x, offset_y, currentState, ekfVerifyParams, ...
                plannerParams, verifyParams, dataStore);

            if ~isempty(localWallStatus)
                localWallStatus.wallIdx = nextWallIdx;
                localWallStatus.wall = S.optWalls(nextWallIdx, :);
                fprintf('[finalCompetition] optional wall %d status=%s reason=%s\n', ...
                    nextWallIdx, localWallStatus.status, localWallStatus.reason);
                wallStatus(nextWallIdx) = copyFinalWallStatusSummary( ...
                    wallStatus(nextWallIdx), localWallStatus, nextWallIdx, S.optWalls(nextWallIdx, :));
                if isempty(wallStatusByVisit)
                    wallStatusByVisit = localWallStatus;
                else
                    wallStatusByVisit(end + 1, 1) = localWallStatus; %#ok<AGROW>
                end
                currentState = updateStateFromWallStatus(currentState, localWallStatus);
            end

            remainingWallIdx(remainingWallIdx == nextWallIdx) = [];
            dataStore = syncFinalGlobal(dataStore, sprintf('optionalWall%dDone', nextWallIdx));
        end
        dataStore.optionalWallStatus = wallStatus;
        dataStore.wallStatus = wallStatus;
        dataStore.wallStatusByVisit = wallStatusByVisit;
        dataStore.optionalWallVisitOrder = optionalWallVisitOrder;
        dataStore.verifiedMap = verifiedMap;
        unknownWallIdx = find(strcmp({wallStatus.status}.', 'unknown'));
        if ~isempty(unknownWallIdx)
            dataStore.final.stopReason = 'optionalWallsIncomplete';
            dataStore.final.optionalWallsIncompleteIdx = unknownWallIdx(:).';
        end
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

    if isempty(dataStore.final.stopReason) && ~isempty(ecGoals) && remainingTime(maxTime, runTimer) > 8.0
        planningWallStatus = wallStatus;
        if ~isempty(wallStatusByVisit)
            planningWallStatus = wallStatusByVisit;
        end
        [ecPlanningStartState, postOptionalWallPlanningStartInfo] = ...
            selectPostOptionalWallPlanningState( ...
                currentState, planningWallStatus, postNormalSafeState, ecGoals, verifiedMap, plannerParams);
        [ecGoals, ecGoalVisitOrder] = orderNearestPoints( ...
            ecPlanningStartState.pose(1:2).', ecGoals);
        [ecPlannedPath, ecPlannerInfo] = planWaypointSequenceKnownMap( ...
            ecPlanningStartState.pose(1:2).', ecGoals, verifiedMap, plannerParams);
        ecPlannerInfo.visitOrder = ecGoalVisitOrder;
        [ecPlannedPath, ecPlannerInfo] = prependBridgePathToEcPlan( ...
            currentState.pose(1:2).', ecPlanningStartState.pose(1:2).', ...
            ecPlannedPath, ecPlannerInfo, verifiedMap, plannerParams);
        if ~ecPlannerInfo.bridgeSuccess
            [currentState, ecPlanningStartState, postOptionalWallPlanningStartInfo, ...
                ecPlannerInfo, ecPlannedPath, dataStore] = runPostOptionalWallEscapeAndReplan( ...
                    Robot, currentState, planningWallStatus, postNormalSafeState, ecGoals, ...
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

    dataStore.normalGoals = normalGoals;
    dataStore.normalGoalVisitOrder = normalGoalVisitOrder;
    dataStore.normalPlanningMap = normalPlanningMap;
    dataStore.normalPlannedPath = normalPlannedPath;
    dataStore.normalPlannerInfo = normalPlannerInfo;
    dataStore.normalSnapInfo = normalSnapInfo;
    dataStore.normalEkfState = normalEkfState;
    dataStore.normalNavState = normalNavState;
    dataStore.postNormalSafeState = postNormalSafeState;
    dataStore.wallStatus = wallStatus;
    dataStore.wallStatusByVisit = wallStatusByVisit;
    dataStore.optionalWallVisitOrder = optionalWallVisitOrder;
    dataStore.verifiedMap = verifiedMap;
    dataStore.ecGoals = ecGoals;
    dataStore.ecGoalVisitOrder = ecGoalVisitOrder;
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

function [orderedPoints, visitOrder] = orderNearestPoints(startXY, points)
if isempty(points)
    orderedPoints = points;
    visitOrder = [];
    return;
end
currentXY = startXY(:).';
remainingIdx = (1:size(points, 1)).';
visitOrder = zeros(size(points, 1), 1);
orderedPoints = zeros(size(points));
for k = 1:size(points, 1)
    deltas = points(remainingIdx, :) - currentXY;
    [~, localIdx] = min(sum(deltas .^ 2, 2));
    chosenIdx = remainingIdx(localIdx);
    visitOrder(k) = chosenIdx;
    orderedPoints(k, :) = points(chosenIdx, :);
    currentXY = points(chosenIdx, :);
    remainingIdx(localIdx) = [];
end
end

function wallIdx = nearestOptionalWallIndex(startXY, optWalls, candidateIdx)
if isempty(candidateIdx)
    wallIdx = [];
    return;
end
midpoints = 0.5 * (optWalls(candidateIdx, 1:2) + optWalls(candidateIdx, 3:4));
deltas = midpoints - startXY(:).';
[~, localIdx] = min(sum(deltas .^ 2, 2));
wallIdx = candidateIdx(localIdx);
end

function summary = copyFinalWallStatusSummary(summary, fullStatus, wallIdx, wall)
summary.wallIdx = wallIdx;
summary.wall = wall;
if isfield(fullStatus, 'status')
    summary.status = fullStatus.status;
end
if isfield(fullStatus, 'confidence')
    summary.confidence = fullStatus.confidence;
end
if isfield(fullStatus, 'reason')
    summary.reason = fullStatus.reason;
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

function [localStatus, dataStore, verifiedMap] = verifyOptionalWallWithRelaxation( ...
    Robot, mapIn, wall, wallIdx, beaconLoc, offset_x, offset_y, currentState, ...
    ekfVerifyParams, plannerParams, baseVerifyParams, dataStore)
verifiedMap = mapIn;
localStatus = repmat(emptyFinalWallStatus(), 0, 1);
retryLog = repmat(emptyOptionalWallRetryLog(), 0, 1);

for attempt = 1:numOptionalWallRelaxationLevels()
    [relaxedPlannerParams, relaxedVerifyParams] = makeOptionalWallRelaxedParams( ...
        plannerParams, baseVerifyParams, wallIdx, attempt);
    [candidateStatus, dataStore, candidateMap] = verifyOptionalWalls( ...
        Robot, mapIn, wall, beaconLoc, offset_x, offset_y, currentState, ...
        ekfVerifyParams, relaxedPlannerParams, relaxedVerifyParams, dataStore);

    if isempty(candidateStatus)
        candidateStatus = emptyFinalWallStatus();
        candidateStatus.wallIdx = wallIdx;
        candidateStatus.wall = wall;
        candidateStatus.reason = 'emptyLocalStatus';
    end
    candidateStatus.wallIdx = wallIdx;
    candidateStatus.wall = wall;
    candidateStatus.relaxationAttempt = attempt;
    candidateStatus.relaxationInfo = describeOptionalWallRelaxation( ...
        relaxedPlannerParams, relaxedVerifyParams);

    retryEntry = emptyOptionalWallRetryLog();
    retryEntry.attempt = attempt;
    retryEntry.status = candidateStatus.status;
    retryEntry.reason = candidateStatus.reason;
    retryEntry.relaxationInfo = candidateStatus.relaxationInfo;
    retryLog(end + 1, 1) = retryEntry; %#ok<AGROW>

    fprintf('[finalCompetition] optional wall %d retry %d status=%s reason=%s clearance=%.2f probeClearance=%.2f\n', ...
        wallIdx, attempt, candidateStatus.status, candidateStatus.reason, ...
        relaxedVerifyParams.observeClearance, relaxedVerifyParams.bumpFallbackClearance);

    localStatus = candidateStatus;
    verifiedMap = candidateMap;
    if ~shouldRelaxOptionalWallSearch(candidateStatus)
        break;
    end
end

localStatus.relaxationRetries = retryLog;
end

function tf = shouldRelaxOptionalWallSearch(status)
tf = strcmp(status.status, 'unknown') && ...
    (contains(status.reason, 'noReachableObservePoint') || ...
     contains(status.reason, 'noReachableProbeStart') || ...
     contains(status.reason, 'planFailed_noPath') || ...
     contains(status.reason, 'planFailed_noGraphPath') || ...
     contains(status.reason, 'bumpFallbackPlanFailed'));
end

function n = numOptionalWallRelaxationLevels()
n = 2;
end

function [plannerOut, verifyOut] = makeOptionalWallRelaxedParams( ...
    plannerIn, verifyIn, wallIdx, attempt)
plannerOut = plannerIn;
verifyOut = verifyIn;
verifyOut.wallIdxLabels = wallIdx;

attempt = max(1, min(numOptionalWallRelaxationLevels(), attempt));
observeClearance = [0.35 0.28 0.22 0.16 0.10];
probeClearance = [0.28 0.22 0.16 0.12 0.08];
navClearance = [0.42 0.34 0.28 0.22 0.16];
cornerRadius = [0.54 0.44 0.34 0.27 0.22];
gridSpacing = [0.45 0.35 0.28 0.22 0.18];

verifyOut.observeClearance = min(verifyOut.observeClearance, observeClearance(attempt));
verifyOut.bumpFallbackClearance = min(verifyOut.bumpFallbackClearance, probeClearance(attempt));
verifyOut.navigationEdgeClearance = navClearance(attempt);
verifyOut.navigationNodeClearance = navClearance(attempt);
verifyOut.navigationCornerOffsetRadius = cornerRadius(attempt);
verifyOut.navigationStartGoalClearance = min(0.08, max(0.02, 0.08 - 0.015 * (attempt - 1)));
verifyOut.observeDistances = unique([verifyOut.observeDistances(:).' ...
    0.35 0.45 0.60 0.75 0.90 1.10]);
verifyOut.observeTangentOffsets = unique([verifyOut.observeTangentOffsets(:).' ...
    -0.55 -0.40 -0.20 0 0.20 0.40 0.55]);
verifyOut.bumpFallbackDistances = unique([verifyOut.bumpFallbackDistances(:).' ...
    0.35 0.42 0.55 0.70 0.90]);
verifyOut.bumpFallbackTangentOffsets = unique([verifyOut.bumpFallbackTangentOffsets(:).' ...
    -0.55 -0.45 -0.25 0 0.25 0.45 0.55]);
verifyOut.followerMaxRunTime = min(verifyOut.followerMaxRunTime, 80);
verifyOut.bumpFallbackMaxRunTime = min(verifyOut.bumpFallbackMaxRunTime, 55);

plannerOut.edgeClearance = navClearance(attempt);
plannerOut.nodeClearance = navClearance(attempt);
plannerOut.cornerOffsetRadius = cornerRadius(attempt);
plannerOut.startGoalClearance = verifyOut.navigationStartGoalClearance;
plannerOut.gridSpacing = gridSpacing(attempt);
end

function info = describeOptionalWallRelaxation(plannerParams, verifyParams)
info = struct( ...
    'edgeClearance', plannerParams.edgeClearance, ...
    'nodeClearance', plannerParams.nodeClearance, ...
    'cornerOffsetRadius', plannerParams.cornerOffsetRadius, ...
    'gridSpacing', plannerParams.gridSpacing, ...
    'observeClearance', verifyParams.observeClearance, ...
    'bumpFallbackClearance', verifyParams.bumpFallbackClearance);
end

function entry = emptyOptionalWallRetryLog()
entry = struct('attempt', NaN, 'status', '', 'reason', '', 'relaxationInfo', struct());
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
candidates{end + 1} = struct('source', 'postNormalSafeState', 'state', postNormalSafeState);
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
    [orderedEcGoals, ecGoalVisitOrder] = orderNearestPoints( ...
        ecPlanningStartState.pose(1:2).', ecGoals);
    [ecTailPath, ecPlannerInfo] = planWaypointSequenceKnownMap( ...
        ecPlanningStartState.pose(1:2).', orderedEcGoals, verifiedMap, plannerParams);
    ecPlannerInfo.visitOrder = ecGoalVisitOrder;
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
