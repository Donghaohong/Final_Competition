function dataStore = testIntegratedWaypointOptionalPracticeMap(Robot, goalWaypoints)
% testIntegratedWaypointOptionalPracticeMap Init + waypoint following + optional walls.
%
% SimulatorGUI Autonomous Start usage:
%   testIntegratedWaypointOptionalPracticeMap
%
% Optional command-line style usage:
%   dataStore = testIntegratedWaypointOptionalPracticeMap(Robot, [x1 y1; x2 y2])

codeDir = fileparts(mfilename('fullpath'));
S = load(fullfile(codeDir, 'PracticeMap2026.mat'), ...
    'map', 'optWalls', 'waypoints', 'beaconLoc');

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
    highLevelGoals = defaultPracticeGoals(S.waypoints, initState.startWaypointIdx);
else
    highLevelGoals = goalWaypoints;
end

plannerParams = knownMapPlannerDefaultParams();
ekfParams = ekfWaypointDefaultParams();
ekfParams.maxRunTime = 500.0;
ekfParams.maxDepthRange = 10.0;
ekfParams.minDepthRange = 0.175;
ekfParams.debugPrint = true;
ekfParams.enableLivePlot = true;

verifyParams = optionalWallDefaultParams();
verifyParams.maxDepthRange = 10.0;
verifyParams.minDepthRange = 0.175;
verifyParams.debugPrint = true;

strategyParams = integratedStrategyDefaultParams();

currentState = initState;
verifiedMap = S.map;
wallStatus = initializeIntegratedWallStatus(S.optWalls);
plannedSegments = repmat(emptyIntegratedSegment(), 0, 1);
ekfState = struct();
navState = struct();
stopReason = 'completed';

for goalIdx = 1:size(highLevelGoals, 1)
    targetGoal = highLevelGoals(goalIdx, :);
    segmentDone = false;
    replanCount = 0;

    while ~segmentDone
        [pathToGoal, planInfo] = planPathKnownMap( ...
            currentState.pose(1:2).', targetGoal, verifiedMap, plannerParams);

        segment = emptyIntegratedSegment();
        segment.goalIdx = goalIdx;
        segment.goal = targetGoal;
        segment.startPose = currentState.pose;
        segment.verifiedMapSize = size(verifiedMap, 1);
        segment.path = pathToGoal;
        segment.planInfo = planInfo;
        segment.replanCount = replanCount;

        if isempty(pathToGoal) || ~planInfo.success
            stopRobotSafe(Robot);
            segment.stopReason = ['planFailed_' planInfo.reason];
            plannedSegments(end + 1, 1) = segment; %#ok<AGROW>
            stopReason = segment.stopReason;
            break;
        end

        [selectedWalls, selectInfo] = selectRelevantOptionalWalls( ...
            pathToGoal, S.optWalls, wallStatus, strategyParams);
        segment.relevantWallIdx = selectedWalls;
        segment.relevantWallInfo = selectInfo;

        if ~isempty(selectedWalls) && replanCount < strategyParams.maxReplansPerGoal
            [wallStatus, verifiedMap, currentState, dataStore, verifyInfo] = ...
                verifySelectedOptionalWalls( ...
                    Robot, verifiedMap, S.optWalls, selectedWalls, wallStatus, ...
                    S.beaconLoc, offset_x, offset_y, currentState, ekfParams, ...
                    plannerParams, verifyParams, dataStore);
            segment.verifyInfo = verifyInfo;
            segment.stopReason = 'verifiedRelevantWall';
            plannedSegments(end + 1, 1) = segment; %#ok<AGROW>
            replanCount = replanCount + 1;
            continue;
        end

        followerParams = ekfParams;
        followerParams.beepMask = false(size(pathToGoal, 1), 1);
        followerParams.beepMask(end) = true;
        [ekfState, dataStore, navState] = runEkfWaypointFollower( ...
            Robot, verifiedMap, S.beaconLoc, pathToGoal, offset_x, offset_y, ...
            currentState, followerParams, dataStore);

        currentState.pose = ekfState.pose;
        currentState.poseCov = ekfState.poseCov;
        segment.ekfState = ekfState;
        segment.navState = navState;
        segment.endPose = currentState.pose;

        if ~ekfState.reachedAllGoals
            stopReason = ['navigationFailed_' ekfState.stopReason];
            segment.stopReason = stopReason;
            plannedSegments(end + 1, 1) = segment; %#ok<AGROW>
            break;
        end

        segment.stopReason = 'reachedGoal';
        plannedSegments(end + 1, 1) = segment; %#ok<AGROW>
        segmentDone = true;
    end

    if startsWith(stopReason, 'planFailed_') || startsWith(stopReason, 'navigationFailed_')
        break;
    end
end

if strcmp(stopReason, 'completed') && strategyParams.verifyRemainingWallsAfterWaypoints
    remainingWalls = find(strcmp({wallStatus.status}.', 'unknown'));
    if ~isempty(remainingWalls)
        postVerifyStartPose = currentState.pose;
        [wallStatus, verifiedMap, currentState, dataStore, postVerifyInfo] = ...
            verifySelectedOptionalWalls( ...
                Robot, verifiedMap, S.optWalls, remainingWalls, wallStatus, ...
                S.beaconLoc, offset_x, offset_y, currentState, ekfParams, ...
                plannerParams, verifyParams, dataStore);
        segment = emptyIntegratedSegment();
        segment.goalIdx = NaN;
        segment.goal = [NaN NaN];
        segment.startPose = postVerifyStartPose;
        segment.endPose = currentState.pose;
        segment.verifiedMapSize = size(verifiedMap, 1);
        segment.relevantWallIdx = remainingWalls(:).';
        segment.verifyInfo = postVerifyInfo;
        segment.stopReason = 'postWaypointOptionalVerification';
        plannedSegments(end + 1, 1) = segment;
        stopReason = 'completedWaypointsAndOptionalWalls';
    end
end

dataStore.initState = initState;
dataStore.ekfState = ekfState;
dataStore.navState = navState;
dataStore.highLevelGoals = highLevelGoals;
dataStore.wallStatus = wallStatus;
dataStore.verifiedMap = verifiedMap;
dataStore.plannedSegments = plannedSegments;
dataStore.integratedStopReason = stopReason;

save(fullfile(codeDir, 'latestIntegratedPracticeRun.mat'), ...
    'initState', 'ekfState', 'navState', 'wallStatus', 'verifiedMap', ...
    'dataStore', 'S', 'highLevelGoals', 'plannedSegments', 'stopReason');

disp('Integrated waypoint + optional wall result:');
fprintf('  stopReason: %s\n', stopReason);
disp('  wallStatus:');
for i = 1:numel(wallStatus)
    fprintf('    optWall %d: status=%s confidence=%.2f attempts=%d reason=%s\n', ...
        i, wallStatus(i).status, wallStatus(i).confidence, ...
        wallStatus(i).attempts, wallStatus(i).reason);
end
end

function params = integratedStrategyDefaultParams()
params = struct();
params.optionalWallPathDistance = 0.35;
params.maxWallsPerSegment = 1;
params.maxVerifyAttemptsPerWall = 1;
params.maxReplansPerGoal = 3;
params.verifyRemainingWallsAfterWaypoints = true;
end

function goals = defaultPracticeGoals(waypoints, startWaypointIdx)
goals = waypoints;
if isfinite(startWaypointIdx) && startWaypointIdx >= 1 && startWaypointIdx <= size(goals, 1)
    goals(startWaypointIdx, :) = [];
end
end

function wallStatus = initializeIntegratedWallStatus(optWalls)
wallStatus = repmat(emptyIntegratedWallStatus(), size(optWalls, 1), 1);
for i = 1:size(optWalls, 1)
    wallStatus(i).wallIdx = i;
    wallStatus(i).wall = optWalls(i, :);
end
end

function status = emptyIntegratedWallStatus()
status = struct( ...
    'wallIdx', NaN, ...
    'wall', NaN(1, 4), ...
    'status', 'unknown', ...
    'confidence', 0, ...
    'reason', '', ...
    'attempts', 0, ...
    'lastLocalStatus', struct(), ...
    'lastVerifiedMapSize', NaN);
end

function segment = emptyIntegratedSegment()
segment = struct( ...
    'goalIdx', NaN, ...
    'goal', NaN(1, 2), ...
    'startPose', [], ...
    'endPose', [], ...
    'verifiedMapSize', NaN, ...
    'path', [], ...
    'planInfo', struct(), ...
    'relevantWallIdx', [], ...
    'relevantWallInfo', struct(), ...
    'verifyInfo', struct(), ...
    'ekfState', struct(), ...
    'navState', struct(), ...
    'replanCount', 0, ...
    'stopReason', '');
end

function [selectedWalls, info] = selectRelevantOptionalWalls(path, optWalls, wallStatus, params)
selectedWalls = [];
info = struct('scores', [], 'distances', [], 'intersects', [], 'candidates', []);
if size(path, 1) < 2 || isempty(optWalls)
    return;
end

numWalls = size(optWalls, 1);
scores = inf(numWalls, 1);
distances = inf(numWalls, 1);
intersects = false(numWalls, 1);
candidates = false(numWalls, 1);

for i = 1:numWalls
    if ~strcmp(wallStatus(i).status, 'unknown') || wallStatus(i).attempts >= params.maxVerifyAttemptsPerWall
        continue;
    end

    wall = optWalls(i, :);
    minDist = inf;
    doesIntersect = false;
    for k = 1:size(path, 1)-1
        a = path(k, :);
        b = path(k + 1, :);
        c = wall(1:2);
        d = wall(3:4);
        segDist = segmentSegmentDistanceLocal(a, b, c, d);
        minDist = min(minDist, segDist);
        doesIntersect = doesIntersect || segmentsIntersectLocal(a, b, c, d);
    end

    distances(i) = minDist;
    intersects(i) = doesIntersect;
    if doesIntersect || minDist <= params.optionalWallPathDistance
        candidates(i) = true;
        scores(i) = minDist - 10 * double(doesIntersect);
    end
end

candidateIdx = find(candidates);
if isempty(candidateIdx)
    info.scores = scores;
    info.distances = distances;
    info.intersects = intersects;
    info.candidates = candidates;
    return;
end

[~, order] = sort(scores(candidateIdx), 'ascend');
selectedWalls = candidateIdx(order(1:min(params.maxWallsPerSegment, numel(order))));
info.scores = scores;
info.distances = distances;
info.intersects = intersects;
info.candidates = candidates;
end

function [wallStatus, verifiedMap, currentState, dataStore, verifyInfo] = ...
    verifySelectedOptionalWalls( ...
        Robot, verifiedMap, optWalls, selectedWalls, wallStatus, beaconLoc, ...
        offset_x, offset_y, currentState, ekfParams, plannerParams, verifyParams, dataStore)
verifyInfo = struct('selectedWalls', selectedWalls, 'localStatus', []);

for i = 1:numel(selectedWalls)
    globalIdx = selectedWalls(i);
    wallStatus(globalIdx).attempts = wallStatus(globalIdx).attempts + 1;
    localVerifyParams = verifyParams;
    localVerifyParams.wallIdxLabels = globalIdx;

    [localStatus, dataStore, verifiedMapCandidate] = verifyOptionalWalls( ...
        Robot, verifiedMap, optWalls(globalIdx, :), beaconLoc, offset_x, offset_y, ...
        currentState, ekfParams, plannerParams, localVerifyParams, dataStore);

    if isempty(localStatus)
        wallStatus(globalIdx).status = 'unknown';
        wallStatus(globalIdx).reason = 'noLocalStatus';
        continue;
    end

    localStatus.wallIdx = globalIdx;
    localStatus.wall = optWalls(globalIdx, :);
    wallStatus(globalIdx).status = localStatus.status;
    wallStatus(globalIdx).confidence = localStatus.confidence;
    wallStatus(globalIdx).reason = localStatus.reason;
    wallStatus(globalIdx).lastLocalStatus = localStatus;
    wallStatus(globalIdx).lastVerifiedMapSize = size(verifiedMapCandidate, 1);
    verifyInfo.localStatus = [verifyInfo.localStatus; localStatus];

    if strcmp(localStatus.status, 'exists')
        verifiedMap = verifiedMapCandidate;
    end

    currentState = updateCurrentStateFromWallStatus(currentState, localStatus);
end
end

function currentState = updateCurrentStateFromWallStatus(currentState, status)
if isfield(status, 'bumpProbe') && isstruct(status.bumpProbe) && ...
        isfield(status.bumpProbe, 'endPose') && numel(status.bumpProbe.endPose) == 3
    currentState.pose = status.bumpProbe.endPose(:);
    if isfield(status, 'ekfCovAtClassify') && isequal(size(status.ekfCovAtClassify), [3 3])
        currentState.poseCov = status.ekfCovAtClassify;
    end
elseif isfield(status, 'ekfPoseAtClassify') && numel(status.ekfPoseAtClassify) == 3
    currentState.pose = status.ekfPoseAtClassify(:);
    currentState.poseCov = status.ekfCovAtClassify;
elseif isfield(status, 'ekfPoseAtObserve') && numel(status.ekfPoseAtObserve) == 3
    currentState.pose = status.ekfPoseAtObserve(:);
    currentState.poseCov = status.ekfCovAtObserve;
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

function tf = segmentsIntersectLocal(a, b, c, d)
tol = 1e-10;
o1 = orientLocal(a, b, c);
o2 = orientLocal(a, b, d);
o3 = orientLocal(c, d, a);
o4 = orientLocal(c, d, b);

if (o1 * o2 < -tol) && (o3 * o4 < -tol)
    tf = true;
    return;
end

tf = (abs(o1) <= tol && pointOnSegmentLocal(c, a, b, tol)) || ...
     (abs(o2) <= tol && pointOnSegmentLocal(d, a, b, tol)) || ...
     (abs(o3) <= tol && pointOnSegmentLocal(a, c, d, tol)) || ...
     (abs(o4) <= tol && pointOnSegmentLocal(b, c, d, tol));
end

function val = orientLocal(a, b, c)
val = (b(1) - a(1)) * (c(2) - a(2)) - ...
      (b(2) - a(2)) * (c(1) - a(1));
end

function tf = pointOnSegmentLocal(p, a, b, tol)
tf = p(1) >= min(a(1), b(1)) - tol && p(1) <= max(a(1), b(1)) + tol && ...
     p(2) >= min(a(2), b(2)) - tol && p(2) <= max(a(2), b(2)) + tol;
end
