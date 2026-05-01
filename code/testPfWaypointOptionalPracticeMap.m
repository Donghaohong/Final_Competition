function dataStore = testPfWaypointOptionalPracticeMap(Robot, goalWaypoints)
% testPfWaypointOptionalPracticeMap Init + PF waypoint following + optional walls.
%
% SimulatorGUI Autonomous Start usage:
%   testPfWaypointOptionalPracticeMap
%
% Optional command-line style usage:
%   dataStore = testPfWaypointOptionalPracticeMap(Robot, [x1 y1; x2 y2])

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
[plannedPath, plannerInfo] = planWaypointSequenceKnownMap( ...
    initState.pose(1:2).', highLevelGoals, S.map, plannerParams);

if isempty(plannedPath) || ~plannerInfo.success
    stopRobotSafe(Robot);
    dataStore.highLevelGoals = highLevelGoals;
    dataStore.plannedPath = plannedPath;
    dataStore.plannerInfo = plannerInfo;
    save(fullfile(codeDir, 'latestPfWaypointOptionalPracticeRun.mat'), ...
        'initState', 'dataStore', 'S', 'highLevelGoals', 'plannedPath', 'plannerInfo');
    error('Known-map path planner failed: %s', plannerInfo.reason);
end

pfParams = pfWaypointDefaultParams();
pfParams.maxRunTime = 500.0;
pfParams.maxDepthRange = 10.0;
pfParams.minDepthRange = 0.175;
pfParams.debugPrint = true;
pfParams.enableLivePlot = true;
if isfield(plannerInfo, 'beepMask')
    pfParams.beepMask = plannerInfo.beepMask;
end

[pfState, dataStore, navState] = runPfWaypointFollower( ...
    Robot, S.map, S.beaconLoc, plannedPath, offset_x, offset_y, ...
    initState, pfParams, dataStore);

currentState = initState;
currentState.pose = pfState.pose;
currentState.poseCov = pfState.poseCov;
currentState.particles = pfState.particles;
currentState.weights = pfState.weights;

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

dataStore.initState = initState;
dataStore.pfState = pfState;
dataStore.navState = navState;
dataStore.highLevelGoals = highLevelGoals;
dataStore.plannedPath = plannedPath;
dataStore.plannerInfo = plannerInfo;
dataStore.wallStatus = wallStatus;
dataStore.verifiedMap = verifiedMap;

save(fullfile(codeDir, 'latestPfWaypointOptionalPracticeRun.mat'), ...
    'initState', 'pfState', 'navState', 'wallStatus', 'verifiedMap', ...
    'dataStore', 'S', 'highLevelGoals', 'plannedPath', 'plannerInfo');

disp('Initialization result:');
disp(initState);
disp('Known-map planned path:');
disp(plannedPath);
disp('PF waypoint follower result:');
disp(pfState);
disp('Navigation result:');
disp(navState);
disp('PF optional wall verification result:');
for i = 1:numel(wallStatus)
    fprintf('  optWall %d: status=%s confidence=%.2f reason=%s\n', ...
        i, wallStatus(i).status, wallStatus(i).confidence, wallStatus(i).reason);
end
end

function goals = defaultPracticeGoals(waypoints, startWaypointIdx)
goals = waypoints;
if isfinite(startWaypointIdx) && startWaypointIdx >= 1 && startWaypointIdx <= size(goals, 1)
    goals(startWaypointIdx, :) = [];
end
end
