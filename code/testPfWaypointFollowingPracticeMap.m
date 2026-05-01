function dataStore = testPfWaypointFollowingPracticeMap(Robot, goalWaypoints)
% testPfWaypointFollowingPracticeMap SimulatorGUI entry for init + PF follower.
%
% Usage from SimulatorGUI Autonomous Start:
%   testPfWaypointFollowingPracticeMap
%
% Optional command-line style usage:
%   dataStore = testPfWaypointFollowingPracticeMap(Robot, [x1 y1; x2 y2])

codeDir = fileparts(mfilename('fullpath'));
S = load(fullfile(codeDir, 'PracticeMap2026.mat'), ...
    'map', 'waypoints', 'beaconLoc');

initParams = initDefaultParams();
initParams.maxInitTime = 25.0;
initParams.minInitTime = 8.0;
initParams.maxDepthRange = 10.0;
initParams.minDepthRange = 0.175;
initParams.debugPrint = true;

% Simulator CreateRobot uses RealSense camera offset [0, 0.08].
offset_x = 0.0;
offset_y = 0.08;

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
    save(fullfile(codeDir, 'latestPfWaypointPracticeRun.mat'), ...
        'initState', 'dataStore', 'S', 'highLevelGoals', 'plannedPath', 'plannerInfo');
    error('Known-map path planner failed: %s', plannerInfo.reason);
end

pfGoalWaypoints = plannedPath;

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
    Robot, S.map, S.beaconLoc, pfGoalWaypoints, offset_x, offset_y, ...
    initState, pfParams, dataStore);

dataStore.initState = initState;
dataStore.pfState = pfState;
dataStore.navState = navState;
dataStore.highLevelGoals = highLevelGoals;
dataStore.plannedPath = plannedPath;
dataStore.plannerInfo = plannerInfo;

save(fullfile(codeDir, 'latestPfWaypointPracticeRun.mat'), ...
    'initState', 'pfState', 'navState', 'dataStore', 'S', ...
    'pfGoalWaypoints', 'highLevelGoals', 'plannedPath', 'plannerInfo');

disp('Initialization result:');
disp(initState);
disp('Known-map planned path:');
disp(plannedPath);
disp('PF waypoint follower result:');
disp(pfState);
disp('Navigation result:');
disp(navState);
end

function goals = defaultPracticeGoals(waypoints, startWaypointIdx)
goals = waypoints;
if isfinite(startWaypointIdx) && startWaypointIdx >= 1 && startWaypointIdx <= size(goals, 1)
    goals(startWaypointIdx, :) = [];
end
end
