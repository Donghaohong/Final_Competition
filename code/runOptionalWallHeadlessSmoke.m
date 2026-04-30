function result = runOptionalWallHeadlessSmoke(startWaypointIdx, optWallIdx, includeOptionalWalls)
% runOptionalWallHeadlessSmoke Headless simulator smoke test for optional-wall verification.
%
% includeOptionalWalls can be:
%   true: include all optional walls in the simulated physical map
%   false: include no optional walls
%   numeric indices: include only those optional walls physically
% The verification code still receives known map + candidate optWalls separately.

codeDir = fileparts(mfilename('fullpath'));
toolboxDir = '/Users/donghaohong/Documents/iRobotCreateSimulatorToolbox-master';
addpath(codeDir);
addpath(toolboxDir);

S = load(fullfile(codeDir, 'PracticeMap2026.mat'), ...
    'map', 'optWalls', 'waypoints', 'beaconLoc');

if nargin < 1 || isempty(startWaypointIdx)
    startWaypointIdx = 1;
end
if nargin < 2 || isempty(optWallIdx)
    optWallIdx = 1;
end
if nargin < 3 || isempty(includeOptionalWalls)
    includeOptionalWalls = true;
end

sim = CreateRobot();
if isnumeric(includeOptionalWalls)
    physicalOptIdx = includeOptionalWalls(:);
    physicalOptIdx = physicalOptIdx(physicalOptIdx >= 1 & physicalOptIdx <= size(S.optWalls, 1));
    physicalMap = [S.map; S.optWalls(physicalOptIdx, :)];
elseif includeOptionalWalls
    physicalMap = [S.map; S.optWalls];
else
    physicalMap = S.map;
end
setMap(sim, physicalMap, [], beaconLocToSimulatorBeacsLocal(S.beaconLoc), []);
setMapStart(sim, [S.waypoints(startWaypointIdx, :) 1.0]);
setAutoEnable(sim, true);
startTimeElap(sim);
updateOutput(sim);

timerSim = startHeadlessSimTimerLocal(sim, 0.05);
cleanupObj = onCleanup(@() stopAndDeleteTimerLocal(timerSim));

initParams = initDefaultParams();
initParams.maxInitTime = 12.0;
initParams.minInitTime = 4.0;
initParams.minSweepTurnForConvergence = pi;
initParams.maxSweeps = 0.75;
initParams.controlDt = 0.10;
initParams.maxDepthRange = 10.0;
initParams.minDepthRange = 0.175;
initParams.debugPrint = true;

[initState, dataStore] = initializeLocalization( ...
    sim, S.map, S.waypoints, S.beaconLoc, 0.0, 0.08, initParams, struct());

ekfParams = ekfWaypointDefaultParams();
ekfParams.maxDepthRange = 10.0;
ekfParams.minDepthRange = 0.175;
ekfParams.debugPrint = true;

plannerParams = knownMapPlannerDefaultParams();

verifyParams = optionalWallDefaultParams();
verifyParams.maxDepthRange = 10.0;
verifyParams.minDepthRange = 0.175;
verifyParams.followerMaxRunTime = 140.0;
verifyParams.debugPrint = true;

[wallStatus, dataStore, verifiedMap] = verifyOptionalWalls( ...
    sim, S.map, S.optWalls(optWallIdx, :), S.beaconLoc, 0.0, 0.08, ...
    initState, ekfParams, plannerParams, verifyParams, dataStore);

stopAndDeleteTimerLocal(timerSim);
clear cleanupObj;
stopRobotSafe(sim);

result = struct();
result.initState = initState;
result.wallStatus = wallStatus;
result.verifiedMap = verifiedMap;
result.dataStore = dataStore;
result.truthPose = getState(sim);
result.startWaypointIdx = startWaypointIdx;
result.optWallIdx = optWallIdx;
result.includeOptionalWalls = includeOptionalWalls;

save(fullfile(codeDir, 'latestOptionalWallHeadlessSmoke.mat'), 'result', 'S');

disp('Optional-wall headless smoke result:');
disp(wallStatus);
end

function timerSim = startHeadlessSimTimerLocal(sim, dt)
timerSim = timer( ...
    'ExecutionMode', 'fixedSpacing', ...
    'Period', dt, ...
    'BusyMode', 'drop', ...
    'TimerFcn', @(~, ~) stepHeadlessSimLocal(sim, dt));
start(timerSim);
end

function stepHeadlessSimLocal(sim, dt)
oldState = getState(sim);
driveNormal(sim, dt);
newState = getState(sim);
updateOdom(sim, oldState, newState);
updateOutput(sim);
end

function stopAndDeleteTimerLocal(timerSim)
if isempty(timerSim) || ~isvalid(timerSim)
    return;
end
try
    stop(timerSim);
catch
end
try
    delete(timerSim);
catch
end
end

function beacs = beaconLocToSimulatorBeacsLocal(beaconLoc)
colors = [ ...
    1 0 0; ...
    0 1 0; ...
    0 0 1; ...
    1 1 0; ...
    1 0 1; ...
    0 1 1; ...
    0.5 0.5 0.5; ...
    1 0.5 0];

beacs = cell(size(beaconLoc, 1), 6);
for i = 1:size(beaconLoc, 1)
    c = colors(mod(i - 1, size(colors, 1)) + 1, :);
    beacs{i, 1} = beaconLoc(i, 2);
    beacs{i, 2} = beaconLoc(i, 3);
    beacs{i, 3} = c(1);
    beacs{i, 4} = c(2);
    beacs{i, 5} = c(3);
    beacs{i, 6} = num2str(beaconLoc(i, 1));
end
end
