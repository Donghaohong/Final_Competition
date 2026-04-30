function results = runPracticeMapInitHeadlessSim(runIdx, rngSeed, initOverride)
% runPracticeMapInitHeadlessSim Dynamic command-line simulator smoke test.
%
% This uses a MATLAB timer to advance the CreateRobot state while
% initializeLocalization is paused between sensor reads. It avoids the GUI
% file picker but exercises real sweep motion and odometry increments.

codeDir = fileparts(mfilename('fullpath'));
toolboxDir = '/Users/donghaohong/Documents/iRobotCreateSimulatorToolbox-master';
addpath(codeDir);
addpath(toolboxDir);

S = load(fullfile(codeDir, 'PracticeMap2026.mat'), ...
    'map', 'waypoints', 'beaconLoc');

if nargin < 1 || isempty(runIdx)
    runIdx = 1:size(S.waypoints, 1);
end
if nargin >= 2 && ~isempty(rngSeed)
    rng(rngSeed);
end

beacs = beaconLocToSimulatorBeacs(S.beaconLoc);
theta0 = 1.1;
numRuns = numel(runIdx);
results = zeros(numRuns, 10);

for runNum = 1:numRuns
    k = runIdx(runNum);
    sim = CreateRobot();
    setMap(sim, S.map, [], beacs, []);
    setMapStart(sim, [S.waypoints(k, :) theta0]);
    setAutoEnable(sim, true);
    startTimeElap(sim);
    updateOutput(sim);

    initParams = initDefaultParams();
    initParams.maxInitTime = 22.0;
    initParams.minInitTime = 8.0;
    initParams.controlDt = 0.10;
    initParams.maxDepthRange = 10.0;
    initParams.minDepthRange = 0.175;
    initParams.debugPrint = false;

    if nargin >= 3 && ~isempty(initOverride)
        fn = fieldnames(initOverride);
        for i = 1:numel(fn)
            initParams.(fn{i}) = initOverride.(fn{i});
        end
    end

    timerSim = startHeadlessSimTimer(sim, 0.05);
    cleanupObj = onCleanup(@() stopAndDeleteTimer(timerSim)); %#ok<NASGU>

    [initState, dataStore] = initializeLocalization( ...
        sim, S.map, S.waypoints, S.beaconLoc, 0.0, 0.08, initParams, struct());

    stopAndDeleteTimer(timerSim);
    clear cleanupObj;

    finalTruth = getState(sim);
    posErr = norm(initState.pose(1:2).' - finalTruth(1:2));
    headingErr = abs(wrapToPiLocal(initState.pose(3) - finalTruth(3)));
    totalOdomTurn = sum(abs(dataStore.odometry(:, 3)));

    results(runNum, :) = [ ...
        k, ...
        initState.startWaypointIdx, ...
        initState.confidence, ...
        double(initState.converged), ...
        posErr, ...
        headingErr, ...
        initState.pose(:).', ...
        totalOdomTurn];
end

disp('cols: trueWP estWP confidence converged finalPosErr finalHeadingErr estX estY estTheta totalAbsOdomTurn');
disp(results);
end

function timerSim = startHeadlessSimTimer(sim, dt)
timerSim = timer( ...
    'ExecutionMode', 'fixedSpacing', ...
    'Period', dt, ...
    'BusyMode', 'drop', ...
    'TimerFcn', @(~, ~) stepHeadlessSim(sim, dt));
start(timerSim);
end

function stepHeadlessSim(sim, dt)
oldState = getState(sim);
driveNormal(sim, dt);
newState = getState(sim);
updateOdom(sim, oldState, newState);
updateOutput(sim);
end

function stopAndDeleteTimer(timerSim)
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

function beacs = beaconLocToSimulatorBeacs(beaconLoc)
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
