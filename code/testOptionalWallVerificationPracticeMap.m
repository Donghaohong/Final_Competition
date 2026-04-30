function dataStore = testOptionalWallVerificationPracticeMap(Robot)
% testOptionalWallVerificationPracticeMap SimulatorGUI entry for optional walls.
%
% Runs initialization, then actively verifies each optional wall in
% PracticeMap2026 using EKF navigation and depth classification.

codeDir = fileparts(mfilename('fullpath'));
S = load(fullfile(codeDir, 'PracticeMap2026.mat'), ...
    'map', 'optWalls', 'waypoints', 'beaconLoc');

offset_x = 0.0;
offset_y = 0.08;

initParams = initDefaultParams();
initParams.maxInitTime = 12.0;
initParams.minInitTime = 4.0;
initParams.minSweepTurnForConvergence = pi;
initParams.maxSweeps = 0.75;
initParams.maxDepthRange = 10.0;
initParams.minDepthRange = 0.175;
initParams.debugPrint = true;

[initState, dataStore] = initializeLocalization( ...
    Robot, S.map, S.waypoints, S.beaconLoc, offset_x, offset_y, initParams, struct());

ekfParams = ekfWaypointDefaultParams();
ekfParams.maxDepthRange = 10.0;
ekfParams.minDepthRange = 0.175;
ekfParams.debugPrint = true;

plannerParams = knownMapPlannerDefaultParams();

verifyParams = optionalWallDefaultParams();
verifyParams.maxDepthRange = 10.0;
verifyParams.minDepthRange = 0.175;
verifyParams.debugPrint = true;

[wallStatus, dataStore, verifiedMap] = verifyOptionalWalls( ...
    Robot, S.map, S.optWalls, S.beaconLoc, offset_x, offset_y, ...
    initState, ekfParams, plannerParams, verifyParams, dataStore);

dataStore.initState = initState;
dataStore.wallStatus = wallStatus;
dataStore.verifiedMap = verifiedMap;

save(fullfile(codeDir, 'latestOptionalWallPracticeRun.mat'), ...
    'initState', 'wallStatus', 'verifiedMap', 'dataStore', 'S');

disp('Optional wall verification result:');
for i = 1:numel(wallStatus)
    fprintf('  optWall %d: status=%s confidence=%.2f rays=%d reason=%s\n', ...
        i, wallStatus(i).status, wallStatus(i).confidence, ...
        wallStatus(i).numRaysUsed, wallStatus(i).reason);
end
end
