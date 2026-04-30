function dataStore = testInitializeLocalizationPracticeMap(Robot)
% testInitializeLocalizationPracticeMap SimulatorGUI entry for init localization.

mapFile = fullfile(fileparts(mfilename('fullpath')), 'PracticeMap2026.mat');
S = load(mapFile, 'map', 'waypoints', 'beaconLoc');

initParams = initDefaultParams();
initParams.maxInitTime = 25.0;
initParams.minInitTime = 8.0;
initParams.maxDepthRange = 10.0;
initParams.minDepthRange = 0.175;
initParams.debugPrint = true;

% Simulator CreateRobot uses RealSense camera offset [0, 0.08].
[initState, dataStore] = initializeLocalization( ...
    Robot, S.map, S.waypoints, S.beaconLoc, 0.0, 0.08, initParams, struct());

save(fullfile(fileparts(mfilename('fullpath')), 'latestInitLocalizationPracticeRun.mat'), ...
    'initState', 'dataStore', 'S');

disp('Initialization result:');
disp(initState);
end
