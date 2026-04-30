function fig = plotInitializationResult(initInput, dataStore, mapData, waypoints, beaconLoc)
% plotInitializationResult Simple visualization for initialization localization.
%
% Common usage after running testInitializeLocalizationPracticeMap:
%   plotInitializationResult('latestInitLocalizationPracticeRun.mat')
%
% Other supported usage:
%   plotInitializationResult(initState, dataStore, S)
%   plotInitializationResult(initState, dataStore, map, waypoints, beaconLoc)

if nargin < 1 || isempty(initInput)
    initInput = fullfile(fileparts(mfilename('fullpath')), ...
                         'latestInitLocalizationPracticeRun.mat');
end

if ischar(initInput) || isstring(initInput)
    runData = load(char(initInput));
    if ~isfield(runData, 'initState')
        error('MAT file must contain initState.');
    end
    initState = runData.initState;

    if isfield(runData, 'dataStore')
        dataStore = runData.dataStore; %#ok<NASGU>
    end

    if isfield(runData, 'S')
        mapData = runData.S;
    else
        mapData = runData;
    end
elseif nargin >= 3
    initState = initInput;
else
    error(['Use plotInitializationResult(fileName), ', ...
           'plotInitializationResult(initState, dataStore, S), or ', ...
           'plotInitializationResult(initState, dataStore, map, waypoints, beaconLoc).']);
end

if nargin >= 5 && isnumeric(mapData)
    S = struct();
    S.map = mapData;
    S.waypoints = waypoints;
    S.beaconLoc = beaconLoc;
    mapData = S;
end

map = getFieldOrDefault(mapData, 'map', []);
waypoints = getFieldOrDefault(mapData, 'waypoints', []);
beaconLoc = getFieldOrDefault(mapData, 'beaconLoc', []);

if isempty(map)
    error('Map data must include field map.');
end
if isempty(waypoints)
    error('Map data must include field waypoints.');
end

pose = getFieldOrDefault(initState, 'pose', [NaN; NaN; NaN]);
particles = getFieldOrDefault(initState, 'particles', []);
waypointProb = getFieldOrDefault(initState, 'waypointProb', []);
startWaypointIdx = getFieldOrDefault(initState, 'startWaypointIdx', NaN);
confidence = getFieldOrDefault(initState, 'confidence', NaN);
converged = getFieldOrDefault(initState, 'converged', false);
headingStd = getFieldOrDefault(initState, 'headingStd', NaN);

if isempty(waypointProb)
    waypointProb = zeros(size(waypoints, 1), 1);
end
waypointProb = waypointProb(:);

fig = figure('Name', 'Initialization Result', 'Color', 'w');
ax = axes(fig);
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');
xlabel(ax, 'x (m)');
ylabel(ax, 'y (m)');

plotKnownMap(ax, map);
plotBeacons(ax, beaconLoc);
plotParticles(ax, particles);
plotWaypoints(ax, waypoints, waypointProb, startWaypointIdx);
plotEstimatedPose(ax, pose);
setNiceLimits(ax, map, waypoints, beaconLoc, particles, pose);

title(ax, sprintf('Init result: WP %d, confidence %.3f, converged %d, headingStd %.1f deg', ...
    startWaypointIdx, confidence, logical(converged), headingStd * 180 / pi));

legend(ax, 'Location', 'bestoutside');
end

function value = getFieldOrDefault(S, fieldName, defaultValue)
if isstruct(S) && isfield(S, fieldName)
    value = S.(fieldName);
else
    value = defaultValue;
end
end

function plotKnownMap(ax, map)
for i = 1:size(map, 1)
    if i == 1
        plot(ax, map(i, [1 3]), map(i, [2 4]), 'k-', ...
            'LineWidth', 1.8, ...
            'DisplayName', 'Known map');
    else
        plot(ax, map(i, [1 3]), map(i, [2 4]), 'k-', ...
            'LineWidth', 1.8, ...
            'HandleVisibility', 'off');
    end
end
end

function plotBeacons(ax, beaconLoc)
if isempty(beaconLoc)
    return;
end
scatter(ax, beaconLoc(:, 2), beaconLoc(:, 3), 70, ...
    'g', 'filled', ...
    'MarkerEdgeColor', [0 0.25 0], ...
    'DisplayName', 'Beacons');
for i = 1:size(beaconLoc, 1)
    text(ax, beaconLoc(i, 2) + 0.04, beaconLoc(i, 3) + 0.04, ...
        sprintf('%d', beaconLoc(i, 1)), ...
        'Color', [0 0.35 0], ...
        'FontWeight', 'bold');
end
end

function plotParticles(ax, particles)
if isempty(particles) || size(particles, 2) < 2
    return;
end
scatter(ax, particles(:, 1), particles(:, 2), 18, ...
    [0.45 0.45 0.45], ...
    'filled', ...
    'MarkerFaceAlpha', 0.35, ...
    'MarkerEdgeAlpha', 0.10, ...
    'DisplayName', 'Particles');
end

function plotWaypoints(ax, waypoints, waypointProb, startWaypointIdx)
numWaypoints = size(waypoints, 1);
if numel(waypointProb) ~= numWaypoints
    waypointProb = zeros(numWaypoints, 1);
end

scatter(ax, waypoints(:, 1), waypoints(:, 2), 120, ...
    [1.00 0.80 0.10], ...
    's', 'filled', ...
    'MarkerEdgeColor', 'k', ...
    'LineWidth', 1.2, ...
    'DisplayName', 'Candidate starts');

for k = 1:numWaypoints
    label = sprintf('WP%d %.2f', k, waypointProb(k));
    text(ax, waypoints(k, 1) + 0.05, waypoints(k, 2) + 0.05, label, ...
        'Color', 'k', ...
        'FontWeight', waypointLabelWeight(k, startWaypointIdx));
end

if isfinite(startWaypointIdx) && startWaypointIdx >= 1 && startWaypointIdx <= numWaypoints
    plot(ax, waypoints(startWaypointIdx, 1), waypoints(startWaypointIdx, 2), ...
        'ro', ...
        'MarkerSize', 15, ...
        'LineWidth', 2.0, ...
        'DisplayName', 'Selected start');
end
end

function fontWeight = waypointLabelWeight(k, startWaypointIdx)
if k == startWaypointIdx
    fontWeight = 'bold';
else
    fontWeight = 'normal';
end
end

function plotEstimatedPose(ax, pose)
if numel(pose) < 3 || any(~isfinite(pose(1:3)))
    return;
end
arrowLength = 0.35;
plot(ax, pose(1), pose(2), 'bo', ...
    'MarkerFaceColor', 'b', ...
    'MarkerSize', 8, ...
    'DisplayName', 'Estimated pose');
quiver(ax, pose(1), pose(2), ...
    arrowLength * cos(pose(3)), arrowLength * sin(pose(3)), ...
    0, ...
    'b', ...
    'LineWidth', 2.0, ...
    'MaxHeadSize', 2.0, ...
    'DisplayName', 'Estimated heading');
end

function setNiceLimits(ax, map, waypoints, beaconLoc, particles, pose)
xy = [map(:, 1:2); map(:, 3:4); waypoints(:, 1:2)];
if ~isempty(beaconLoc)
    xy = [xy; beaconLoc(:, 2:3)];
end
if ~isempty(particles) && size(particles, 2) >= 2
    xy = [xy; particles(:, 1:2)];
end
if numel(pose) >= 2 && all(isfinite(pose(1:2)))
    xy = [xy; pose(1:2).'];
end
xy = xy(all(isfinite(xy), 2), :);
if isempty(xy)
    return;
end

padding = 0.5;
xlim(ax, [min(xy(:, 1)) - padding, max(xy(:, 1)) + padding]);
ylim(ax, [min(xy(:, 2)) - padding, max(xy(:, 2)) + padding]);
end
