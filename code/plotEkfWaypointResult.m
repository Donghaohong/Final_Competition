function fig = plotEkfWaypointResult(inputArg, varargin)
% plotEkfWaypointResult Simple plot for EKF waypoint follower runs.
%
% Common usage:
%   plotEkfWaypointResult('latestEkfWaypointPracticeRun.mat')
%
% Other supported usage:
%   plotEkfWaypointResult(dataStore, S, goalWaypoints)

if nargin < 1 || isempty(inputArg)
    inputArg = fullfile(fileparts(mfilename('fullpath')), ...
                        'latestEkfWaypointPracticeRun.mat');
end

if ischar(inputArg) || isstring(inputArg)
    runData = load(char(inputArg));
    if ~isfield(runData, 'dataStore')
        error('MAT file must contain dataStore.');
    end
    dataStore = runData.dataStore;
    if isfield(runData, 'S')
        mapData = runData.S;
    else
        mapData = runData;
    end
    if isfield(runData, 'goalWaypoints')
        goalWaypoints = runData.goalWaypoints;
    else
        goalWaypoints = [];
    end
elseif nargin >= 2
    dataStore = inputArg;
    mapData = varargin{1};
    if numel(varargin) >= 2
        goalWaypoints = varargin{2};
    else
        goalWaypoints = [];
    end
else
    error('Use plotEkfWaypointResult(fileName) or plotEkfWaypointResult(dataStore, S, goalWaypoints).');
end

map = getFieldOrDefault(mapData, 'map', []);
waypoints = getFieldOrDefault(mapData, 'waypoints', []);
beaconLoc = getFieldOrDefault(mapData, 'beaconLoc', []);

if isempty(map)
    error('Map data must include field map.');
end
if ~isfield(dataStore, 'ekfMu') || isempty(dataStore.ekfMu)
    error('dataStore.ekfMu is empty.');
end

fig = figure('Name', 'EKF Waypoint Result', 'Color', 'w');
axMap = axes(fig);
hold(axMap, 'on');
axis(axMap, 'equal');
grid(axMap, 'on');
xlabel(axMap, 'x (m)');
ylabel(axMap, 'y (m)');
title(axMap, 'EKF Path and Goals');

plotMapWalls(axMap, map);
plotBeacons(axMap, beaconLoc);
plotCandidateWaypoints(axMap, waypoints);
plotGoals(axMap, goalWaypoints);
plotEkfPath(axMap, dataStore);
plotFinalCovariance(axMap, dataStore);
setMapLimits(axMap, map, waypoints, beaconLoc, goalWaypoints, dataStore.ekfMu);
legend(axMap, 'Location', 'bestoutside');
end

function value = getFieldOrDefault(S, fieldName, defaultValue)
if isstruct(S) && isfield(S, fieldName)
    value = S.(fieldName);
else
    value = defaultValue;
end
end

function plotMapWalls(ax, map)
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
scatter(ax, beaconLoc(:, 2), beaconLoc(:, 3), 55, ...
    'g', 'filled', ...
    'MarkerEdgeColor', [0 0.25 0], ...
    'DisplayName', 'Beacons');
for i = 1:size(beaconLoc, 1)
    text(ax, beaconLoc(i, 2) + 0.04, beaconLoc(i, 3) + 0.04, ...
        sprintf('%d', beaconLoc(i, 1)), ...
        'Color', [0 0.35 0]);
end
end

function plotCandidateWaypoints(ax, waypoints)
if isempty(waypoints)
    return;
end
scatter(ax, waypoints(:, 1), waypoints(:, 2), 70, ...
    [1.00 0.82 0.10], ...
    's', 'filled', ...
    'MarkerEdgeColor', 'k', ...
    'DisplayName', 'Map waypoints');
for k = 1:size(waypoints, 1)
    text(ax, waypoints(k, 1) + 0.04, waypoints(k, 2) + 0.04, ...
        sprintf('WP%d', k), ...
        'Color', 'k');
end
end

function plotGoals(ax, goalWaypoints)
if isempty(goalWaypoints)
    return;
end
plot(ax, goalWaypoints(:, 1), goalWaypoints(:, 2), 'r--o', ...
    'LineWidth', 1.2, ...
    'MarkerFaceColor', 'r', ...
    'DisplayName', 'Goal sequence');
end

function plotEkfPath(ax, dataStore)
ekfMu = dataStore.ekfMu;
plot(ax, ekfMu(:, 2), ekfMu(:, 3), 'b-', ...
    'LineWidth', 2.0, ...
    'DisplayName', 'EKF path');
plot(ax, ekfMu(1, 2), ekfMu(1, 3), 'bo', ...
    'MarkerFaceColor', [0.65 0.80 1.00], ...
    'MarkerSize', 8, ...
    'DisplayName', 'EKF start');

xf = ekfMu(end, 2);
yf = ekfMu(end, 3);
thf = ekfMu(end, 4);
plot(ax, xf, yf, 'bo', ...
    'MarkerFaceColor', 'b', ...
    'MarkerSize', 8, ...
    'DisplayName', 'EKF final');
quiver(ax, xf, yf, 0.35 * cos(thf), 0.35 * sin(thf), ...
    0, ...
    'b', ...
    'LineWidth', 2.0, ...
    'MaxHeadSize', 2.0, ...
    'DisplayName', 'Final heading');
end

function plotFinalCovariance(ax, dataStore)
if ~isfield(dataStore, 'ekfSigma') || isempty(dataStore.ekfSigma)
    return;
end
ekfMu = dataStore.ekfMu;
P = dataStore.ekfSigma(1:2, 1:2, end);
mu = ekfMu(end, 2:3).';

if any(~isfinite(P(:))) || any(~isfinite(mu))
    return;
end
P = (P + P.') / 2;
[V, D] = eig(P);
evals = max(diag(D), 0);
if all(evals <= eps)
    return;
end

ang = linspace(0, 2 * pi, 100);
ellipse = mu + 2.0 * V * diag(sqrt(evals)) * [cos(ang); sin(ang)];
plot(ax, ellipse(1, :), ellipse(2, :), 'm-', ...
    'LineWidth', 1.4, ...
    'DisplayName', 'Final 2-sigma covariance');
end

function setMapLimits(ax, map, waypoints, beaconLoc, goalWaypoints, ekfMu)
xy = [map(:, 1:2); map(:, 3:4)];
if ~isempty(waypoints)
    xy = [xy; waypoints(:, 1:2)];
end
if ~isempty(beaconLoc)
    xy = [xy; beaconLoc(:, 2:3)];
end
if ~isempty(goalWaypoints)
    xy = [xy; goalWaypoints(:, 1:2)];
end
if ~isempty(ekfMu)
    xy = [xy; ekfMu(:, 2:3)];
end
xy = xy(all(isfinite(xy), 2), :);
if isempty(xy)
    return;
end
padding = 0.55;
xlim(ax, [min(xy(:, 1)) - padding, max(xy(:, 1)) + padding]);
ylim(ax, [min(xy(:, 2)) - padding, max(xy(:, 2)) + padding]);
end
