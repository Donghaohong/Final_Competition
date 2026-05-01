function fig = plotPfWaypointResult(inputArg, varargin)
% plotPfWaypointResult Simple plot for PF waypoint follower runs.
%
% Usage:
%   plotPfWaypointResult('latestPfWaypointPracticeRun.mat')
%   plotPfWaypointResult(dataStore, S, goalWaypoints)

if nargin == 1 && (ischar(inputArg) || isstring(inputArg))
    loaded = load(inputArg);
    if isfield(loaded, 'dataStore')
        dataStore = loaded.dataStore;
    else
        error('File must contain dataStore.');
    end
    if isfield(loaded, 'S')
        S = loaded.S;
    else
        S = struct();
    end
    if isfield(loaded, 'pfGoalWaypoints')
        goalWaypoints = loaded.pfGoalWaypoints;
    elseif isfield(loaded, 'plannedPath')
        goalWaypoints = loaded.plannedPath;
    else
        goalWaypoints = [];
    end
elseif nargin >= 3
    dataStore = inputArg;
    S = varargin{1};
    goalWaypoints = varargin{2};
else
    error('Use plotPfWaypointResult(fileName) or plotPfWaypointResult(dataStore, S, goalWaypoints).');
end

map = getStructFieldOrDefault(S, 'map', []);
beaconLoc = getStructFieldOrDefault(S, 'beaconLoc', []);
waypoints = getStructFieldOrDefault(S, 'waypoints', []);

fig = figure('Name', 'PF Waypoint Result', 'Color', 'w');
axMap = axes(fig);
hold(axMap, 'on');
axis(axMap, 'equal');
grid(axMap, 'on');
xlabel(axMap, 'x (m)');
ylabel(axMap, 'y (m)');
title(axMap, 'PF Path and Goals');

plotPfMapWalls(axMap, map);
plotPfBeacons(axMap, beaconLoc);
plotPfCandidateWaypoints(axMap, waypoints);
plotPfGoals(axMap, goalWaypoints);
plotPfPath(axMap, dataStore);
plotPfFinalParticles(axMap, dataStore);
plotPfFinalCovariance(axMap, dataStore);

legend(axMap, 'Location', 'bestoutside');
setPfPlotLimits(axMap, map, beaconLoc, waypoints, goalWaypoints, dataStore);
end

function plotPfMapWalls(ax, map)
if isempty(map)
    return;
end
for i = 1:size(map, 1)
    if i == 1
        plot(ax, map(i, [1 3]), map(i, [2 4]), 'k-', ...
            'LineWidth', 2.0, 'DisplayName', 'Known map');
    else
        plot(ax, map(i, [1 3]), map(i, [2 4]), 'k-', ...
            'LineWidth', 2.0, 'HandleVisibility', 'off');
    end
end
end

function plotPfBeacons(ax, beaconLoc)
if isempty(beaconLoc)
    return;
end
scatter(ax, beaconLoc(:, 2), beaconLoc(:, 3), 45, 'g', 'filled', ...
    'MarkerEdgeColor', [0 0.3 0], 'DisplayName', 'Beacons');
for i = 1:size(beaconLoc, 1)
    text(ax, beaconLoc(i, 2) + 0.04, beaconLoc(i, 3), sprintf('%g', beaconLoc(i, 1)), ...
        'Color', [0 0.3 0], 'FontSize', 8);
end
end

function plotPfCandidateWaypoints(ax, waypoints)
if isempty(waypoints)
    return;
end
scatter(ax, waypoints(:, 1), waypoints(:, 2), 35, 's', ...
    'MarkerFaceColor', [1.0 0.85 0.1], 'MarkerEdgeColor', [0.25 0.2 0], ...
    'DisplayName', 'Map waypoints');
end

function plotPfGoals(ax, goalWaypoints)
if isempty(goalWaypoints)
    return;
end
plot(ax, goalWaypoints(:, 1), goalWaypoints(:, 2), 'r--o', ...
    'LineWidth', 1.2, 'MarkerFaceColor', 'r', 'DisplayName', 'Goal sequence');
end

function plotPfPath(ax, dataStore)
if ~isfield(dataStore, 'pfMu') || isempty(dataStore.pfMu)
    return;
end
pfMu = dataStore.pfMu;
plot(ax, pfMu(:, 2), pfMu(:, 3), 'b-', ...
    'LineWidth', 2.0, 'DisplayName', 'PF path');
plot(ax, pfMu(1, 2), pfMu(1, 3), 'bo', ...
    'MarkerSize', 8, 'MarkerFaceColor', [0.65 0.80 1.0], ...
    'DisplayName', 'PF start');
xf = pfMu(end, 2);
yf = pfMu(end, 3);
theta = pfMu(end, 4);
plot(ax, xf, yf, 'bo', ...
    'MarkerSize', 8, 'MarkerFaceColor', 'b', ...
    'DisplayName', 'PF final');
quiver(ax, xf, yf, 0.35 * cos(theta), 0.35 * sin(theta), 0, ...
    'b', 'LineWidth', 1.8, 'MaxHeadSize', 2, ...
    'DisplayName', 'Final heading');
end

function plotPfFinalParticles(ax, dataStore)
if ~isfield(dataStore, 'pfParticles') || isempty(dataStore.pfParticles)
    return;
end
particles = dataStore.pfParticles{end};
if isempty(particles)
    return;
end
scatter(ax, particles(:, 1), particles(:, 2), 16, [0.55 0.55 0.55], ...
    'filled', 'MarkerFaceAlpha', 0.35, 'DisplayName', 'Final particles');
end

function plotPfFinalCovariance(ax, dataStore)
if ~isfield(dataStore, 'pfFinalState') || ~isfield(dataStore.pfFinalState, 'poseCov')
    return;
end
pose = dataStore.pfFinalState.pose;
P = dataStore.pfFinalState.poseCov(1:2, 1:2);
if any(~isfinite(P(:))) || any(~isfinite(pose(:)))
    return;
end
P = (P + P.') / 2;
[V, D] = eig(P);
evals = max(diag(D), 0);
if all(evals <= eps)
    return;
end
ang = linspace(0, 2 * pi, 80);
ellipse = pose(1:2) + 2.0 * V * diag(sqrt(evals)) * [cos(ang); sin(ang)];
plot(ax, ellipse(1, :), ellipse(2, :), 'm-', ...
    'LineWidth', 1.5, 'DisplayName', 'Final 2-sigma covariance');
end

function setPfPlotLimits(ax, map, beaconLoc, waypoints, goalWaypoints, dataStore)
xy = [];
if ~isempty(map)
    xy = [xy; map(:, 1:2); map(:, 3:4)];
end
if ~isempty(beaconLoc)
    xy = [xy; beaconLoc(:, 2:3)];
end
if ~isempty(waypoints)
    xy = [xy; waypoints(:, 1:2)];
end
if ~isempty(goalWaypoints)
    xy = [xy; goalWaypoints(:, 1:2)];
end
if isfield(dataStore, 'pfMu') && ~isempty(dataStore.pfMu)
    xy = [xy; dataStore.pfMu(:, 2:3)];
end
if isfield(dataStore, 'pfParticles') && ~isempty(dataStore.pfParticles)
    particles = dataStore.pfParticles{end};
    if ~isempty(particles)
        xy = [xy; particles(:, 1:2)];
    end
end
xy = xy(all(isfinite(xy), 2), :);
if isempty(xy)
    return;
end
padding = 0.55;
xlim(ax, [min(xy(:, 1)) - padding, max(xy(:, 1)) + padding]);
ylim(ax, [min(xy(:, 2)) - padding, max(xy(:, 2)) + padding]);
end

function value = getStructFieldOrDefault(S, fieldName, defaultValue)
if isstruct(S) && isfield(S, fieldName)
    value = S.(fieldName);
else
    value = defaultValue;
end
end
