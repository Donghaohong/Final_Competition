function fig = plotOptionalWallVerificationResult(inputArg)
% plotOptionalWallVerificationResult Plot optional-wall verification output.
%
% Usage:
%   plotOptionalWallVerificationResult('latestOptionalWallPracticeRun.mat')

if nargin < 1 || isempty(inputArg)
    inputArg = fullfile(fileparts(mfilename('fullpath')), ...
                        'latestOptionalWallPracticeRun.mat');
end

runData = load(char(inputArg));
if ~isfield(runData, 'S') || ~isfield(runData, 'wallStatus')
    error('MAT file must contain S and wallStatus.');
end

S = runData.S;
wallStatus = runData.wallStatus;
dataStore = getFieldOrDefault(runData, 'dataStore', struct());

fig = figure('Name', 'Optional Wall Verification Result', 'Color', 'w');
ax = axes(fig);
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');
xlabel(ax, 'x (m)');
ylabel(ax, 'y (m)');
title(ax, 'Optional Wall Verification');

plotWalls(ax, S.map, 'k-', 1.8, 'Known map');

for i = 1:numel(wallStatus)
    wall = wallStatus(i).wall;
    switch wallStatus(i).status
        case 'exists'
            style = 'r-';
            width = 2.8;
        case 'absent'
            style = 'g--';
            width = 2.2;
        otherwise
            style = 'm:';
            width = 2.4;
    end
    plot(ax, wall([1 3]), wall([2 4]), style, ...
        'LineWidth', width, ...
        'DisplayName', sprintf('opt %d %s', i, wallStatus(i).status));
    mid = 0.5 * [wall(1) + wall(3), wall(2) + wall(4)];
    text(ax, mid(1) + 0.04, mid(2) + 0.04, ...
        sprintf('%d:%s %.2f', i, wallStatus(i).status, wallStatus(i).confidence), ...
        'FontWeight', 'bold');

    if ~isempty(wallStatus(i).observePoint)
        plot(ax, wallStatus(i).observePoint(1), wallStatus(i).observePoint(2), ...
            'co', 'MarkerFaceColor', 'c', ...
            'DisplayName', 'observe point');
    end
end

if isfield(S, 'waypoints') && ~isempty(S.waypoints)
    scatter(ax, S.waypoints(:, 1), S.waypoints(:, 2), 70, ...
        [1.00 0.82 0.10], 's', 'filled', ...
        'MarkerEdgeColor', 'k', ...
        'DisplayName', 'waypoints');
end

if isfield(S, 'beaconLoc') && ~isempty(S.beaconLoc)
    scatter(ax, S.beaconLoc(:, 2), S.beaconLoc(:, 3), 55, ...
        'g', 'filled', ...
        'MarkerEdgeColor', [0 0.25 0], ...
        'DisplayName', 'beacons');
end

if isfield(dataStore, 'ekfMu') && ~isempty(dataStore.ekfMu)
    plot(ax, dataStore.ekfMu(:, 2), dataStore.ekfMu(:, 3), 'b-', ...
        'LineWidth', 1.5, ...
        'DisplayName', 'EKF path');
end

xy = [S.map(:, 1:2); S.map(:, 3:4)];
if isfield(S, 'optWalls')
    xy = [xy; S.optWalls(:, 1:2); S.optWalls(:, 3:4)];
end
padding = 0.55;
xlim(ax, [min(xy(:, 1)) - padding, max(xy(:, 1)) + padding]);
ylim(ax, [min(xy(:, 2)) - padding, max(xy(:, 2)) + padding]);
legend(ax, 'Location', 'bestoutside');
end

function value = getFieldOrDefault(S, fieldName, defaultValue)
if isstruct(S) && isfield(S, fieldName)
    value = S.(fieldName);
else
    value = defaultValue;
end
end

function plotWalls(ax, walls, style, width, name)
for i = 1:size(walls, 1)
    if i == 1
        plot(ax, walls(i, [1 3]), walls(i, [2 4]), style, ...
            'LineWidth', width, 'DisplayName', name);
    else
        plot(ax, walls(i, [1 3]), walls(i, [2 4]), style, ...
            'LineWidth', width, 'HandleVisibility', 'off');
    end
end
end
