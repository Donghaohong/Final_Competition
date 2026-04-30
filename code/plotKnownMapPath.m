function fig = plotKnownMapPath(map, path, pathInfo, waypoints, beaconLoc)
% plotKnownMapPath Plot a known-map planned path for debugging.

if nargin < 3
    pathInfo = struct();
end
if nargin < 4
    waypoints = [];
end
if nargin < 5
    beaconLoc = [];
end

fig = figure('Name', 'Known Map Planned Path', 'Color', 'w');
ax = axes(fig);
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');
xlabel(ax, 'x (m)');
ylabel(ax, 'y (m)');
title(ax, 'Known Map Planned Path');

for i = 1:size(map, 1)
    if i == 1
        plot(ax, map(i, [1 3]), map(i, [2 4]), 'k-', ...
            'LineWidth', 1.8, 'DisplayName', 'Known map');
    else
        plot(ax, map(i, [1 3]), map(i, [2 4]), 'k-', ...
            'LineWidth', 1.8, 'HandleVisibility', 'off');
    end
end

if isstruct(pathInfo) && isfield(pathInfo, 'nodes') && ~isempty(pathInfo.nodes)
    scatter(ax, pathInfo.nodes(:, 1), pathInfo.nodes(:, 2), 12, ...
        [0.72 0.72 0.72], 'filled', 'DisplayName', 'Roadmap nodes');
end

if isstruct(pathInfo) && isfield(pathInfo, 'edges') && ~isempty(pathInfo.edges) && ...
        isfield(pathInfo, 'nodes') && ~isempty(pathInfo.nodes)
    for i = 1:size(pathInfo.edges, 1)
        p1 = pathInfo.nodes(pathInfo.edges(i, 1), :);
        p2 = pathInfo.nodes(pathInfo.edges(i, 2), :);
        plot(ax, [p1(1) p2(1)], [p1(2) p2(2)], '-', ...
            'Color', [0.88 0.88 0.88], ...
            'LineWidth', 0.5, ...
            'HandleVisibility', 'off');
    end
end

if ~isempty(waypoints)
    scatter(ax, waypoints(:, 1), waypoints(:, 2), 70, ...
        [1.00 0.82 0.10], 's', 'filled', ...
        'MarkerEdgeColor', 'k', ...
        'DisplayName', 'Map waypoints');
end

if ~isempty(beaconLoc)
    scatter(ax, beaconLoc(:, 2), beaconLoc(:, 3), 55, ...
        'g', 'filled', ...
        'MarkerEdgeColor', [0 0.25 0], ...
        'DisplayName', 'Beacons');
end

if ~isempty(path)
    plot(ax, path(:, 1), path(:, 2), 'r-o', ...
        'LineWidth', 2.4, ...
        'MarkerFaceColor', 'r', ...
        'DisplayName', 'Planned path');
    plot(ax, path(1, 1), path(1, 2), 'bo', ...
        'MarkerFaceColor', 'b', ...
        'MarkerSize', 8, ...
        'DisplayName', 'Start');
    plot(ax, path(end, 1), path(end, 2), 'p', ...
        'MarkerFaceColor', 'y', ...
        'MarkerEdgeColor', 'k', ...
        'MarkerSize', 12, ...
        'DisplayName', 'Goal');
end

xy = [map(:, 1:2); map(:, 3:4)];
if ~isempty(path)
    xy = [xy; path];
end
if ~isempty(waypoints)
    xy = [xy; waypoints];
end
if ~isempty(beaconLoc)
    xy = [xy; beaconLoc(:, 2:3)];
end
padding = 0.55;
xlim(ax, [min(xy(:, 1)) - padding, max(xy(:, 1)) + padding]);
ylim(ax, [min(xy(:, 2)) - padding, max(xy(:, 2)) + padding]);
legend(ax, 'Location', 'bestoutside');
end
