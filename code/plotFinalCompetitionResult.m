function fig = plotFinalCompetitionResult(dataStore)
% plotFinalCompetitionResult Final report plot for the competition entry.
%
% Optional wall drawing rule:
%   known map walls: black
%   optional wall exists: black
%   optional wall unknown: red
%   optional wall absent: not drawn

S = getMapStruct(dataStore);
if isempty(S)
    error('plotFinalCompetitionResult:missingMap', ...
        'dataStore does not contain S or compMap map fields.');
end

fig = figure('Name', 'Final Competition Result', 'Color', 'w');
ax = axes(fig);
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');
xlabel(ax, 'x (m)');
ylabel(ax, 'y (m)');
title(ax, 'Final Competition Result');

plotWallSet(ax, S.map, 'k-', 2.0, 'Known map');
plotOptionalWalls(ax, S, getFieldOrDefault(dataStore, 'wallStatus', []));
plotBeacons(ax, S);
plotWaypoints(ax, S, dataStore);
plotRobotTrajectory(ax, dataStore);

legend(ax, 'Location', 'bestoutside');
hold(ax, 'off');
end

function S = getMapStruct(dataStore)
S = [];
if isfield(dataStore, 'S') && isstruct(dataStore.S) && isfield(dataStore.S, 'map')
    S = dataStore.S;
elseif isfield(dataStore, 'compMap') && isstruct(dataStore.compMap) && isfield(dataStore.compMap, 'map')
    S = dataStore.compMap;
end
if ~isempty(S) && ~isfield(S, 'optWalls')
    S.optWalls = zeros(0, 4);
end
if ~isempty(S) && ~isfield(S, 'waypoints')
    S.waypoints = zeros(0, 2);
end
if ~isempty(S) && ~isfield(S, 'ECwaypoints')
    S.ECwaypoints = zeros(0, 2);
end
if ~isempty(S) && ~isfield(S, 'beaconLoc')
    S.beaconLoc = zeros(0, 3);
end
end

function plotWallSet(ax, walls, lineSpec, lineWidth, displayName)
if isempty(walls)
    return;
end
for i = 1:size(walls, 1)
    args = {'Color', lineSpec(1), 'LineStyle', lineSpec(2:end), ...
        'LineWidth', lineWidth};
    if i == 1
        args = [args {'DisplayName', displayName}]; %#ok<AGROW>
    else
        args = [args {'HandleVisibility', 'off'}]; %#ok<AGROW>
    end
    plot(ax, walls(i, [1 3]), walls(i, [2 4]), args{:});
end
end

function plotOptionalWalls(ax, S, wallStatus)
if isempty(S.optWalls)
    return;
end
for i = 1:size(S.optWalls, 1)
    status = 'unknown';
    if numel(wallStatus) >= i && isfield(wallStatus(i), 'status') && ~isempty(wallStatus(i).status)
        status = wallStatus(i).status;
    end
    switch lower(status)
        case 'exists'
            color = 'k';
            style = '-';
            width = 3.4;
            name = 'Optional wall exists';
        case 'absent'
            continue;
        otherwise
            color = 'r';
            style = '--';
            width = 2.6;
            name = 'Optional wall unknown';
    end
    hasName = ~hasLegendEntry(ax, name);
    args = {'Color', color, 'LineStyle', style, 'LineWidth', width};
    if hasName
        args = [args {'DisplayName', name}]; %#ok<AGROW>
    else
        args = [args {'HandleVisibility', 'off'}]; %#ok<AGROW>
    end
    plot(ax, S.optWalls(i, [1 3]), S.optWalls(i, [2 4]), args{:});
end
end

function plotBeacons(ax, S)
if isempty(S.beaconLoc)
    return;
end
scatter(ax, S.beaconLoc(:, 2), S.beaconLoc(:, 3), 45, ...
    'MarkerFaceColor', [0 0.9 0], 'MarkerEdgeColor', 'k', ...
    'DisplayName', 'Beacons');
for i = 1:size(S.beaconLoc, 1)
    text(ax, S.beaconLoc(i, 2) + 0.03, S.beaconLoc(i, 3) + 0.03, ...
        num2str(S.beaconLoc(i, 1)), 'Color', [0 0.35 0], ...
        'FontSize', 9, 'HandleVisibility', 'off');
end
end

function plotWaypoints(ax, S, dataStore)
if ~isempty(S.waypoints)
    scatter(ax, S.waypoints(:, 1), S.waypoints(:, 2), 52, 's', ...
        'MarkerFaceColor', [1.0 0.85 0.15], 'MarkerEdgeColor', 'k', ...
        'DisplayName', 'Waypoints');
end
if ~isempty(S.ECwaypoints)
    scatter(ax, S.ECwaypoints(:, 1), S.ECwaypoints(:, 2), 52, 'd', ...
        'MarkerFaceColor', [1.0 0.55 0.05], 'MarkerEdgeColor', 'k', ...
        'DisplayName', 'EC waypoints');
end

visitedNormal = collectVisitedGoals(dataStore, 'normalNavState');
visitedEC = collectVisitedGoals(dataStore, 'ecNavState');
visited = [visitedNormal; visitedEC];
if ~isempty(visited)
    scatter(ax, visited(:, 1), visited(:, 2), 95, 'o', ...
        'MarkerFaceColor', [0.15 0.45 1.0], 'MarkerEdgeColor', 'k', ...
        'LineWidth', 1.2, 'DisplayName', 'Visited goals');
end
end

function visited = collectVisitedGoals(dataStore, navField)
visited = zeros(0, 2);
if ~isfield(dataStore, navField) || ~isstruct(dataStore.(navField))
    return;
end
navState = dataStore.(navField);
if ~isfield(navState, 'goalWaypoints') || ~isfield(navState, 'reachedGoals')
    return;
end
goals = navState.goalWaypoints;
reached = logical(navState.reachedGoals(:));
if isfield(navState, 'beepMask') && ~isempty(navState.beepMask)
    realGoalMask = logical(navState.beepMask(:));
else
    realGoalMask = true(size(goals, 1), 1);
end
n = min([size(goals, 1), numel(reached), numel(realGoalMask)]);
if n < 1
    return;
end
keep = reached(1:n) & realGoalMask(1:n);
visited = goals(keep, :);
end

function plotRobotTrajectory(ax, dataStore)
traj = zeros(0, 2);
if isfield(dataStore, 'ekfMu') && size(dataStore.ekfMu, 2) >= 3
    traj = dataStore.ekfMu(:, 2:3);
elseif isfield(dataStore, 'pfMu') && size(dataStore.pfMu, 2) >= 3
    traj = dataStore.pfMu(:, 2:3);
end
if isempty(traj)
    return;
end
plot(ax, traj(:, 1), traj(:, 2), 'b-', 'LineWidth', 2.2, ...
    'DisplayName', 'Robot trajectory');
scatter(ax, traj(1, 1), traj(1, 2), 70, 'bo', ...
    'MarkerFaceColor', [0.6 0.8 1.0], 'DisplayName', 'Start');
scatter(ax, traj(end, 1), traj(end, 2), 80, 'bo', ...
    'MarkerFaceColor', 'b', 'DisplayName', 'Final pose');
end

function hasEntry = hasLegendEntry(ax, displayName)
children = findobj(ax, '-property', 'DisplayName');
names = get(children, 'DisplayName');
if ischar(names)
    names = {names};
end
hasEntry = any(strcmp(names, displayName));
end

function value = getFieldOrDefault(S, fieldName, defaultValue)
if isstruct(S) && isfield(S, fieldName)
    value = S.(fieldName);
else
    value = defaultValue;
end
end
