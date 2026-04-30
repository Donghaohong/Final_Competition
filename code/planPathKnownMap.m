function [path, pathInfo] = planPathKnownMap(startXY, goalXY, map, plannerParams)
% planPathKnownMap Plan a collision-free path using known wall segments only.
%
% The roadmap is a visibility graph over start, goal, safe corner-offset
% samples, and optional sparse grid samples. Edges are added only when the
% straight segment keeps the requested clearance from every known wall.

if nargin < 4 || isempty(plannerParams)
    plannerParams = struct();
end

params = knownMapPlannerDefaultParams(plannerParams);

if size(map, 2) ~= 4
    error('map must be an N x 4 wall matrix [x1 y1 x2 y2].');
end

startXY = startXY(:).';
goalXY = goalXY(:).';
if numel(startXY) ~= 2 || numel(goalXY) ~= 2
    error('startXY and goalXY must be 1 x 2 points.');
end

bounds = inferMapBounds(map, params);

path = zeros(0, 2);
pathInfo = struct();
pathInfo.success = false;
pathInfo.reason = '';
pathInfo.params = params;
pathInfo.bounds = bounds;
pathInfo.nodes = [];
pathInfo.edges = zeros(0, 2);
pathInfo.rawPath = [];
pathInfo.pathCost = inf;

if ~pointInsideBounds(startXY, bounds, params.startGoalClearance)
    pathInfo.reason = 'startOutsideBounds';
    return;
end
if ~pointInsideBounds(goalXY, bounds, params.startGoalClearance)
    pathInfo.reason = 'goalOutsideBounds';
    return;
end
if pointWallClearance(startXY, map) < params.startGoalClearance
    pathInfo.reason = 'startTooCloseToWall';
    return;
end
if pointWallClearance(goalXY, map) < params.startGoalClearance
    pathInfo.reason = 'goalTooCloseToWall';
    return;
end

if edgeIsCollisionFree(startXY, goalXY, map, bounds, params)
    path = [startXY; goalXY];
    pathInfo.success = true;
    pathInfo.reason = 'directPath';
    pathInfo.nodes = path;
    pathInfo.edges = [1 2];
    pathInfo.rawPath = path;
    pathInfo.pathCost = norm(goalXY - startXY);
    return;
end

candidateNodes = generateRoadmapNodes(map, bounds, params);
nodes = [startXY; goalXY; candidateNodes];
nodes = uniqueRowsTolerance(nodes, params.duplicateNodeTol);

% Preserve exact start and goal at fixed indices after duplicate removal.
nodes = [startXY; goalXY; nodes(3:end, :)];
nodes = uniqueRowsTolerancePreserveFirst(nodes, params.duplicateNodeTol);

n = size(nodes, 1);
adj = inf(n, n);
adj(1:n+1:end) = 0;
edges = zeros(0, 2);

for i = 1:n-1
    for j = i+1:n
        if edgeIsCollisionFree(nodes(i, :), nodes(j, :), map, bounds, params)
            w = norm(nodes(i, :) - nodes(j, :));
            adj(i, j) = w;
            adj(j, i) = w;
            edges(end+1, :) = [i j]; %#ok<AGROW>
        end
    end
end

[nodePath, pathCost] = runDijkstraLocal(adj, 1, 2);
pathInfo.nodes = nodes;
pathInfo.edges = edges;
pathInfo.adjacency = adj;
pathInfo.nodePath = nodePath;
pathInfo.pathCost = pathCost;

if isempty(nodePath)
    pathInfo.reason = 'noGraphPath';
    return;
end

rawPath = nodes(nodePath, :);
if params.smoothPath
    path = smoothVisibilityPath(rawPath, map, bounds, params);
else
    path = rawPath;
end

pathInfo.success = true;
pathInfo.reason = 'graphPath';
pathInfo.rawPath = rawPath;
pathInfo.path = path;
pathInfo.pathCost = pathLength(path);
end

function nodes = generateRoadmapNodes(map, bounds, params)
wallEndpoints = [map(:, 1:2); map(:, 3:4)];
wallEndpoints = uniqueRowsTolerance(wallEndpoints, params.duplicateNodeTol);

angles = linspace(0, 2 * pi, params.numCornerSamples + 1);
angles(end) = [];
nodes = zeros(0, 2);

for i = 1:size(wallEndpoints, 1)
    corner = wallEndpoints(i, :);
    for a = angles
        p = corner + params.cornerOffsetRadius * [cos(a), sin(a)];
        if pointIsValidRoadmapNode(p, map, bounds, params)
            nodes(end+1, :) = p; %#ok<AGROW>
        end
    end
end

if params.addGridSamples
    xs = (bounds(1) + params.nodeClearance):params.gridSpacing:(bounds(2) - params.nodeClearance);
    ys = (bounds(3) + params.nodeClearance):params.gridSpacing:(bounds(4) - params.nodeClearance);
    for ix = 1:numel(xs)
        for iy = 1:numel(ys)
            p = [xs(ix), ys(iy)];
            if pointIsValidRoadmapNode(p, map, bounds, params)
                nodes(end+1, :) = p; %#ok<AGROW>
            end
        end
    end
end

nodes = uniqueRowsTolerance(nodes, params.duplicateNodeTol);
end

function tf = pointIsValidRoadmapNode(p, map, bounds, params)
tf = pointInsideBounds(p, bounds, params.nodeClearance) && ...
     pointWallClearance(p, map) >= params.nodeClearance - params.clearanceTol;
end

function bounds = inferMapBounds(map, params)
xs = [map(:, 1); map(:, 3)];
ys = [map(:, 2); map(:, 4)];
bounds = [ ...
    min(xs) - params.boundsPadding, ...
    max(xs) + params.boundsPadding, ...
    min(ys) - params.boundsPadding, ...
    max(ys) + params.boundsPadding];
end

function tf = pointInsideBounds(p, bounds, clearance)
tf = p(1) >= bounds(1) + clearance && ...
     p(1) <= bounds(2) - clearance && ...
     p(2) >= bounds(3) + clearance && ...
     p(2) <= bounds(4) - clearance;
end

function dMin = pointWallClearance(p, map)
dMin = inf;
for i = 1:size(map, 1)
    d = pointSegmentDistance(p, map(i, 1:2), map(i, 3:4));
    dMin = min(dMin, d);
end
end

function tf = edgeIsCollisionFree(p1, p2, map, bounds, params)
tf = false;
if ~pointInsideBounds(p1, bounds, params.startGoalClearance) || ...
   ~pointInsideBounds(p2, bounds, params.startGoalClearance)
    return;
end

for i = 1:size(map, 1)
    wallA = map(i, 1:2);
    wallB = map(i, 3:4);
    d = segmentSegmentDistance(p1, p2, wallA, wallB);
    if d < params.edgeClearance - params.clearanceTol
        return;
    end
end

tf = true;
end

function d = pointSegmentDistance(p, a, b)
ab = b - a;
den = dot(ab, ab);
if den < eps
    d = norm(p - a);
    return;
end
t = dot(p - a, ab) / den;
t = max(0, min(1, t));
proj = a + t * ab;
d = norm(p - proj);
end

function d = segmentSegmentDistance(a, b, c, dpt)
if segmentsIntersect(a, b, c, dpt)
    d = 0;
    return;
end

d = min([ ...
    pointSegmentDistance(a, c, dpt), ...
    pointSegmentDistance(b, c, dpt), ...
    pointSegmentDistance(c, a, b), ...
    pointSegmentDistance(dpt, a, b)]);
end

function tf = segmentsIntersect(a, b, c, d)
tol = 1e-10;
o1 = orientation(a, b, c);
o2 = orientation(a, b, d);
o3 = orientation(c, d, a);
o4 = orientation(c, d, b);

if (o1 * o2 < -tol) && (o3 * o4 < -tol)
    tf = true;
    return;
end

tf = (abs(o1) <= tol && pointOnSegment(c, a, b, tol)) || ...
     (abs(o2) <= tol && pointOnSegment(d, a, b, tol)) || ...
     (abs(o3) <= tol && pointOnSegment(a, c, d, tol)) || ...
     (abs(o4) <= tol && pointOnSegment(b, c, d, tol));
end

function val = orientation(a, b, c)
val = (b(1) - a(1)) * (c(2) - a(2)) - ...
      (b(2) - a(2)) * (c(1) - a(1));
end

function tf = pointOnSegment(p, a, b, tol)
tf = p(1) >= min(a(1), b(1)) - tol && p(1) <= max(a(1), b(1)) + tol && ...
     p(2) >= min(a(2), b(2)) - tol && p(2) <= max(a(2), b(2)) + tol;
end

function rows = uniqueRowsTolerance(rows, tol)
if isempty(rows)
    return;
end
keep = true(size(rows, 1), 1);
for i = 1:size(rows, 1)
    if ~keep(i)
        continue;
    end
    diffs = rows(i+1:end, :) - rows(i, :);
    dup = sqrt(sum(diffs .^ 2, 2)) < tol;
    keep(i + find(dup)) = false;
end
rows = rows(keep, :);
end

function rowsOut = uniqueRowsTolerancePreserveFirst(rows, tol)
rowsOut = zeros(0, 2);
for i = 1:size(rows, 1)
    if isempty(rowsOut)
        rowsOut(end+1, :) = rows(i, :); %#ok<AGROW>
        continue;
    end
    diffs = rowsOut - rows(i, :);
    if all(sqrt(sum(diffs .^ 2, 2)) >= tol)
        rowsOut(end+1, :) = rows(i, :); %#ok<AGROW>
    end
end
end

function [nodePath, totalCost] = runDijkstraLocal(adj, startIdx, goalIdx)
n = size(adj, 1);
visited = false(n, 1);
dist = inf(n, 1);
prev = zeros(n, 1);
dist(startIdx) = 0;

for iter = 1:n
    masked = dist;
    masked(visited) = inf;
    [best, u] = min(masked);
    if isinf(best)
        break;
    end
    if u == goalIdx
        break;
    end

    visited(u) = true;
    neighbors = find(isfinite(adj(u, :)) & ~visited.');
    for v = neighbors
        alt = dist(u) + adj(u, v);
        if alt < dist(v)
            dist(v) = alt;
            prev(v) = u;
        end
    end
end

if isinf(dist(goalIdx))
    nodePath = [];
    totalCost = inf;
    return;
end

totalCost = dist(goalIdx);
nodePath = goalIdx;
while nodePath(1) ~= startIdx
    nodePath = [prev(nodePath(1)), nodePath]; %#ok<AGROW>
end
end

function path = smoothVisibilityPath(rawPath, map, bounds, params)
if size(rawPath, 1) <= 2
    path = rawPath;
    return;
end

path = rawPath(1, :);
i = 1;
while i < size(rawPath, 1)
    bestJ = i + 1;
    for j = size(rawPath, 1):-1:(i + 1)
        if edgeIsCollisionFree(rawPath(i, :), rawPath(j, :), map, bounds, params)
            bestJ = j;
            break;
        end
    end
    path(end+1, :) = rawPath(bestJ, :); %#ok<AGROW>
    i = bestJ;
end
end

function len = pathLength(path)
if size(path, 1) < 2
    len = 0;
    return;
end
steps = diff(path, 1, 1);
len = sum(sqrt(sum(steps .^ 2, 2)));
end
