function depthPred = raycastKnownMap(pose, mapGeom, beamAngles, sensorOffset, params)
% raycastKnownMap Predict depth values against known fixed walls only.

if isnumeric(mapGeom)
    mapGeom = precomputeKnownMapGeometry(mapGeom, params.wallThickness);
end

beamAngles = beamAngles(:);
numBeams = numel(beamAngles);
depthPred = params.maxDepthRange * ones(numBeams, 1);

pose = pose(:).';
theta = pose(3);

c = cos(theta);
s = sin(theta);

origin = [ ...
    pose(1) + c * sensorOffset(1) - s * sensorOffset(2), ...
    pose(2) + s * sensorOffset(1) + c * sensorOffset(2)];

for k = 1:numBeams
    relAng = beamAngles(k);
    dirVec = [cos(theta + relAng), sin(theta + relAng)];
    bestT = inf;

    for i = 1:numel(mapGeom.edges)
        edges = mapGeom.edges{i};
        for e = 1:size(edges, 1)
            p1 = edges(e, 1:2);
            p2 = edges(e, 3:4);
            [t, u, valid] = rayIntersectSegment(origin, dirVec, p1, p2);
            if valid && t < bestT && u >= 0 && u <= 1
                bestT = t;
            end
        end
    end

    if isfinite(bestT)
        depthPred(k) = max(params.minDepthRange, min(params.maxDepthRange, bestT * cos(relAng)));
    end
end
end

function [t, u, valid] = rayIntersectSegment(origin, dirVec, p1, p2)
segment = p2 - p1;
den = dirVec(1) * segment(2) - dirVec(2) * segment(1);

if abs(den) < 1e-12
    t = inf;
    u = NaN;
    valid = false;
    return;
end

delta = p1 - origin;
t = (delta(1) * segment(2) - delta(2) * segment(1)) / den;
u = (delta(1) * dirVec(2) - delta(2) * dirVec(1)) / den;

valid = (t >= 0);
end
