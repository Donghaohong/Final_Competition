function waypointProb = computeWaypointPosterior(weights, waypointIds, numWaypoints)
% computeWaypointPosterior Aggregate particle mass by start waypoint.

if nargin < 3 || isempty(numWaypoints)
    numWaypoints = max(waypointIds);
end

weights = weights(:);
waypointIds = waypointIds(:);

waypointProb = zeros(numWaypoints, 1);

for k = 1:numWaypoints
    waypointProb(k) = sum(weights(waypointIds == k));
end

totalProb = sum(waypointProb);
if totalProb > 0
    waypointProb = waypointProb / totalProb;
end
end
