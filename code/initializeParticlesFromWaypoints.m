function [particles, weights, waypointIds, debug] = initializeParticlesFromWaypoints(waypoints, params)
% initializeParticlesFromWaypoints Initialize PF support on known start waypoints.

if isempty(waypoints) || size(waypoints, 2) ~= 2
    error('waypoints must be a nonempty N x 2 matrix.');
end

numWaypoints = size(waypoints, 1);
numParticles = max(params.numParticles, numWaypoints);

counts = floor(numParticles / numWaypoints) * ones(numWaypoints, 1);
counts(1:mod(numParticles, numWaypoints)) = counts(1:mod(numParticles, numWaypoints)) + 1;

particles = zeros(numParticles, 3);
waypointIds = zeros(numParticles, 1);

cursor = 1;
for k = 1:numWaypoints
    nk = counts(k);
    idx = cursor:(cursor + nk - 1);

    headings = linspace(-pi, pi, nk + 1);
    headings(end) = [];

    particles(idx, 1) = waypoints(k, 1) + params.startPosStd * randn(nk, 1);
    particles(idx, 2) = waypoints(k, 2) + params.startPosStd * randn(nk, 1);
    particles(idx, 3) = wrapToPiLocal(headings(:) + params.startHeadingJitter * randn(nk, 1));
    waypointIds(idx) = k;

    cursor = cursor + nk;
end

weights = ones(numParticles, 1) / numParticles;

debug = struct();
debug.countsPerWaypoint = counts;
debug.numParticles = numParticles;
end
