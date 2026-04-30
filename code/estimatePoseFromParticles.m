function est = estimatePoseFromParticles(particles, weights, waypointIds, waypointProb)
% estimatePoseFromParticles Pose estimate from the dominant start-hypothesis cluster.

weights = weights(:);
weights = weights / max(sum(weights), eps);

if nargin < 4 || isempty(waypointProb)
    waypointProb = computeWaypointPosterior(weights, waypointIds, max(waypointIds));
end

[clusterMass, dominantIdx] = max(waypointProb);
clusterMask = (waypointIds(:) == dominantIdx);

if ~any(clusterMask)
    clusterMask = true(size(weights));
    clusterMass = 1.0;
end

clusterWeights = weights(clusterMask);
clusterWeights = clusterWeights / max(sum(clusterWeights), eps);
clusterParticles = particles(clusterMask, :);

xHat = sum(clusterWeights .* clusterParticles(:, 1));
yHat = sum(clusterWeights .* clusterParticles(:, 2));
thetaHat = atan2(sum(clusterWeights .* sin(clusterParticles(:, 3))), ...
                 sum(clusterWeights .* cos(clusterParticles(:, 3))));

dx = clusterParticles(:, 1) - xHat;
dy = clusterParticles(:, 2) - yHat;
dtheta = wrapToPiLocal(clusterParticles(:, 3) - thetaHat);

X = [dx, dy, dtheta];
weightedX = bsxfun(@times, X, clusterWeights);
posCov = X' * weightedX;

est = struct();
est.pose = [xHat; yHat; thetaHat];
est.posCov = posCov;
est.headingStd = circularStdWeighted(clusterParticles(:, 3), clusterWeights);
est.dominantWaypointIdx = dominantIdx;
est.clusterMask = clusterMask;
est.clusterMass = clusterMass;
est.clusterWeights = clusterWeights;
est.clusterParticles = clusterParticles;
end
