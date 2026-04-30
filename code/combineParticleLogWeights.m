function logW = combineParticleLogWeights(prevWeights, beaconLogLike, depthLogLike, params)
% combineParticleLogWeights Combine prior and measurement terms in log-space.

if nargin < 1 || isempty(prevWeights)
    prevWeights = [];
end

numTerms = [numel(prevWeights), numel(beaconLogLike), numel(depthLogLike)];
N = max(numTerms);
if N == 0
    logW = [];
    return;
end

if isempty(prevWeights)
    prevWeights = ones(N, 1) / N;
else
    prevWeights = prevWeights(:);
end

if isempty(beaconLogLike)
    beaconLogLike = zeros(N, 1);
else
    beaconLogLike = beaconLogLike(:);
end

if isempty(depthLogLike)
    depthLogLike = zeros(N, 1);
else
    depthLogLike = depthLogLike(:);
end

logW = log(max(prevWeights, params.weightFloor));
logW = logW + params.beaconWeight * beaconLogLike;
logW = logW + params.depthWeight * depthLogLike;
logW = logW - max(logW);
end
