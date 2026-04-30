function [logLike, debug] = scoreParticlesWithDepth(particles, obs, mapGeom, params)
% scoreParticlesWithDepth Log-likelihood from known-wall depth prediction.

N = size(particles, 1);
logLike = zeros(N, 1);

debug = struct();
debug.numBeamsUsed = 0;
debug.beamIndices = [];

if ~params.useDepth || ~isfield(obs, 'depthRanges') || isempty(obs.depthRanges)
    return;
end

depthRanges = obs.depthRanges(:);
depthAngles = obs.depthAngles(:);
validMask = obs.validDepthMask(:) & isfinite(depthRanges) & ...
            depthRanges >= params.minDepthRange & depthRanges <= params.maxDepthRange;

validIdx = find(validMask);
if isempty(validIdx)
    return;
end

if numel(validIdx) > params.depthMaxBeams
    samplePos = round(linspace(1, numel(validIdx), params.depthMaxBeams));
    validIdx = validIdx(samplePos);
    validIdx = unique(validIdx(:), 'stable');
end

debug.numBeamsUsed = numel(validIdx);
debug.beamIndices = validIdx;

obsSel = depthRanges(validIdx);
angSel = depthAngles(validIdx);

for i = 1:N
    predSel = raycastKnownMap(particles(i, :), mapGeom, angSel, params.sensorOffset, params);
    residual = obsSel - predSel;
    residual = max(-params.depthResidualClip, min(params.depthResidualClip, residual));
    logLike(i) = -0.5 * sum((residual ./ params.depthSigma) .^ 2);
end
end
