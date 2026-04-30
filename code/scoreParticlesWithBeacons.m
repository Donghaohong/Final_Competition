function [logLike, debug] = scoreParticlesWithBeacons(particles, obs, beaconLoc, offset_x, offset_y, params)
% scoreParticlesWithBeacons Log-likelihood from beacon observations.

N = size(particles, 1);
logLike = zeros(N, 1);

debug = struct();
debug.numDetections = 0;
debug.usedCartesian = true;

if ~params.useBeacons || ~isfield(obs, 'beacons') || isempty(obs.beacons)
    return;
end

numDetections = numel(obs.beacons);
debug.numDetections = numDetections;

for i = 1:N
    pose = particles(i, :);
    ll = 0.0;

    for j = 1:numDetections
        b = obs.beacons(j);
        pred = predictBeaconObservations(pose, beaconLoc, offset_x, offset_y, b.tagNum);

        if isempty(pred) || isnan(pred(1).tagNum)
            ll = ll + params.unknownTagLogPenalty;
            continue;
        end

        pred = pred(1);
        conf = 1.0;
        if isfield(b, 'confidence') && isfinite(b.confidence)
            conf = max(0.25, min(1.0, b.confidence));
        end

        if ~pred.visible || pred.xCam < params.minVisibleCameraX
            llObs = params.beaconInvisibleLogPenalty;
        elseif isfield(b, 'xCam') && isfield(b, 'yCam') && isfinite(b.xCam) && isfinite(b.yCam)
            dx = max(-params.beaconResidualClipXY, min(params.beaconResidualClipXY, b.xCam - pred.xCam));
            dy = max(-params.beaconResidualClipXY, min(params.beaconResidualClipXY, b.yCam - pred.yCam));
            llObs = -0.5 * ((dx / params.beaconSigmaX) ^ 2 + (dy / params.beaconSigmaY) ^ 2);
        else
            dr = max(-params.beaconResidualClipRange, ...
                     min(params.beaconResidualClipRange, b.range - pred.range));
            db = wrapToPiLocal(b.bearing - pred.bearing);
            db = max(-params.beaconResidualClipBearing, min(params.beaconResidualClipBearing, db));
            llObs = -0.5 * ((dr / params.beaconSigmaRange) ^ 2 + ...
                            (db / params.beaconSigmaBearing) ^ 2);
        end

        ll = ll + conf * llObs;
    end

    logLike(i) = ll;
end
end
