function [converged, confidence, debug] = checkInitializationConvergence(summary, history, params)
% checkInitializationConvergence Multi-criterion convergence check for init PF.

numWaypoints = numel(summary.waypointProb);
uniformProb = 1 / max(numWaypoints, 1);
dominantProb = max(summary.waypointProb);

if isempty(summary.posCov)
    posStd = inf;
else
    posStd = sqrt(max(0, trace(summary.posCov(1:2, 1:2))));
end

headingStd = summary.headingStd;

stablePose = inf;
stableHeading = inf;
stableWaypoint = false;

if size(history.poses, 1) >= params.stabilityWindow
    recentPoses = history.poses(end - params.stabilityWindow + 1:end, :);
    recentWaypoints = history.dominantWaypointIdx(end - params.stabilityWindow + 1:end);

    if all(recentWaypoints == recentWaypoints(end))
        poseRef = recentPoses(end, :);
        posDiff = sqrt(sum((recentPoses(:, 1:2) - poseRef(1:2)) .^ 2, 2));
        headingDiff = abs(wrapToPiLocal(recentPoses(:, 3) - poseRef(3)));

        stablePose = max(posDiff);
        stableHeading = max(headingDiff);
        % The robot is intentionally rotating during initialization, so the
        % absolute heading estimate should move. Heading quality is checked
        % separately via headingStd; stability here means the start
        % hypothesis and in-place position estimate remain consistent.
        stableWaypoint = (stablePose <= params.stablePosThreshold);
    end
end

cDom = max(0, min(1, (dominantProb - uniformProb) / max(1 - uniformProb, eps)));
cPos = exp(-0.5 * (posStd / max(params.maxPosStdForConvergence, eps)) ^ 2);
cHead = exp(-0.5 * (headingStd / max(params.maxHeadingStdForConvergence, eps)) ^ 2);
cStable = double(stableWaypoint);

confidence = min(1, max(0, 0.45 * cDom + 0.25 * cPos + 0.20 * cHead + 0.10 * cStable));

converged = summary.numUpdates >= params.minConvergenceUpdates && ...
            dominantProb >= params.dominantWaypointThresh && ...
            posStd <= params.maxPosStdForConvergence && ...
            headingStd <= params.maxHeadingStdForConvergence && ...
            stableWaypoint;

if ~converged
    confidence = min(confidence, params.maxUnconvergedConfidence);
end

debug = struct();
debug.dominantWaypointProb = dominantProb;
debug.posStd = posStd;
debug.headingStd = headingStd;
debug.stablePose = stablePose;
debug.stableHeading = stableHeading;
debug.stableWaypoint = stableWaypoint;
end
