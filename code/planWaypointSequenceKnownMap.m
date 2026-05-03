function [plannedPath, sequenceInfo] = planWaypointSequenceKnownMap(startXY, goalWaypoints, map, plannerParams)
% planWaypointSequenceKnownMap Plan through multiple high-level goals.

if nargin < 4 || isempty(plannerParams)
    plannerParams = struct();
end

startXY = startXY(:).';
if isempty(goalWaypoints)
    plannedPath = startXY;
    sequenceInfo = struct( ...
        'success', true, ...
        'reason', 'noGoals', ...
        'segmentInfo', {{}}, ...
        'highLevelGoals', goalWaypoints);
    return;
end

if size(goalWaypoints, 2) ~= 2
    error('goalWaypoints must be an N x 2 matrix.');
end

plannedPath = startXY;
segmentInfo = cell(size(goalWaypoints, 1), 1);
currentStart = startXY;

sequenceInfo = struct();
sequenceInfo.success = false;
sequenceInfo.reason = '';
sequenceInfo.segmentInfo = segmentInfo;
sequenceInfo.highLevelGoals = goalWaypoints;
sequenceInfo.beepMask = false(size(plannedPath, 1), 1);

remainingGoals = goalWaypoints;

for k = 1:size(goalWaypoints, 1)
    % sorting points by nearest neighbor 
    distances = sqrt(sum((remainingGoals - currentStart).^2, 2));
    [~, nearestLocalIdx] = min(distances);


    goal =remainingGoals(nearestLocalIdx, :);

    [segmentPath, info] = planPathKnownMap(currentStart, goal, map, plannerParams);
    segmentInfo{k} = info;

    if isempty(segmentPath) || ~info.success
        sequenceInfo.reason = sprintf('segment%d_%s', k, info.reason);
        sequenceInfo.segmentInfo = segmentInfo;
        return;
    end

    if size(segmentPath, 1) >= 2
        plannedPath = [plannedPath; segmentPath(2:end, :)]; %#ok<AGROW>
    end
    sequenceInfo.beepMask = [sequenceInfo.beepMask; false(max(0, size(segmentPath, 1) - 2), 1); true];
    currentStart = goal;
    remainingGoals(nearestLocalIdx, :) = [];
end

sequenceInfo.success = true;
sequenceInfo.reason = 'ok';
sequenceInfo.segmentInfo = segmentInfo;
sequenceInfo.plannedPath = plannedPath;
sequenceInfo.beepMask = sequenceInfo.beepMask(:);
sequenceInfo.pathLength = pathLengthLocal(plannedPath);
end

function len = pathLengthLocal(path)
if size(path, 1) < 2
    len = 0;
    return;
end
steps = diff(path, 1, 1);
len = sum(sqrt(sum(steps .^ 2, 2)));
end
