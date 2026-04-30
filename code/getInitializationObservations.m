function obs = getInitializationObservations(Robot, offset_x, offset_y, params)
% getInitializationObservations Collect raw init-stage depth, beacon, and bump data.
%
% Confirmed course APIs used here:
%   - RealSenseDist
%   - RealSenseTag
%   - BumpsWheelDropsSensorsRoomba
%
% TODO:
%   Verify the exact RealSenseTag column ordering on the competition machine.
%   This wrapper currently assumes the format used elsewhere in this repo:
%   [latency, id, x_tag, y_tag, theta_tag].

obs = struct();
obs.depthRaw = [];
obs.depthRanges = [];
obs.depthAngles = [];
obs.validDepthMask = [];
obs.rawTagMatrix = [];
obs.beacons = struct('tagNum', {}, 'latency', {}, 'xCam', {}, 'yCam', {}, ...
                     'thetaCam', {}, 'range', {}, 'bearing', {}, 'confidence', {});
obs.bump = struct('right', 0, 'left', 0, 'dropRight', 0, 'dropLeft', 0, ...
                  'dropCaster', 0, 'front', 0, 'any', false, 'valid', false);

try
    depthArray = RealSenseDist(Robot);
    depthRaw = depthArray(:).';
    obs.depthRaw = depthRaw;

    if params.dropFirstDepthSample && numel(depthRaw) > 1
        depthRanges = depthRaw(2:end);
    else
        depthRanges = depthRaw;
    end

    numDepth = numel(depthRanges);
    fovRad = deg2rad(params.depthFovDeg);
    depthAngles = linspace(0.5 * fovRad, -0.5 * fovRad, numDepth).';

    obs.depthRanges = depthRanges(:);
    obs.depthAngles = depthAngles;
    obs.validDepthMask = isfinite(obs.depthRanges) & ...
                         obs.depthRanges >= params.minDepthRange & ...
                         obs.depthRanges <= params.maxDepthRange;
catch
    % Leave the depth fields empty.
end

try
    CreatePort = getCreateSensorPort(Robot);
    [BumpRight, BumpLeft, DropRight, DropLeft, DropCaster, BumpFront] = ...
        BumpsWheelDropsSensorsRoomba(CreatePort);

    obs.bump = struct( ...
        'right', BumpRight, ...
        'left', BumpLeft, ...
        'dropRight', DropRight, ...
        'dropLeft', DropLeft, ...
        'dropCaster', DropCaster, ...
        'front', BumpFront, ...
        'any', logical(BumpRight || BumpLeft || BumpFront), ...
        'valid', true);
catch
    % Leave the bump struct in its default invalid state.
end

try
    tagMatrix = RealSenseTag(Robot);
    if ~isempty(tagMatrix)
        obs.rawTagMatrix = tagMatrix;
        obs.beacons = parseRawTagDetections(tagMatrix);
    end
catch
    % Leave the beacon fields empty.
end
end

function beacons = parseRawTagDetections(tagMatrix)
tagMatrix = double(tagMatrix);
numRows = size(tagMatrix, 1);
numCols = size(tagMatrix, 2);

template = struct('tagNum', NaN, 'latency', NaN, 'xCam', NaN, 'yCam', NaN, ...
                  'thetaCam', NaN, 'range', NaN, 'bearing', NaN, 'confidence', 1.0);
beacons = repmat(template, numRows, 1);

for i = 1:numRows
    row = tagMatrix(i, :);

    if numCols >= 5
        latency = row(1);
        tagNum = row(2);
        xCam = row(3);
        yCam = row(4);
        thetaCam = row(5);
    elseif numCols == 4
        latency = NaN;
        tagNum = row(1);
        xCam = row(2);
        yCam = row(3);
        thetaCam = row(4);
    elseif numCols == 3
        latency = NaN;
        tagNum = row(1);
        xCam = row(2);
        yCam = row(3);
        thetaCam = NaN;
    else
        continue;
    end

    beacons(i).tagNum = tagNum;
    beacons(i).latency = latency;
    beacons(i).xCam = xCam;
    beacons(i).yCam = yCam;
    beacons(i).thetaCam = thetaCam;
    beacons(i).range = hypot(xCam, yCam);
    beacons(i).bearing = atan2(yCam, xCam);
    beacons(i).confidence = 1.0;
end

validRows = arrayfun(@(b) isfinite(b.tagNum) && isfinite(b.xCam) && isfinite(b.yCam), beacons);
beacons = beacons(validRows);
end
