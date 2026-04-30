function pred = predictBeaconObservations(pose, beaconLoc, offset_x, offset_y, tagNums)
% predictBeaconObservations Predict camera-frame beacon observations from pose.

if nargin < 5 || isempty(tagNums)
    tagNums = beaconLoc(:, 1);
end

tagNums = tagNums(:);
pred = repmat(struct( ...
    'tagNum', NaN, ...
    'xCam', NaN, ...
    'yCam', NaN, ...
    'range', NaN, ...
    'bearing', NaN, ...
    'visible', false), numel(tagNums), 1);

poseRow = pose(:).';

for i = 1:numel(tagNums)
    tagNum = tagNums(i);
    idx = find(beaconLoc(:, 1) == tagNum, 1, 'first');
    if isempty(idx)
        continue;
    end

    xyGlobal = beaconLoc(idx, 2:3);
    xyRobot = global2robot(poseRow, xyGlobal);

    xCam = xyRobot(1) - offset_x;
    yCam = xyRobot(2) - offset_y;

    pred(i).tagNum = tagNum;
    pred(i).xCam = xCam;
    pred(i).yCam = yCam;
    pred(i).range = hypot(xCam, yCam);
    pred(i).bearing = atan2(yCam, xCam);
    pred(i).visible = (xCam > 0);
end
end
