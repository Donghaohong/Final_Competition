function params = knownMapPlannerDefaultParams(override)
% knownMapPlannerDefaultParams Defaults for known-wall visibility planning.

params = struct();

% Clearance for the robot center relative to all known wall segments.
params.robotRadius = 0.18;
params.wallClearance = 0.17;
params.edgeClearance = params.robotRadius + params.wallClearance;
params.nodeClearance = params.edgeClearance;
params.startGoalClearance = 0.05;

% Roadmap node generation.
params.cornerOffsetRadius = params.edgeClearance + 0.12;
params.numCornerSamples = 16;
params.addGridSamples = true;
params.gridSpacing = 0.45;
params.duplicateNodeTol = 0.05;

% Edge checking and path cleanup.
params.intersectionTol = 1e-9;
params.clearanceTol = 1e-6;
params.smoothPath = true;
params.maxSegmentLength = 0.18;

% Workspace boundary inferred from the outer wall extents.
params.boundsPadding = 0.0;

if nargin < 1 || isempty(override)
    return;
end

if ~isstruct(override)
    error('override must be a struct.');
end

fn = fieldnames(override);
for i = 1:numel(fn)
    params.(fn{i}) = override.(fn{i});
end
end
