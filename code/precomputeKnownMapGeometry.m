function mapGeom = precomputeKnownMapGeometry(map, wallThickness)
% precomputeKnownMapGeometry Build thick-wall polygons from centerline segments.

if isempty(map) || size(map, 2) ~= 4
    error('map must be an N x 4 matrix of wall centerlines.');
end

numWalls = size(map, 1);
mapGeom = struct();
mapGeom.map = map;
mapGeom.wallThickness = wallThickness;
mapGeom.polygons = cell(numWalls, 1);
mapGeom.edges = cell(numWalls, 1);

halfT = 0.5 * wallThickness;

for i = 1:numWalls
    p1 = map(i, 1:2);
    p2 = map(i, 3:4);

    d = p2 - p1;
    L = norm(d);
    if L < 1e-12
        poly = repmat(p1, 4, 1);
    else
        tangent = d / L;
        normal = [-tangent(2), tangent(1)];
        poly = [ ...
            p1 + halfT * normal; ...
            p2 + halfT * normal; ...
            p2 - halfT * normal; ...
            p1 - halfT * normal];
    end

    edges = [poly, poly([2 3 4 1], :)];

    mapGeom.polygons{i} = poly;
    mapGeom.edges{i} = edges;
end
end
