function sigma = circularStdWeighted(theta, w)
% circularStdWeighted Weighted circular standard deviation in radians.

theta = theta(:);

if nargin < 2 || isempty(w)
    w = ones(size(theta));
else
    w = w(:);
end

if isempty(theta) || isempty(w) || numel(theta) ~= numel(w)
    sigma = inf;
    return;
end

w = max(w, 0);
wSum = sum(w);
if wSum <= 0
    sigma = inf;
    return;
end

w = w / wSum;

C = sum(w .* cos(theta));
S = sum(w .* sin(theta));
R = hypot(C, S);
R = min(max(R, 1e-12), 1.0);

sigma = sqrt(max(0, -2 * log(R)));
end
