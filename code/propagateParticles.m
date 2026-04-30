function particlesNew = propagateParticles(particles, odom, params)
% propagateParticles Differential-drive propagation using odometry increments.

if isempty(particles)
    particlesNew = particles;
    return;
end

if isstruct(odom)
    d = odom.d;
    phi = odom.phi;
else
    d = odom(1);
    phi = odom(2);
end

N = size(particles, 1);
theta = particles(:, 3);

sigmaD = params.odomSigmaDBase + params.odomSigmaDPerMeter * abs(d);
sigmaPhi = params.odomSigmaPhiBase + params.odomSigmaPhiPerRad * abs(phi);

dSample = d + sigmaD * randn(N, 1);
phiSample = phi + sigmaPhi * randn(N, 1);

xNew = particles(:, 1);
yNew = particles(:, 2);

smallTurn = abs(phiSample) < 1e-6;

xNew(smallTurn) = xNew(smallTurn) + dSample(smallTurn) .* cos(theta(smallTurn));
yNew(smallTurn) = yNew(smallTurn) + dSample(smallTurn) .* sin(theta(smallTurn));

turnIdx = ~smallTurn;
if any(turnIdx)
    R = dSample(turnIdx) ./ phiSample(turnIdx);
    xNew(turnIdx) = xNew(turnIdx) + R .* (sin(theta(turnIdx) + phiSample(turnIdx)) - sin(theta(turnIdx)));
    yNew(turnIdx) = yNew(turnIdx) + R .* (-cos(theta(turnIdx) + phiSample(turnIdx)) + cos(theta(turnIdx)));
end

xNew = xNew + params.processXYJitter * randn(N, 1);
yNew = yNew + params.processXYJitter * randn(N, 1);
thetaNew = wrapToPiLocal(theta + phiSample + params.processThetaJitter * randn(N, 1));

particlesNew = [xNew, yNew, thetaNew];
end
