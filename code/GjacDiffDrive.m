function G = GjacDiffDrive(x, u)
% GjacDiffDrive: output the jacobian of the dynamics. Returns the G matrix
%
%   INPUTS
%       x            3-by-1 vector of pose
%       u            2-by-1 vector [d, phi]'
%
%   OUTPUTS
%       G            Jacobian matrix partial(g)/partial(x)
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Last, First Name
    x = x(:);
    u = u(:);

    theta = x(3);
    d     = u(1);
    phi   = u(2);

    epsPhi = 1e-6;

    G = eye(3);

    if abs(phi) < epsPhi
        G(1,3) = -d * sin(theta);
        G(2,3) =  d * cos(theta);
    else
        G(1,3) = (d/phi) * (cos(theta + phi) - cos(theta));
        G(2,3) = (d/phi) * (sin(theta + phi) - sin(theta));
    end
end