function ang = wrapToPiLocal(ang)
% wrapToPiLocal Wrap angle(s) to [-pi, pi).

ang = atan2(sin(ang), cos(ang));
end
