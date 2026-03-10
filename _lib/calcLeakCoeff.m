function Ct_est = calcLeakCoeff

% Objective function: RMS error between measured and estimated velocity
objFun = @(Ct) rms(y_dot - ...
    ( (lsim(delayTf, Kq .* x_v .* sqrt(P_supply - (sign(x_v).*P_L)) - Ct .* P_L, t)) / Aeff )) ...
    + abs(mean(y_dot - y_dot_est));   % offset cezası

% Initial guess for Ct
Ct0 = 1e-12;  % m^3/(s·Pa), tipik başlangıç
% Lower and upper bounds (gerekirse ayarlayabilirsin)
lb = 1e-13;       
ub = 1e-7;    

options = optimset('Display','iter');

Ct_est = fminsearch(objFun, 1e-10, options);
end 
