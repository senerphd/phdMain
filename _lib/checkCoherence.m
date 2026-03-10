function [coh, f] = checkCoherence(u,y,Ts)
Fs = 1/Ts; 
% Hanning pencere uzunluğu (ör: 4 s)
Nwin = 4*Fs;
[coh, f] = mscohere(u, y, hanning(Nwin), [], Nwin, Fs);




end 
