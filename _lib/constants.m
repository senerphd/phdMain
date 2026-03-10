%% General Parameters 
Ts = 1e-3;                  % s Sampling Time
Fs = 1/Ts;                  % Hz Sampling Frequency
%% Servo Valve Parameters 
XV_MAX     = 0.001016;                 % m Max spool position
XV_MIN     = -XV_MAX;                 % m Min spool position
W           = 2.5088e-3;                % m Orifice width
%% Hydraulic Cylinder Parameters
M           = 3.4; % kg
D_PISTON    = 34e-3;                    % Piston Diameter m
D_ROD       = 30e-3;                    % Rod Diameter m
A_PISTON    = (pi * D_PISTON^2)/4;      % Piston area m^2
A_ROD       = (pi * D_ROD^2)/4;         % Rod area m^2
AEFF        = A_PISTON - A_ROD;         % Effective Area m2
L_STROKE    = 190.16e-3;                % Total Stroke Length m
V01         = AEFF * (L_STROKE/2);      % Initial Chamber Volume in neural position m^3
V02         = AEFF * (L_STROKE/2);      % Initial Chamber Volume in neural position m^3
V_DEAD      = 1.5315e-05;                % 15.3 cm^3 dead volume <- 

C_T = 3.9933e-13;

%% Hydraulic Oil Parameters 
B_E         = 1.6e9;                    % Effective Bulk Modulus (100 ksi) Pa
RHO         = 802.25;
%% CONVERSIONS 
m3PsToLpm   = 60e3; 
PaToBar     = 1/100e3;

%% Valve Params 

K           = 0.00000334; 
Qs_at_x0    = 0.00001146; 
x0          = 0.00053428 ; 
k           = 1.82709080; 
