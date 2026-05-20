%% General Parameters 
Ts = 1e-3;                  % s Sampling Time
Fs = 1/Ts;                  % Hz Sampling Frequency
%% Servo Valve Parameters 
XV_MAX     = 0.001016;                  % m Max spool position
XV_MIN     = -XV_MAX;                   % m Min spool position
W           = 2.5088e-3;                % m Orifice width
%% Hydraulic Cylinder Parameters
M           = 3.4;                      % Movable mass (Piston + Rod) kg
D_PISTON    = 34e-3;                    % Piston Diameter m
D_ROD       = 30e-3;                    % Rod Diameter m
A_PISTON    = (pi * D_PISTON^2)/4;      % Piston area m^2
A_ROD       = (pi * D_ROD^2)/4;         % Rod area m^2
AEFF        = A_PISTON - A_ROD;         % Effective Area m2
L_STROKE    = 190.16e-3;                % Total Stroke Length m
V01         = AEFF * (L_STROKE/2);      % Initial Chamber Volume in neural position m^3
V02         = AEFF * (L_STROKE/2);      % Initial Chamber Volume in neural position m^3
V_DEAD      = 1.5315e-05;               % 15.3 cm^3 dead volume <- 
V0 = V01 + V_DEAD;                      % Total Volume m3
C_T = 3.9933e-13;                       % Internal leakage constant m3/s/Pa
B_V = 28.08;                            % Viskoz sürtünme katsayısı N.s/m
F_s = 88.30;                            % Statik sürtünme değeri N 
F_c = 16.63;                            % Coulomb sürtünme değeri N 
tau = 0.03695;                          % Stribeck velocity coefficient 

eps = 1e-6;                             % Epsilon for numerical stability 
%% Hydraulic Oil Parameters 
B_E         = 1.6e9;                    % Effective Bulk Modulus (100 ksi) Pa
RHO         = 802.25;
%% CONVERSIONS 
m3PsToLpm   = 60e3; 
PaToBar     = 1/100e3;

%% Valve Params 

K           = 0.00000334; 
K_T         = K / sqrt(2); 

Qs_at_x0    = 0.00001146; 
x0          = 0.00053428 ; 
k           = 1.82709080; 


x0_S = 0.46e-3 ;  % Supply tarafı (0.46 mA) --> bunu editledim
x0_R = 0.64e-3 ;  % Return tarafı (0.64 mA)

% ------------------------------------- 
% K değeri 	    : 	 0.00000340 
% Qs_at_x0 	    : 	 0.00001133 
% x0 	 	 	: 	 0.00052030 
% k 	 	 	:  	 1.38401705 
% ------------------------------------- 