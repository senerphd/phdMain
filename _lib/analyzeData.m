% Calculated Parameters

% Conversion Constants 
m3PsToLpm   = 60e3; 
barToPa     = 1e5; 
barToPsi    = 14.5037738; 

PaToBar = 1/barToPa; 


L_stroke    = 190.17e-3;                % Total Stroke Length m 
m           = 3.4;                      % kg (Piston + Rod : 3.5 kg) 

x_v_max     = 0.001016;                 % m Max spool position
x_v_min     = -x_v_max;                 % m Min spool position

i_max = 50e-3; 
Ki          = x_v_max / i_max;          % m/A Spool gain, assuming spool position is linear with current
D_piston    = 34e-3;                    % Piston Diameter m
D_rod       = 30e-3;                    % Rod Diameter m
A_piston    = (pi * D_piston^2)/4;      % Piston area m^2
A_rod       = (pi * D_rod^2)/4;         % Rod area m^2
Aeff        = A_piston - A_rod;         % Effective Area m2

C_d         = 0.7;                      % Flow discharge coefficient 
w           = 1.8538e-3;                % Orifice window length m
rho         = 854.56;                      % kg/m3; 
    
Kq          = C_d * w * sqrt(2/rho);    % m2/s Flow Gain wrt spool position per DeltaP
Kt          = Kq*Ki;                    % m3/s/A per DeltaP 



C_t        = 1.000000000000046e-11;    % Servo-valve leak coefficient used in Q_L m³/(s·Pa)
% C_t          = 8.0423e-12;               % Servo-valve leak coefficient used in Q_L m³/(s·Pa)
B_e          = 1.6e9;          % Effective Bulk Modulus (100 ksi) Pa

V01         = Aeff * (L_stroke/2);     % Initial Chamber Volume in neural position m^3
V02         = Aeff * (L_stroke/2);     % Initial Chamber Volume in neural position m^3

%% 

% Options   
s = tf('s'); 
filter = false; 
windowTime = 2;

includeServoDynamics = false;
includeDelay = false; 
delayTime = 0.0;
delayTf = exp(-delayTime * s); %10 ms delay

checkCoher = false;


%
if includeServoDynamics
    % Sistem tanımı
    f_cutoff = 120;
    wn = 2*pi*f_cutoff;
    zeta = 0.707;
    s = tf('s');
    if includeDelay
        H = (wn^2 / (s^2 + 2*zeta*wn*s + wn^2)) * delayTf ;
    else
        H = (wn^2 / (s^2 + 2*zeta*wn*s + wn^2));
    end
    x_v = lsim(H,u,t,u(1)) .* (x_v_max/i_max);
else
    if includeDelay
        x_v         = Ki * u_perc/100 * 50e-3;
        x_v = lsim(delayTf,x_v,t,x_v    (1));
    else
        x_v         = Ki * u_perc/100 * 50e-3;                   % Spool Position m
    end
end


%
if filter
    windowLength = round(windowTime / Ts);
    sgOrder = 3;
    if mod(windowLength, 2) == 0, windowLength = windowLength + 1; end
    % r   = sgolayfilt(r, sgOrder, windowLength);
    % u   = sgolayfilt(u, sgOrder, windowLength);
    % y   = sgolayfilt(y, sgOrder, windowLength);
    Q_return   = sgolayfilt(Q_return, sgOrder, windowLength);
    P_A = sgolayfilt(P_A, sgOrder, windowLength);
    P_B = sgolayfilt(P_B, sgOrder, windowLength);
    P_L = sgolayfilt(P_L, sgOrder, windowLength);
    P_supply = sgolayfilt(P_supply, sgOrder, windowLength);
    P_return = sgolayfilt(P_return, sgOrder, windowLength);
    % e = sgolayfilt(e, sgOrder, windowLength);
    % xc_dot = sgolayfilt(xc_dot, sgOrder, windowLength);
end

%% Plot Test Data (Position & Current) 
figure; 
subplot(2,1,1)
plot(t,r,t,xc); 
legend('r','xc'); 
xlabel('Time (s)')
ylabel('Position (m)');
grid on 


subplot(2,1,2)
plot(t,u_perc); 
legend('u,y'); 
xlabel('Time (s)')
ylabel('Applied Current (%)');
grid on 

%% Check Input - Output Coherence
if checkCoher 
    [coh, f] = checkCoherence(u,y,Ts); 

    figure;
    semilogx(f, coh, 'LineWidth', 1.5);
    grid on; ylim([0 1]);
    xlabel('Frequency (Hz)'); ylabel('Coherence');
    title('Input-Output Coherence');
end
%% Load flow calculation Q_L -> Tek denklem
close all 

% Total flow
Q_L = (Kq) .* x_v .* sqrt(P_supply - (sign(x_v).* P_L)) ; % Kq = Cd * w * sqrt(2/rho) geliyor. 
Q_L = Q_L + C_t.* P_L; 


xc_dot_est_Q_L = Q_L/Aeff; 

figure
plot(t,xc_dot,t,xc_dot_est_Q_L)
grid on
legend('y_{dot} measured','y_{dot} Q_L')

%% Servo valve flows (nonlinear orifice eqns.) -> İki denklem
close all 
% Switch (step) selectors: 1 veya 0


s_pos = double(x_v >= 0);
s_neg = 1 - s_pos;   % u < 0

Q1 = Kq .* x_v .* (s_pos .* sqrt(max(P_supply - P_A,0)) +  s_neg.* sqrt(max(P_A - P_return,0)));   % [m^3/s] flow to chamber A
Q2 = Kq .* x_v .* (s_pos .* sqrt(max(P_B - P_return,0)) +  s_neg.* sqrt(max(P_supply - P_B,0)));   % [m^3/s] flow to chamber B
Q_L_model = (Q1+Q2)/2;

Q_L_leak_calc = -C_t .* P_L; 
Q_L_model = Q_L_model + Q_L_leak_calc; 

Q_L_meas = xc_dot*Aeff; % m3/s
figure
plot(t,Q_L_meas * 60e3 ,t,Q_L_model * 60e3)
grid on
legend('Q_L measured (lpm)','Q_L_model (lpm)')


% Fark
differ = Q_L_meas - Q_L_model;
meanErr = mean(differ); 

% RMS hata
rmsErr = rms(differ);

% Sonuçları yazdır
fprintf('RMS error between Q_L_meas and Q_L_model: %g [m^3/s]\n', rmsErr);
fprintf('Mean error between Q_L_meas and Q_L_model: %g [m^3/s]\n', meanErr);
%% === Pressure dynamics =====================================

% Chamber volumes
V1 = V01 + Aeff * xc;   % [m^3] chamber A volume
V2 = V02 - Aeff * xc;   % [m^3] chamber B volume
Vt = V1 + V2;          % [m^3] total volume

% Chamber pressures
P1 = P_A;              % Ölçülen chamber A pressure (Pa)
P2 = P_B;              % Ölçülen chamber B pressure (Pa)
PL = P1 - P2;          % Load pressure (Pa)

Q_leak = C_t .* P_L; 

% Chamber pressure dynamics
dP1 = (B_e ./ V1) .* (Q1 - Aeff .* xc_dot - Q_leak);  % [Pa/s]
dP2 = (B_e ./ V2) .* (Q2 + Aeff .* xc_dot + Q_leak);  % [Pa/s]

% Alternative: load pressure dynamic
dPL = (4*B_e ./ Vt) .* ( (Q1+Q2)/2 - Aeff .* xc_dot - C_t .* PL ); % [Pa/s]

%% 3. Initialize
N  = length(t);
dt = 1e-3;
P1_model = zeros(N,1);
P2_model = zeros(N,1);


% initial condition (ölçümle başlatıyoruz)
P1_model(1) = P_A(1);
P2_model(1) = P_B(1);

%% 4. Time marching (Euler integration)
for k = 1:N-1
    % Volumes
    V1 = V01 + Aeff * xc(k);
    V2 = V02 - Aeff * xc(k);

    % Load pressure (model)
    PL = P1_model(k) - P2_model(k);

    % Flows (Q1,Q2) spool ve model basınçlarına göre
    s_pos = double(x_v(k) >= 0);
    s_neg = 1 - s_pos;
    Q1 = Kq .* x_v(k) .* (s_pos.*sqrt(max(P_supply - P1_model(k),0)) + ...
                          s_neg.*sqrt(max(P1_model(k) - P_return,0)));
    Q2 = Kq .* x_v(k) .* (s_pos.*sqrt(max(P2_model(k) - P_return,0)) + ...
                          s_neg.*sqrt(max(P_supply - P2_model(k),0)));

    % Leakage
    Q_leak = C_t * PL;

    % Pressure dynamics (ODE)
    dP1 = (B_e / V1) * (Q1 - Aeff*xc_dot(k) - Q_leak);
    dP2 = (B_e / V2) * (Q2 + Aeff*xc_dot(k) + Q_leak);

    % Integrate
    P1_model(k+1) = P1_model(k) + dP1(k)*dt;
    P2_model(k+1) = P2_model(k) + dP2(k)*dt;
end

% 5. Compare with measured data
figure; 
plot(t,P1/1e5,t,P2/1e5)
grid on; 
legend('P1 (bar)','P2 (bar)')

figure;
subplot(2,1,1)
plot(t, P_A/1e5, 'b', 'LineWidth',1.5); hold on;
plot(t, P1_model/1e5, 'r--', 'LineWidth',1.5);
xlabel('Time [s]'); ylabel('P1 [bar]');
legend('Measured','Model');
grid on;

subplot(2,1,2)
plot(t, P_B/1e5, 'b', 'LineWidth',1.5); hold on;
plot(t, P2_model/1e5, 'r--', 'LineWidth',1.5);
xlabel('Time [s]'); ylabel('P2 [bar]');
legend('Measured','Model');
grid on;

% 6. Error metric
errP1 = rms(P_A - P1_model);
errP2 = rms(P_B - P2_model);
fprintf('RMS Error: P1 = %.2f bar, P2 = %.2f bar\n', errP1/1e5, errP2/1e5);


%% === Mechanical dynamics ==================================
F_hyd  = Aeff .* PL;                        % [N] hydraulic force
F_fric = Bp .* xc_dot + Fc .* sign(xc_dot);   % [N] friction (viscous + Coulomb)
y_ddot = (F_hyd - F_fric) ./ m;             % [m/s^2] piston acceleration

%% 


