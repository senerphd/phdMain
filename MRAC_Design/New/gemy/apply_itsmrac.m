%% FINAL WORKING MRAC DESIGN (FIXED STRUCTURE)
% Fixed Reference Model Swapped Blocks
% Optimized for Sinusoidal Tracking (Target < 2mm RMS)

clear all; clc; close all;
projectPath = 'C:\_PhD\4_Tez\3_Tests\0_Modelling\phdMain\MRAC_Design\New\gemy';
addpath(projectPath); addpath('C:\_PhD\4_Tez\3_Tests\0_Modelling\phdMain'); 
addpath('C:\_PhD\4_Tez\3_Tests\0_Modelling\phdMain\_lib');

%% 1. PHYSICAL PARAMETERS
Ps_supply = 199.85e5; P_return = 4.46e5; Ps_net = Ps_supply - P_return;
M = 3.4; AEFF = 2.0106e-04; B_V = 38.00; B_E = 1.6e9; C_T = 3.9933e-13;
L_STROKE = 190.16e-3; V_DEAD = 1.5315e-05; 
V01 = AEFF * (L_STROKE/2); V02 = V01; Ve = 2*(V01 + V_DEAD);
Kq_ref = 6.6397e-3; Ps_fg_net = 71.14e5; Kq = Kq_ref * sqrt(Ps_net / Ps_fg_net); 
Kc_ref = 3.8649e-13; Kc = Kc_ref * sqrt(205.47e5 / Ps_net); 
K = 0.00000334; k = 1.82709080; x0 = 0.00053428; XV_MAX = 0.001016; XV_MIN = -XV_MAX; W = 2.5088e-3;
RHO = 802.25; m3PsToLpm = 60e3; PaToBar = 1/100e3; Kv = 1.0;
F_s = 88.30; F_c = 16.63; tau = 0.03695; Ts = 1e-3; Fs = 1000;

% Initial conditions and dummy data
t_d = [0 100]; p_d = Ps_supply/2 * [1 1]; d_d = [0 0];
v_vars = {'xc_m','v_mPs','PA_Pa','PB_Pa','Q_return_m3Ps','r_xc_m','r_v_mPs','r_PA_Pa','r_PB_Pa','u_perc_m','PA_bar','PB_bar','PA_bar_m','PB_bar_m'};
for j=1:length(v_vars), assignin('base', v_vars{j}, timeseries(contains(v_vars{j},'P')*p_d + ~contains(v_vars{j},'P')*d_d, t_d)); end

%% 2. MRAC DESIGN
A = [0, 1, 0; 0, -B_V/M, AEFF/M; 0, -(4*B_E*AEFF)/Ve, -4*B_E*(Kc + C_T)/Ve]; 
B = [0; 0; (4*B_E/Ve)*(Kq*Kv)];
T = diag([1e-3, 1e-3, 1e5]); T_inv = inv(T);
A_bar = T_inv * A * T; B_bar_mA = (T_inv * B) * 1e-3;

% Reference Model - 1 Hz Optimized
wn = 40; zeta = 0.8;
p_ref = [-wn*zeta + 1i*wn*sqrt(1-zeta^2), -wn*zeta - 1i*wn*sqrt(1-zeta^2), -150];
Km = place(A_bar, B_bar_mA, p_ref); Am_bar = A_bar - B_bar_mA * Km;
DC_gain = [1 0 0] * inv(-Am_bar) * B_bar_mA; Bm_bar_mm = B_bar_mA / DC_gain;

P_bar = lyap(Am_bar', eye(3)); 
PB_vec = [P_bar(1,:) * B_bar_mA; 0; 0]; 

% Adaptation Gains
Gamma_L = 1e-6;   
Gamma_R = 1e-5;   
sigma_L = 0.001;  
sigma_M = 0.001;

L_hat_IC = -Km; M_hat_IC = 1 / DC_gain;
PA_IC = Ps_supply/2; PB_IC = Ps_supply/2;

% Workspace Assignment
vars_to_assign = {'PB_vec', 'Gamma_L', 'Gamma_R', 'sigma_L', 'sigma_M', 'L_hat_IC', 'M_hat_IC', 'PA_IC', 'PB_IC', 'Km', 'Am_bar', 'Bm_bar_mm','k','K','x0','XV_MAX','XV_MIN','W','RHO','F_s','F_c','tau'};
for i=1:length(vars_to_assign), assignin('base', vars_to_assign{i}, eval(vars_to_assign{i})); end

%% 3. APPLY TO SIMULINK AND SAVE
modelName = 'NonLinModelMracScaled'; load_system(modelName);

% FIX THE SWAPPED BLOCKS PERMANENTLY
set_param([modelName '/ReferenceModel/Constant'], 'Value', 'Am_bar');
set_param([modelName '/ReferenceModel/Constant1'], 'Value', 'Bm_bar_mm');

% Configure Simulation
set_param(modelName, 'FixedStep', '1e-4');
set_param([modelName '/Step'], 'After', '1');
set_param([modelName '/Sine Wave'], 'Frequency', num2str(2*pi*1.0));
set_param([modelName '/Sine Wave'], 'Amplitude', '10');

% Update Adaptation Parameters
set_param([modelName '/AdaptationMechanism/Constant6'], 'Value', 'PB_vec');
set_param([modelName '/AdaptationMechanism/Constant4'], 'Value', 'Gamma_L');
set_param([modelName '/AdaptationMechanism/Constant7'], 'Value', 'Gamma_R');
set_param([modelName '/Gain_sigL'], 'Gain', 'sigma_L');
set_param([modelName '/Gain_sigM'], 'Gain', 'sigma_M');

save_system(modelName);

%% 4. RUN SIMULATION
fprintf('\n--- Running Final MRAC Simulation (1 Hz) ---\n');
out = sim(modelName, 'StopTime', '15');

%% 5. VALIDATION
t = out.zm.Time; zm = out.zm.Data(:); 
z_data = out.z.Data; if ndims(z_data) == 3, z_data = squeeze(z_data); end
if size(z_data, 2) > 3, z_data = z_data'; end
z = z_data(:,1);

len = min([length(t), length(zm), length(z)]);
t = t(1:len); zm = zm(1:len); z = z(1:len);

idx = t > (t(end) - 5);
rms_error = rms(zm(idx) - z(idx));
max_error = max(abs(zm(idx) - z(idx)));

fprintf('RMS Error: %.4f mm\n', rms_error);
fprintf('Max Error: %.4f mm\n', max_error);
fprintf('Max Position: %.2f mm\n', max(z(idx)));

% Plot
figure('Color','w','Name','Final MRAC Result');
subplot(2,1,1);
plot(t, zm, 'b', 'LineWidth', 1.5); hold on;
plot(t, z, 'r--', 'LineWidth', 1.2);
grid on; ylabel('Position (mm)'); legend('Ref Model','Plant');
title(sprintf('1 Hz Sinusoidal Tracking | RMS: %.3f mm', rms_error));
xlim([t(end)-2 t(end)]);

subplot(2,1,2);
plot(t, zm-z, 'k'); grid on; ylabel('Error (mm)'); xlabel('Time (s)');
title('Tracking Error'); xlim([t(end)-2 t(end)]);

saveas(gcf, 'final_validated_mrac.png');
save('final_mrac_results.mat');
