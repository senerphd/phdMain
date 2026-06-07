%% MRAC DEBUG - 0.5 Hz Trajectory Analysis (Fixed Names)
clear all; clc; close all;
projectPath = 'C:\_PhD\4_Tez\3_Tests\0_Modelling\phdMain\MRAC_Design\New\gemy';
addpath(projectPath); addpath('C:\_PhD\4_Tez\3_Tests\0_Modelling\phdMain'); 
addpath('C:\_PhD\4_Tez\3_Tests\0_Modelling\phdMain\_lib');

%% 1. PARAMETRELER
Ts = 1e-3; Fs = 1000;
Ps_supply = 199.85e5; P_return = 4.46e5; Ps_net = Ps_supply - P_return;
M = 3.4; AEFF = 2.0106e-04; B_V = 38.00; B_E = 1.6e9; C_T = 3.9933e-13;
L_STROKE = 190.16e-3; V_DEAD = 1.5315e-05; 
V01 = AEFF * (L_STROKE/2); V02 = V01; Ve = 2*(V01 + V_DEAD);
Kq_ref = 6.6397e-3; Ps_fg_net = 71.14e5; Kq = Kq_ref * sqrt(Ps_net / Ps_fg_net); 
Kc_ref = 3.8649e-13; Kc = Kc_ref * sqrt(205.47e5 / Ps_net); 
K = 0.00000334; k = 1.82709080; x0 = 0.00053428; XV_MAX = 0.001016; XV_MIN = -XV_MAX; W = 2.5088e-3;
RHO = 802.25; m3PsToLpm = 60e3; PaToBar = 1/100e3; Kv = 1.0;
F_s = 88.30; F_c = 16.63; tau = 0.03695;

% Dummy data
t_d = [0 100]; d_d = [0 0]; p_d = Ps_supply/2 * [1 1];
v_vars = {'xc_m','v_mPs','PA_Pa','PB_Pa','Q_return_m3Ps','r_xc_m','r_v_mPs','r_PA_Pa','r_PB_Pa','u_perc_m','PA_bar','PB_bar','PA_bar_m','PB_bar_m'};
for j=1:length(v_vars), assignin('base', v_vars{j}, timeseries(contains(v_vars{j},'P')*p_d + ~contains(v_vars{j},'P')*d_d, t_d)); end

%% 2. TASARIM
A = [0, 1, 0; 0, -B_V/M, AEFF/M; 0, -(4*B_E*AEFF)/Ve, -4*B_E*(Kc + C_T)/Ve]; 
B = [0; 0; (4*B_E/Ve)*(Kq*Kv)];
T = diag([1e-3, 1e-3, 1e5]); T_inv = inv(T);
A_bar = T_inv * A * T; B_bar_mA = (T_inv * B) * 1e-3;
wn = 40; zeta = 0.707;
p_ref = [-zeta*wn + 1i*wn*sqrt(1-zeta^2), -zeta*wn - 1i*wn*sqrt(1-zeta^2), -150];
Km = place(A_bar, B_bar_mA, p_ref); Am_bar = A_bar - B_bar_mA * Km;
DC_gain = [1 0 0] * inv(-Am_bar) * B_bar_mA; Bm_bar_mm = B_bar_mA / DC_gain;
P_bar = lyap(Am_bar', eye(3)); 
PB_vec = [P_bar(1,:) * B_bar_mA; 0; 0]; 
Gamma_L = 1e-10; Gamma_R = 1e-9; sigma_L = 0.01; sigma_M = 0.01;
L_hat_IC = -Km; M_hat_IC = 1 / DC_gain; PA_IC = Ps_supply/2; PB_IC = Ps_supply/2;

%% 3. SİMÜLASYON
modelName = 'NonLinModelMracScaled'; load_system(modelName);
set_param(modelName, 'FixedStep', '1e-4');
set_param([modelName '/Sine Wave'], 'Frequency', num2str(2*pi*0.5));
set_param([modelName '/Sine Wave'], 'Amplitude', '10');
set_param([modelName '/AdaptationMechanism/Constant6'], 'Value', 'PB_vec');
set_param([modelName '/AdaptationMechanism/Constant4'], 'Value', 'Gamma_L');
set_param([modelName '/AdaptationMechanism/Constant7'], 'Value', 'Gamma_R');
set_param([modelName '/Gain_sigL'], 'Gain', 'sigma_L');
set_param([modelName '/Gain_sigM'], 'Gain', 'sigma_M');

fprintf('Debug Sim basliyor (0.5 Hz)...\n');
out = sim(modelName, 'StopTime', '15');

%% 4. PLOT
t = out.zm.Time; zm = out.zm.Data(:); 
z_data = out.z.Data; if ndims(z_data) == 3, z_data = squeeze(z_data); end
if size(z_data, 2) > 3, z_data = z_data'; end
z = z_data(:,1); u = squeeze(out.u_mA.Data);
L = squeeze(out.L_hat.Data); M = squeeze(out.M_hat_out.Data);

len = min([length(t), length(zm), length(z), length(u), length(L), length(M)]);
t=t(1:len); zm=zm(1:len); z=z(1:len); u=u(1:len); L=L(1:len,:); M=M(1:len);

figure('Name','MRAC Debug','Color','w','Position',[50 50 1000 900]);
subplot(4,1,1); plot(t, zm, 'b', t, z, 'r--'); grid on; legend('zm','z'); title('Tracking');
subplot(4,1,2); plot(t, zm-z); grid on; title('Error (mm)');
subplot(4,1,3); plot(t, u); grid on; title('Control (mA)');
subplot(4,1,4); plot(t, L); hold on; plot(t, M, '--k'); grid on; title('Adaptive Gains');
saveas(gcf, 'mrac_debug_plot.png');
fprintf('Bitti. Max Error: %.2f mm\n', max(abs(zm-z)));
