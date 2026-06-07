%% MRAC FINAL SOLUTION AND FREQUENCY SWEEP - FIXED
% Tüm parametreleri (V02 dahil) ve bağımlılıkları tanımlar.

clear all; clc; close all;
projectPath = 'C:\_PhD\4_Tez\3_Tests\0_Modelling\phdMain\MRAC_Design\New\gemy';
addpath(projectPath); addpath('C:\_PhD\4_Tez\3_Tests\0_Modelling\phdMain'); 
addpath('C:\_PhD\4_Tez\3_Tests\0_Modelling\phdMain\_lib');

%% 1. FİZİKSEL PARAMETRELER (SI)
Ps_supply = 199.85e5; P_return = 4.46e5; Ps_net = Ps_supply - P_return;
M = 3.4; AEFF = 2.0106e-04; B_V = 38.00; B_E = 1.6e9; C_T = 3.9933e-13;
L_STROKE = 190.16e-3; V_DEAD = 1.5315e-05; 
V01 = AEFF * (L_STROKE/2); V02 = V01; Ve = 2*(V01 + V_DEAD);
Kq_ref = 6.6397e-3; Ps_fg_net = 71.14e5; Kq = Kq_ref * sqrt(Ps_net / Ps_fg_net); 
Kc_ref = 3.8649e-13; Kc = Kc_ref * sqrt(205.47e5 / Ps_net); 
K = 0.00000334; k = 1.82709080; x0 = 0.00053428; XV_MAX = 0.001016; XV_MIN = -XV_MAX; W = 2.5088e-3;
RHO = 802.25; m3PsToLpm = 60e3; PaToBar = 1/100e3; Kv = 1.0;
F_s = 88.30; F_c = 16.63; tau = 0.03695; Ts = 1e-3; Fs = 1000;

%% 2. DUMMY VERİLER
t_d = [0 100]; d_d = [0 0]; p_d = Ps_supply/2 * [1 1];
vars = {'xc_m','v_mPs','PA_Pa','PB_Pa','Q_return_m3Ps','r_xc_m','r_v_mPs','r_PA_Pa','r_PB_Pa','u_perc_m','PA_bar','PB_bar','PA_bar_m','PB_bar_m'};
for i=1:length(vars)
    if contains(vars{i}, 'P'), val = p_d; else, val = d_d; end
    assignin('base', vars{i}, timeseries(val, t_d));
end

%% 3. MRAC TASARIMI (mm, mm/s, bar)
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

Gamma_L = 1e-10; Gamma_R = 1e-9; sigma_L = 0.05; sigma_M = 0.05;
L_hat_IC = -Km; M_hat_IC = 1 / DC_gain; PA_IC = Ps_supply/2; PB_IC = Ps_supply/2;

%% 4. FREKANS SWEEP
freqs = [0.5, 1.0, 2.0]; results = struct();
modelName = 'NonLinModelMracScaled'; load_system(modelName);
set_param(modelName, 'FixedStep', '1e-4');
set_param([modelName '/AdaptationMechanism/Constant6'], 'Value', 'PB_vec');
set_param([modelName '/AdaptationMechanism/Constant4'], 'Value', 'Gamma_L');
set_param([modelName '/AdaptationMechanism/Constant7'], 'Value', 'Gamma_R');
set_param([modelName '/Gain_sigL'], 'Gain', 'sigma_L');
set_param([modelName '/Gain_sigM'], 'Gain', 'sigma_M');

fprintf('\n--- MRAC Final Sweep Basliyor ---\n');
for i = 1:length(freqs)
    f = freqs(i); set_param([modelName '/Sine Wave'], 'Frequency', num2str(2*pi*f));
    set_param([modelName '/Sine Wave'], 'Amplitude', '10');
    simTime = 10; fprintf('Test: %.1f Hz...', f);
    try
        out = sim(modelName, 'StopTime', num2str(simTime));
        t = out.zm.Time; zm = out.zm.Data(:);
        z_raw = squeeze(out.z.Data); if size(z_raw,2)>3, z_raw=z_raw'; end
        z = z_raw(:,1);
        len = min([length(t), length(zm), length(z)]);
        idx = t(1:len) > (t(len) - 3);
        results(i).freq = f; results(i).rms = rms(zm(idx)-z(idx));
        results(i).t = t(1:len); results(i).zm = zm(1:len); results(i).z = z(1:len);
        fprintf(' RMS Hata: %.3f mm\n', results(i).rms);
    catch ME
        fprintf(' Hata: %s\n', ME.message);
    end
end

%% 5. RAPOR
figure('Color', 'w', 'Position', [100 100 1000 600]);
for i = 1:length(results)
    if isfield(results(i), 't')
        subplot(length(results), 1, i);
        plot(results(i).t, results(i).zm, 'b', results(i).t, results(i).z, 'r--');
        title(['Freq: ', num2str(results(i).freq), ' Hz | RMS: ', num2str(results(i).rms), ' mm']);
        grid on; xlim([results(i).t(end)-2, results(i).t(end)]);
    end
end
save('mrac_final_results_fixed.mat', 'results');
