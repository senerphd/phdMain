%% OPTIMIZING MRAC FOR 1HZ SINE
clear all; clc; close all;
projectPath = 'C:\_PhD\4_Tez\3_Tests\0_Modelling\phdMain\MRAC_Design\New\gemy';
addpath(projectPath); addpath('C:\_PhD\4_Tez\3_Tests\0_Modelling\phdMain'); 
addpath('C:\_PhD\4_Tez\3_Tests\0_Modelling\phdMain\_lib');

% PHYSICAL PARAMETERS
Ps_supply = 199.85e5; P_return = 4.46e5; Ps_net = Ps_supply - P_return;
M = 3.4; AEFF = 2.0106e-04; B_V = 38.00; B_E = 1.6e9; C_T = 3.9933e-13;
L_STROKE = 190.16e-3; V_DEAD = 1.5315e-05; 
V01 = AEFF * (L_STROKE/2); V02 = V01; Ve = 2*(V01 + V_DEAD);
Kq_ref = 6.6397e-3; Ps_fg_net = 71.14e5; Kq = Kq_ref * sqrt(Ps_net / Ps_fg_net); 
Kc_ref = 3.8649e-13; Kc = Kc_ref * sqrt(205.47e5 / Ps_net); 
K = 0.00000334; k = 1.82709080; x0 = 0.00053428; XV_MAX = 0.001016; XV_MIN = -XV_MAX; W = 2.5088e-3;
RHO = 802.25; m3PsToLpm = 60e3; PaToBar = 1/100e3; Kv = 1.0;
F_s = 88.30; F_c = 16.63; tau = 0.03695; Ts = 1e-3; Fs = 1000;

% DUMMY DATA
t_d = [0 100]; p_d = Ps_supply/2 * [1 1]; d_d = [0 0];
v_vars = {'xc_m','v_mPs','PA_Pa','PB_Pa','Q_return_m3Ps','r_xc_m','r_v_mPs','r_PA_Pa','r_PB_Pa','u_perc_m','PA_bar','PB_bar','PA_bar_m','PB_bar_m'};
for j=1:length(v_vars), assignin('base', v_vars{j}, timeseries(contains(v_vars{j},'P')*p_d + ~contains(v_vars{j},'P')*d_d, t_d)); end

% DESIGN
A = [0, 1, 0; 0, -B_V/M, AEFF/M; 0, -(4*B_E*AEFF)/Ve, -4*B_E*(Kc + C_T)/Ve]; 
B = [0; 0; (4*B_E/Ve)*(Kq*Kv)];
T = diag([1e-3, 1e-3, 1e5]); T_inv = inv(T);
A_bar = T_inv * A * T; B_bar_mA = (T_inv * B) * 1e-3;

% Tune Reference Model for better tracking of 1Hz
wn = 45; zeta = 0.8;
p_ref = [-zeta*wn + 1i*wn*sqrt(1-zeta^2), -zeta*wn - 1i*wn*sqrt(1-zeta^2), -150];
Km = place(A_bar, B_bar_mA, p_ref); Am_bar = A_bar - B_bar_mA * Km;
DC_gain = [1 0 0] * inv(-Am_bar) * B_bar_mA; Bm_bar_mm = B_bar_mA / DC_gain;
P_bar = lyap(Am_bar', eye(3)); 
PB_vec = [P_bar(1,:) * B_bar_mA; 0; 0]; 

L_hat_IC = -Km; M_hat_IC = 1 / DC_gain;
PA_IC = Ps_supply/2; PB_IC = Ps_supply/2;

% SWEEP ADAPTATION GAINS
gammas = [1e-6, 5e-6, 1e-5];
best_rms = inf; best_gamma = 0;

modelName = 'NonLinModelMracScaled'; load_system(modelName);
set_param(modelName, 'FixedStep', '1e-4');
set_param([modelName '/Sine Wave'], 'Frequency', num2str(2*pi*1.0));
set_param([modelName '/Sine Wave'], 'Amplitude', '10');
set_param([modelName '/Gain_sigL'], 'Gain', '0.001');
set_param([modelName '/Gain_sigM'], 'Gain', '0.001');

for g = gammas
    Gamma_L = g; Gamma_R = g*10;
    assignin('base', 'Gamma_L', Gamma_L);
    assignin('base', 'Gamma_R', Gamma_R);
    set_param([modelName '/AdaptationMechanism/Constant4'], 'Value', 'Gamma_L');
    set_param([modelName '/AdaptationMechanism/Constant7'], 'Value', 'Gamma_R');
    
    try
        out = sim(modelName, 'StopTime', '10');
        t = out.zm.Time; zm = out.zm.Data(:);
        z_raw = squeeze(out.z.Data); if size(z_raw,2)>3, z_raw=z_raw'; end
        z = z_raw(:,1);
        idx = t > (t(end) - 3);
        curr_rms = rms(zm(idx)-z(idx));
        fprintf('Gamma: %.1e, RMS: %.4f mm\n', g, curr_rms);
        if curr_rms < best_rms
            best_rms = curr_rms; best_gamma = g;
        end
    catch
        fprintf('Gamma: %.1e failed\n', g);
    end
end

fprintf('Best Gamma: %.1e, Best RMS: %.4f mm\n', best_gamma, best_rms);
save('best_mrac_params.mat', 'best_gamma', 'best_rms', 'wn', 'zeta');
