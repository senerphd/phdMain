%% runSim.m — MRAC simülasyonu çalıştır ve sonuçları kaydet
% Kullanım: test_modu = 'step' veya 'sinus' olarak ayarla, sonra çalıştır
%
% Sonuçlar: sim_results.mat dosyasına kaydedilir

%% Ortamı yükle
run('C:\_PhD\4_Tez\3_Tests\0_Modelling\phdMain\MRAC_Design\new_ForClaude\newDesign.m');

%% ---- TEST MODU SEÇ ----
% 'step' : Step referans, L_hat_IC(2)=0 (overshoot azaltma)
% 'sinus': 6Hz sinüs, L_hat_IC = Km (tam nominal kazanç)
test_modu = 'step';   % <-- BURAYI DEĞİŞTİR: 'step' veya 'sinus'
% -------------------------

%% Adaptasyon parametreleri (model içinde Constant olarak atanmış)
Gamma_L = 0.05;
Gamma_M = 0.05;

%% L_hat başlangıç koşulu: moda göre
if strcmp(test_modu, 'step')
    L_hat_IC = [Km(1), 0, Km(3)];   % hız terimi sıfır → overshoot azalır
    Constant4_val = '1';             % MultiPortSwitch: step girişi
    sim_sure = 5;                    % s
    fprintf('TEST MODU: STEP (L_hat_IC(2)=0)\n');
else
    L_hat_IC = Km;                   % tam Km → anında iyi 6Hz takibi
    Constant4_val = '2';             % MultiPortSwitch: 6Hz sinüs
    sim_sure = 10;                   % s (adaptasyonun oturması için)
    fprintf('TEST MODU: 6Hz SİNÜS (L_hat_IC = Km)\n');
end
fprintf('L_hat_IC = [%.5f, %.5f, %.5f]\n', L_hat_IC);
fprintf('Gamma_L = %.4f, Gamma_M = %.4f\n', Gamma_L, Gamma_M);

%% Modeli hazırla
load_system('NonLinModelMracScaled');

% To Workspace bloklarını Array formatına çevir (simOut'a yazmak için)
twBlocks = find_system('NonLinModelMracScaled','BlockType','ToWorkspace');
for i = 1:length(twBlocks)
    set_param(twBlocks{i},'SaveFormat','Array');
end

% Referans seçici ve süre
set_param('NonLinModelMracScaled/Constant4','Value', Constant4_val);
set_param('NonLinModelMracScaled','StopTime', num2str(sim_sure));
set_param('NonLinModelMracScaled','SaveTime','on','TimeSaveName','tout');
set_param('NonLinModelMracScaled','SaveOutput','off');

%% Simülasyonu çalıştır
fprintf('Simülasyon çalışıyor (%s, T=%.0fs)...\n', test_modu, sim_sure);
simOut = sim('NonLinModelMracScaled');
fprintf('Simülasyon tamamlandı.\n');

%% Sonuçları topla — simOut'dan al (MATLAB 2021b+: simOut.get veya simOut.varname)
% Simulink matris formatı: [signal_dim1, signal_dim2, N_time] → squeeze ile [N×dim]
t_v   = simOut.tout;                        % [N×1] s
zm_v  = squeeze(simOut.get('zm'))';         % [N×3] ref model: pos_mm, vel_mm/s, press_bar
z_v   = squeeze(simOut.get('z'))';          % [N×3] plant durumu
r_v   = simOut.get('r');                    % [N×1] referans sinyal
L_v   = squeeze(simOut.get('L_hat'))';      % [N×3] adaptif L_hat
M_v   = simOut.get('M_hat');               % [N×1] adaptif M_hat
u_v   = squeeze(simOut.get('u_mA'));        % [N×1] kontrol sinyali [mA]
fprintf('Veri boyutları: t=%dx1, zm=%dx3, z=%dx3\n', length(t_v), size(zm_v,1), size(z_v,1));

pos_ref  = zm_v(:,1);   % referans model konumu [mm]
pos_act  = z_v(:,1);    % plant konumu [mm]
e1       = pos_ref - pos_act;  % konum hatası [mm]

%% Moda özgü performans metrikleri
if strcmp(test_modu, 'step')
    % Step başlangıcını bul
    idx_step = find(abs(r_v) > 0.5, 1);
    if isempty(idx_step), idx_step = 10; end
    t0 = t_v(idx_step);
    r_final = r_v(end);

    % Step cevabını çıkar
    idx_resp = t_v >= t0;
    t_resp   = t_v(idx_resp) - t0;
    pos_resp = pos_act(idx_resp);

    % Metrikler
    overshoot_pct = (max(pos_resp) - r_final) / abs(r_final) * 100;
    ss_error      = mean(e1(end-100:end));
    rise_idx      = find(pos_resp >= 0.9*r_final, 1);
    rise_time     = t_resp(rise_idx);

    fprintf('\n=== STEP PERFORMANS ===\n');
    fprintf('Hedef: %.1f mm\n', r_final);
    fprintf('Overshoot: %.1f%%\n', overshoot_pct);
    fprintf('SS hatası: %.4f mm (sigma=0)\n', ss_error);
    fprintf('Rise time (%%90): %.1f ms\n', rise_time*1000);

    perf.overshoot_pct = overshoot_pct;
    perf.ss_error_mm   = ss_error;
    perf.rise_time_ms  = rise_time*1000;
    perf.modu          = 'step';

else
    % 6Hz sinüs: son 3 saniyeyi analiz et
    idx_ss = t_v >= (t_v(end) - 3);
    e1_ss  = e1(idx_ss);
    pos_ss = pos_act(idx_ss);
    ref_ss = pos_ref(idx_ss);

    A_ref = (max(ref_ss) - min(ref_ss)) / 2;
    A_act = (max(pos_ss) - min(pos_ss)) / 2;
    e_rms = rms(e1_ss);
    e_peak= max(abs(e1_ss));
    ss_err= mean(e1_ss);

    fprintf('\n=== 6Hz SİNÜS PERFORMANS ===\n');
    fprintf('A_ref = %.2f mm, A_act = %.2f mm\n', A_ref, A_act);
    fprintf('A_act/A_ref = %.4f (%.1f%%)\n', A_act/A_ref, 100*A_act/A_ref);
    fprintf('e1 RMS = %.4f mm\n', e_rms);
    fprintf('e1 peak= %.4f mm\n', e_peak);
    fprintf('SS bias= %.4f mm (sigma=0 → sıfıra yakın beklenir)\n', ss_err);

    perf.A_ref    = A_ref;
    perf.A_act    = A_act;
    perf.e_rms_mm = e_rms;
    perf.e_peak_mm= e_peak;
    perf.ss_bias_mm=ss_err;
    perf.modu     = 'sinus';
end

%% Kaydet
save_path = 'C:\_PhD\4_Tez\3_Tests\0_Modelling\phdMain\MRAC_Design\new_ForClaude\sim_results.mat';
save(save_path, 't_v','zm_v','z_v','r_v','L_v','M_v','u_v','e1','perf','test_modu');
fprintf('\nSonuçlar kaydedildi: sim_results.mat\n');

%% Grafik
figure('Name',['MRAC - ' upper(test_modu)], 'NumberTitle','off');
subplot(3,1,1);
plot(t_v, pos_ref, 'b--', 'LineWidth',1.2); hold on;
plot(t_v, pos_act, 'r',   'LineWidth',1.2);
legend('Ref Model','Plant'); ylabel('Konum (mm)'); title(['MRAC ' upper(test_modu) ' Testi']);
grid on;

subplot(3,1,2);
plot(t_v, e1, 'k', 'LineWidth',1); ylabel('e_1 (mm)');
title('Konum Hatası'); grid on;

subplot(3,1,3);
plot(t_v, u_v, 'b', 'LineWidth',1); ylabel('u (mA)');
xlabel('Zaman (s)'); title('Kontrol Sinyali'); grid on;
