%%
clear all 
clc
close all 
addpath("C:\_PhD\4_Tez\3_Tests\0_Modelling\phdMain");
run("C:\_PhD\4_Tez\3_Tests\0_Modelling\phdMain\phdShell.m");
%%
simTime = 30; % s
%% PARAMETRELER
d_p     = 34e-3;
d_r     = 30e-3;
Ac     = pi/4*(d_p^2 - d_r^2); % Efektif alan [m^2]
V_dead  = 1.5315e-5;
V_0     = Ac*(190.16e-3/2) + V_dead;

m      = 3.4;                   % Piston rotu eşdeğer kütlesi [kg]
Bp     = 38;                    % Viskoz sönümleme katsayısı [N/(m/s)]
beta_e = 1.6e9;                 % Hidrolik yağın efektif elastikiyet (bulk) modülü [Pa]
Ve     = 2*V_0;                 % Efektif hacim [m^3]
Cip    = 3.9933e-13;            % Silindir içi kaçak katsayısı [(m^3/s)/Pa]
%% Servo valf parametreleri test'ten gelen 
% Referans test basınçları
Ps_fg_net   = 1032 * 6894.75729;           % Flow gain test net ΔP [Pa]  (71.14 bar)
Ps_pg_net   = 2980 * 6894.75729;           % Pressure gain test net Ps [Pa] (205.47 bar)

% Gerçek sistem basıncı — dinamik test verisinden ölçüldü (20260107)
Ps_supply   = 199.85e5;                    % Besleme basıncı  [Pa]  (≈200 bar)
P_return    = 4.46e5;                      % Return hattı     [Pa]  (≈4.46 bar)
Ps_net      = Ps_supply - P_return;        % Efektif net basınç [Pa] (≈195.39 bar)

% Referans değerler (statik valf testlerinden)
Kq_ref      = 6.6397e-3;                  % @ 71.14 bar net  [m³/(s·A)]
Kc_ref      = 3.8649e-13;                 % @ 205.47 bar net [m³/(s·Pa)]
%   Kc_ref: ±2 mA pressure gain regresyonu (R²=0.825)
%   Önceki (±5 mA, R²=0.62): 7.9179e-13  ← doyum bölgesi dahil, güvenilmez

% Sistem basıncına ölçekle -> ~200 bar çalışma noktası için 
Kq          = Kq_ref * sqrt(Ps_net  / Ps_fg_net);   % [m³/(s·A)]
Kc          = Kc_ref * sqrt(Ps_pg_net / Ps_net);    % [m³/(s·Pa)]

Kv          = 1.0;                         % Valf kazancı (Kq'ya absorb edildi)
%% Sistemin Lineer Dinamik Modelinin Oluşturulması 

A = [0,     1,                      0; 
     0,     -Bp/m,                  Ac/m; 
     0,     -(4*beta_e*Ac)/Ve,      -4*beta_e*(Kc + Cip)/Ve]; 

B = [0; 0; (4*beta_e/Ve)*(Kq*Kv)];

%% Ölçeklendirilmiş uzaya geçiş 
T = diag([1e-3,1e-3,1e5]); %mm,mm,bar
T_inv = inv(T);

%% Ölçeklendirilmiş lineer dinamik sistem modeli 
A_bar = T_inv * A * T; 
B_bar = T_inv * B; 
B_bar_mA = B_bar .* 1e-3; 

%% Ölçeklendirilmiş uzayda referans model 
run("placePoles.m"); 

%% Lyapunov denkleminin yazılıp ölçekli uzayda P_bar'ın bulunması
% Q = eye(3) seçimi: P_bar genel amaçlı Lyapunov çözümü.
% NOT: Bu sistemde B_bar_mA = [0; 0; b3] yapısı nedeniyle
% e'*P*B ifadesinde e3 (basınç hatası) her zaman baskındır (P33/P13 ~ 770).
% Bu yüzden adaptasyon scalarTerm'i sadece konum hatası e1 üzerinden
% hesaplanır (bkz. Gamma/scalarTerm bölümü).
Q_bar = eye(3);
P_bar = lyap(Am_bar', Q_bar);

%% Adaptif Kazanç Başlangıç Koşulları
% u = -L_hat'*z + M_hat*r  (Controller/Gain = -1)
% L_hat_IC = -Km: nominal geri besleme kazancını sağlar (u_0 = Km*z)
% M_hat_IC = 1/DC_gain: nominal feedforward (referans modelin DC kazancı = 1)
L_hat_IC = -Km;                          % [1x3], NonLinModelMracScaled/Integrator3 IC
DC_gain  = C_pos * inv(-Am_bar) * B_bar_mA;  % referans modelin DC kazancı [mm/mA]
M_hat_IC = 1 / DC_gain;                  % nominal feedforward kazancı [mA/mm]

%% Adaptasyon Kazançları ve scalarTerm Seçimi
%
% Teorik: scalarTerm = e' * P_bar * B_bar_mA  (tam Lyapunov türevi)
% Pratik sorun: B_bar_mA = [0; 0; b3] olduğundan
%   scalarTerm = b3 * (P13*e1 + P23*e2 + P33*e3)
%   P33/P13 ~ 770 => e3 (basınç hatası) adaptasyonu domine eder.
%   Plant hareket etmediğinde z3 >> zm3 => e3 < 0 => scalarTerm < 0
%   => M_hat azalır, u küçülür => kısır döngü (plant yine hareket etmez).
%
% Çözüm: scalarTerm'de sadece e1 (konum hatası) kanalını kullan:
%   scalarTerm = e' * [P_bar(1,:)*B_bar_mA; 0; 0]
%              = e1 * P13 * b3
% Simulink'te AdaptationMechanism/Constant2 = [P_bar(1,:)*B_bar_mA; 0; 0]
%
% Adaptasyon yasaları:
%   L_hat_dot = -Gamma_L * scalarTerm * z    [AdaptMech/Gain=-1, Constant=Gamma_L]
%   M_hat_dot = +Gamma_R * scalarTerm * r    [AdaptMech/Constant3=Gamma_R, pozitif]
%
% Ayar rehberi: scalarTerm ~ e1 * 126 (e1=5mm => ~630)
%   L_hat_dot ~ Gamma_L * 630 * z1 => 1s'de ~1e-3 değişim için Gamma_L ~ 1e-6
%   M_hat_dot ~ Gamma_R * 630 * r  => 1s'de ~1e-3 değişim için Gamma_R ~ 1e-5
%   ~30s adaptasyon süresi sonunda yakınsama sağlanır.
Gamma_L = 1e-6;   % AdaptMech/Constant  -> L_hat_dot = -Gamma_L * scalarTerm * z
Gamma_R = 1e-4;   % AdaptMech/Constant3 -> M_hat_dot = +Gamma_R * scalarTerm * r

%% sigma-Modification (UUB Stabilite Garantisi)
% Adaptasyon yasaları sigma-mod ile:
%   L_hat_dot = -Gamma_L * scalarTerm * z - sigma_L * L_hat   [Sum_sigL]
%   M_hat_dot = +Gamma_R * scalarTerm * r - sigma_M * M_hat   [Sum_sigM]
%
% Stabilite ispatı (V_dot analizi):
%   V = e'Pe + (1/Gamma_L)||L_tilde||^2 + (1/Gamma_R)M_tilde^2
%   V_dot = -e'Qe + 2*delta*(M_tilde*r - L_tilde'z)
%           - sigma_L/Gamma_L*||L_tilde||^2 - sigma_M/Gamma_R*M_tilde^2
%           + C_sigma   (sınırlı sabit terim)
%   Burada delta = e'PB - phi = b3*(P23*e2 + P33*e3)  (basitleştirmeden kayıp)
%   Young eşitsizliği ile: V_dot <= -(lambda_Q - eps)*||e||^2 - sigma_min*(||L_tilde||^2+M_tilde^2) + C
%   => V sınırlı => tüm sinyaller UUB (Uniformly Ultimately Bounded)
%
% sigma küçük  => steady-state hata küçük, kazanç baskılama az
% sigma büyük  => hızlı yakınsama, fakat steady-state bias artar
% Optimum: sigma = 0.001 (RMS ~0.45mm, yakınsama ~5s)
sigma_L = 0.001;  % [Gain_sigL] L_hat sigma-mod kazancı
sigma_M = 0.001;  % [Gain_sigM] M_hat sigma-mod kazancı

%% Plant Başlangıç Koşulları (timeseries IC sorununu önler)
xc_dot_IC = v_mPs.Data(1);    % Başlangıç hızı [m/s]
xc_IC     = xc_m.Data(1);     % Başlangıç konumu [m]
PA_IC     = PA_Pa.Data(1);    % Başlangıç A-odası basıncı [Pa]
PB_IC     = PB_Pa.Data(1);    % Başlangıç B-odası basıncı [Pa]

%% Simülasyon
% Not: sigma-mod ile yakınsama ~5-7s, steady-state RMS ~0.45mm (%3).
% simTime = 30 önerilir; performans değerlendirmesi t>20s yapılır.
close all;
% assignin('base','sigma_L', sigma_L);
% assignin('base','sigma_M', sigma_M);
% out = sim("NonLinModelMracScaled.slx");

% %% Sonuçlar
% t_sim  = out.get('zm').Time;
% zm_sim = squeeze(out.get('zm').Data);
% z_sim  = squeeze(out.get('z').Data);
% L_sim  = squeeze(out.get('L_hat').Data);
% M_sim  = squeeze(out.get('M_hat').Data);
% 
% t_eval = max(t_sim)*0.67;  % son 1/3'te performans değerlendir
% idx_eval = t_sim >= t_eval;
% rms_pos = rms(zm_sim(1,idx_eval) - z_sim(1,idx_eval));
% 
% figure('Name','MRAC - Nonlineer Plant Takip','Color','w','Position',[100 50 900 700])
% 
% subplot(3,1,1)
% plot(t_sim, zm_sim(1,:), 'b', 'LineWidth', 1.5); hold on
% plot(t_sim, z_sim(1,:),  'r--', 'LineWidth', 1.5)
% grid on; ylabel('Konum (mm)')
% title(sprintf('Referans Model Takibi  |  RMS hata (t>%.0fs): %.2f mm (%.1f%%)', ...
%     t_eval, rms_pos, rms_pos/15*100))
% legend('z_{m1} (ref model)', 'z_1 (plant)')
% ylim([-20 20])
% 
% subplot(3,1,2)
% plot(t_sim, zm_sim(1,:)-z_sim(1,:), 'k', 'LineWidth', 1.2)
% yline(0,'--r'); grid on; ylabel('e_1 (mm)'); title('Konum Takip Hatası')
% 
% subplot(3,1,3)
% plot(t_sim, L_sim(1,:), 'LineWidth', 1.2); hold on
% plot(t_sim, L_sim(2,:), 'LineWidth', 1.2)
% plot(t_sim, L_sim(3,:), 'LineWidth', 1.2)
% plot(t_sim, M_sim,      '--k', 'LineWidth', 1.5)
% grid on; xlabel('t (s)'); ylabel('Kazanç')
% title(sprintf('Adaptif Kazançlar  |  M_{hat}: %.4f -> %.4f mA/mm', M_sim(1), M_sim(end)))
% legend('L_1','L_2','L_3','M_{hat}','Location','best')