%%
clear all
clc
close all
addpath('C:\_PhD\4_Tez\3_Tests\0_Modelling\phdMain');
run('C:\_PhD\4_Tez\3_Tests\0_Modelling\phdMain\phdShell.m');
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
run('placePoles.m');

%% Lyapunov denkleminin yazılıp ölçekli uzayda P_bar'ın bulunması

% Q_bar = eye(3);
% Konuma yüksek(1), hıza düşük(0.01), basınca ise sıfıra yakın(1e-12) ağırlık veriyoruz:
Q_bar = diag([1, 0.01, 1e-12]); 

P_bar = lyap(Am_bar', Q_bar);

%% Dead-Zone Eşiği g0 (Fiziksel: e1_tol × |P_bar(1,:)*B_bar_mA|)
% wn değişince P_bar değişir → g0 otomatik güncellenir
% Kaynak: Ioannou & Sun (1996) §8.4.33 — gürültü sınırına göre eşik
e1_tol = 0.1;                           % mm — konum sensör gürültü toleransı
g0 = e1_tol * abs(P_bar(1,:) * B_bar_mA);
fprintf('g0 = %.4f  (e1_tol=%.1fmm × |PB1|=%.4f)\n', g0, e1_tol, P_bar(1,:)*B_bar_mA);

%% Komut Pre-Filter (Step Overshoot Eliminasyonu)
% Amaç:  Step komutu referans modele doğrudan gitmez;
%        2. derece overdamped filtreden geçer → MRAC'a keskin geçiş gitmez → overshoot yok.
% Kısıt: wn_pf >> 2*pi*6 = 37.7 rad/s  (6Hz'de < 1% zayıflama)
%        zeta_pf >= 1.0                  (filtre kendisi overshoot yaratmasın)
% Kararlılık: r → r_filtered sınırlı, MRAC Lyapunov ispatı bozulmaz (Ioannou & Sun §6.3)
wn_pf   = 400;   % rad/s  →  @6Hz: |G|=0.991, phase=-10.8° (ihmal edilebilir)
zeta_pf = 1.0;   % kritik sönümlü: step'e karşı sıfır overshoot
pf_num  = wn_pf^2;
pf_den  = [1, 2*zeta_pf*wn_pf, wn_pf^2];
fprintf('Pre-filter: wn_pf=%d rad/s, zeta_pf=%.1f  →  settling~%.1fms\n', ...
    wn_pf, zeta_pf, 2/wn_pf*1000);

%% Adaptasyon Kazançları
% Gamma_L, Gamma_M: adaptasyon hızı (model içinde Constant bloğu olarak tanımlı)
% Yüksek Γ → hızlı adaptasyon (6Hz için gerekli), düşük Γ → smooth step
Gamma_L = 0.05;   % Durum geri besleme adaptasyon hızı
Gamma_M = 0.05;   % Feedforward adaptasyon hızı

%% Parameter Projection Sınırları (Ioannou §8.4.2 / §8.5.5)
% Amaç: σ=0 koşulunda üçgen profil gibi testlerde parameter drift'i önle
% Kural: L_hat(i) sınıra ulaştığında ve update sınır dışına itecekse → update=0
% Kaynak: Ioannou & Sun (1996) Eq.(8.4.28) / (8.5.73)
%
% Fiziksel temellendirme:
%   L1 (konum geri beslemesi): Km(1)=0.0357 → pozitif, max 5× nominal
%   L2 (hız geri beslemesi):   Km(2)=-0.0177 → IC=0 kullanılıyor, ±8× nominal
%   L3 (basınç geri beslemesi): Λ=[1,1,0] ile dondurulmuş → projection pasif
% L_hat(2) NEGATIF olmali (sönümleme): Km(2)=-0.0177
% Pozitif L_hat(2) → pozitif hız geri beslemesi → kararsız kapalı döngü
% Üst sınır: 0 (pozitife izin yok)  Alt sınır: -0.10 (5× nominal)
L_hat_min = [0,             -0.10,   0     ];  % [mA/mm, mA/(mm/s), mA/bar]
L_hat_max = [5*Km(1),        0.0,    5*Km(3)];  % [0.178, 0.0, 0.137]

% M_hat için sınırlar (feedforward kazancı)
M_hat_min = 0.001;   % pozitif olmalı (DC kazanç işareti)
M_hat_max = 10.0;    % nominal ~1/DC_gain ≈ 0.036 mA/mm

fprintf('Projection sınırları:\n');
fprintf('  L1: [%.4f, %.4f]\n', L_hat_min(1), L_hat_max(1));
fprintf('  L2: [%.4f, %.4f]  (drift önleme - kritik)\n', L_hat_min(2), L_hat_max(2));
fprintf('  L3: [%.4f, %.4f]\n', L_hat_min(3), L_hat_max(3));

%% Adaptif Kazanç Başlangıç Koşulları
% u = -L_hat'*z + M_hat*r  (Controller/Gain = -1)
% L_hat_IC = Km: nominal geri besleme kazancını sağlar (u_0 = Km*z)
% M_hat_IC = 1/DC_gain: nominal feedforward (referans modelin DC kazancı = 1)
% NOT: Step modunda L_hat_IC(2)=0 (hız feedforward'u kaldır → overshoot azalt)
%      Sinüs modunda L_hat_IC = Km (nominal kazanç → hemen iyi 6Hz takibi)
test_modu = 'step';   % 'step' veya 'sinus'
if strcmp(test_modu, 'step')
    L_hat_IC = [Km(1), 0, Km(3)];   % hız terimi sıfırlanır
else
    L_hat_IC = Km;                   % tam nominal kazanç
end

DC_gain  = C_pos * inv(-Am_bar) * B_bar_mA;  % referans modelin DC kazancı [mm/mA]
M_hat_IC = 1 / DC_gain;                  % nominal feedforward kazancı [mA/mm]

%% Plant Başlangıç Koşulları (timeseries IC sorununu önler)
xc_dot_IC = 0;    % Başlangıç hızı [m/s]
xc_IC     = -45e-3;     % Başlangıç konumu [m]
PA_IC     = PA_Pa.Data(1);    % Başlangıç A-odası basıncı [Pa]
PB_IC     = PB_Pa.Data(1);    % Başlangıç B-odası basıncı [Pa]

%% Referans Model Başlangıç Koşulları
% zm, plant ile aynı noktadan başlamalı; aksi hâlde büyük başlangıç
% hatası adaptasyonu kontrolsüz saptırır.
% T_inv = diag([1000, 1000, 1e-5])  =>  z = T_inv * x
zm_IC = [xc_IC * 1000*0;          % konum:  m  -> mm
         xc_dot_IC * 1000*0;      % hız:   m/s -> mm/s
         (PA_IC - PB_IC)*1e-5*0]; % deltaP: Pa -> ölçekli

%% Simülasyon

close all;
load_system('NonLinModelMracScaled');
set_param('NonLinModelMracScaled/Integrator', 'InitialCondition', 'zm_IC');
open_system('NonLinModelMracScaled');
