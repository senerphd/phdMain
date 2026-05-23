%% mrac_setup.m
% Lyapunov Tabanlı MRAC - Parametre Kurulum Scripti
%
% LinModel (MATLAB Function bloğu) parametrelerine dayalı olarak
% plant, referans model ve adaptasyon parametrelerini hesaplar.
%
% TASARIM YÖNTEMİ:
%   1. Plant fiziksel koordinatlarda linearize edildi (3 durum).
%   2. P_L basınç durumu Ps=21 MPa ile normalize edildi (sayısal kararlılık).
%   3. Referans model: LQR ile kazanç tasarlanıp Am = Ap - Bp*K_lqr yapısı
%      kullanıldı → MRAC eşleşme koşulu otomatik sağlandı.
%   4. Lyapunov denklemi: Am^T*P + P*Am = -I çözüldü.
%
% Hasan Şener - PhD Tezi

clear; clc;
fprintf('=== MRAC Lyapunov Parametre Kurulumu ===\n\n');

%% --------------------------------------------------------
%  1. PLANT PARAMETRELERİ (LinModel'den)
% ---------------------------------------------------------
fprintf('[1] Plant parametreleri hesaplanıyor...\n');

d_p     = 34e-3;                            % Piston çapı [m]
d_r     = 30e-3;                            % Rod çapı [m]
A_c     = pi/4*(d_p^2 - d_r^2);            % Efektif alan [m²]
V_dead  = 1.5315e-5;                        % Ölü hacim [m³]
V_0     = A_c*(190.16e-3/2) + V_dead;      % Başlangıç hacmi [m³]

m       = 3.4;                              % Piston + rod kütlesi [kg]
Ae      = A_c;                              % Efektif alan [m²]
Bp_phys = 38;                               % Viskoz sönüm [N·s/m]
beta_e  = 1.6e9;                            % Efektif bulk modülü [Pa]
Ve      = 2*V_0;                            % Efektif hacim [m³]
Kq      = 1.1003e-2;                        % Akış kazancı [(m³/s)/m]
Kv      = 1;                                % Servo valf kazancı [m/V]
Kc      = 3.9633e-13;                       % Basınç-akış katsayısı [(m³/s)/Pa]
Cip     = 3.9933e-13;                       % İç kaçak katsayısı [(m³/s)/Pa]

% Fiziksel koordinatlarda lineer durum uzayı
% x = [x_p (m), xdot_p (m/s), P_L (Pa)]^T
Ap = [0,    1,                        0;
      0,    -Bp_phys/m,              -Ae/m;
      0,    4*beta_e*Ae/Ve,          -4*beta_e*(Kc+Cip)/Ve];

Bp_vec = [0; 0; -4*beta_e*Kq*Kv/Ve];  % 3×1 giriş vektörü

Cp = [1, 0, 0];                         % Çıkış: konum x_p
Dp = 0;

fprintf('   Plant özdeğerleri: '); disp(eig(Ap).');

%% --------------------------------------------------------
%  2. DURUM NORMALİZASYONU (sayısal kararlılık için)
% ---------------------------------------------------------
fprintf('[2] Durum normalizasyonu uygulanıyor...\n');

% Normalize koordinat: x_s = D * x, x_s = [x_p, xdot_p, P_L/Ps]
Ps = 21e6;              % Referans basınç [Pa] (yaklaşık supply pressure)
D    = diag([1, 1, 1/Ps]);
Dinv = diag([1, 1, Ps]);

Ap_s   = D * Ap   * Dinv;   % Normalize plant A matrisi
Bp_s   = D * Bp_vec;         % Normalize plant B vektörü
fprintf('   Normalize Ap özdeğerleri: '); disp(eig(Ap_s).');

%% --------------------------------------------------------
%  3. REFERANS MODEL - LQR TABANLI TASARIM
%     Am = Ap_s - Bp_s * K_lqr  →  MRAC eşleşme koşulu otomatik sağlanır
% ---------------------------------------------------------
fprintf('[3] Referans model tasarlanıyor (LQR)...\n');

Q_lqr = diag([100, 10, 0.1]);  % Konum ağırlıklı LQR ağırlıkları
R_lqr = 1;                       % Kontrol girişi ağırlığı

[K_lqr, ~, ev_lqr] = lqr(Ap_s, Bp_s, Q_lqr, R_lqr);
Am_s = Ap_s - Bp_s * K_lqr;    % Ulaşılabilir referans model (normalize)

fprintf('   Referans model özdeğerleri:\n'); disp(ev_lqr.');
if all(real(ev_lqr) < 0)
    fprintf('   Referans model Hurwitz (kararlı) ✓\n');
end

% Eşleşme koşulu doğrulama
Kx_star = -K_lqr;               % MRAC ideal kazanç: K_x* = -K_lqr
match_err = norm(Ap_s + Bp_s * Kx_star - Am_s);
fprintf('   Eşleşme hatası: %.2e (0 olmalı) ✓\n', match_err);

% Bm tasarımı: DC kazancı = 1 (r → konum)
dc_coef = -[1,0,0] * inv(Am_s) * Bp_s;
Kr_star  = 1 / dc_coef;
Bm_s     = Bp_s * Kr_star;

fprintf('   K_x* = [%.4f, %.4f, %.6f]\n', Kx_star(1), Kx_star(2), Kx_star(3));
fprintf('   K_r* = %.4f\n', Kr_star);

%% --------------------------------------------------------
%  4. LYAPUNOV DENKLEMİ: Am^T*P + P*Am = -Q
% ---------------------------------------------------------
fprintf('[4] Lyapunov denklemi çözülüyor...\n');

Q_lyap = eye(3);
P_s    = lyap(Am_s', Q_lyap);   % Am^T*P + P*Am = -I

eigP = eig(P_s);
if all(eigP > 0)
    fprintf('   P pozitif tanımlı ✓  (min özdeğer: %.4e)\n', min(eigP));
else
    error('P pozitif tanımlı değil!');
end

PBp_norm = norm(P_s * Bp_s);
fprintf('   norm(P*Bp) = %.4f  (adaptasyon ölçeği)\n', PBp_norm);

%% --------------------------------------------------------
%  5. ADAPTASYON PARAMETRELERİ
% ---------------------------------------------------------
fprintf('[5] Adaptasyon parametreleri ayarlanıyor...\n');

% Gerekli Gamma hesabı:
% K_dot ≈ Gamma * |xp| * sigma → Gamma ≈ K_target / (|xp| * sigma * T_conv)
Kx_target   = max(abs(Kx_star));
e_typical   = 0.03;        % Tipik konum hatası [m]
sigma_typ   = e_typical * (P_s * Bp_s)' * [1;0;0];  % konum ekseni sigma
Gamma_x     = Kx_target / (e_typical * abs(sigma_typ) * 2.0);
Gamma_r     = Kr_star    / (0.03 * abs(sigma_typ) * 2.0);

% Makul aralıkta tut
Gamma_x = min(max(Gamma_x, 1e3), 5e4);
Gamma_r = min(max(Gamma_r, 1e3), 5e4);

fprintf('   Gamma_x = %.2e\n', Gamma_x);
fprintf('   Gamma_r = %.2e\n', Gamma_r);

% Kontrol sinyali doyum sınırı
u_max = 10.0;   % [V]
fprintf('   u_max = %.1f V\n', u_max);

%% --------------------------------------------------------
%  6. SİMÜLASYON PARAMETRELERİ
% ---------------------------------------------------------
fprintf('[6] Simülasyon parametreleri...\n');

Ts_sim    = 1e-3;         % Örnekleme süresi [s]
T_sim     = 5.0;          % Simülasyon süresi [s]
x0_plant  = [0; 0; 0];   % Fiziksel başlangıç koşulu
x0_s      = [0; 0; 0];   % Normalize başlangıç koşulu
x0_ref    = [0; 0; 0];   % Referans model başlangıç koşulu
Kx0       = 0.5 * Kx_star(:);  % Perturbed başlangıç (%50 ideal)
Kr0       = 0.5 * Kr_star;      % Perturbed başlangıç (%50 ideal)

r_amp     = 0.05;   % 5 cm step referans [m]
r_time    = 0.5;    % Step zamanı [s]

fprintf('   T_sim = %.1f s | r = %.0f mm step @ t=%.1f s\n', T_sim, r_amp*100, r_time);
fprintf('   Başlangıç K_x = 50%% K_x*\n');

%% --------------------------------------------------------
%  7. S-FUNCTION İÇİN PARAMETRELER (orijinal koordinatlarda)
% ---------------------------------------------------------
% S-Function fiziksel koordinatlarda çalışır (normalize değil)
% P ve Bp'yi orijinal koordinatlara dönüştür
P_phys   = Dinv * P_s * D;     % P fiziksel koordinatlarda
Bp_phys2 = Bp_vec;              % Bp fiziksel koordinatlarda (Bp_vec)
Kx_star_phys = Kx_star * D;    % K_x* fiziksel koordinatlarda (satır vektörü)
Kr_star_phys = Kr_star;

fprintf('\n[7] S-Function parametreleri hazırlandı.\n');
fprintf('    P_phys norm: %.4e\n', norm(P_phys));

%% --------------------------------------------------------
%  8. ÖZET
% ---------------------------------------------------------
fprintf('\n=== KURULUM TAMAMLANDI ===\n');
fprintf('Referans model özdeğerleri: %.1f, %.1f±%.1fj rad/s\n', ...
        real(ev_lqr(1)), real(ev_lqr(2)), imag(ev_lqr(2)));
fprintf('K_x* = [%.4f, %.4f, %.6f]\n', Kx_star(1), Kx_star(2), Kx_star(3));
fprintf('K_r* = %.4f\n', Kr_star);
fprintf('Gamma_x = %.2e, Gamma_r = %.2e\n', Gamma_x, Gamma_r);
fprintf('u_max = %.1f V\n\n', u_max);
fprintf('>> mrac_test.m ile simülasyonu çalıştırın.\n');
