% PARAMETRELER
d_p     = 34e-3;
d_r     = 30e-3;
A_c     = pi/4*(d_p^2 - d_r^2);
V_dead  = 1.5315e-5;
V_0     = A_c*(190.16e-3/2) + V_dead;

m      = 3.4;                   % Piston rotu eşdeğer kütlesi [kg]
Ae     = A_c;                   % Efektif alan [m^2] (Çift kollu silindirde doğrudan A'ya eşittir)
Bp     = 38;                    % Viskoz sönümleme katsayısı [N/(m/s)]
beta_e = 1.6e9;                 % Hidrolik yağın efektif elastikiyet (bulk) modülü [Pa]
Ve     = 2*V_0;                 % Efektif hacim [m^3]
Kq     = 1.1003e-2;              % Akış kazancı [(m^3/s)/m]
Kv     = 2.5e-4;                % Servo valf kazancı [m/u_perc%] (Simulink: xv=Const2*Const1*u_perc=2.5e-4*u_perc)
Kc     = 3.9633e-13; %3.9633e-13;            % Basınç-akış katsayısı [(m^3/s)/Pa]
Cip    = 3.9933e-13;            % Silindir içi kaçak katsayısı [(m^3/s)/Pa]
%% Referans modelin tanımlanması
% https://gemini.google.com/app/677af4a79d39da7f

am1 = 2.707E9; 
am2 = 9.528E7; 
am3 = 145.384; 

Am = [0,        1,          0;
      0,        -Bp/m,      -A_c/m;
      am1,      am2,        -am3];  % FIX: +am1, +am2 (işaret düzeltildi)

% DÜZELTME: bm3 negatif olmalı (DC kazanç için x1_ss = r sağlanır)
% https://gemini.google.com/app/677af4a79d39da7f

bm3 = -am1;
Bm = [0; 0; bm3];

Cm = eye(3);
%% Dinamik sistemin lineer halinin tanımlanması
A_mat = [0,     1,                  0; 
         0,     -Bp/m,              -Ae/m; 
         0,     4*beta_e*Ae/Ve,     -4*beta_e*(Kc + Cip)/Ve]; 

B_mat = [0; 0; (-4*beta_e/Ve)*(Kq*Kv)]; 

%% Ölçeklendirme
x1_scale = 0.1;      % m  — maksimum beklenen strok yarısı
x2_scale = 0.05;     % m/s — tipik maksimum hız
x3_scale = 1e7;      % Pa — tipik maksimum basınç farkı

% x1_scale = 170.16e-3/2;     % m  — maksimum beklenen strok yarısı
% x2_scale = 1.2;             % m/s — tipik maksimum hız
% x3_scale = 196e5;           % Pa — tipik maksimum basınç farkı

T     = diag([x1_scale, x2_scale, x3_scale]);
T_inv = diag([1/x1_scale, 1/x2_scale, 1/x3_scale]);

%% Ölçeklendirilmiş sistem matrisleri
Am_z  = T_inv * Am * T;     % özdeğerler korunur
Bm_z  = T_inv * Bm;         % referans girişi r boyutsuz
B_z   = T_inv * B_mat;

%% Lyapunov — artık iyi koşullandırılmış
Q_z = eye(3);
P_z = lyap(Am_z', Q_z);     % cond(P_z) ≈ 2e4 (önceki 1e15 yerine)

%% Adaptasyon çekirdeği
PB_vec_z = P_z * B_z;       % veya P_z * Bm_z (ikisi de çalışır)

%% Gamma — artık anlamlı bir değer
gamma_M = 1 / (norm(B_z)^2 * norm(P_z));   % ≈ 2.1e-5 başlangıç
gamma_L = gamma_M;
% Simülasyonda 10x veya 100x artırmayı dene: gamma_M * 10, * 100
