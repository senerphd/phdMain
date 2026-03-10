%% Clear screen & data
close all
clear all
clc
%% Define library
% addpath('..\_lib\');
addpath('C:\_PhD\4_Tez\3_Tests\1_TestData\_lib');
%% Import Data
% txtDataFolder = '.\20251030\txtData\';
% txtDataName = 'RMC200_all_18.txt';                         % Open Loop
% txtDataName = 'RMC200_all_19.txt';                           % SINE SWEEP FULL -
% txtDataName = 'RMC200_all_19 - Trimmed.txt';               % SINE SWEEP SLOW +
% txtDataName = 'RMC200_all_19 - Trimmed.txt';               % SINE SWEEP SLOW +
% txtDataName = 'RMC200_all_32 - Trimmed.txt';               % +/- 100 OL
% txtDataName = 'RMC200_all_31 - Trimmed.txt';               % Open Loop


% txtDataFolder = '.\20251105\txtData\';
% txtDataName = 'RMC200_all_1.txt';                         % SLOW SINE +
% txtDataName = 'RMC200_all_7 - Trimmed2.txt';                 % FAST SINE + 
% txtDataName = 'RMC200_all_8 - Trimmed.txt';               % FAST SINE + 
% txtDataName = 'RMC200_all_17 - Trimmed - Trimmed.txt';    % SLOW TRIANGLE -
% txtDataName = 'RMC200_all_17 - Trimmed_old.txt';          % SLOW TRIANGLE -
% txtDataName = 'RMC200_all_17 - Trimmed_openLoop.txt';       % U: Const -> Open Loop

txtDataFolder = '.\20251211\txtData\';
% txtDataName = 'RMC200_all_11.txt';        % 11. 20 mm/s saniye üçgen profil testleri ++
% txtDataName = 'RMC200_all_22.txt';          % 22. 0.5 hz 30 genlik 
% txtDataName = 'RMC200_all_22 - Trimmed.txt';          % 22. 0.5 hz 30 genlik 
% txtDataName = 'RMC200_all_24.txt';          % 24. 1 hz 30 genlik 
txtDataName = 'RMC200_all_25.txt';          % 25. 1.5 hz 30 genlik 
% txtDataName = 'RMC200_all_32 - Trimmed_stepOL_100.txt';          % Step Open Loop %100
finalTxtData = [txtDataFolder,txtDataName];

[t,r,xc,u_perc,Q_return,P_A, P_B, P_supply, P_return,T_supply,e,r_dot, xc_dot,P_L, F_piston, Ts,Fs] = importfile3(finalTxtData);

% mass and acceleration
m = 3.4;

xc_dot =  gradient(xc,Ts);
xc_ddot = gradient(xc_dot,Ts);

B_e         = 1.6e9;          % Effective Bulk Modulus (100 ksi) Pa
D_piston    = 34e-3;                    % Piston Diameter m
D_rod       = 30e-3;                    % Rod Diameter m
A_piston    = (pi * D_piston^2)/4;      % Piston area m^2
A_rod       = (pi * D_rod^2)/4;         % Rod area m^2
Aeff        = A_piston - A_rod;         % Effective Area m2

L_stroke    = 190.16e-3;               % Total Stroke Length m 
V01         = Aeff * (L_stroke/2);     % Initial Chamber Volume in neural position m^3
V02         = Aeff * (L_stroke/2);     % Initial Chamber Volume in neural position m^3
V_dead      = 1e-5;                    % 10 cm^3 dead volume <- 

%% OPTIONS 
useKalman = false; 
fitCurve = false; 

% Conversion Constants 
m3PsToLpm = 60e3; 
lpmTom3Ps = 1/m3PsToLpm; 
barToPa    = 1e5; 
barToPsi   = 14.5037738; 
PaToBar = 1/barToPa; 
%%
if useKalman
    % --- Kalman Filtresi Ayar Parametreleri (BU DEĞERLERİ AYARLAMALISINIZ) ---
    close all
    % q_jerk_intensity (Süreç Gürültüsü Yoğunluğu, $q$): Modelinize ne kadar güveneceğinizi belirler.
    % Büyük $q$: Modelin belirsizliği artar, tahminler ölçümlere daha çok güvenir ve daha hızlı değişir
    % (daha az yumuşatma, daha gürültülü ivme).Küçük $q$: Model daha katı hale gelir, tahminler geçmişe daha çok güvenir (daha fazla yumuşatma, daha yavaş tepki).

    % R_variance (Ölçüm Gürültüsü Kovaryansı, $R$): Konum sensörü verisindeki gürültü varyansıdır.
    % Büyük $R$: Sensörün gürültülü olduğu varsayılır, tahminler ölçüme daha az güvenir.
    % Küçük $R$: Sensörün hassas olduğu varsayılır, tahminler ölçüme daha çok güvenir.

    noise_level = 0.000001;           % Sensör gürültüsü seviyesi
    q_jerk_intensity = 10;         % Süreç gürültüsü yoğunluğu (TUNE: 0.01 ile 100 arasında deneyin)
    R_variance = noise_level^2 * 10000;     % Ölçüm gürültüsü varyansı. Genellikle (sensör hassasiyeti)^2
    % veya veri setinin gürültü varyansı alınır.
    [x_est, v_est, a_est] = kalman_filter_position(xc, Ts, q_jerk_intensity, R_variance);


    xc = x_est';
    xc_dot = v_est';
    xc_ddot = a_est';
    F_friction = F_piston_filtered - (m .* xc_ddot);

    % Plot all signals
    figure;
    subplot(3,1,1)
    plot(t,xc);
    hold on
    plot(t,x_est,'LineWidth',1.5)
    ylabel('Cylinder Position ($x_c$)','Interpreter','latex')
    xlabel('Time (s)','Interpreter','latex')
    title('Position - Velocity - Acceleration Estimation with Kalman Filter')
    grid on;

    subplot(3,1,2)
    plot(t,xc_dot);
    hold on
    plot(t,v_est,'LineWidth',1.5)
    ylabel('Cylinder Velocity ($\dot{x_c}$)','Interpreter','latex')
    xlabel('Time (s)','Interpreter','latex')

    grid on;

    subplot(3,1,3)
    hold on
    plot(t,xc_ddot);
    plot(t,a_est,'LineWidth',1.5)
    plot(t,xc_ddot_filtered,'LineWidth',1.5)
    ylabel('Cylinder Acceleration ($\ddot{x_c}$)','Interpreter','latex')
    xlabel('Time (s)','Interpreter','latex')
    legend('xc_{ddot}','a_{est} - Kalman','a_{est} - Filtered')
    grid on;

else
    % --- 2. Filtre Tasarımı (Butterworth Alçak Geçiren Filtre) ---

    %------------------------------------------
    % Filter Position Data
    %------------------------------------------
    % Filtrenin kesim frekansını belirleyin.
    Fc = 60; % Kesim frekansı (Hz) - Ayarlanması gereken parametre
    Wn = Fc / (Fs/2); % Normalleştirilmiş kesim frekansı
    % Butterworth filtresi katsayıları
    order = 2;
    [b, a] = butter(order, Wn, 'low');

    % --- 3. Sıfır Fazlı Filtreleme (filtfilt) ---
    xc_filtered = filtfilt(b, a, xc);

    Fc = 6; % Kesim frekansı (Hz) - Ayarlanması gereken parametre
    Wn = Fc / (Fs/2); % Normalleştirilmiş kesim frekansı
    % Butterworth filtresi katsayıları
    order = 2;
    [b, a] = butter(order, Wn, 'low');
    P_A_filtered = filtfilt(b, a, P_A);
    P_B_filtered = filtfilt(b, a, P_B);


    P_supply_filtered = filtfilt(b, a, P_supply);
    P_return_filtered = filtfilt(b, a, P_return); 

    Fc = 20; % Kesim frekansı (Hz) - Ayarlanması gereken parametre
    Wn = Fc / (Fs/2); % Normalleştirilmiş kesim frekansı
    % Butterworth filtresi katsayıları
    order = 2;
    [b, a] = butter(order, Wn, 'low');
    Q_return_filtered = filtfilt(b, a, Q_return);

    % Calculate velocity & acceleration with filtered position data 
    v_from_xc_filtered = gradient(xc_filtered, Ts);                   % Hız
    a_from_xc_filtered = gradient(v_from_xc_filtered, Ts);  % İvme


    % --- ÇÖZÜM: KENARLARI KIRPMA ---
    % "Kenar Etkisi" (Edge Transient)
    % Verinin başından ve sonundan (örneğin 0.2 saniye veya 50-100 sample) atın.
    % Örnekleme sürenize (Ts) göre sample sayısını belirleyin.
    trim_time = 1.75; % saniye (Filtrenin oturması için pay)
    trim_samples = round(trim_time / Ts);

    % Zaman vektörünü ve verileri kırpıyoruz
    t_final = t(trim_samples : end-trim_samples);
    xc_final = xc_filtered(trim_samples : end-trim_samples);
    v_final  = v_from_xc_filtered(trim_samples : end-trim_samples);
    a_final  = a_from_xc_filtered(trim_samples : end-trim_samples);

    % (Opsiyonel) Basınç verilerinizi de aynı indexlerle kırpmayı unutmayın!
    P_A_final = P_A_filtered(trim_samples : end-trim_samples);
    P_B_final = P_B_filtered(trim_samples : end-trim_samples);
    
    P_supply_final = P_supply_filtered(trim_samples : end-trim_samples);
    P_return_final = P_return_filtered(trim_samples : end-trim_samples);
    Q_return_final = Q_return_filtered(trim_samples : end-trim_samples);
    
    
    u_perc_final = u_perc(trim_samples : end-trim_samples);


    % Plotlamayı bu _final verilerle yapın.
     % Plot all signals
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    figure;
    subplot(3,1,1)
    plot(t,xc);
    hold on
    plot(t_final,xc_final,'LineWidth',1.5)
    ylabel('Cylinder Position(${x_c}$)','Interpreter','latex')
    xlabel('Time (s)','Interpreter','latex')
    title('Position - Velocity - Acceleration LowPass Filtered')
    legend('${x_c}$','${x_c}$ - Filtered','Interpreter','latex')
    grid on;

    subplot(3,1,2)
    plot(t,xc_dot);
    hold on
    plot(t_final,v_final,'LineWidth',1.5)
    ylabel('Cylinder Velocity ($\dot{x_c}$)','Interpreter','latex')
    xlabel('Time (s)','Interpreter','latex')
    legend('$\dot{x_c}$','$\dot{x_c}$ - Filtered','Interpreter','latex')
    grid on;

    subplot(3,1,3)
    plot(t,xc_ddot);
    hold on
    plot(t_final,a_final,'LineWidth',1.5)
    ylabel('Cylinder Acceleration ($\ddot{x_c}$)','Interpreter','latex')
    xlabel('Time (s)','Interpreter','latex')
    legend('$\ddot{x_c}$','$\ddot{x_c}$ - Filtered','Interpreter','latex')
    grid on;
    
    % Plot pressures
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    figure;
    subplot(2,2,1)
    plot(t,P_A);
    hold on
    plot(t_final,P_A_final,'LineWidth',1.5)
    ylabel('Cylinder Pressure A','Interpreter','latex')
    xlabel('Time (s)','Interpreter','latex')
    legend('${P_A}$','${P_A}$ - Filtered','Interpreter','latex')
    grid on;

    subplot(2,2,2)
    plot(t,P_B);
    hold on
    plot(t_final,P_B_final,'LineWidth',1.5)
    ylabel('Cylinder Pressure B','Interpreter','latex')
    xlabel('Time (s)','Interpreter','latex')    
    legend('${P_B}$','${P_B}$ - Filtered','Interpreter','latex')
    grid on;

    subplot(2,2,3)
    plot(t,P_supply);
    hold on
    plot(t_final,P_supply_final,'LineWidth',1.5)
    ylabel('Supply Pressure','Interpreter','latex')
    xlabel('Time (s)','Interpreter','latex')    
    legend('$P_{supply}$','$P_{supply}$ - Filtered','Interpreter','latex')
    grid on;

    subplot(2,2,4)
    plot(t,P_return);
    hold on
    plot(t_final,P_return_final,'LineWidth',1.5)
    ylabel('Return Pressure','Interpreter','latex')
    xlabel('Time (s)','Interpreter','latex')    
    legend('$P_{return}$','$P_{return}$ - Filtered','Interpreter','latex')
    grid on;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    figure;
    subplot(2,2,1)
    plot(t_final,xc_final,'LineWidth',1.5)
    ylabel('Cylinder Position(${x_c}$)','Interpreter','latex')
    xlabel('Time (s)','Interpreter','latex')
    legend('${x_c}$ - Filtered','Interpreter','latex')
    grid on;

    subplot(2,2,2)
    plot(t_final,v_final);
    ylabel('Cylinder Velocity ($\dot{x_c}$)','Interpreter','latex')
    xlabel('Time (s)','Interpreter','latex')
    legend('$\dot{x_c}$ - Filtered','Interpreter','latex')
    grid on;

    subplot(2,2,3)
    plot(t,(P_A - P_B));
    hold on; 
    plot(t_final,(P_A_final- P_B_final));
    ylabel('Delta Pressure $P_L$','Interpreter','latex')
    xlabel('Time (s)','Interpreter','latex')
    legend('$P_L$','$P_L$ - Filtered','Interpreter','latex')
    grid on;

    subplot(2,2,4)
    plot(t,Q_return);
    hold on; 
    plot(t_final,Q_return_final);
    ylabel('Return Flow $Q_{return}$','Interpreter','latex')
    xlabel('Time (s)','Interpreter','latex')
    legend('$Q_{return}$ - Filtered','Interpreter','latex')
    grid on;
    
end

%%
% Alanları tanımlayın (Kendi sisteminize göre güncelleyin)
D_piston    = 34e-3;                    % Piston Diameter m
D_rod       = 30e-3;                    % Rod Diameter m
A_piston    = (pi * D_piston^2)/4;      % Piston area m^2
A_rod       = (pi * D_rod^2)/4;         % Rod area m^2
Aeff        = A_piston - A_rod;         % Effective Area m2

Area_A = Aeff; % m^2
Area_B = Aeff; % m^2
Mass   = 3.4;   % kg (Hareket eden toplam kütle)

x_v_max     = 0.001016;                 % m Max spool position
x_v_min     = -x_v_max;                 % m Min spool position

i_max       = 50e-3; 
Ki          = x_v_max / i_max;          % m/A Spool gain, assuming spool position is linear with current
xv          = u_perc/100 .* 50e-3 .* Ki; 
xv_final    = u_perc_final/100 .* 50e-3 .* Ki; 
%% 
% 1. Hidrolik Kuvvet (Net itme/çekme kuvveti)
F_hydraulic = (P_A_final * Area_A) - (P_B_final * Area_B);

% 2. Atalet Kuvveti (F = m*a)
F_inertial = Mass * a_final;

% 3. Sürtünme Kuvveti (F_sürtünme = F_hidrolik - F_atalet)
F_friction = F_hydraulic - F_inertial;

figure;
plot(v_final, F_friction, 'b.'); % Nokta bulutu olarak çizmek detayları gösterir
grid on;
xlabel('Hız (m/s)');
ylabel('Sürtünme Kuvveti (N)');
title('Sürtünme Kuvveti vs. Hız (Stribeck Eğrisi)');

%% Statik Parametrelerin Belirlenmesi 
% Grafiğinize bakarak şu değerleri not edin (veya MATLAB'da lsqcurvefit ile fit edin):
% 1.	$F_c$ (Coulomb Sürtünmesi): Yüksek hızlarda grafiğin oturduğu en düşük seviye (Stribeck çukurunun dibi değil, lineer artışın başladığı yer).
% o	Tahmini Okuma: Pozitif için 22 N, Negatif için 32 N. (Ortalama 27 N alabilirsiniz).
% 2.	$F_s$ (Statik Sürtünme): Hızın 0 olduğu yerdeki o ilk sivri tepe.
% o	Tahmini Okuma: Pozitif için 27 N, Negatif için 35 N. (Ortalama 31 N alabilirsiniz).
% 3.	$v_s$ (Stribeck Hızı): Sürtünmenin azalmayı bırakıp tekrar artmaya (veya sabitlenmeye) başladığı hız değeri.
% o	Tahmini Okuma: Grafiğinizde eğrinin büküldüğü yer yaklaşık 0.04 m/s gibi duruyor.
% 4.	$\sigma_2$ (Viskoz Sürtünme): Grafiğin kuyruk kısmındaki (0.1 m/s sonrası) eğim.
% o	Tahmini Okuma: Hız 0.1'den 0.3'e çıkarken kuvvet çok az artmış (belki 2-3 N). Yani viskoziteniz düşük. Eğim $\approx (25-22)/(0.3-0.1) = 15 Ns/m$ gibi küçük bir değer olabilir. 
%% Stribeck Eğrisi Fit (Düzeltilmiş Sınırlar ile) - Statik


if fitCurve == true 
    % 1. Veri Hazırlığı (Aynı)
    v_threshold = 0.001; 
    idx_pos = v_final > v_threshold;
    v_pos = v_final(idx_pos);
    F_pos = F_friction(idx_pos);
    
    idx_neg = v_final < -v_threshold;
    v_neg = abs(v_final(idx_neg));      
    F_neg = abs(F_friction(idx_neg));   
    
    % Model Fonksiyonu (Aynı)
    stribeck_fun = @(x, v) x(1) + (x(2) - x(1)) .* exp(-(v ./ x(3)).^2) + x(4) .* v;
    % x(1): Fc, x(2): Fs, x(3): vs, x(4): sigma2
    
    % --- KRİTİK DÜZELTME: SINIRLAR (BOUNDS) ---
    % Algoritmanın Fs'yi 0 yapmasını engellemek için alt sınırları (lb) yükseltiyoruz.
    % Grafiğe bakarak: Fc en az 15N, Fs en az 20N olmalı dedik.
    
    % Parametre Sırası: [Fc, Fs, vs, sigma2]
    
    % --- POZİTİF YÖN ---
    x0_pos = [22,   26,    0.02,   12];   % Başlangıç tahmini (Grafikten)
    lb_pos = [15,   38,    0.001,  5];    % ALT SINIR: Fs en az 22N olsun dedik!
   ub_pos = [30,   80,    0.1,    100];  
    
    % --- NEGATİF YÖN ---
    x0_neg = [29,   35,    0.02,   5];    % Başlangıç tahmini (Grafikten)
    lb_neg = [20,   48,    0.001,  0];    % ALT SINIR: Fs en az 30N olsun dedik!
   ub_neg = [40,   90,    0.1,    100];
    
    % LSQCURVEFIT İşlemi
    options = optimoptions('lsqcurvefit', 'Display', 'final', 'MaxIter', 1000);
    
    fprintf('Pozitif Yön Fit Ediliyor...\n');
    [x_est_pos, resnorm_pos] = lsqcurvefit(stribeck_fun, x0_pos, v_pos, F_pos, lb_pos, ub_pos, options);
    
    fprintf('Negatif Yön Fit Ediliyor...\n');
    [x_est_neg, resnorm_neg] = lsqcurvefit(stribeck_fun, x0_neg, v_neg, F_neg, lb_neg, ub_neg, options);
    
    % Sonuçları Yazdırma
    disp('--------------------------------------------------');
    disp('       PARAMETRE      |  POZİTİF (+) |  NEGATİF (-) ');
    disp('--------------------------------------------------');
    fprintf('Fc (Coulomb)          :  %8.4f N  |  %8.4f N\n', x_est_pos(1), x_est_neg(1));
    fprintf('Fs (Static/Peak)      :  %8.4f N  |  %8.4f N\n', x_est_pos(2), x_est_neg(2));
    fprintf('vs (Stribeck Vel.)    :  %8.4f m/s|  %8.4f m/s\n', x_est_pos(3), x_est_neg(3));
    fprintf('sigma2 (Viscous)      :  %8.4f    |  %8.4f\n', x_est_pos(4), x_est_neg(4));
    disp('--------------------------------------------------');
    
    % Görselleştirme
    figure('Name', 'Corrected Stribeck Fit', 'Color', 'white');
    hold on; grid on;
    plot(v_final, F_friction, 'b.', 'MarkerSize', 2, 'DisplayName', 'Deneysel Veri');
    v_range = linspace(0, max(v_pos), 200);
    plot(v_range, stribeck_fun(x_est_pos, v_range), 'r-', 'LineWidth', 2.5, 'DisplayName', 'Fit (+) CORRECTED');
    plot(-v_range, -stribeck_fun(x_est_neg, v_range), 'm-', 'LineWidth', 2.5, 'DisplayName', 'Fit (-) CORRECTED');
    xlabel('Hız (m/s)'); ylabel('Sürtünme Kuvveti (N)'); legend('Location', 'best');

end

%% 
% --- SİMULASYON HAZIRLIĞI ---

% 1. Zaman ve Veri Vektörlerinin Boyut Kontrolü
% (Bazen kırpma işleminden sonra transpoze hatası olabilir, garantiye alalım)
if size(t_final,1) < size(t_final,2), t_final = t_final'; end
if size(F_hydraulic,1) < size(F_hydraulic,2), F_hydraulic = F_hydraulic'; end
if size(v_final,1) < size(v_final,2), v_final = v_final'; end

% 2. Simulink İçin Timeseries Objeleri Oluşturma
% Giriş: Hidrolik Kuvvet (P1*A1 - P2*A2)
ts_F_input = timeseries(F_hydraulic, t_final);

% Referans: Karşılaştırma yapacağımız gerçek hız
ts_v_ref   = timeseries(v_final, t_final);

% 3. Model Parametrelerinin Workspace'te Olduğundan Emin Olun
m = Mass; % Kütle (3.4 kg demiştiniz)
% Diğer parametreler zaten tanımlı (Fc, Fs, vs, sigma_0...)


%% 

%% 
% figure;
% plot(ts_F_input.Time, ts_F_input.Data / max(abs(ts_F_input.Data)), 'b'); hold on;
% plot(ts_v_ref.Time, ts_v_ref.Data / max(abs(ts_v_ref.Data)), 'r--');
% legend('Normalize Kuvvet', 'Normalize Hız');
% title('İşaret Kontrolü');
%% 
%---------------------------------------
% OPTIMIZATION 
%---------------------------------------
 close all;
simParamEster
open_system('cologniModel')
