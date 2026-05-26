%% SİMÜLASYON KURULUMU VE MRAC PARAMETRELERİ

% 2. Simülasyon Zaman Adımı ve Vektörü
dt = 1e-4;          % 0.1 ms örnekleme (Katı hidrolik dinamikler için güvenli sınır)
t_end = 2.0;        % 2 saniyelik simülasyon süresi
t = 0:dt:t_end;
N = length(t);

% 3. Durum Değişkenleri [mm; mm/s; bar]
z   = zeros(3, N); % Gerçek sistem durumları
zm  = zeros(3, N); % Referans model durumları
u_log = zeros(1, N); % Kontrol sinyali günlüğü (mA)

% 4. Adaptif Kazanç Matrisleri (Kocagil'deki gibi Sütun Vektörleri)
Kx = zeros(3, N);
Kr = zeros(1, N);

% 5. Adaptasyon Hızları (Tuning Parameters)
% DİKKAT: B_bar_mA(3) elemanı ~20600 mertebesinde olduğu için e' * P * B 
% çarpımı çok büyük çıkacaktır. Şişmeyi (windup) ve salınımı (chattering)
% engellemek için Gamma değerleri küçük tutulmalıdır.
% Gamma_x(2,2) en küçüktür çünkü z2 (hız) genliği 565 mm/s seviyelerine çıkacaktır.
Gamma_x = diag([1e-4, 1e-7, 1e-5]); 
Gamma_r = 5e-4;

%% SİMÜLASYON DÖNGÜSÜ (Euler İleri Farklar Yöntemi)
for k = 1:(N-1)
    % a. Referans Sinyali: 15 mm, 6 Hz Sinüs
    r_mm = 15 * sin(2 * pi * 6 * t(k));
    
    % b. Hata Dinamiği (Kocagil Formatı: e = zm - z)
    e = zm(:, k) - z(:, k);
    
    % c. Kontrol Kuralı (u = Kx^T * z + Kr * r)
    u_mA = Kx(:, k)' * z(:, k) + Kr(k) * r_mm;
    
    % Fiziksel Valf Doyumu (Moog G761 sınırı: ±25 mA)
    u_mA_sat = max(min(u_mA, 25), -25);
    u_log(k) = u_mA_sat;
    
    % d. Adaptasyon Yasalarının Hesaplanması
    % Skaler çarpan: (1x3) * (3x3) * (3x1) = 1x1
    mrac_scalar = e' * P_bar * B_bar_mA; 
    
    % Kazanç Türevleri
    Kx_dot = Gamma_x * z(:, k) * mrac_scalar; 
    Kr_dot = Gamma_r * r_mm    * mrac_scalar;
    
    % e. Sistem Diferansiyel Denklemleri
    z_dot  = A_bar * z(:, k) + B_bar_mA * u_mA_sat;
    zm_dot = Am_bar * zm(:, k) + Bm_bar_mm * r_mm;
    
    % f. Ayrık Zamanlı İntegrasyon (Durum Güncellemesi)
    z(:, k+1)  = z(:, k)  + z_dot  * dt;
    zm(:, k+1) = zm(:, k) + zm_dot * dt;
    Kx(:, k+1) = Kx(:, k) + Kx_dot * dt;
    Kr(k+1)    = Kr(k)    + Kr_dot * dt;
end
u_log(N) = u_log(N-1); % Son endeks dengelemesi

%% GRAFİKLER VE SONUÇ ANALİZİ
figure('Name','MRAC Performans Analizi','Color','w');

% 1. Konum Takibi
subplot(3,1,1);
plot(t, zm(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(t, z(1,:), 'r--', 'LineWidth', 1.5);
grid on; ylabel('Konum (mm)');
legend('Referans Model (z_{m1})', 'Sistem Çıkışı (z_1)');
title('6 Hz, 15 mm Yörünge Takibi');

% 2. Kontrol Sinyali
subplot(3,1,2);
plot(t, u_log, 'k', 'LineWidth', 1.2); hold on;
plot(t, 25*ones(size(t)), 'r:'); plot(t, -25*ones(size(t)), 'r:');
grid on; ylabel('Akım (mA)');
legend('u_{kontrol}', 'Fiziksel Doyum');
title('Valf Kontrol Sinyali');

% 3. Adaptif Kazançların Evrimi
subplot(3,1,3);
plot(t, Kx(1,:), 'LineWidth', 1.5); hold on;
plot(t, Kx(2,:), 'LineWidth', 1.5);
plot(t, Kx(3,:), 'LineWidth', 1.5);
plot(t, Kr, '--', 'LineWidth', 1.5);
grid on; xlabel('Zaman (s)'); ylabel('Kazanç Değerleri');
legend('K_{x1} (Konum)', 'K_{x2} (Hız)', 'K_{x3} (Basınç)', 'K_r (Referans)');
title('Zamanla Değişen Kontrolör Parametreleri');
%%
figure 
% 1. Konum Takibi
subplot(3,1,1);
plot(t, zm(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(t, z(1,:), 'r--', 'LineWidth', 1.5);
grid on; ylabel('Konum (mm)');
legend('Referans Model (z_{m1})', 'Sistem Çıkışı (z_1)');
title('6 Hz, 15 mm Yörünge Takibi');

% 2. Hız Takibi
subplot(3,1,2);
plot(t, zm(2,:), 'b', 'LineWidth', 1.5); hold on;
plot(t, z(2,:), 'r--', 'LineWidth', 1.5);
grid on; ylabel('Hız (mm/s)');
legend('Referans Model (z_{m2})', 'Sistem Çıkışı (z_2)');
title('6 Hz, 15 mm Hız Takibi');

% 2. Hız Takibi
subplot(3,1,3);
plot(t, zm(3,:), 'b', 'LineWidth', 1.5); hold on;
plot(t, z(3,:), 'r--', 'LineWidth', 1.5);
grid on; ylabel('PL (bar)');
legend('Referans Model (z_{m3})', 'Sistem Çıkışı (z_3)');
title('6 Hz, 15 mm PL Takibi');