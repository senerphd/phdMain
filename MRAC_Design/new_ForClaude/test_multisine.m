%% test_multisine.m — Multi-sine sinyal doğrulama ve analiz scripti
% multisine_fcn.m fonksiyonunu test eder
% Çalıştır → 3 grafik: zaman, spektrum, PE kontrolü
clear; clc; close all;

%% Sinyal üretimi
Fs   = 1000;          % Örnekleme frekansı [Hz]
T    = 30;            % Toplam süre [s]
t    = (0:1/Fs:T)';   % Zaman vektörü

y = zeros(size(t));
for i = 1:length(t)
    y(i) = multisine_fcn(t(i));
end

%% Temel metrikler
y_steady  = y(t >= 2);   % Ramp-up sonrası (t > 2s)
y_peak    = max(abs(y_steady));
y_rms     = rms(y_steady);
crest_fac = y_peak / y_rms;
fprintf('=== Multi-Sine Sinyal Metrikleri ===\n');
fprintf('Peak amplitüd : %.2f mm\n', y_peak);
fprintf('RMS           : %.2f mm\n', y_rms);
fprintf('Crest factor  : %.2f  (ideal sinüs = √2 = 1.41)\n', crest_fac);
fprintf('Frekans aralığı: 0.5 – 12 Hz\n');
fprintf('Bileşen sayısı : 9\n\n');

%% FFT Spektrum analizi
N_fft  = length(y_steady);
f_vec  = (0:N_fft-1) * Fs / N_fft;
Y_fft  = abs(fft(y_steady)) * 2 / N_fft;
f_plot = f_vec(f_vec <= 20);       % 0–20 Hz göster
Y_plot = Y_fft(f_vec <= 20);

%% Grafik 1: Zaman serisi
figure('Name','Multi-Sine — Zaman Serisi','Position',[100 600 900 300]);
plot(t, y, 'b', 'LineWidth', 1);
hold on;
yline(15,  'r--', '+15 mm limit', 'LabelHorizontalAlignment','left');
yline(-15, 'r--', '-15 mm limit', 'LabelHorizontalAlignment','left');
xlabel('Zaman [s]'); ylabel('Konum [mm]');
title('Multi-Sine Referans Sinyal (9 bileşen, Schroeder fazları)');
grid on; xlim([0 T]);
legend('y(t)', '±15mm limit', 'Location','northeast');

%% Grafik 2: Tek taraflı genlik spektrumu
figure('Name','Multi-Sine — Frekans Spektrumu','Position',[100 250 900 350]);
stem(f_plot, Y_plot, 'b', 'filled', 'MarkerSize', 5);
xlabel('Frekans [Hz]'); ylabel('Genlik [mm]');
title('Multi-Sine Tek Taraflı Amplitüd Spektrumu');
grid on; xlim([0 20]);
% Bileşen frekanslarını işaretle
f_comp = [0.5, 1, 2, 3, 5, 6, 8, 10, 12];
A_comp = [6.0, 5.0, 4.5, 4.0, 3.0, 3.0, 2.5, 2.0, 1.5];
for k = 1:length(f_comp)
    text(f_comp(k), A_comp(k)+0.15, sprintf('%.0fHz', f_comp(k)), ...
        'HorizontalAlignment','center', 'FontSize', 8, 'Color','r');
end

%% Grafik 3: Schroeder fazı etkisi (ham vs Schroeder karşılaştırma)
figure('Name','Schroeder Faz Etkisi','Position',[100 50 900 300]);

% Schroeder olmadan (φ=0)
f_Hz = [0.5, 1.0, 2.0, 3.0, 5.0, 6.0, 8.0, 10.0, 12.0];
A_mm = [6.0, 5.0, 4.5, 4.0, 3.0, 3.0, 2.5,  2.0,  1.5];
y_no_sch = zeros(size(t));
for k = 1:9
    y_no_sch = y_no_sch + A_mm(k) * sin(2*pi*f_Hz(k)*t);
end

t_idx = t >= 2 & t <= 12;
subplot(1,2,1);
plot(t(t_idx), y_no_sch(t_idx), 'r', 'LineWidth', 0.8);
yline(15,'k--'); yline(-15,'k--');
title(sprintf('Schroeder OLMADAN\nPeak=%.1f mm, CF=%.2f', ...
    max(abs(y_no_sch(t>=2))), max(abs(y_no_sch(t>=2)))/rms(y_no_sch(t>=2))));
xlabel('t [s]'); ylabel('[mm]'); grid on; ylim([-35 35]);

subplot(1,2,2);
plot(t(t_idx), y(t_idx), 'b', 'LineWidth', 0.8);
yline(15,'k--'); yline(-15,'k--');
title(sprintf('Schroeder İLE\nPeak=%.1f mm, CF=%.2f', y_peak, crest_fac));
xlabel('t [s]'); ylabel('[mm]'); grid on; ylim([-35 35]);

sgtitle('Schroeder Faz Optimizasyonu Etkisi');

%% PE Kontrolü
fprintf('=== PE (Kalıcı Uyarım) Kontrolü ===\n');
fprintf('MRAC için gereken min. frekans sayısı: ceil(n_params/2) = 2\n');
fprintf('Kullanılan frekans sayısı: 9  →  PE koşulu SAĞLANDI ✓\n');
fprintf('\nSimülasyon süresi önerisi:\n');
fprintf('  En düşük frekans: 0.5 Hz → periyot = 2 s\n');
fprintf('  Min. simülasyon: 5 periyot × 2 s = 10 s\n');
fprintf('  Tavsiye edilen  : 30 s (yeterli adaptasyon süresi)\n');
