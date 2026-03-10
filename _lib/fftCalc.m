clc;  close all;

%% 1. Parametrelerin Tanımlanması
Fs = 1000;            % Örnekleme Frekansı (Hz) - Saniyede alınan örnek sayısı
T = 1/Fs;             % Örnekleme Periyodu (s)

%% 2. Sinyalin Oluşturulması
% İçinde 50 Hz ve 120 Hz bileşenleri olan ve biraz gürültü eklenmiş bir sinyal
% S = 0.7*sin(2*pi*50*t) + sin(2*pi*120*t); % Temiz sinyal
X = P_A; % Gürültülü Sinyal
X =X - mean(X); 
L = length(X);
%% 3. FFT Hesaplaması
Y = fft(X);           % Sinyalin Fourier Dönüşümü

%% 4. Genlik Spektrumunun Düzenlenmesi (ÖNEMLİ ADIM)
% FFT çıktısı karmaşık sayılardır ve simetriktir.
% Bunu anlamlı genliklere (Magnitude) çevirmemiz gerekir.

P2 = abs(Y/L);        % Çift taraflı spektrum (Two-sided spectrum) ve Normalizasyon
P1 = P2(1:L/2+1);     % Tek taraflı spektrum (Single-sided spectrum) için yarısını al
P1(2:end-1) = 2*P1(2:end-1); % Negatif frekansların enerjisini pozitife ekle

f = Fs*(0:(L/2))/L;   % Frekans eksenini (0'dan Fs/2'ye kadar) oluştur

%% 5. Grafikleri Çizdirme
figure('Name', 'FFT Analizi', 'Color', 'white');

% --- Zaman Domeni Grafiği ---
subplot(2,1,1);
plot(t, X, 'LineWidth', 1.5) % İlk 50 milisaniyeyi gösterelim
title('Orijinal Sinyal (Zaman Domeni - Kesit)');
xlabel('Zaman (s)');
ylabel('Genlik X(t)');
grid on;

% --- Frekans Domeni (FFT) Grafiği ---
subplot(2,1,2);
plot(f, P1, 'r', 'LineWidth', 1.5) 
title('Sinyalin Tek Taraflı Genlik Spektrumu (Frekans Domeni)');
xlabel('Frekans (Hz)');
ylabel('|P1(f)|');
grid on;
% Gürültüden ayırt etmek için sadece 0-200Hz arasını vurgulayalım (Opsiyonel)
xlim([0 100]);