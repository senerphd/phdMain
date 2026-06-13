%% 1. İstenen Kapalı-Çevrim Kutupları
wn   = 120;   % Doğal frekans (rad/s)  — 6Hz sinüs için yüksek bant genişliği
zeta = 0.9;  % Sönümleme oranı       — underdamped, step overshoot pre-filter ile engelleniyor
p3   = -150; % 3. kutup              — hidrolik bandwidth ~73 rad/s'in altında

% Sönümleme oranına göre kutup hesabı (Sayısal olarak daha güvenli)
if zeta < 1
    p1 = -zeta*wn + 1i*wn*sqrt(1-zeta^2);
    p2 = -zeta*wn - 1i*wn*sqrt(1-zeta^2);
else
    p1 = -zeta*wn - wn*sqrt(zeta^2-1);
    p2 = -zeta*wn + wn*sqrt(zeta^2-1);
end

       

hedef_kutuplar = [p1, p2, p3];

%% 2. Nominal Durum Geribesleme Kazancı (Pole Placement)
% Kutupların çakışmadığından (distinct) emin olunmalıdır.
Km = place(A_bar, B_bar_mA, hedef_kutuplar);

%% 3. Referans Model Matrisleri
Am_bar = A_bar - B_bar_mA * Km;

%% 4. DC Kazancı 1 Olacak Şekilde Bm Ayarı
C_pos  = [1, 0, 0];              % Konum çıkışı
% inv() yerine daha kararlı olan backslash (\) operatörü kullanıldı:
DC_gain = C_pos * (-Am_bar \ B_bar_mA);
Bm_bar_mm = B_bar_mA / DC_gain;

%% 5. Doğrulama
sys_refModel = ss(Am_bar, Bm_bar_mm, C_pos, zeros(1,1));
ev = eig(Am_bar);
fprintf('Referans Model Kutupları:\n');
for i = 1:length(ev)
    if imag(ev(i)) ~= 0
        fprintf('  p%d = %.2f ± j%.2f  (wn=%.1f rad/s, zeta=%.3f)\n', ...
            i, real(ev(i)), abs(imag(ev(i))), abs(ev(i)), -real(ev(i))/abs(ev(i)));
    else
        fprintf('  p%d = %.2f\n', i, real(ev(i)));
    end
end
fprintf('DC Kazancı = %.4f\n', DC_gain);
fprintf('Km = [%.5f  %.5f  %.5f]\n', Km(1), Km(2), Km(3));