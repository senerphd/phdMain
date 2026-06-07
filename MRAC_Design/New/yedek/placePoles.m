%% İDEAL REFERANS MODEL TASARIMI (Pole Placement Yöntemi ile)

% 2. İstenen Kutupların Belirlenmesi
% (Not: Fiziksel sınırları zorlamayacak, sönümlemesi iyi kutuplar)
wn = 120;            % Doğal frekans (rad/s)
zeta = 0.707;       % Sönümleme oranı

p1 = -zeta*wn + 1i*wn*sqrt(1-zeta^2);
p2 = -zeta*wn - 1i*wn*sqrt(1-zeta^2);
p3 = -150;          % Hızlı basınç kutbu

hedef_kutuplar = [p1, p2, p3];

% 3. Nominal durum geribesleme kazancının bulunması
Km = place(A_bar, B_bar_mA, hedef_kutuplar);

% 4. Referans Model Matrislerinin Oluşturulması
Am_bar = A_bar - B_bar_mA * Km;

% 5. Bm Matrisinin DC Kazancı 1 Yapacak Şekilde Ayarlanması
% Sistemin steady-state'de referansı (r) tam yakalaması için:
C_pos = [1, 0, 0]; % Konum çıkışı
DC_gain = C_pos * inv(-Am_bar) * B_bar_mA;
Bm_bar_mm = B_bar_mA / DC_gain;

sys_refModel = ss(Am_bar,Bm_bar_mm,C_pos,zeros(1,1)); 
bode(sys_refModel) 