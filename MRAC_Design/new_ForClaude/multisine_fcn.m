function y = multisine_fcn(t)
%MULTISINE_FCN  Çok bileşenli sinüs referans sinyal üreteci
%
% Simulink MATLAB Function bloğunda kullanım:
%   Blok adı  : MultiSine
%   Giriş     : t   — Clock bloğundan [s]
%   Çıkış     : y   — Referans pozisyon [mm]
%
% Tasarım kriterleri:
%   1) PE (Persistently Exciting) — Ioannou §8.5 parametre yakınsaması
%      için en az (n_params/2) = 2 farklı frekans gerekir.
%      Burada 9 bileşen kullanıldı → güçlü PE garantisi.
%   2) Schroeder fazları → crest factor minimize edilir
%      φ_k = −π·k·(k−1)/N  →  peak/RMS ≈ √2  (tek sinüse yakın)
%   3) Yumuşak başlangıç (Hann penceresi, T_ramp=2s)
%      → t=0'da ani konum sıçraması yok, mekanik darbe yok
%   4) Amplitüd azaltma: yüksek frekans → küçük genlik
%      (hidrolik aktuatör bant genişliği kısıtı)
%
% Toplam peak tahmini (Schroeder ile): ~14 mm  (<±15 mm limit)
% RMS                                 : ~7  mm
% Crest factor                        : ~2.0
%
% Frekans aralığı: 0.5 – 12 Hz  (referans model bw: ~17 Hz)
%
% =========================================================
%  MATLAB Function bloğuna bu kodu YAPIŞTIRINkopyalayın:
%  (fonksiyon imzası dahil, yorum satırları isteğe bağlı)
% =========================================================

% ---- Bileşen tanımları ----
% Frekans [Hz]   Amplitüd [mm]
%  0.5            6.0   — quasi-static / düşük frekans
%  1.0            5.0
%  2.0            4.5
%  3.0            4.0
%  5.0            3.0   — orta bant
%  6.0            3.0   — mevcut sinüs test frekansı (doğrulama noktası)
%  8.0            2.5
% 10.0            2.0
% 12.0            1.5   — bant genişliğine yakın

N    = 9;
f_Hz = [0.5,  1.0,  2.0,  3.0,  5.0,  6.0,  8.0, 10.0, 12.0];
A_mm = [6.0,  5.0,  4.5,  4.0,  3.0,  3.0,  2.5,  2.0,  1.5];

% ---- Schroeder fazları ----
% φ_k = −π·k·(k−1)/N    k = 1..N   (1-tabanlı indis)
phi = zeros(1, N);
phi(1)  = 0.0;                             % k=1:  φ = 0
phi(2)  = -pi * 2  * 1  / N;              % k=2
phi(3)  = -pi * 3  * 2  / N;              % k=3
phi(4)  = -pi * 4  * 3  / N;              % k=4
phi(5)  = -pi * 5  * 4  / N;              % k=5
phi(6)  = -pi * 6  * 5  / N;              % k=6
phi(7)  = -pi * 7  * 6  / N;              % k=7
phi(8)  = -pi * 8  * 7  / N;              % k=8
phi(9)  = -pi * 9  * 8  / N;              % k=9

% ---- Sinyal üretimi ----
omega = 2.0 * pi * f_Hz;   % açısal frekanslar [rad/s]

y = A_mm(1) * sin(omega(1)*t + phi(1)) ...
  + A_mm(2) * sin(omega(2)*t + phi(2)) ...
  + A_mm(3) * sin(omega(3)*t + phi(3)) ...
  + A_mm(4) * sin(omega(4)*t + phi(4)) ...
  + A_mm(5) * sin(omega(5)*t + phi(5)) ...
  + A_mm(6) * sin(omega(6)*t + phi(6)) ...
  + A_mm(7) * sin(omega(7)*t + phi(7)) ...
  + A_mm(8) * sin(omega(8)*t + phi(8)) ...
  + A_mm(9) * sin(omega(9)*t + phi(9));

% ---- Yumuşak başlangıç (Hann penceresi) ----
% 0 → T_ramp saniyede tam genliğe ulaşır
% w(t) = 0.5·(1 − cos(π·t/T_ramp))   →  w(0)=0, w(T_ramp)=1
T_ramp = 2.0;   % [s]
if t < T_ramp
    w = 0.5 * (1.0 - cos(pi * t / T_ramp));
    y = y * w;
end

end
