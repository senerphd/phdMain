%% projection_notes.m
% Simulink'te Parameter Projection implementasyonu — adım adım talimatlar
% Kaynak: Ioannou & Sun §8.4.2, Eq.(8.4.28)
%
% MEVCUT adaptasyon yasası (model içinde):
%   dL_hat/dt = -Gamma_L * Lambda .* z * f(phi)   [dead-zone aktif]
%   dM_hat/dt = -Gamma_M * r * f(phi)
%
% HEDEF: L_hat(i) sınırlarını aşmasın.
% En temiz Simulink implementasyonu: Integratörün girişine
% "projection_switch" MATLAB Function bloğu ekle.
%
% -------------------------------------------------------------------------
% SIMULINK'E EKLENECEK MATLAB FUNCTION BLOĞU:
% -------------------------------------------------------------------------
%
% Blok adı: "ProjectionSwitch_L"
% Girişler: dL_raw [3×1], L_hat [3×1]
% Çıkış:    dL_proj [3×1]
%
% function dL_proj = fcn(dL_raw, L_hat)
%   % Projection sınırları (newDesign.m'den workspace'e aktarılmalı)
%   L_min = coder.load('proj_bounds.mat','L_hat_min').L_hat_min;
%   L_max = coder.load('proj_bounds.mat','L_hat_max').L_hat_max;
%   dL_proj = dL_raw;
%   for i = 1:3
%     if (L_hat(i) >= L_max(i) && dL_raw(i) > 0)
%         dL_proj(i) = 0;   % sınır dışına itecek → durdur
%     elseif (L_hat(i) <= L_min(i) && dL_raw(i) < 0)
%         dL_proj(i) = 0;   % sınır dışına itecek → durdur
%     end
%   end
% end
%
% -------------------------------------------------------------------------
% ALTERNATIF (daha basit): Integratör saturation kullan
% -------------------------------------------------------------------------
% Simulink integratör bloğunda "Limit output" özelliği var.
% ANCAK bu tek başına yetmez — upper saturation'da olduğunda
% pozitif update hala integratörü sıkıştırmaya devam eder (anti-windup yok).
%
% Doğru yol: Integratör + External Reset veya
%            MATLAB Function bloğu (yukarıdaki gibi)
%
% -------------------------------------------------------------------------
% EN HIZLI ÇÖZÜM: Integratör bloğuna "Saturate output" +
%                 "Prevent output from leaving saturation" (anti-windup)
% -------------------------------------------------------------------------
% Simulink integratör parametreleri:
%   - Upper saturation limit: L_hat_max(i)  (her eksen için ayrı blok)
%   - Lower saturation limit: L_hat_min(i)
%   - Anti-windup method: "back-calculation" VEYA
%     "Limit output" işaretlendi + harici hesaplama
%
% Simulink'te 3 ayrı 1-D integratör kullanılıyorsa her biri için:
%   Upper limit: L_hat_max(i) değeri
%   Lower limit: L_hat_min(i) değeri
%
% Bu yöntem §8.4.2 denklem (8.4.28)'in component-wise versiyonudur:
%   "θ_dot = 0 if |θ| = M₀ AND θ·ε₁·u > 0"
% Integratör saturation tam olarak bunu yapar.
%
% -------------------------------------------------------------------------
% KRITIK FARK: Integratör saturation vs true projection
% -------------------------------------------------------------------------
% Integratör saturation: L_hat(i) değerini sabit tutar sınırda
%   → L_hat(i) sınırı geçemez
%   → AMA update terimi hala hesaplanır, integratör "stuck" kalır
%   → Anti-windup ile bu düzeltilir: update dışarı itecekse sıfırlanır
%
% True projection: update terimi sıfırlanır (Lyapunov ispatı için gerekli)
%   → Matematiksel olarak daha doğru
%   → Implementasyonu biraz daha karmaşık
%
% Pratik olarak her ikisi de aynı etkiyi verir çünkü
% integratör sınırında kaldığında Lyapunov fonksiyonu artmaz.
%
% =========================================================================
% TAVSIYE: Simulink'te en basit implementasyon:
% =========================================================================
% 1. L_hat integratörlerini bul (3 adet: L1, L2, L3 için)
% 2. Her integratörde "Limit output" kutusunu işaretle
% 3. Upper/lower limitleri L_hat_max/min değerlerine bağla (Constant blok)
% 4. Anti-windup için "clamping" yöntemini seç
%
% Workspace'den:
%   L1_max = L_hat_max(1);  L1_min = L_hat_min(1);
%   L2_max = L_hat_max(2);  L2_min = L_hat_min(2);
%   L3_max = L_hat_max(3);  L3_min = L_hat_min(3);
%   M_max  = M_hat_max;     M_min  = M_hat_min;

% newDesign.m çalıştırıldıktan sonra bu değerler workspace'de mevcut olacak.

%% Projection sınırlarını kaydet (Simulink constant bloğu için)
% newDesign.m çalıştırıldıktan sonra:
L1_min = L_hat_min(1);  L1_max = L_hat_max(1);
L2_min = L_hat_min(2);  L2_max = L_hat_max(2);
L3_min = L_hat_min(3);  L3_max = L_hat_max(3);
M_min  = M_hat_min;     M_max  = M_hat_max;

fprintf('Simulink Constant blokları için değerler:\n');
fprintf('L1: [%.4f, %.4f]\n', L1_min, L1_max);
fprintf('L2: [%.4f, %.4f]  ← Bu en kritik\n', L2_min, L2_max);
fprintf('L3: [%.4f, %.4f]\n', L3_min, L3_max);
fprintf('M:  [%.4f, %.4f]\n', M_min, M_max);
