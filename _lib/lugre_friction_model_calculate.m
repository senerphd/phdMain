function F_friction = lugre_friction_model_calculate(zaman, xc_dot)
% LUGRE_FRICTION_MODEL_CALCULATE
%
% Optimize edilmiş parametreler kullanılarak Lugre sürtünme modelini hesaplar.
% Model denklemi: F_friction = sigma0 * z + sigma1 * dz/dt + Fv * xc_dot
% z dinamiği: dz/dt = xc_dot - (|xc_dot| / g(xc_dot)) * z
% g(xc_dot): Statik sürtünme fonksiyonu
%
% KULLANIM:
%   dt = 0.001; % Örnekleme zamanı
%   zaman = (0:length(xc_dot)-1)' * dt; 
%   F_friction_tahmin = lugre_friction_model_calculate(zaman, xc_dot);
%
% GİRDİLER:
%   zaman  : Zaman vektörü [s]
%   xc_dot : Silindir hızı vektörü [m/s]
%
% ÇIKTILAR:
%   F_friction : Tahmin edilen sürtünme kuvveti vektörü [N]

    % --- 1. Optimize Edilmiş Parametreler ---
    % Sizin son optimizasyon çıktınızdan alınan sabitler
    P_opt(1) = 5.00e6;    % sigma0 (Kıl Sertliği) [N/m]
    P_opt(2) = 100.0000;  % sigma1 (Kıl Sönümleme) [Ns/m]
    P_opt(3) = 80.0000;   % Fv (Viskoz Katsayısı) [Ns/m]
    P_opt(4) = 21.0000;   % Fc (Coulomb Sürtünmesi) [N]
    P_opt(5) = 54.0000;   % Fs (Statik Sürtünme) [N]
    P_opt(6) = 0.0100;    % vs (Stribeck Hızı) [m/s]
    
    dt = mean(diff(zaman)); % Zaman adımını hesapla
    
    % --- 2. ODE Çözümü için Hazırlık ---
    
    % Hız verisini zaman fonksiyonu olarak interpolasyon ile tanımlama
    % ODE çözücüsünün ara değerleri okuması için gereklidir
    xc_dot_interp = @(t) interp1(zaman, xc_dot, t, 'linear', 'extrap');
    
    z0 = 0; % z için başlangıç koşulu
    
    % --- 3. ODE Çözümü (z dinamiği) ---
    
    % ode45 ile z durum değişkeninin zamana bağlı değişimini çözme
    [~, z_ode] = ode45(@(t, z) lugre_ode_fcn(t, z, xc_dot_interp, P_opt), zaman, z0);
    z = z_ode(:, 1);
    
    % z'nin Türevi (dz/dt)
    dz_dt = gradient(z, dt);
    
    % --- 4. Sürtünme Kuvveti Hesaplama ---
    
    sigma0 = P_opt(1);
    sigma1 = P_opt(2);
    Fv     = P_opt(3);
    
    % F_friction = sigma0 * z + sigma1 * dz/dt + Fv * xc_dot
    F_friction = sigma0 * z + sigma1 * dz_dt + Fv * xc_dot;

end

% --- Lugre ODE Fonksiyonu (z Dinamiği) ---
function dz_dt = lugre_ode_fcn(t, z, xc_dot_interp, p)
% Durum denklemi: dz/dt = xc_dot - (|xc_dot| / g(xc_dot)) * z
    
    sigma0 = p(1);
    Fc     = p(4);
    Fs     = p(5);
    vs     = p(6);
    
    % Mevcut zamandaki hızı interpolasyon ile bulma
    xc_dot_val = xc_dot_interp(t);
    
    % Statik Sürtünme Fonksiyonu g(xc_dot)
    % g_val = Fd / sigma0
    g_val = (Fc + (Fs - Fc) * exp(-(xc_dot_val / vs)^2)) / sigma0;
    
    % z'nin dinamiği: dz/dt = xc_dot - (|xc_dot| / g(xc_dot)) * z
    if abs(xc_dot_val) < 1e-6 % Hız sıfıra yakınken stabilite için
        dz_dt = 0;
    else
        dz_dt = xc_dot_val - (abs(xc_dot_val) / g_val) * z;
    end
end