function F_friction = lugre_asymmetric_model_calculate(zaman, xc_dot)
% % LUGRE_ASYMMETRIC_MODEL_CALCULATE
% % Hız yönüne (+ / -) göre farklı parametre setleri kullanan asimetrik Lugre modeli.
% %
% % P_asym yapısı (12 elemanlı vektör):
% %   [sigma0+, sigma1+, Fv+, Fc+, Fs+, vs+, sigma0-, sigma1-, Fv-, Fc-, Fs-, vs-]
% 
%     % --- 1. Parametrelerin Ayrılması ---
% 
%     % Pozitif Yön Parametreleri (P+)
% 
%     % --- 1. Optimize Edilmiş Parametreler ---
    % Sizin son optimizasyon çıktınızdan alınan sabitler
    sigma0_plus = 1e7;    % sigma0 (Kıl Sertliği) [N/m]
    sigma1_plus = 0;  % sigma1 (Kıl Sönümleme) [Ns/m]
    Fv_plus     = 18;   % Fv (Viskoz Katsayısı) [Ns/m] 18/0,05
    Fc_plus     = 17.32;   % Fc (Coulomb Sürtünmesi) [N]
    Fs_plus     = 72.5264;   % Fs (Statik Sürtünme) [N]
    vs_plus     = 0.01;       % vs (Stribeck Hızı) [m/s]

    sigma0_minus = sigma0_plus;    % sigma0 (Kıl Sertliği) [N/m]
    sigma1_minus = sigma1_plus ;  % sigma1 (Kıl Sönümleme) [Ns/m]
    Fv_minus     = 30; %21.526;   % Fv (Viskoz Katsayısı) [Ns/m]
    Fc_minus     = 27.2383;   % Fc (Coulomb Sürtünmesi) [N]
    Fs_minus     = 72;    % Fs (Statik Sürtünme) [N]
    vs_minus     = 0.02;    % vs (Stribeck Hızı) [m/s]

    P_asym = [sigma0_plus, sigma1_plus, Fv_plus, Fc_plus, Fs_plus, vs_plus,sigma0_minus, sigma1_minus, Fv_minus, Fc_minus, Fs_minus, vs_minus]; 


    dt = mean(diff(zaman)); % Zaman adımını hesapla

    % --- 2. ODE Çözümü için Hazırlık ---

    % Hız verisini zaman fonksiyonu olarak interpolasyon ile tanımlama
    xc_dot_interp = @(t) interp1(zaman, xc_dot, t, 'linear', 'extrap');

    z0 = 0; % z için başlangıç koşulu

    % --- 3. ODE Çözümü (z dinamiği) ---

    % ODE çözücüsüne tüm asimetrik parametreleri gönderiyoruz.
    [~, z_ode] = ode45(@(t, z) lugre_ode_fcn_asym(t, z, xc_dot_interp, P_asym), zaman, z0);
    z = z_ode(:, 1);

    % z'nin Türevi (dz/dt)
    dz_dt = gradient(z, dt);

    % --- 4. Sürtünme Kuvveti Hesaplama (Asimetrik) ---

    % Her bir zaman adımı için doğru parametreyi seçerek F_friction'ı hesaplıyoruz
    N = length(xc_dot);
    F_friction = zeros(N, 1);

    for i = 1:N
        xc_dot_val = xc_dot(i);

        if xc_dot_val >= 0 % Pozitif veya durma (torkun pozitif olduğu varsayımı)
            sigma0 = sigma0_plus;
            sigma1 = sigma1_plus;
            Fv     = Fv_plus;
        else % Negatif
            sigma0 = sigma0_minus;
            sigma1 = sigma1_minus;
            Fv     = Fv_minus;
        end

        % F_friction = sigma0 * z + sigma1 * dz/dt + Fv * xc_dot
        F_friction(i) = sigma0 * z(i)    + sigma1 * dz_dt(i) + Fv * xc_dot_val;
    end
end


% --- Asimetrik Lugre ODE Fonksiyonu (z Dinamiği) ---
function dz_dt = lugre_ode_fcn_asym(t, z, xc_dot_interp, P_asym)

    % Hız parametresini al
    xc_dot_val = xc_dot_interp(t);

    % Yön belirleme ve parametreleri seçme
    if xc_dot_val >= 0 % Pozitif yön parametreleri (1:6)
        p = P_asym(1:6);
    else % Negatif yön parametreleri (7:12)
        p = P_asym(7:12);
    end

    % Seçilen parametrelerden değerleri çıkarma
    sigma0 = p(1);
    Fc     = p(4);
    Fs     = p(5);
    vs     = p(6);

    % Statik Sürtünme Fonksiyonu g(xc_dot)
    g_val = (Fc + (Fs - Fc) * exp(-1 * ((xc_dot_val / vs)^2))) / sigma0;

    % z'nin dinamiği: dz/dt = xc_dot - (|xc_dot| / g(xc_dot)) * z

        dz_dt = xc_dot_val - (abs(xc_dot_val) / g_val) * z;
    
end

%% 
% --- SÜREKLİ ZAMANLI LUGRE MODEL SİMÜLASYONU FONKSİYONU ---
function F_friction = lugre_simulation(params, time, xc_dot)
    
    % Zaman adımı (ode45'in dahili çözümü daha hassastır)
    dt = time(2) - time(1); 
    
    % Parametre ataması
    sigma0 = params(1);
    sigma1 = params(2);
    sigma2 = params(3);
    Fc_pos = params(4);
    Fs_pos = params(5);
    vs_pos = params(6);
    Fc_neg = params(7);
    Fs_neg = params(8);
    vs_neg = params(9);
    F0     = params(10);

    
    % Başlangıç koşulu
    z0 = 0; % Başlangıç kıl sapması
    
    % Hız verisini diferansiyel denklem için erişilebilir hale getirme
    v_interp = griddedInterpolant(time, xc_dot, 'linear', 'none');
    
    % ODE Çözücüsü (Diferansiyel Denklemi Çözme)
    [~, Z] = ode45(@(t, z) lugre_ode_system(t, z, v_interp, params), time, z0);
    
    z = Z(:, 1);
    
    % Kıl Sapması Türevi (dz/dt) için yaklaşık hesaplama
    % Geriye doğru fark (Backwards Difference) ile dz/dt hesaplanır.
    dz_dt = zeros(size(xc_dot));
    for k = 2:length(z)
        dz_dt(k) = (z(k) - z(k-1)) / dt;
    end
    dz_dt(1) = dz_dt(2); % İlk noktayı eşitleme

    % Sürtünme Kuvveti (F_r) - Denklem (9) [cite: 258]
    F_friction = sigma0 * z + sigma1 * dz_dt + sigma2 * xc_dot + F0;

end

% --- LUGRE ODE SİSTEMİ (ode45 İÇİN) ---
function dzdt = lugre_ode_system(t, z, v_interp, params)
    
    % Parametre ataması (lugre_simulation'daki ile aynı)
    sigma0 = params(1);
    Fc_pos = params(4);
    Fs_pos = params(5);
    vs_pos = params(6);
    Fc_neg = params(7);
    Fs_neg = params(8);
    vs_neg = params(9);
    
    % Anlık hızı bulma
    v = v_interp(t);
    
    % Hızın yönüne göre Stribeck parametrelerini seçme (Pozitif/Negatif Hız)
    if v >= 0
        Fc_val = Fc_pos;
        Fs_val = Fs_pos;
        vs_val = vs_pos;
    else
        Fc_val = Fc_neg;
        Fs_val = Fs_neg;
        vs_val = vs_neg;
    end
    
    % Stribeck Fonksiyonu g(v) - Denklem (11) [cite: 417]
    g_v = Fc_val + (Fs_val - Fc_val) * exp(-(v / vs_val)^2);
    
    % Kıl Sapması Türevi (dz/dt) - Denklem (10) [cite: 413]
    % dz/dt = v - (sigma0 * |v| / g(v)) * z
    if g_v == 0
        dzdt = 0; % Bölme hatasını önleme
    else
        dzdt = v - sigma0 * abs(v) / g_v * z;
    end
end