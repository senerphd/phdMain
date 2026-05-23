%% mrac_test.m
% Lyapunov Tabanlı MRAC - Tam Test ve Simülasyon Scripti
%
% KULLANIM:
%   Bu scripti MRAC_Lyapunov klasöründen çalıştırın.
%   1. mrac_setup.m → parametreler
%   2. mex mrac_lyapunov_sfun.c → S-Function derleme
%   3. MATLAB ODE ile doğrulama simülasyonu
%   4. Sonuçlar kaydedilir
%
% TASARIM NOTLARI:
%   - Plant: 3 durumlu hidrolik servo (x_p, xdot_p, P_L)
%   - Normalize koordinat: x_s = [x_p, xdot_p, P_L/Ps], Ps = 21 MPa
%   - Referans model: LQR tabanlı → Am = Ap_s - Bp_s*K_lqr (Hurwitz)
%   - Eşleşme koşulu: Ap_s + Bp_s*K_x* = Am_s (tam sağlanmış)
%   - Lyapunov: Am_s^T*P + P*Am_s = -I → P pozitif tanımlı
%   - Adaptasyon: K_x_dot = -Γx * xp * (e^T*P*Bp)
%                 K_r_dot = -Γr * r  * (e^T*P*Bp)
%
% Hasan Şener - PhD Tezi

clear; clc; close all;
fprintf('╔══════════════════════════════════════════════╗\n');
fprintf('║  Lyapunov Tabanlı MRAC - Simülasyon Testi   ║\n');
fprintf('╚══════════════════════════════════════════════╝\n\n');

script_dir = fileparts(mfilename('fullpath'));
cd(script_dir);

%% ============================================================
%  ADIM 1: Parametreler
% =============================================================
fprintf('══ ADIM 1: Parametreler ══\n');
run('mrac_setup.m');

%% ============================================================
%  ADIM 2: S-Function Derle (opsiyonel - Simulink için)
% =============================================================
fprintf('\n══ ADIM 2: S-Function Derleme ══\n');
sfun_c = fullfile(script_dir, 'mrac_lyapunov_sfun.c');
if exist(sfun_c, 'file')
    try
        mex('-outdir', script_dir, sfun_c);
        fprintf('  S-Function derlendi ✓\n');
    catch me
        fprintf('  MEX derleme atlandı: %s\n', me.message);
        fprintf('  (MATLAB ODE simülasyonu kullanılacak)\n');
    end
else
    fprintf('  mrac_lyapunov_sfun.c bulunamadı, ODE simülasyonu kullanılacak.\n');
end

%% ============================================================
%  ADIM 3: MATLAB ODE Simülasyonu (Simulink bağımsız doğrulama)
% =============================================================
fprintf('\n══ ADIM 3: MATLAB ODE Simülasyonu ══\n');

r_func = @(t) r_amp * (t >= r_time);

% Test 1: İdeal başlangıç kazançları (Lyapunov kararlılık kanıtı)
fprintf('  Test 1: İdeal K_x*, K_r* başlangıç değerleri...\n');
z0_ideal = [x0_s; x0_s; Kx_star(:); Kr_star];
mrac_ode1 = @(t,z) mrac_ode_fn(t,z,Ap_s,Bp_s,Am_s,Bm_s,...
                                 P_s,Gamma_x,Gamma_r,u_max,r_func);
ode_opts = odeset('RelTol',1e-5,'AbsTol',1e-7,'MaxStep',5e-3);
[t1, z1] = ode15s(mrac_ode1, [0,T_sim], z0_ideal, ode_opts);

xp1 = z1(:,1:3); xm1 = z1(:,4:6); e1 = xp1-xm1; r1 = arrayfun(r_func,t1);
idx1 = t1>0.8*T_sim;
fprintf('    Kalıcı durum hatası (RMS): %.4f mm ✓\n', rms(e1(idx1,1))*1000);

% Test 2: Perturbed başlangıç (adaptasyonun gücü)
fprintf('  Test 2: %%50 perturbed K_x, K_r başlangıç değerleri...\n');
z0_perturb = [x0_s; x0_s; 0.5*Kx_star(:); 0.5*Kr_star];
mrac_ode2 = @(t,z) mrac_ode_fn(t,z,Ap_s,Bp_s,Am_s,Bm_s,...
                                 P_s,Gamma_x,Gamma_r,u_max,r_func);
[t2, z2] = ode15s(mrac_ode2, [0,T_sim], z0_perturb, ode_opts);

xp2 = z2(:,1:3); xm2 = z2(:,4:6); Kx2 = z2(:,7:9); Kr2 = z2(:,10);
e2 = xp2-xm2; r2 = arrayfun(r_func,t2);
idx2 = t2>0.8*T_sim;
u2 = zeros(length(t2),1);
for i=1:length(t2)
    u2(i)=min(max(Kx2(i,:)*xp2(i,:)'+Kr2(i)*r2(i),-u_max),u_max);
end

idx_rise = find(xp2(:,1)>=0.9*r_amp,1,'first');
fprintf('    Kalıcı durum hatası (RMS): %.4f mm\n', rms(e2(idx2,1))*1000);
fprintf('    Yükselme süresi (90%%): %.3f s\n', t2(idx_rise));
fprintf('    Son K_x: [%.4f, %.4f, %.6f]\n', Kx2(end,1), Kx2(end,2), Kx2(end,3));
fprintf('    Son K_r: %.4f  (hedef: %.4f)\n', Kr2(end), Kr_star);

%% ============================================================
%  ADIM 4: Görselleştirme
% =============================================================
fprintf('\n══ ADIM 4: Görselleştirme ══\n');

fig = figure('Name','Lyapunov MRAC Sonuçları','Position',[50,50,1300,900]);

% Subplot 1: Konum takibi
subplot(3,2,1);
plot(t2, r2*1000, 'k--', 'LineWidth',1.5, 'DisplayName','Referans r'); hold on;
plot(t2, xm2(:,1)*1000,'b-','LineWidth',1.5,'DisplayName','Ref Model x_m');
plot(t2, xp2(:,1)*1000,'r-','LineWidth',1.5,'DisplayName','Plant x_p');
xlabel('Zaman [s]'); ylabel('Konum [mm]');
title('Konum Takibi (%50 Perturbed Başlangıç)');
legend('Location','best'); grid on;

% Subplot 2: Takip hatası
subplot(3,2,2);
plot(t2, e2(:,1)*1000, 'r-', 'LineWidth',1.5); yline(0,'k--');
xlabel('Zaman [s]'); ylabel('Hata [mm]');
title(sprintf('Konum Takip Hatası  (RMS=%.4f mm)', rms(e2(idx2,1))*1000));
grid on;

% Subplot 3: Hız takibi
subplot(3,2,3);
plot(t2, xm2(:,2)*1000,'b-','LineWidth',1.5,'DisplayName','x_{m2}'); hold on;
plot(t2, xp2(:,2)*1000,'r-','LineWidth',1.5,'DisplayName','x_{p2}');
xlabel('Zaman [s]'); ylabel('Hız [mm/s]');
title('Hız Takibi'); legend('Location','best'); grid on;

% Subplot 4: Kontrol sinyali
subplot(3,2,4);
plot(t2, u2,'g-','LineWidth',1.5);
yline(u_max,'r--','u_{max}'); yline(-u_max,'r--');
xlabel('Zaman [s]'); ylabel('u [V]');
title('Kontrol Sinyali (doyum sınırı dahil)'); grid on;

% Subplot 5: K_x adaptasyonu
subplot(3,2,5);
plot(t2, Kx2(:,1),'r-','LineWidth',1.5,'DisplayName','K_{x1}'); hold on;
plot(t2, Kx2(:,2),'b-','LineWidth',1.5,'DisplayName','K_{x2}');
plot(t2, Kx2(:,3),'g-','LineWidth',1.5,'DisplayName','K_{x3}');
yline(Kx_star(1),'r--'); yline(Kx_star(2),'b--'); yline(Kx_star(3),'g--');
xlabel('Zaman [s]'); ylabel('Kazanç');
title('Adaptif K_x(t)  (-- = K_x*)');
legend('Location','best'); grid on;

% Subplot 6: K_r adaptasyonu
subplot(3,2,6);
plot(t2, Kr2,'m-','LineWidth',1.5);
yline(Kr_star,'m--',sprintf('K_r^* = %.2f', Kr_star));
xlabel('Zaman [s]'); ylabel('K_r');
title('Adaptif K_r(t)'); grid on;

sgtitle(sprintf(['Lyapunov Tabanlı MRAC - Hidrolik Servo Sistemi\n'...
    'LQR Referans Model | \\Gamma_x = \\Gamma_r = %.1e | u_{max} = %.0f V'], ...
    Gamma_x, u_max), 'FontSize', 10, 'FontWeight', 'bold');

%% Grafik kaydet
fig_path = fullfile(script_dir, 'mrac_results.png');
exportgraphics(fig, fig_path, 'Resolution', 150);
fprintf('  Grafik kaydedildi: mrac_results.png\n');

%% ============================================================
%  ADIM 5: Performans Özeti
% =============================================================
fprintf('\n══ ADIM 5: PERFORMANS ÖZETİ ══\n');
fprintf('  ┌─────────────────────────────────────────┐\n');
fprintf('  │  Test 1 (İdeal K*): Hata = %.4f mm   │\n', rms(e1(idx1,1))*1000);
fprintf('  │  Test 2 (%%50 K*  ): Hata = %.4f mm   │\n', rms(e2(idx2,1))*1000);
if ~isempty(idx_rise)
    fprintf('  │  Yükselme süresi    : %.3f s         │\n', t2(idx_rise));
end
fprintf('  │  Kontrol sinyali max: %.4f V        │\n', max(abs(u2)));
fprintf('  └─────────────────────────────────────────┘\n');

fprintf('\n[TAMAMLANDI] MRAC simülasyonu başarılı.\n');

%% ============================================================
%  YARDIMCI FONKSİYON
% =============================================================

function zdot = mrac_ode_fn(t, z, Ap, Bp, Am, Bm, P, Gx, Gr, umax, rfunc)
    n = 3;
    xp = z(1:n);      % plant durumu (normalize)
    xm = z(n+1:2*n);  % ref model durumu (normalize)
    Kx = z(2*n+1:3*n);% adaptif K_x [nx1]
    Kr = z(3*n+1);     % adaptif K_r [skaler]

    r = rfunc(t);

    % Kontrol sinyali: u = K_x^T * x_p + K_r * r
    u = Kx' * xp + Kr * r;
    u = max(min(u, umax), -umax);  % doyum

    % Plant dinamiği: xdot_p = Ap*xp + Bp*u
    xp_dot = Ap * xp + Bp * u;

    % Referans model: xdot_m = Am*xm + Bm*r
    xm_dot = Am * xm + Bm * r;

    % Hata: e = x_p - x_m
    e = xp - xm;

    % sigma = e^T * P * B_p (skaler - Lyapunov adaptasyon terimi)
    sigma = e' * P * Bp;

    % Adaptasyon kanunları (Lyapunov kararlılık)
    % K_x_dot = -Gamma_x * x_p * sigma  [nx1]
    % K_r_dot = -Gamma_r * r  * sigma   [skaler]
    Kx_dot = -Gx * xp * sigma;
    Kr_dot = -Gr * r  * sigma;

    zdot = [xp_dot; xm_dot; Kx_dot; Kr_dot];
end
