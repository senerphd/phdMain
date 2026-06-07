%% REFERANS MODEL OPTİMİZASYONU (wn, zeta, p3)
% ZOH ayrıklaştırma - stabil ve hızlı
% Kullanım: newDesign.m sonrası çalıştır

%% Veri hazırlama
T_opt = 300; ds = 1;
idx    = xc_m.Time <= T_opt;
t_ds   = xc_m.Time(idx); t_ds = t_ds(1:ds:end);
xc_mm  = xc_m.Data(idx)*1e3;   xc_mm  = xc_mm(1:ds:end);
v_mmps = v_mPs.Data(idx)*1e3;  v_mmps = v_mmps(1:ds:end);
PL_bar = PL_Pa.Data(idx)/1e5;  PL_bar = PL_bar(1:ds:end);
r_mm   = r_m.Data(idx)*1e3;    r_mm   = r_mm(1:ds:end);
dt_ds  = t_ds(2)-t_ds(1);
z0     = [xc_mm(1); v_mmps(1); PL_bar(1)];
fprintf('N=%d, dt=%.1fms, T=%.0fs\n', length(t_ds), dt_ds*1e3, t_ds(end));

%% Başlangıç ve optimizasyon
p0 = [609.2, 0.123, -90.6];
c0 = refCostZOH(p0, A_bar, B_bar_mA, r_mm, xc_mm, v_mmps, PL_bar, z0, dt_ds);
fprintf('Başlangıç maliyeti: %.4f\n', c0);

opts = optimset('Display','iter','MaxIter',1e3,'TolFun',1e-8,'TolX',1e-8);
fn   = @(p) refCostZOH(p, A_bar, B_bar_mA, r_mm, xc_mm, v_mmps, PL_bar, z0, dt_ds);
tic; [p_opt, c_opt] = fminsearch(fn, p0, opts); t_opt = toc;

fprintf('\n=== SONUÇ (%.1fs) ===\n', t_opt);
fprintf('wn   = %.4f  (eski: %.1f)\n', p_opt(1), p0(1));
fprintf('zeta = %.6f  (eski: %.4f)\n', p_opt(2), p0(2));
fprintf('p3   = %.4f  (eski: %.1f)\n', p_opt(3), p0(3));
fprintf('Maliyet: %.4f -> %.4f\n', c0, c_opt);

%% Görselleştir
[zm0] = refIntZOH(p0,    A_bar, B_bar_mA, r_mm, z0, dt_ds);
[zmO] = refIntZOH(p_opt, A_bar, B_bar_mA, r_mm, z0, dt_ds);

figure('Name','Ref Model Opt','Color','w','Position',[50 50 1100 750]);
subplot(3,1,1)
plot(t_ds,xc_mm,'k','LineWidth',1.2); hold on
plot(t_ds,zm0(:,1),'b--','LineWidth',1); plot(t_ds,zmO(:,1),'r','LineWidth',1.4)
grid on; ylabel('Konum (mm)')
legend('Gerçek x_c','zm_0','zm_{opt}','Location','best')
title(sprintf('wn=%.1f  zeta=%.4f  p3=%.1f  |  cost: %.4f->%.4f',p_opt(1),p_opt(2),p_opt(3),c0,c_opt))

subplot(3,1,2)
plot(t_ds,v_mmps,'k','LineWidth',1.2); hold on
plot(t_ds,zm0(:,2),'b--','LineWidth',1); plot(t_ds,zmO(:,2),'r','LineWidth',1.4)
grid on; ylabel('Hız (mm/s)')
legend('Gerçek v','zm_0','zm_{opt}','Location','best')

subplot(3,1,3)
plot(t_ds,PL_bar,'k','LineWidth',1.2); hold on
plot(t_ds,zm0(:,3),'b--','LineWidth',1); plot(t_ds,zmO(:,3),'r','LineWidth',1.4)
grid on; ylabel('P_L (bar)'); xlabel('t (s)')
legend('Gerçek P_L','zm_0','zm_{opt}','Location','best')
title('P_L — düşük frekansta sapma normaldir')

%% placePoles.m güncelle
fprintf('\n--- placePoles.m güncelleme:\n');
fprintf('wn   = %.4f;\n', p_opt(1));
fprintf('zeta = %.6f;\n', p_opt(2));
fprintf('p3   = %.4f;\n', p_opt(3));

%% ===== YEREL FONKSİYONLAR =====

function cost = refCostZOH(p, A_bar, B_bar_mA, r_mm, xc_mm, v_mmps, PL_bar, z0, dt)
    wn=p(1); ze=p(2); p3=p(3);
    if wn<50||wn>3000||ze<0.02||ze>0.98||p3>-5||p3<-2000, cost=1e9; return; end
    zm = refIntZOH(p, A_bar, B_bar_mA, r_mm, z0, dt);
    if isempty(zm), cost=1e9; return; end
    s1=rms(xc_mm)+1e-6; s2=rms(v_mmps)+1e-6; s3=rms(PL_bar)+1e-6;
    cost = 3*rms(zm(:,1)-xc_mm)/s1 + 2*rms(zm(:,2)-v_mmps)/s2 + 1*rms(zm(:,3)-PL_bar)/s3;
end

function zm = refIntZOH(p, A_bar, B_bar_mA, r_mm, z0, dt)
    zm = [];
    try
        wn=p(1); ze=p(2); p3=p(3);
        p1 = -ze*wn + 1i*wn*sqrt(1-ze^2);
        Km = place(A_bar, B_bar_mA, [p1, conj(p1), p3]);
        Am = A_bar - B_bar_mA*Km;
        DC = [1,0,0]*(-Am\B_bar_mA);
        if abs(DC)<1e-12||~isreal(DC), return; end
        Bm = B_bar_mA/DC;
        Ad = expm(Am*dt);
        Bd = (Am\(Ad-eye(3)))*Bm;
        N  = length(r_mm);
        zm = zeros(N,3); zm(1,:) = z0';
        for k=1:N-1, zm(k+1,:)=(Ad*zm(k,:)' + Bd*r_mm(k))'; end
        if any(~isfinite(zm(:))), zm=[]; end
    catch
    end
end
