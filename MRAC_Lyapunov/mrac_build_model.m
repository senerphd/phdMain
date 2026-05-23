%% mrac_build_model.m
% Lyapunov MRAC - Simulink Modelini Programatik Olarak Oluşturur
%
% Bu script mrac_setup.m çalıştırıldıktan sonra çalıştırılmalıdır.
% Gerekli değişkenler workspace'te hazır olmalıdır.
%
% Hasan Şener - PhD Tezi

fprintf('=== MRAC Simulink Modeli Oluşturuluyor ===\n');

%% Parametrelerin var olduğunu kontrol et
required_vars = {'Ap','Bp_vec','Am','Bm','P_mat','Gamma_x','Gamma_r','u_max', ...
                 'T_sim','Ts_sim','x0_plant','x0_ref','r_amp','r_time','Kx0','Kr0'};
for i = 1:length(required_vars)
    if ~exist(required_vars{i}, 'var')
        error('"%s" bulunamadı. Önce mrac_setup.m çalıştırın.', required_vars{i});
    end
end
fprintf('  Gerekli değişkenler mevcut ✓\n');

%% S-Function derlenmiş mi?
sfun_path = fullfile(fileparts(mfilename('fullpath')), 'mrac_lyapunov_sfun');
if isempty(dir([sfun_path, '.*mex*']))
    fprintf('  UYARI: S-Function derlenmemiş. mrac_test.m çalıştırarak derleyin.\n');
end

%% Eski model varsa kapat
model_name = 'mrac_lyapunov';
if bdIsLoaded(model_name)
    close_system(model_name, 0);
    fprintf('  Eski model kapatıldı.\n');
end

%% Yeni model oluştur
new_system(model_name);
set_param(model_name, 'StopTime', num2str(T_sim));
set_param(model_name, 'Solver', 'ode45');
set_param(model_name, 'MaxStep', num2str(Ts_sim));
set_param(model_name, 'RelTol', '1e-4');
set_param(model_name, 'AbsTol', '1e-6');

fprintf('  Model oluşturuldu: %s\n', model_name);

%% ============================================================
%  BLOK KONUMLARI (piksel)
% =============================================================
% Sütun X konumları
xRef   = 50;
xCtrl  = 300;
xPlant = 600;
xScope = 900;

% Satır Y konumları
yRef   = 200;
yPlant = 400;
yKx    = 600;

bw = 120;  % blok genişliği
bh = 60;   % blok yüksekliği
bh2 = 80;

%% ============================================================
%  REFERANS SİNYAL (Step)
% =============================================================
add_block('simulink/Sources/Step', [model_name '/Reference_r'], ...
    'Time',           num2str(r_time), ...
    'Before',         '0', ...
    'After',          num2str(r_amp), ...
    'Position',       [xRef, yRef, xRef+bw, yRef+bh]);

%% ============================================================
%  REFERANS MODEL (State-Space)
% =============================================================
% Am ve Bm string olarak aktar
Am_str  = mat2str(Am,  8);
Bm_str  = mat2str(Bm,  8);
Cm_str  = mat2str(eye(3), 8);
Dm_str  = mat2str(zeros(3,1), 8);
x0_str  = mat2str(x0_ref, 8);

add_block('simulink/Continuous/State-Space', [model_name '/Ref_Model'], ...
    'A',          Am_str, ...
    'B',          Bm_str, ...
    'C',          Cm_str, ...
    'D',          Dm_str, ...
    'X0',         x0_str, ...
    'Position',   [xCtrl, yRef-10, xCtrl+bw, yRef+bh+10]);

%% ============================================================
%  PLANT (State-Space, doğrusal model)
% =============================================================
Ap_str  = mat2str(Ap,       8);
Bpv_str = mat2str(Bp_vec,   8);
Cpp_str = mat2str(eye(3),   8);
Dp_str  = mat2str(zeros(3,1),8);
x0p_str = mat2str(x0_plant, 8);

add_block('simulink/Continuous/State-Space', [model_name '/Plant'], ...
    'A',          Ap_str, ...
    'B',          Bpv_str, ...
    'C',          Cpp_str, ...
    'D',          Dp_str, ...
    'X0',         x0p_str, ...
    'Position',   [xCtrl, yPlant-10, xCtrl+bw, yPlant+bh+10]);

%% ============================================================
%  MRAC KONTROLCÜSÜ (C S-Function)
% =============================================================
% S-Function parametreleri: Gamma_x, Gamma_r, P_mat (9 elem), Bp_vec (3 elem), u_max
P_flat  = P_mat(:);    % 9 elemanlı sütun vektörü (column-major)
Bp_flat = Bp_vec(:);   % 3 elemanlı

sfun_params = sprintf('%g, %g, [%s], [%s], %g', ...
    Gamma_x, Gamma_r, ...
    num2str(P_flat.',  '%.6e '), ...
    num2str(Bp_flat.', '%.6e '), ...
    u_max);

add_block('simulink/User-Defined Functions/S-Function', [model_name '/MRAC_Controller'], ...
    'FunctionName',  'mrac_lyapunov_sfun', ...
    'Parameters',    sfun_params, ...
    'Position',      [xCtrl-150, yPlant+100, xCtrl-150+bw, yPlant+100+bh2]);

%% ============================================================
%  MUX BLOKU: [x_p(3); x_m(3); r(1)] → S-Function girişi
% =============================================================
add_block('simulink/Signal Routing/Mux', [model_name '/Mux_In'], ...
    'Inputs',     '3', ...
    'Position',   [xCtrl-220, yPlant+110, xCtrl-210, yPlant+130]);

%% ============================================================
%  SCOPE'LAR
% =============================================================
% 1. Konum karşılaştırma (y_plant vs y_ref)
add_block('simulink/Sinks/Scope', [model_name '/Scope_Position'], ...
    'NumInputPorts', '2', ...
    'Position',   [xScope, yRef, xScope+bw, yRef+bh]);

% 2. Kontrol sinyali u
add_block('simulink/Sinks/Scope', [model_name '/Scope_u'], ...
    'NumInputPorts', '1', ...
    'Position',   [xScope, yPlant, xScope+bw, yPlant+bh]);

% 3. Adaptif kazançlar K_x, K_r
add_block('simulink/Sinks/Scope', [model_name '/Scope_Gains'], ...
    'NumInputPorts', '2', ...
    'Position',   [xScope, yKx, xScope+bw, yKx+bh]);

%% ============================================================
%  SELECTOR BLOKLARI: Durum vektörünü elemanlara ayır
% =============================================================
% Plant'tan konum çıkarma (x_p[0])
add_block('simulink/Signal Routing/Selector', [model_name '/Sel_Plant_Pos'], ...
    'InputPortWidth', '3', ...
    'Indices',    '1', ...
    'Position',   [xPlant+50, yPlant-30, xPlant+70, yPlant-10]);

% Ref model'den konum çıkarma (x_m[0])
add_block('simulink/Signal Routing/Selector', [model_name '/Sel_Ref_Pos'], ...
    'InputPortWidth', '3', ...
    'Indices',    '1', ...
    'Position',   [xPlant+50, yRef+30, xPlant+70, yRef+50]);

% MRAC çıkışından K_x çıkarma (çıkış [2:4])
add_block('simulink/Signal Routing/Selector', [model_name '/Sel_Kx'], ...
    'InputPortWidth', '5', ...
    'Indices',    '2:4', ...
    'Position',   [xCtrl+bw+50, yPlant+120, xCtrl+bw+70, yPlant+140]);

%% ============================================================
%  BAĞLANTILARI KUR
% =============================================================
fprintf('  Bağlantılar kuruluyor...\n');

% r → Referans Model
add_line(model_name, 'Reference_r/1', 'Ref_Model/1', 'autorouting','on');

% r → Mux giriş 3
add_line(model_name, 'Reference_r/1', 'Mux_In/3', 'autorouting','on');

% Ref_Model çıkışı → Mux giriş 2
add_line(model_name, 'Ref_Model/1', 'Mux_In/2', 'autorouting','on');

% Plant çıkışı → Mux giriş 1
add_line(model_name, 'Plant/1', 'Mux_In/1', 'autorouting','on');

% Mux → MRAC_Controller girişi
add_line(model_name, 'Mux_In/1', 'MRAC_Controller/1', 'autorouting','on');

% MRAC_Controller çıkışı [1=u] → Plant girişi
add_line(model_name, 'MRAC_Controller/1', 'Plant/1', 'autorouting','on');

% Plant pos → Scope_Position (1)
add_line(model_name, 'Plant/1',     'Sel_Plant_Pos/1', 'autorouting','on');
add_line(model_name, 'Sel_Plant_Pos/1', 'Scope_Position/1', 'autorouting','on');

% Ref pos → Scope_Position (2)
add_line(model_name, 'Ref_Model/1', 'Sel_Ref_Pos/1', 'autorouting','on');
add_line(model_name, 'Sel_Ref_Pos/1', 'Scope_Position/2', 'autorouting','on');

% MRAC u → Scope_u
add_line(model_name, 'MRAC_Controller/1', 'Scope_u/1', 'autorouting','on');

% K_x → Scope_Gains (1)
add_line(model_name, 'MRAC_Controller/1', 'Sel_Kx/1', 'autorouting','on');
add_line(model_name, 'Sel_Kx/1', 'Scope_Gains/1', 'autorouting','on');

% K_r → Scope_Gains (2)  [MRAC çıkışı indis 5]
add_block('simulink/Signal Routing/Selector', [model_name '/Sel_Kr'], ...
    'InputPortWidth', '5', ...
    'Indices',    '5', ...
    'Position',   [xCtrl+bw+50, yKx-20, xCtrl+bw+70, yKx]);
add_line(model_name, 'MRAC_Controller/1', 'Sel_Kr/1', 'autorouting','on');
add_line(model_name, 'Sel_Kr/1', 'Scope_Gains/2', 'autorouting','on');

%% ============================================================
%  MODELİ KAYDET
% =============================================================
model_path = fullfile(fileparts(mfilename('fullpath')), [model_name '.slx']);
save_system(model_name, model_path);
fprintf('  Model kaydedildi: %s\n', model_path);

open_system(model_name);
fprintf('\n=== Model başarıyla oluşturuldu: %s ===\n', model_name);
fprintf('    sim(''%s'') ile çalıştırabilirsiniz.\n', model_name);
