%% Parametreler
T_total = 60;       % toplam süre [s]
Fs      = 1000;     % örnekleme frekansı [Hz]
f_tri   = 0.1;      % üçgen sinyal frekansı [Hz] (periyot = 10 s)
A_tri   = 1;        % genlik [mm] (ör: ±1 mm)
T_plateau = 3;      % baş/son 0 plateau süresi [s]

%% Zaman vektörü
t = (0:1/Fs:T_total)';   % [s]

%% Üçgen kısmın zaman vektörü
t_tri = (0:1/Fs:(T_total - 2*T_plateau))';

%% Üçgen sinyal (orta kısım)
r_tri = A_tri * sawtooth(2*pi*f_tri*t_tri, 0.5);

%% Baştaki ve sondaki sıfır plateau
r0_start = zeros(Fs*T_plateau,1);
r0_end   = zeros(Fs*T_plateau,1);

%% Tam sinyal oluştur
r = [r0_start; r_tri; r0_end];



%% Üçgen sinyal oluşturma
% Yöntem-1: hazır sawtooth fonksiyonu (π ile kaydırınca üçgen olur)
r = A_tri * sawtooth(2*pi*f_tri*t, 0.5);  % duty=0.5 → simetrik üçgen

% Yöntem-2: kendi fonksiyonumuz (isteğe bağlı)
% periyot = 1/f_tri;
% r = A_tri * (2*abs(2*(t/periyot - floor(t/periyot + 0.5))) - 1);

%% Grafik
figure; hold on; grid on;
plot(t, r+1, 'b','LineWidth',1.2); % mm cinsinden çiz
xlabel('Zaman [s]');
ylabel('Referans [mm]');
title('Üçgen Profil (r(t))');


%% Tablo oluştur
dataTable = table(t, r, ...
    'VariableNames', {'Time_s', 'Ref_mm'});

%% CSV olarak yaz
csvFileName = 'triangle_profile.csv';
writetable(dataTable, csvFileName);