%% Referans Dithering (pozisyon komutu için)
Fs   = 1000;            % Hz
Ts   = 1/Fs;
Tper = 30;              % periyot süresi [s]
Nper = 8;               % periyot sayısı
T    = Tper * Nper;     % toplam süre
t    = (0:Ts:T-Ts).';   % zaman vektörü

% Frekans bandı
fmin = 0.1; fmax = 3.0; % Hz
f0 = 1/Tper;
kmin = ceil(fmin/f0); kmax = floor(fmax/f0);
fgrid = (kmin:kmax)*f0;
Nf = numel(fgrid);

% Genlik shaping (pozisyon, metre cinsinden)
Amax = 0.010;             % 10 mm
vlim = 0.060;             % 60 mm/s
A_f = min(Amax, vlim./(2*pi*fgrid).');   % frekansa göre genlik

% Rastgele fazlar
phi = 2*pi*rand(Nf,1);

% Multisine oluştur
r_ms = zeros(size(t));
for i=1:Nf
    r_ms = r_ms + A_f(i)*sin(2*pi*fgrid(i)*t + phi(i));
end

% Fade-in/out (ilk ve son 2 s)
fade = min(1,(t/2).*((T - t)/2));
fade = max(fade,0);
r_ms = r_ms .* fade;

% Çıktı [s, referans (m)]
data = [t r_ms];
writematrix(data, 'dither_reference.csv');
