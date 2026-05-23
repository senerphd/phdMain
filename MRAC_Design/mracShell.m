
% PARAMETRELER
d_p     = 34e-3;
d_r     = 30e-3;
A_c     = pi/4*(d_p^2 - d_r^2);
V_dead  = 1.5315e-5;
V_0     = A_c*(190.16e-3/2) + V_dead;

m      = 3.4;                   % Piston rotu eşdeğer kütlesi [kg]
Ae     = A_c;                   % Efektif alan [m^2] (Çift kollu silindirde doğrudan A'ya eşittir)
Bp     = 38;                    % Viskoz sönümleme katsayısı [N/(m/s)]
beta_e = 1.6e9;                 % Hidrolik yağın efektif elastikiyet (bulk) modülü [Pa]
Ve     = 2*V_0;                 % Efektif hacim [m^3]
Kq     = 1.1003e-2;              % Akış kazancı [(m^3/s)/m]
Kv     = 1;                     % Servo valf spool kazancı [m/V]
Kc     = 3.9633e-13; %3.9633e-13;            % Basınç-akış katsayısı [(m^3/s)/Pa]
Cip    = 3.9933e-13;            % Silindir içi kaçak katsayısı [(m^3/s)/Pa]
%% Referans modelin tanımlanması
am1 = 2.707E9; 
am2 = 9.528E7; 
am3 = 145.384; 

bm3 = am1; % DC Gain olmasın? 

Am = [0,        1,          0; 
      0,        -Bp/m,      -A_c/m; 
      -am1,     -am2,       -am3]; 

Bm = [0; 0; bm3]; 
Cm = eye(3); 

%% Dinamik sistemin lineer halinin tanımlanması
% Durum değişkenlerini okuma
% x1 = x(1); % x_p
% x2 = x(2); % \dot{x}_p
% x3 = x(3); % P_L
% 
% % Denklem 9: Durum uzayı türevlerinin (State derivatives) hesaplanması
% x1_dot = x2;
% x2_dot = (-Ae * x3 - Bp * x2 - f) / m;
% x3_dot = (4 * beta_e / Ve) * (-Kq * Kv *u - (Kc + Cip) * x3 + Ae * x2); % u teriminin önüne eksi (-) geldi
% 
% % Çıkış vektörünü oluşturma (Sütun vektörü olmalıdır)
% xdot = [x1_dot; x2_dot; x3_dot];

A_mat = [0,     1,                  0; 
         0,     -Bp/m,              -Ae/m; 
         0,     4*beta_e*Ae/Ve,     -4*beta_e*(Kc + Cip)/Ve]; 

B_mat = [0; 0; (-4*beta_e/Ve)*(Kq*Kv)]; 

% A32, A33 ve b3 değerlerinin senin parametrelerinle hesaplanması
A32 = A_mat(3,2);
A33 = A_mat(3,3);
b3  = B_mat(3,1);

% İdeal Başlangıç Kazançları (Kx0 ve Kr0)
Kx1_0 = -am1 / b3;
Kx2_0 = (-am2 - A32) / b3;
Kx3_0 = (-am3 - A33) / b3;

Kx0 = [Kx1_0; Kx2_0; Kx3_0]; % 3x1 Kx Integrator Başlangıç Koşulu
Kr0 = am1 / b3;             % Skaler Kr Integrator Başlangıç Koşulu

%% Lyapunov Denklemi
Q = eye(3);
P = lyap(Am', Q);
PB_vec = P * B_mat; % 3x1 Sabit Vektör
%%