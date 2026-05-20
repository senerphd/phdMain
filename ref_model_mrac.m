% Sistem Parametreleri
m = 3.4; 
Ap = 2.0106e-04; 
bv = 38;

% Kanonik Matrisler (Önceki tasarımdan)
Ac = [0 1 0; 0 0 1; -136687.5 -6885 -139.5];
Bc = [0; 0; 136687.5];

% Dönüşüm Matrisi
T = [1 0 0; 
     0 1 0; 
     0 bv/Ap m/Ap];

% Fiziksel Referans Model Matrisleri
Am = T * Ac * inv(T);
Bm = T * Bc;
Cm = eye(3); 
Dm = zeros(3,1); 

ref_model = ss(Am, Bm, Cm, Dm); 

