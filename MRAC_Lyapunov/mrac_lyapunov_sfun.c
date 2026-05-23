/*
 * mrac_lyapunov_sfun.c
 *
 * Lyapunov Kararlılık Teorisine Dayalı Model Reference Adaptive Controller (MRAC)
 * C Level-2 S-Function Implementasyonu
 *
 * Hidrolik Servo Sistem için 3-Durumlu MRAC Kontrolcüsü
 *
 * DURUM UZAYI (Plant):
 *   x = [x_p, xdot_p, P_L]^T  (konum, hız, yük basıncı)
 *   xdot_p = A_p*x + B_p*u
 *   u = K_x^T * x_p + K_r * r
 *
 * REFERANS MODEL:
 *   xdot_m = A_m * x_m + B_m * r
 *
 * HATA:
 *   e = x_p - x_m
 *
 * ADAPTASYON KANUNLARI (Lyapunov Kararlılık):
 *   K_x_dot = -Gamma_x * x_p * (e^T * P * B_p)
 *   K_r_dot = -Gamma_r * r  * (e^T * P * B_p)
 *
 *   Lyapunov Denklemi: A_m^T*P + P*A_m = -Q (Q > 0)
 *
 * GİRİŞLER (7 adet):
 *   [0..2]  : x_p  - Plant durumu (3x1)
 *   [3..5]  : x_m  - Referans model durumu (3x1)
 *   [6]     : r    - Referans sinyal (skaler)
 *
 * ÇIKIŞLAR (5 adet):
 *   [0]     : u    - Kontrol sinyali
 *   [1..3]  : K_x  - Adaptif kazanç vektörü (3x1)
 *   [4]     : K_r  - Adaptif besleme ileri kazancı
 *
 * SÜREKLİ DURUMLAR (4 adet):
 *   [0..2]  : K_x(t) - Adaptasyon durumu
 *   [3]     : K_r(t) - Adaptasyon durumu
 *
 * PARAMETRELER (S-Function bloğundan aktarılır):
 *   P[1]    : Gamma_x  - x adaptasyon kazancı (skaler)
 *   P[2]    : Gamma_r  - r adaptasyon kazancı (skaler)
 *   P[3]    : P_mat    - Lyapunov matrisi (9 eleman, satır sırası)
 *   P[4]    : Bp_vec   - Plant giriş matrisi B_p (3 eleman)
 *   P[5]    : u_max    - Kontrol sinyali doyum sınırı
 *
 * Tasarım: Lyapunov Tabanlı MRAC (Narendra & Annaswamy, 1989)
 *
 * Hasan Şener - PhD Tezi
 */

#define S_FUNCTION_NAME  mrac_lyapunov_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>
#include <string.h>

/* ===== SABİTLER ===== */
#define N_STATES    3       /* Plant / referans model durum sayısı       */
#define N_INPUTS    7       /* [x_p(3), x_m(3), r(1)]                   */
#define N_OUTPUTS   5       /* [u(1), K_x(3), K_r(1)]                   */
#define N_CONT_ST   4       /* Sürekli durum: [K_x(3), K_r(1)]          */

#define IDX_GAMMA_X 0
#define IDX_GAMMA_R 1
#define IDX_P_MAT   2
#define IDX_BP_VEC  3
#define IDX_UMAX    4
#define N_PARAMS    5

/* ===== YARDIMCI FONKSİYONLAR ===== */

/* 3x1 * 1x3 iç çarpım (dot product) */
static real_T dot3(const real_T *a, const real_T *b)
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

/* Doyum (saturation) fonksiyonu */
static real_T saturate(real_T val, real_T limit)
{
    if (val >  limit) return  limit;
    if (val < -limit) return -limit;
    return val;
}

/* ===== S-FUNCTION METODLARI ===== */

/* mdlInitializeSizes: Port ve parametre boyutlarını tanımla */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, N_PARAMS);

    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;  /* Hata: MATLAB hata mesajı gösterecek */
    }

    /* Parametreler değiştirilemez (simülasyon sırasında) */
    for (int i = 0; i < N_PARAMS; i++) {
        ssSetSFcnParamNotTunable(S, i);
    }

    /* Sürekli durum sayısı */
    ssSetNumContStates(S, N_CONT_ST);
    ssSetNumDiscStates(S, 0);

    /* Giriş portları */
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, N_INPUTS);
    ssSetInputPortRequiredContiguous(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    /* Çıkış portları */
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, N_OUTPUTS);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

/* mdlInitializeSampleTimes */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

/* mdlInitializeConditions: Başlangıç koşulları */
#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S)
{
    real_T *x0 = ssGetContStates(S);
    /* K_x = [0, 0, 0], K_r = 1 (başlangıç) */
    x0[0] = 0.0;   /* K_x[0] */
    x0[1] = 0.0;   /* K_x[1] */
    x0[2] = 0.0;   /* K_x[2] */
    x0[3] = 1.0;   /* K_r    */
}

/* mdlOutputs: Çıkış hesapla (u, K_x, K_r) */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    /* Girişleri oku */
    const real_T *u_in = ssGetInputPortRealSignal(S, 0);
    const real_T *xp   = &u_in[0];   /* Plant durumu */
    /* const real_T *xm   = &u_in[3];   Referans model durumu (çıkış için gerekli değil) */
    real_T r           = u_in[6];    /* Referans sinyal */

    /* Durumları oku: K_x, K_r */
    const real_T *states = ssGetContStates(S);
    const real_T *Kx     = &states[0];  /* K_x [3x1] */
    real_T        Kr     = states[3];   /* K_r [skaler] */

    /* Parametre: u_max */
    real_T u_max = mxGetScalar(ssGetSFcnParam(S, IDX_UMAX));

    /* Kontrol sinyali: u = K_x^T * x_p + K_r * r */
    real_T u_ctrl = dot3(Kx, xp) + Kr * r;

    /* Doyum uygula */
    u_ctrl = saturate(u_ctrl, u_max);

    /* Çıkışları yaz */
    real_T *y = ssGetOutputPortRealSignal(S, 0);
    y[0] = u_ctrl;
    y[1] = Kx[0];
    y[2] = Kx[1];
    y[3] = Kx[2];
    y[4] = Kr;
}

/* mdlDerivatives: Adaptasyon kanunlarını entegre et */
#define MDL_DERIVATIVES
static void mdlDerivatives(SimStruct *S)
{
    /* Girişleri oku */
    const real_T *u_in = ssGetInputPortRealSignal(S, 0);
    const real_T *xp   = &u_in[0];   /* x_p: plant durumu [3] */
    const real_T *xm   = &u_in[3];   /* x_m: ref model durumu [3] */
    real_T r           = u_in[6];    /* r: referans sinyal */

    /* Parametreleri oku */
    real_T Gamma_x = mxGetScalar(ssGetSFcnParam(S, IDX_GAMMA_X));
    real_T Gamma_r = mxGetScalar(ssGetSFcnParam(S, IDX_GAMMA_R));

    const mxArray *P_mx  = ssGetSFcnParam(S, IDX_P_MAT);
    const mxArray *Bp_mx = ssGetSFcnParam(S, IDX_BP_VEC);

    /* P matrisi (3x3, satır sırası) ve Bp vektörü (3x1) */
    const real_T *P_data  = mxGetPr(P_mx);   /* 9 eleman */
    const real_T *Bp_data = mxGetPr(Bp_mx);  /* 3 eleman */

    /*
     * NOT: MATLAB matrisler sütun-sırası (column-major) saklar.
     * P_data[i + 3*j] = P(i,j)
     * P = [P00 P01 P02; P10 P11 P12; P20 P21 P22]
     * P_data = [P00, P10, P20, P01, P11, P21, P02, P12, P22]
     */

    /* Hata: e = x_p - x_m */
    real_T e[3];
    e[0] = xp[0] - xm[0];
    e[1] = xp[1] - xm[1];
    e[2] = xp[2] - xm[2];

    /*
     * Skaler: sigma = e^T * P * B_p
     *
     * Adım 1: v = P * B_p  (3x3 * 3x1 = 3x1)
     * MATLAB column-major: P(i,j) = P_data[i + 3*j]
     */
    real_T v[3];
    int i, j;
    for (i = 0; i < 3; i++) {
        v[i] = 0.0;
        for (j = 0; j < 3; j++) {
            v[i] += P_data[i + 3*j] * Bp_data[j];
        }
    }

    /* Adım 2: sigma = e^T * v = e^T * P * B_p */
    real_T sigma = dot3(e, v);

    /* Adaptasyon türevleri */
    real_T *dx = ssGetdX(S);

    /*
     * K_x_dot = -Gamma_x * x_p * sigma  [3x1]
     * K_r_dot = -Gamma_r * r   * sigma  [skaler]
     */
    dx[0] = -Gamma_x * xp[0] * sigma;
    dx[1] = -Gamma_x * xp[1] * sigma;
    dx[2] = -Gamma_x * xp[2] * sigma;
    dx[3] = -Gamma_r * r      * sigma;
}

/* mdlTerminate */
static void mdlTerminate(SimStruct *S)
{
    UNUSED_ARG(S);
}

/* S-Function kayıt makrosu */
#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
