/*********************************************************
! @file     Expert4Control.h
! @brief    Expert4用制御工学ライブラリhファイル
! @date     2026/02/02
! @author   Isshin Kimura

 変更履歴
 260202 : Expert4Control.h作成
*********************************************************/

#ifndef EXPERT4CONTROL
#define EXPERT4CONTROL

#include "mwio4.h"
#include "TamagawaModule.h"
#include "MacroDefine.h"
#include "math.h"
#include "stdbool.h"

// 積分方法
#define    BACKWARD_EULER       0   // 後退オイラー
#define    FORWARD_EULER        1   // 前進オイラー
#define    TRAPEZOIDAL          2   // 台形積分


/***************************構造体定義******************************/

// FRA発生器管理構造体
typedef struct{
    float   fmin;
    float   fmax;
    float   fstep;
    float   Ni;
    float   Au;
    float   Bu;
    float   Tsta;
    bool    isEnd;
    float   f;
    float   tini;
}Exp4FRA;

// PI制御器管理構造体
typedef struct{
    float rl;       // リミット偏差
    float uZ1;      // 前回の入力
    float yZ1;      // 前回の出力
    float Ts;   // 制御周期
}Exp4PICont;

typedef struct{
    float uZ1;      // 前回の入力
    float yZ1;      // 前回の出力
    float Ts;       // 制御周期
}Exp4Diff;

// 積分器管理構造体
typedef struct{
    float uZ1;      // 前回の入力
    float yZ1;      // 前回の出力
    float Ts;       // 制御周期
    int type;       // 積分方法
}Exp4Int;

// 経過時間カウンタ用構造体
typedef struct{
    unsigned long int count; // 制御周期カウンタ
    float time;              // 経過時間
}Exp4TimeCounter;

/***************************プロトタイプ宣言************************/

// 波形生成関数
float SquareWave(const float freq, const float phase, float time);
float StairsWave(float Time, float Tini, float Ystp, float Tstp, float Nstp);
float TriangleWave(float freq, float time);
// FRA関数
void FRA_Init(Exp4FRA* fra, float FreqMin, float FreqMax, float FreqStep, float NumIntg, 
    float Ampl, float Bias, float TimeSta); 
float FRA_GetSignal(Exp4FRA* fra, float t, float* out_freq);

// リミッタ
float Limiter1(const float limit, float input);
float Limiter2(Exp4PICont *s, const float limit, float input); // PI制御器リミット偏差フィードバック用

// 微分器・積分器
void InitDiff(Exp4Diff *s, const float Ts); // 擬似微分器初期化
float PseudoDiff1(Exp4Diff *s, float input);    // LPFなし
float PseudoDiff2(Exp4Diff *s, float input, const float gpd);   // LPFあり

void InitInt(Exp4Int *s, const float Ts, int type); // 積分器初期化関数
float Integrator(Exp4Int *s, float input);    // 積分器

// PI制御器
void InitPIcont(Exp4PICont *s, const float Ts); // PI制御器初期化関数
float GetPIcont(Exp4PICont *s, float u, const float Kp, const float Ki);    // PI制御器

// オブザーバ

// その他関数
void InitTimeCounter(Exp4TimeCounter *s);
void GetElapsedTime(Exp4TimeCounter *s, float Ts);

#endif