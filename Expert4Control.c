/*********************************************************
! @file     Expert4Control.c
! @brief    Expert4用制御工学ライブラリcファイル
! @date     2026/02/02
! @author   Isshin Kimura

 変更履歴
 260202 : Expert4Control.h作成
 260204 : FRAgenerator.hをExpert4Controlに統合
*********************************************************/

#include "Expert4Control.h"

/////////////////
/* 波形生成関連 */
/////////////////

//! @brief 方形波を出力する関数
//! @param[in]	freq	[Hz] 周波数
//! @param[in]	phase	[rad]位相
//! @param[in]	time	[s]  時刻
//! @return	方形波
float SquareWave(const float freq, const float phase, float time)
{
	float y;
	double r = sin((double)(2.0*PI*freq*time + phase));
	if(0 < r){
		y =  1;
	}else{
		y = -1;
	}
	return y;
}


//! @brief 階段波発生器
//! @param[in]	Time	[s] 現在時刻
//! @param[in]	Tini	[s] 階段の初期時刻
//! @param[in]	Ystp	[-] 階段1つ分の高さ
//! @param[in]	Tstp	[s] 階段1つ分の時間長さ
//! @param[in]	Nstp	[-] 階段の数
//! @return		階段波出力
float StairsWave(float Time, float Tini, float Ystp, float Tstp, float Nstp)
{
	int n = (int)((Time-Tini)/Tstp);	// 現在の階段の段数を計算
	float y;
	
	if(n <= Nstp){
		y = Ystp*(float)n;
	}else{
		y = 0;
	}
	
	return y;
}




//! @brief 三角波発振器
//! @param[in]	freq	三角波の周波数
//! @param[in]	time	時刻
//! @return 三角波
float TriangleWave(float freq, float time){
	const float Tp = 1.0/freq;
	const float a = 2.0/Tp;
	float t, y;
	
	t = fmod(time,Tp);	// 時刻を0～Tpの時間範囲に収める
	
	// 三角波の生成
	if(0 <= t && t < Tp/2.0){
		y = a*t;		// 正の傾き
	}else{
		y = -a*t + 2.0;	// 負の傾き
	}
	
	return 2.0*y - 1.0;	// ±1になるようにする
}

//! @brief FRAの設定関数
//! @param[in]	fra		FRA管理構造体
//! @param[in]	FreqMin	開始周波数[Hz]
//! @param[in]	FreqMax	終了周波数[Hz]
//! @param[in]	FreqStep	周波数ステップ[Hz]
//! @param[in]	NumIntg	積分回数（各周波数で何周期 signal を出力するか）
//! @param[in]	Ampl	振幅
//! @param[in]	Bias	バイアス
//! @param[in]	TimeSta	FRA開始時刻 [s]
void FRA_Init(Exp4FRA* fra,
              float FreqMin,
              float FreqMax,
              float FreqStep,
              float NumIntg,
              float Ampl,
              float Bias,
              float TimeSta)
{
    fra->fmin  = FreqMin;
    fra->fmax  = FreqMax;
    fra->fstep = FreqStep;
    fra->Ni    = NumIntg;
    fra->Au    = Ampl;
    fra->Bu    = Bias;
    fra->Tsta  = TimeSta;

    fra->isEnd = false;
    fra->f     = fra->fmin;
    fra->tini  = 0.0;
}


//! @brief FRA信号生成関数
//! @param[in]	fra		FRA管理構造体
//! @param[in]	t		時刻 [s]
//! @param[out]	out_freq	現在の周波数 [Hz]
//! @return		FRA信号
float FRA_GetSignal(Exp4FRA* fra, float t, float* out_freq)
{
    float outsig = fra->Bu;   // デフォルトはバイアス

    // FRA開始時刻を過ぎ、かつ終了していなければ
    if (t >= fra->Tsta && fra->isEnd == false)
    {
        outsig = fra->Au * cos(2.0 * PI * fra->f * (t - fra->tini - fra->Tsta)) + fra->Bu;

        // Ni 回分の時間を満たしたか？
        if ((fra->Ni / fra->f) <= (t - fra->tini - fra->Tsta))
        {
            if (fra->f <= fra->fmax)
            {
                fra->tini = t - fra->Tsta;  // 次の周波数の開始時刻
                fra->f += fra->fstep;       // 次の周波数へ
            }
            else
            {
                fra->isEnd = true;          // 終了
            }
        }
    }
    else
    {
        outsig = fra->Bu;  // FRA動作前・終了後はバイアス
    }

    // 呼び出し元に返す
    *out_freq = fra->f;
    return outsig;
}

/////////////////
/* リミッタ関連 */
/////////////////

//! @brief リミッタ
//! @param[in]	input	リミッタ入力
//! @param[in]	limit	リミット値
//! @return リミッタ出力
float Limiter1(float input, const float limit){
	if(limit<input)input=limit;		// 任意の数値で入力を制限して出力
	if(input<-limit)input=-limit;
	return input;
}

//! @brief リミッタ(PI制御器リミット偏差フィードバック用)
//! @param[in]	input	リミッタ入力
//! @param[in]	limit	リミット値
//! @param[in]  s    PI制御器管理構造体
//! @return リミッタ出力
float Limiter2(Exp4PICont *s, float input, const float limit){
    float nolim=0.0, lim=0.0;

    nolim = input;
    lim = Limiter1(nolim, limit);
    s->rl = lim - nolim;
    return lim;
}

/************************ 微分器・積分器 ****************************/

//! @brief 擬似微分器初期化関数
//! @param[in]	s	擬似微分器管理構造体
//! @param[in]	Ts	制御周期
void InitDiff(Exp4Diff *s, const float Ts){
	s->uZ1 = 0.0;
	s->yZ1 = 0.0;
	s->Ts = Ts;
}

//! @brief 擬似微分器(後退差分・LPFなし)
//! @param[in]	s	擬似微分器管理構造体
//! @param[in]	input 	擬似微分器の入力
//! @return 擬似微分器出力
float PseudoDiff1(Exp4Diff *s, float input){
	float y = 0.0;

	y = (input - s->uZ1) / s->Ts;	// 後退差分
	s->uZ1 = input;	// 前回値保存

	return y;
}

//! @brief 擬似微分器(双一次変換・LPFあり)
//! @param[in]	s	擬似微分器管理構造体
//! @param[in]	input 	擬似微分器の入力
//! @return 擬似微分器出力
float PseudoDiff2(Exp4Diff *s, float input, const float gpd){
	float y;

	y = ( 2.0*gpd*(input - s->uZ1) + (2.0 - s->Ts*gpd)*s->yZ1 )/(2.0 + s->Ts*gpd);

	s->uZ1 = input;
	s->yZ1 = y;
	return y;
}

//! @brief 積分器初期化関数
//! @param[in]	s	積分器管理構造体
//! @param[in]	Ts	制御周期
//! @param[in]	type	積分方法
void InitInt(Exp4Int *s, const float Ts, int type){
	s->uZ1 = 0.0;
	s->yZ1 = 0.0;
	s->Ts = Ts;
	s->type = type;
}

//! @brief 積分器
//! @param[in]	s	積分器管理構造体
//! @param[in]	input	積分器入力
//! @return 積分器出力
float Integrator(Exp4Int *s, float input){
	float y;

	// 積分方法選択
	switch(s->type){
		case FORWARD_EULER:
			y = s->Ts*s->uZ1 + s->yZ1; // 前進オイラー(前進差分)
			break;
		case BACKWARD_EULER:
			y = s->Ts*input + s->yZ1; // 後退オイラー(後退差分)
			break;
		case TRAPEZOIDAL:
			y = s->Ts/2.0*(input + s->uZ1) + s->yZ1; // 台形積分(双1次変換, Tustin変換)
			break;
		default:
			y = s->Ts*input + s->yZ1; // デフォルトは後退オイラー
			break;
	}

	// 状態変数の更新
	s->uZ1 = input;
	s->yZ1 = y;

	return y;
}


/************************ PID制御器 ****************************/

//! @brief PI制御器初期化関数
//! @param[in]	s	PI制御器管理構造体
void InitPIcont(Exp4PICont *s, const float Ts){
	s->rl = 0.0;
	s->uZ1 = 0.0;
	s->yZ1 = 0.0;
	s->Ts = Ts;
}


//! @brief アンチワインドアップ付きPI制御器
//! @param[in]	u		PI制御器入力
//! @param[in]	s	PI制御器管理構造体
//! @return PI制御器出力
//! @note 双一次変換で離散化、リミットにかかったら積分項を0にする方式
float GetPIcont(Exp4PICont *s, float u, const float Kp, const float Ki){
	float y;
	float cor_Ki = Ki;
	if(s->rl!=0.0)	cor_Ki = 0.0;

	y = (Kp + (s->Ts*cor_Ki)/2.0)*u + ((s->Ts*cor_Ki)/2.0 - Kp)*s->uZ1 + s->yZ1; // 双一次変換
	s->uZ1 = u;
	s->yZ1 = y;
	return y;
}


/************************ オブザーバ関連 ****************************/


//! @brief 時間カウンタ初期化関数
//! @param[in]	s	経過時間カウンタ管理構造体
void InitTimeCounter(Exp4TimeCounter *s){
	s->count = 0;
	s->time = 0.0;
}

//! @brief 経過時間取得関数
//! @param[in]	s	経過時間カウンタ管理構造体
//! @param[in]	Ts 		[s]	制御周期
void GetElapsedTime(Exp4TimeCounter *s, float Ts){
	s->time = s->count*Ts;
	s->count++;
}