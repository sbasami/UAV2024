/* ====================================================================
 * Include files
 * ==================================================================== */
#include "ServoDriver.h"

/* ====================================================================
 * Public functions
 * ==================================================================== */
/**
 * @brief コンストラクタ関数
 * @param [in] pin_pwm PWM出力ピン
 * @param [in] pin_dir 回転方向ピン
 */
ServoDriver::ServoDriver(int pin_pwm, int pin_dir)
{
    // メンバ変数の初期化
    pinPwm_ = pin_pwm;
    pinDir_ = pin_dir;
}

/**
 * @brief 初期化関数
 * @attention
 * - analogWriteFrequency関数を使用すると指定したピンと同じタイマーを使用しているピンのPWM周波数が全て変更されるので予め影響調査を行うこと
 * - analogWriteResolution関数を使用すると全てのanalogWrite関数の分解能が指定した値になるので予め影響調査を行うこと
 * @sa https://www.pjrc.com/teensy/td_pulse.html
 */
void ServoDriver::begin()
{
    // ピン設定の初期化
    pinMode(pinDir_, OUTPUT);
    pinMode(pinPwm_, OUTPUT);

    // PMWの周波数と分解能の設定
    analogWriteFrequency(pinPwm_, PWM_FREQUENCY_);
    analogWriteResolution(PWM_RESOLUTION_);

    // モータの回転を停止
    analogWrite(pinPwm_, 0);
}

/**
 * @brief PWM出力関数
 * @param [in] duty_norm Duty比[-1 ~ 1]
 * @note -1 ~ 1の範囲で正規化した値を使用することで正転と反転を表現している
 */
void ServoDriver::outputPWM(float duty_norm)
{
    int pwm_val;  // PWM出力値

    // Duty比[-1 ~ 1] → Duty比[PWM分解能]への変換
    pwm_val = (int)(duty_norm * (float)(PWM_RESOLUTION_MAX_));

    // 出力制限(上限、下限のチェック)
    if (pwm_val > PWM_RESOLUTION_MAX_)
    {
        pwm_val = PWM_RESOLUTION_MAX_;
    }
    else if (pwm_val < -PWM_RESOLUTION_MAX_)
    {
        pwm_val = -PWM_RESOLUTION_MAX_;
    }
    else
    {
        /* Do nothing */
    }

    // 回転方向の判定と出力
    if (pwm_val >= 0)
    {
        digitalWrite(pinDir_, HIGH);
    }
    else
    {
        pwm_val = -pwm_val;
        digitalWrite(pinDir_, LOW);
    }

    // PWM出力
    analogWrite(pinPwm_, pwm_val);
}