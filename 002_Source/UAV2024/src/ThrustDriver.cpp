/* ====================================================================
 * Include files
 * ==================================================================== */
#include "ThrustDriver.h"

/* ====================================================================
 * Public functions
 * ==================================================================== */
/**
 * @brief EscDriverコンストラクタ
 * @param [in] pin_pwm モータドライバへのPWM出力ピン
 */
ThrustDriver::ThrustDriver(int pin_pwm)
{
    pinPwm_ = pin_pwm;
}

/**
 * @brief 初期化処理
 * @attention
 * - analogWriteFrequency関数を使用すると指定したピンと同じタイマーを使用しているピンのPWM周波数が全て変更されるので予め影響調査を行うこと
 * - analogWriteResolution関数を使用すると全てのanalogWrite関数の分解能が指定した値になるので予め影響調査を行うこと
 * @sa https://www.pjrc.com/teensy/td_pulse.html
 */
void ThrustDriver::begin()
{
    float motor_stop;

    // ピン設定の初期化
    pinMode(pinPwm_, OUTPUT);

    // PMWの周波数と分解能の設定
    analogWriteFrequency(pinPwm_, PWM_FREQUENCY_);
    analogWriteResolution(PWM_RESOLUTION_);

    // モータの回転を停止
    motor_stop = (float)PWM_RESOLUTION_MAX_ * ESC_NORM_MIN_;
    analogWrite(pinPwm_, (int)motor_stop);
}

/**
 * @brief PWM出力処理
 * @param [in] duty_norm Duty比[0 ~ 1]
 */
void ThrustDriver::outputPWM(float duty_norm)
{
    float pwm_val;  // PWM出力値
    float duty_esc;

    // Duty比[0 ~ 1] → Duty比[ESC_NORM_MIN ~ ESC_NORM_MAX]への変換
    duty_esc = duty_norm * (ESC_NORM_MAX_ - ESC_NORM_MIN_) + ESC_NORM_MIN_;

    // Duty比[ESC_NORM_MIN ~ ESC_NORM_MAX] → Duty比[PWM分解能]への変換
    pwm_val = duty_esc * (float)PWM_RESOLUTION_MAX_;

    // 出力制限(上限、下限のチェック)
    if (pwm_val > ((float)PWM_RESOLUTION_MAX_ * ESC_NORM_MAX_))
    {
        pwm_val = (float)PWM_RESOLUTION_MAX_ * ESC_NORM_MAX_;
    }
    else if (pwm_val < ((float)PWM_RESOLUTION_MAX_ * ESC_NORM_MIN_))
    {
        pwm_val = (float)PWM_RESOLUTION_MAX_ * ESC_NORM_MIN_;
    }

    // PWM出力
    analogWrite(pinPwm_, (int)pwm_val);
}