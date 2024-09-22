#ifndef THRUST_DRIVER_H
#define THRUST_DRIVER_H

/* ====================================================================
 * Include files
 * ==================================================================== */
// Arduino標準ライブラリ
#include <Arduino.h>

/* ====================================================================
 * Class declarations
 * ==================================================================== */
/**
 * @brief ThrustDriverクラス
 * @par 説明
 * 以下の処理を提供する
 * - PWMピンの初期化
 * - PWMピンへの出力
 */
class ThrustDriver
{
   private:
    // Oneshot42 12000
    // Multishot 32000
    static constexpr float PWM_FREQUENCY_      = 32000.0f;             /**< PWM周波数のデフォルト値[Hz] */
    static constexpr int   PWM_RESOLUTION_     = 12;                   /**< PWM分解能[bit] */
    static constexpr int   PWM_RESOLUTION_MAX_ = 1 << PWM_RESOLUTION_; /**< PWM分解能最大値 */

    // ESCのノルム計算方法
    // Multishotの有効なシグナル範囲は5 ~ 25us幅のパルス
    // MultishotのPWM周波数は32kHz → 31.25us
    // 25us / 31.25us = 0.8
    //  5us / 31.25us = 0.16
    // |←5us→| ←-------------Multishot protocolが認識できる最小パルス幅
    //  _____               __
    // |     |             |
    // |     |_____________|
    // |←-----31.25us-----→|
    //
    // |←----25us----→| ←----Multishot protocolが認識できる最大パルス幅
    //  ______________      __
    // |              |    |
    // |              |____|
    // |←-----31.25us-----→|
    // 参考文献：https://bluerobotics.com/wp-content/uploads/2018/10/BLHeli_S-manual-SiLabs-Rev16.x.pdf
    // Oneshot42 0.50 ~ 0.99
    // Multishot 0.16 ~ 0.8
    static constexpr float ESC_NORM_MAX_ = 0.8f;  /**< ESCノルム最大値 */
    static constexpr float ESC_NORM_MIN_ = 0.16f; /**< ESCノルム最小値 */

    int pinPwm_; /**< PWM出力ピン */
   public:
    ThrustDriver(int pin_pwm);
    void begin();
    void outputPWM(float duty_norm);
};

#endif /* THRUST_DRIVER_H */