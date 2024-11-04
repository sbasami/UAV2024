#ifndef SERVODRIVER_H
#define SERVODRIVER_H

/* ====================================================================
 * Include files
 * ==================================================================== */
// Arduino標準ライブラリ
#include <Arduino.h>

/* ====================================================================
 * Class declarations
 * ==================================================================== */
/**
 * @brief ServoDriverクラス
 * @par 説明
 * 以下の処理を提供する
 * - PWMピン、回転方向ピンの初期化
 * - PWMピン、回転方向ピンへの出力
 */
class ServoDriver
{
   private:
    // PMW周波数とPWM分解能の最適な組み合わせは下記ページの【PWM Resolution】参照
    // https://www.pjrc.com/teensy/td_pulse.html
    static constexpr float PWM_FREQUENCY_      = 36621.09f;            /**< PWM周波数[Hz] */
    static constexpr int   PWM_RESOLUTION_     = 12;                   /**< PWM分解能[bit] */
    static constexpr int   PWM_RESOLUTION_MAX_ = 1 << PWM_RESOLUTION_; /**< PWM分解能最大値 */

    int pinPwm_;                 /**< PWM出力ピン */
    int pinDir_;                 /**< 回転方向ピン */
    int rotation_direction_ = 0; /**< 回転方向 */

   public:
    explicit ServoDriver(int pin_pwm, int pin_dir);
    void SetRotationDirection(int rotation_direction);
    void begin();
    void outputPWM(float duty_norm);
};

#endif /* SERVODRIVER_H */