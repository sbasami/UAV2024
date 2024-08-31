#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

/* ====================================================================
 * Include files
 * ==================================================================== */
// C++標準ライブラリ
#include <vector>
// Arduino標準ライブラリ
#include <Arduino.h>

/* ====================================================================
 * Class declarations
 * ==================================================================== */
/**
 * @brief ServoControllerクラス
 * @par 説明
 * 以下の処理を提供する
 * - I-PD制御の計算
 * - I-PD制御の計算結果の取得
 */
class ServoController
{
   private:
    static constexpr float GEAR_RATIO_     = (32.0f * 33.0f * 35.0f * 38.0f) / (15.0f * 14.0f * 13.0f * 10.0f); /**< ギア比 */
    static constexpr float ENCODER_PULSE_  = 12.0f;                                                             /**< 360deg回転した時のエンコーダパルス */
    static constexpr float RESOLUTION_RAD_ = (2.0f * PI) / (ENCODER_PULSE_ * GEAR_RATIO_);                      /**< 角度分解能[rad] */
    static constexpr float RESOLUTION_DEG_ = 360.0f / (ENCODER_PULSE_ * GEAR_RATIO_);                           /**< 角度分解能[deg] */
    static constexpr float KP_             = 40.22;                                                             /**< I-PD制御のPゲインのデフォルト値 */
    static constexpr float KI_             = 672.5;                                                             /**< I-PD制御のIゲインのデフォルト値 */
    static constexpr float KD_             = 0.6012;                                                            /**< I-PD制御のDゲインのデフォルト値 */

    float Tc_        = 0; /**< 制御周期[s] */
    float retIPD_    = 0; /**< I-PD制御の計算結果[V] */
    float yOld_      = 0; /**< D制御用の出力保持 */
    float integral_  = 0; /**< I制御用の積分値 */
    float batVolt_v_ = 0; /**< バッテリ電圧[V] */
   public:
    ServoController(float control_cycle_s);
    void  calculateIPD(float ref_angle_rad, float act_angle_rad, float bat_volt_v);
    void  resetIntegral();
    float getIPD_V();
    float getIPD_duty();
};

#endif /* SERVOCONTROLLER_H */