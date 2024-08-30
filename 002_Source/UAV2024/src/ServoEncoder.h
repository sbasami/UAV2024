#ifndef SERVOENCODER_H
#define SERVOENCODER_H

/* ====================================================================
 * Include files
 * ==================================================================== */
// Arduino標準ライブラリ
#include <Arduino.h>
#include <Encoder.h>

/* ====================================================================
 * Class declarations
 * ==================================================================== */
/**
 * @brief ServoEncoderクラス
 * @par 説明
 * 以下の処理を提供する
 * - エンコーダ値の読み取り
 * - 角度の取得
 */
class ServoEncoder
{
   private:
    static constexpr float GEAR_RATIO_     = (32.0f * 33.0f * 35.0f * 38.0f) / (15.0f * 14.0f * 13.0f * 10.0f); /**< ギア比 */
    static constexpr float ENCODER_PULSE_  = 12.0f;                                                             /**< 360deg回転した時のエンコーダパルス */
    static constexpr float RESOLUTION_RAD_ = (2.0f * PI) / (ENCODER_PULSE_ * GEAR_RATIO_);                      /**< 角度分解能[rad] */
    static constexpr float RESOLUTION_DEG_ = 360.0f / (ENCODER_PULSE_ * GEAR_RATIO_);                           /**< 角度分解能[deg] */

    Encoder encoder;
    float   readData_; /**< エンコーダパルスの読出し値 */
   public:
    ServoEncoder(int pin_encA, int pin_encB);
    void  readSensor();
    void  resetSensor(int reset_value);
    float getAngle_rad();
    float getAngle_deg();
};

#endif /* SERVOENCODER_H */