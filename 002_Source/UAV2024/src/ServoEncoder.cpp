/* ====================================================================
 * Include files
 * ==================================================================== */
#include "ServoEncoder.h"

/* ====================================================================
 * Public functions
 * ==================================================================== */
/**
 * @brief ServoEncoderコンストラクタ関数
 * @param [in] pin_encA エンコーダのA相ピン
 * @param [in] pin_encB エンコーダのB相ピン
 */
ServoEncoder::ServoEncoder(int pin_encA, int pin_encB)
    : encoder(pin_encA, pin_encB)
{
    // Do nothing
}

/**
 * @brief エンコーダカウント読み出し関数
 * - エンコーダカウントの読み出し
 */
void ServoEncoder::readSensor()
{
    readData_ = (float)encoder.read();
}

/**
 * @brief エンコーダカウントリセット関数
 * @param [in] reset_value エンコーダカウントのリセット値
 */
void ServoEncoder::resetSensor(int reset_value)
{
    encoder.write(reset_value);
}

/**
 * @brief 角度[rad]取得関数
 * @return 角度[rad]
 */
float ServoEncoder::getAngle_rad()
{
    return (readData_ * RESOLUTION_RAD_);
}

/**
 * @brief 角度[deg取得関数
 * @return 角度[deg]
 */
float ServoEncoder::getAngle_deg()
{
    return (readData_ * RESOLUTION_DEG_);
}
