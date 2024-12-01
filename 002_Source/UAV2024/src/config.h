#ifndef CONFIG_H
#define CONFIG_H

/* ====================================================================
 * constant definitions
 * ==================================================================== */
/* --------------------------------------------------------------------
 * Pin allocation
 * -------------------------------------------------------------------- */
// 姿勢センサ関係
constexpr int CS_PIN_ACCEL = 10; /**< 加速度センサのSPIのCSピン */
constexpr int CS_PIN_GYRO  = 9;  /**< ジャイロセンサのSPIのCSピン */
// サーボモータ関係
constexpr int CONFIG_PIN_SERVO1_PWM   = 15; /**< サーボモータ1のPWM出力ピン */
constexpr int CONFIG_PIN_SERVO1_DIR   = 17; /**< サーボモータ1の回転方向ピン */
constexpr int CONFIG_PIN_SERVO1_ENC_A = 20; /**< サーボモータ1のエンコーダのA相ピン */
constexpr int CONFIG_PIN_SERVO1_ENC_B = 21; /**< サーボモータ1のエンコーダのB相ピン */

constexpr int CONFIG_PIN_SERVO2_PWM   = 28; /**< サーボモータ2のPWM出力ピン */
constexpr int CONFIG_PIN_SERVO2_DIR   = 31; /**< サーボモータ2の回転方向ピン */
constexpr int CONFIG_PIN_SERVO2_ENC_A = 39; /**< サーボモータ2のエンコーダのA相ピン */
constexpr int CONFIG_PIN_SERVO2_ENC_B = 38; /**< サーボモータ2のエンコーダのB相ピン */

constexpr int CONFIG_PIN_SERVO3_PWM   = 29; /**< サーボモータ3のPWM出力ピン */
constexpr int CONFIG_PIN_SERVO3_DIR   = 30; /**< サーボモータ3の回転方向ピン */
constexpr int CONFIG_PIN_SERVO3_ENC_A = 27; /**< サーボモータ3のエンコーダのA相ピン */
constexpr int CONFIG_PIN_SERVO3_ENC_B = 26; /**< サーボモータ3のエンコーダのB相ピン */

constexpr int CONFIG_PIN_SERVO4_PWM   = 14; /**< サーボモータ4のPWM出力ピン */
constexpr int CONFIG_PIN_SERVO4_DIR   = 16; /**< サーボモータ4の回転方向ピン */
constexpr int CONFIG_PIN_SERVO4_ENC_A = 4;  /**< サーボモータ4のエンコーダのA相ピン */
constexpr int CONFIG_PIN_SERVO4_ENC_B = 5;  /**< サーボモータ4のエンコーダのB相ピン */
// ESC関係
constexpr int PIN_ESC1_PWM = 23; /**< ESC1のPWM出力ピン */
constexpr int PIN_ESC2_PWM = 22; /**< ESC2のPWM出力ピン */
constexpr int PIN_ESC3_PWM = 2;  /**< ESC3のPWM出力ピン */
constexpr int PIN_ESC4_PWM = 3;  /**< ESC4のPWM出力ピン */

/* --------------------------------------------------------------------
 * Servo setting
 * -------------------------------------------------------------------- */
constexpr int CONFIG_SERVO_NUM     = 4; /**< サーボモータの数 */
constexpr int CONFIG_PROPELLER_NUM = 4; /**< プロペラの数 */

/* --------------------------------------------------------------------
 * Cycle setting
 * -------------------------------------------------------------------- */
// 制御関係
constexpr float CONFIG_CONTROL_FREQUENCY_Hz          = 1000.0f;                                  /**< 制御周波数[Hz] */
constexpr float CONFIG_CONTROL_CYCLE_sec             = 1 / CONFIG_CONTROL_FREQUENCY_Hz;          /**< 制御周期[s] */
constexpr float CONFIG_POSITION_CONTROL_FREQUENCY_Hz = 50;                                       /**< 位置制御周波数[Hz] */
constexpr float CONFIG_POSITION_CONTROL_PERIOD_sec   = 1 / CONFIG_POSITION_CONTROL_FREQUENCY_Hz; /**< 位置制御周期[s] */

constexpr float CONFIG_S_TO_US = 1000000; /**< [s] → [us]変換 */

#endif /* CONFIG_H */