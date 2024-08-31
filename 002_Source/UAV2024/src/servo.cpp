/* ====================================================================
 * Include files
 * ==================================================================== */
#include "servo.h"
// 自作クラスの.hファイル
#include "ServoEncoder.h"
#include "ServoController.h"
#include "ServoDriver.h"
// その他必要な.hファイル
#include "config.h"

/* ====================================================================
 * constant definitions
 * ==================================================================== */
static constexpr int SERVO_NUM = CONFIG_SERVO_NUM; /**< サーボモータの数 */

/* ====================================================================
 * Class declarations
 * ==================================================================== */
// エンコーダに関するクラス
static ServoEncoder servoEncoder[SERVO_NUM] =
    {ServoEncoder(CONFIG_PIN_SERVO1_ENC_A, CONFIG_PIN_SERVO1_ENC_B),
     ServoEncoder(CONFIG_PIN_SERVO2_ENC_A, CONFIG_PIN_SERVO2_ENC_B),
     ServoEncoder(CONFIG_PIN_SERVO3_ENC_A, CONFIG_PIN_SERVO3_ENC_B),
     ServoEncoder(CONFIG_PIN_SERVO4_ENC_A, CONFIG_PIN_SERVO4_ENC_B)};
// サーボ制御に関するクラス
static ServoController servoController[SERVO_NUM] =
    {ServoController(CONFIG_CONTROL_CYCLE_sec),
     ServoController(CONFIG_CONTROL_CYCLE_sec),
     ServoController(CONFIG_CONTROL_CYCLE_sec),
     ServoController(CONFIG_CONTROL_CYCLE_sec)};
// モータ出力に関するクラス
static ServoDriver servoMotor[SERVO_NUM] =
    {ServoDriver(CONFIG_PIN_SERVO1_PWM, CONFIG_PIN_SERVO1_DIR),
     ServoDriver(CONFIG_PIN_SERVO2_PWM, CONFIG_PIN_SERVO2_DIR),
     ServoDriver(CONFIG_PIN_SERVO3_PWM, CONFIG_PIN_SERVO3_DIR),
     ServoDriver(CONFIG_PIN_SERVO4_PWM, CONFIG_PIN_SERVO4_DIR)};

/* ====================================================================
 * Prototype declarations
 * ==================================================================== */

/* ====================================================================
 * Public functions
 * ==================================================================== */
/**
 * @brief 初期化処理
 */
void servo_Init()
{
    for (int i = 0; i < SERVO_NUM; i++)
    {
        servoMotor[i].begin();
    }
}

/**
 * @brief 現在サーボ角度取得処理
 * @return 現在サーボ角度[rad]
 */
std::vector<float> servo_ReadSensor()
{
    std::vector<float> act_angle(SERVO_NUM);

    // 現在角度の取得
    for (int i = 0; i < SERVO_NUM; i++)
    {
        servoEncoder[i].readSensor();
        act_angle[i] = servoEncoder[i].getAngle_rad();
    }

    return act_angle;
}

/**
 * @brief サーボ制御処理
 * @param [in] ref_angle_rad 目標サーボ角度[rad]
 * @param [in] act_angle_rad 現在サーボ角度[rad]
 * @return サーボの操作量[-1 ~ 1]
 */
std::vector<float> servo_Control(std::vector<float> ref_angle_rad, std::vector<float> act_angle_rad)
{
    std::vector<float> ref_angle(SERVO_NUM);
    std::vector<float> act_angle(SERVO_NUM);
    std::vector<float> duty(SERVO_NUM);

    ref_angle = ref_angle_rad;
    act_angle = act_angle_rad;

    // 角度制御
    for (int i = 0; i < SERVO_NUM; i++)
    {
        servoController[i].calculateIPD(ref_angle[i], act_angle[i], 7.4);
        duty[i] = servoController[i].getIPD_duty();
    }

    return duty;
}

/**
 * @brief サーボ出力処理
 * @param [in] duty_norm サーボの操作量[-1 ~ 1]
 */
void servo_Output(std::vector<float> duty_norm)
{
    for (int i = 0; i < SERVO_NUM; i++)
    {
        servoMotor[i].outputPWM(duty_norm[i]);
    }
}

/* ====================================================================
 * static functions
 * ==================================================================== */
