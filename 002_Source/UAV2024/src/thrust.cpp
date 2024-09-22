/* ====================================================================
 * Include files
 * ==================================================================== */
#include "thrust.h"
// 自作クラスの.hファイル
#include "ThrustController.h"
#include "ThrustDriver.h"
// その他必要な.hファイル
#include "config.h"

/* ====================================================================
 * Constant definitions
 * ==================================================================== */

/* ====================================================================
 * Class declarations
 * ==================================================================== */
// 推力制御に関するクラス
static ThrustController thrustController[CONFIG_PROPELLER_NUM];
// ESC出力に関するクラス
static ThrustDriver thrustDriver[CONFIG_PROPELLER_NUM] =
    {
        ThrustDriver(PIN_ESC1_PWM),
        ThrustDriver(PIN_ESC2_PWM),
        ThrustDriver(PIN_ESC3_PWM),
        ThrustDriver(PIN_ESC4_PWM)};

/* ====================================================================
 * Public functions
 * ==================================================================== */
/**
 * @brief 初期化処理
 */
void thrust_Init()
{
    for (int i = 0; i < CONFIG_PROPELLER_NUM; i++)
    {
        thrustDriver[i].begin();
    }
}

/**
 * @brief 推力制御処理
 * @param [in] ref_thrust_N 目標推力[N]
 * @param [in] command プロポからの指令値
 * @return サーボの操作量[0 ~ 1]
 */
std::vector<float> thrust_Control(std::vector<float> ref_thrust_N, T6L_Command command)
{
    std::vector<float> ref_thrust(CONFIG_PROPELLER_NUM);
    std::vector<float> duty(CONFIG_PROPELLER_NUM);

    ref_thrust = ref_thrust_N;

    // 推力制御
    for (int i = 0; i < CONFIG_PROPELLER_NUM; i++)
    {
        thrustController[i].calculateFF(ref_thrust[i]);
        duty[i] = thrustController[i].getFF();
    }

    // スロットルが閾値を超えていない場合は、出力を0にする
    if (command.throttle < THROTTLE_THRESHOLD)
    {
        for (int i = 0; i < CONFIG_PROPELLER_NUM; i++)
        {
            duty[i] = 0;
        }
    }

    return duty;
}

/**
 * @brief 推力出力処理
 * @param [in] duty_norm サーボの操作量[0 ~ 1]
 */
void thrust_Output(std::vector<float> duty_norm)
{
    for (int i = 0; i < CONFIG_PROPELLER_NUM; i++)
    {
        thrustDriver[i].outputPWM(duty_norm[i]);
    }
}