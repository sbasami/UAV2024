/* ====================================================================
 * Include files
 * ==================================================================== */
#include "ServoController.h"

/* ====================================================================
 * Public functions
 * ==================================================================== */
/**
 * @brief ServoControllerコンストラクタ
 */
ServoController::ServoController(float control_cycle_s)
{
    // メンバ変数の初期化
    Tc_ = control_cycle_s;
}

/**
 * @brief I-PD制御計算処理
 * @param [in] ref_angle_rad 目標角度[rad]
 * @param [in] act_angle_rad 現在角度[rad]
 * @param [in] bat_volt_v モータ電源電圧[V]
 */
void ServoController::calculateIPD(float ref_angle_rad, float act_angle_rad, float bat_volt_v)
{
    float r;  // 目標値[rad]
    float y;  // 制御対象の出力[rad]
    float u;  // 制御対象の入力[V]
    float e;  // 偏差[rad]

    r = ref_angle_rad;
    y = act_angle_rad;

    e = r - y;
    // エンコーダ分解能以下の偏差は0にする
    if (abs(e) < RESOLUTION_RAD_)
    {
        e = 0;
    }

    // I-PD制御
    integral_ += e * Tc_;
    u     = -KP_ * y + KI_ * integral_ - KD_ * (y - yOld_) / Tc_;
    yOld_ = y;

    // 入力制限とアンチワインドアップ制御
    batVolt_v_ = bat_volt_v;
    if (u > batVolt_v_)
    {
        u = batVolt_v_;        // 入力制限
        integral_ -= e * Tc_;  // アンチワインドアップ制御
    }
    else if (u < -batVolt_v_)
    {
        u = -batVolt_v_;       // 入力制限
        integral_ -= e * Tc_;  // アンチワインドアップ制御
    }

    // 計算結果をメンバ変数へ渡す
    retIPD_ = u;
}

/**
 * @brief I-PD制御積分項リセット処理
 */
void ServoController::resetIntegral()
{
    integral_ = 0;
}

/**
 * @brief I-PD操作量[V]取得処理
 * @return I-PD操作量[V]
 */
float ServoController::getIPD_V()
{
    return retIPD_;
}

/**
 * @brief I-PD操作量[-1 ~ 1]取得処理
 * @return I-PD操作量[-1 ~ 1]
 */
float ServoController::getIPD_duty()
{
    return (retIPD_ / batVolt_v_);
}
