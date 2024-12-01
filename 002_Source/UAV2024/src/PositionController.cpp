/* ====================================================================
 * Include files
 * ==================================================================== */
#include "PositionController.h"

/* ====================================================================
 * Public functions
 * ==================================================================== */
/**
 * @brief PositionControllerコンストラクタ関数
 */
PositionController::PositionController(float control_cycle)
{
    // メンバ変数の初期化
    Tc_ = control_cycle;
}

/**
 * @brief FF制御計算関数
 * @param [in] command プロポからの指令値
 */
void PositionController::calculateFF(T6L_Command command)
{
    // プロポの信号を正規化したものを位置の制御出力とする
    retFF_[0] = command.elevator * (FORCE_X_MAX_ - FORCE_X_MIN_) + FORCE_X_MIN_;
    retFF_[1] = command.aileron * (FORCE_Y_MAX_ - FORCE_Y_MIN_) + FORCE_Y_MIN_;
    retFF_[2] = command.throttle * (FORCE_Z_MAX_ - FORCE_Z_MIN_) + FORCE_Z_MIN_;
}

/**
 * @brief PID制御計算関数
 */
void PositionController::calculatePID(std::vector<float> ref_xyz_m, std::vector<float> cur_xyz_m)
{
    error_[0] = ref_xyz_m[0] - cur_xyz_m[0];  // x
    error_[1] = ref_xyz_m[1] - cur_xyz_m[1];  // y
    error_[2] = ref_xyz_m[2] - cur_xyz_m[2];  // z

    // 積分器の更新
    integral_[0] += error_[0] * Tc_;
    integral_[1] += error_[1] * Tc_;
    integral_[2] += error_[2] * Tc_;

    // PID制御
    retPID_[0] = KP_[0] * error_[0] + KI_[0] * integral_[0] + KD_[0] * (error_[0] - errorOld_[0]) / Tc_;
    retPID_[1] = KP_[1] * error_[1] + KI_[1] * integral_[1] + KD_[1] * (error_[1] - errorOld_[1]) / Tc_;
    retPID_[2] = KP_[2] * error_[2] + KI_[2] * integral_[2] + KD_[2] * (error_[2] - errorOld_[2]) / Tc_;

    // D制御用に偏差を保持
    errorOld_[0] = error_[0];
    errorOld_[1] = error_[1];
    errorOld_[2] = error_[2];
}

/**
 * @brief FF操作量取得処理
 * @return FF操作量
 */
std::vector<float> PositionController::getFF()
{
    return retFF_;
}

/**
 * @brief PID操作量取得処理
 * @return PID操作量
 */
std::vector<float> PositionController::getPID()
{
    return retPID_;
}