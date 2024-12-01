/* ====================================================================
 * Include files
 * ==================================================================== */
#include "AttitudeController.h"


/* ====================================================================
 * Public functions
 * ==================================================================== */
/**
 * @brief AttitudeControllerコンストラクタ
 */
AttitudeController::AttitudeController(float control_cycle)
{
    Tc_ = control_cycle;
}


/**
 * @brief PID制御計算処理
 * @param [in] R_ref 目標姿勢の回転行列
 * @param [in] R_cur 現在姿勢の回転行列
 */
void AttitudeController::calculatePID(Eigen::Matrix3f R_ref, Eigen::Matrix3f R_cur)
{
    Eigen::Matrix3f R_e;

    // 誤差回転行列の作成
    R_e = R_ref.transpose() * R_cur;

    // 誤差回転行列からZYXオイラー角で偏差角度を取り出す
    error_[0] = atan2(R_e(1, 2), R_e(2, 2)); // Roll
	error_[1] = asin(-R_e(0, 2));            // Pitch
	error_[2] = atan2(R_e(0, 1), R_e(0, 0)); // Yaw

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
 * @brief PID操作量取得処理
 * @return PID操作量
 */
std::vector<float> AttitudeController::getPID()
{
    return retPID_;
}


/**
 * @brief 偏差姿勢[rad]取得処理
 * @return 偏差姿勢[rad]
 */
std::vector<float> AttitudeController::getErrRPY_rad()
{
    return error_;	
}


/**
 * @brief 偏差姿勢[deg]取得処理
 * @return 偏差姿勢[deg]
 */
std::vector<float> AttitudeController::getErrRPY_deg()
{
    std::vector<float>  ret(3);

	ret[0] = error_[0] * RAD_TO_DEG;
	ret[1] = error_[1] * RAD_TO_DEG;
	ret[2] = error_[2] * RAD_TO_DEG;
	
    return ret;
}
