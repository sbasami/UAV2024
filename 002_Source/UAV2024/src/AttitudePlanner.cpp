/* ====================================================================
 * Include files
 * ==================================================================== */
#include "AttitudePlanner.h"

/* ====================================================================
 * Public functions
 * ==================================================================== */
/**
 * @brief AttitudePlannerコンストラクタ
 */
AttitudePlanner::AttitudePlanner()
{
    // Do nothing
}

/**
 * @brief 目標姿勢計算処理
 * @param [in] command プロポからの指令値
 * @param [in] cur_rpy_deg 現在姿勢[deg]
 */
void AttitudePlanner::calculateRefAttitude(T6L_Command command, std::vector<float> cur_rpy_deg)
{
    float ref_roll, ref_pitch, ref_yaw;

    // スロットルが閾値を超えるまで現在のroll, pitchを目標姿勢にする
    // 姿勢センサのデータにオフセットがある場合の対策
    if (command.throttle < THROTTLE_THRESHOLD)
    {
        ref_offset_deg_[0] = cur_rpy_deg[0];
        ref_offset_deg_[1] = cur_rpy_deg[1];
    }

    // 目標姿勢の設定
    ref_roll  = ref_offset_deg_[0];
    ref_pitch = ref_offset_deg_[1];
    // プロポの信号をYAW_ADD_MIN     ~ YAW_ADD_MAXの範囲で正規化
    ref_yaw = command.rudder * (YAW_ADD_MAX - YAW_ADD_MIN) + YAW_ADD_MIN;

    // スイッチが閾値を超えていた場合はチルト飛行のために目標roll姿勢を更新
    if (command.sw > SWITCH_THRESHOLD)
    {
        ref_roll = ref_roll + 15;
    }

    // プロポから送信される値はAD値なので操作していなくても値が僅かにブレて意図せず加算される可能性がある
    // よって、デッドバンドを設け範囲内ならば値を0にする
    if ((ref_yaw < YAW_ADD_DEADBAND) && (ref_yaw > -YAW_ADD_DEADBAND)) ref_yaw = 0;
    // Yaw軸に関してはYAW_ANGLE_MAX ~ YAW_ANGLE_MINの範囲で制限
    if (ref_yaw > YAW_ANGLE_MAX)
    {
        ref_yaw = YAW_ANGLE_MAX;
    }
    else if (ref_yaw < YAW_ANGLE_MIN)
    {
        ref_yaw = YAW_ANGLE_MIN;
    }
    else
    {
        // do nothing
    }
    // メンバ変数に値を代入
    refRPY_deg_[0] = ref_roll;
    refRPY_deg_[1] = ref_pitch;
    refRPY_deg_[2] += ref_yaw;

    // for debug
    // Serial.print(refRPY_deg_[0]);
    // Serial.print("\t");
    // Serial.print(refRPY_deg_[1]);
    // Serial.print("\t");
    // Serial.print(refRPY_deg_[2]);
    // Serial.print("\n");
}

/**
 * @brief 目標姿勢[rad]取得処理
 * @return 目標姿勢[rad]
 */
std::vector<float> AttitudePlanner::getRefRPY_rad()
{
    std::vector<float> ret(3);

    ret[0] = refRPY_deg_[0] * DEG_TO_RAD;
    ret[1] = refRPY_deg_[1] * DEG_TO_RAD;
    ret[2] = refRPY_deg_[2] * DEG_TO_RAD;

    return ret;
}

/**
 * @brief 目標姿勢[deg]取得処理
 * @return 目標姿勢[deg]
 */
std::vector<float> AttitudePlanner::getRefRPY_deg()
{
    return refRPY_deg_;
}

/**
 * @brief 目標姿勢回転行列取得処理
 * @return 目標姿勢の回転行列
 */
Eigen::Matrix3f AttitudePlanner::getRefRotMatrix()
{
    Eigen::Matrix3f    R;
    std::vector<float> ref_rpy_rad(3);

    ref_rpy_rad = getRefRPY_rad();

    // 目標角度から回転行列を作成
    R = Eigen::AngleAxisf(ref_rpy_rad[0], Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(ref_rpy_rad[1], Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(ref_rpy_rad[2], Eigen::Vector3f::UnitZ());

    return R;
}
