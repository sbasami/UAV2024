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
    /* do nothing */
}

/**
 * @brief チルト飛行の目標角度[deg]設定処理
 * @param [in] tilt_flight_target_angle_deg 目標角度[deg]
 */
void AttitudePlanner::SetTiltFlightTargetAngle(float tilt_flight_target_angle_deg)
{
    tilt_flight_target_angle_deg_ = tilt_flight_target_angle_deg;
}

/**
 * @brief チルト飛行の更新周波数[Hz]設定処理
 * @param [in] tilt_flight_update_frequency_Hz 更新周波数[Hz]
 */
void AttitudePlanner::SetTiltFlightUpdateFrequency(float tilt_flight_update_frequency_Hz)
{
    tilt_flight_update_frequency_Hz_ = tilt_flight_update_frequency_Hz;
}

/**
 * @brief チルト飛行の遷移時間[s]設定処理
 * @param [in] tilt_flight_transition_time_s 遷移時間[s]
 */
void AttitudePlanner::SetTiltFlightTransitionTime(float tilt_flight_transition_time_s)
{
    tilt_flight_transition_time_s_ = tilt_flight_transition_time_s;
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

    // チルト飛行のための目標roll姿勢を計算
    tilt_flight_angle_step_deg_ = calculateTiltFlightAngleStep(ref_offset_deg_[0], tilt_flight_target_angle_deg_, tilt_flight_update_frequency_Hz_, tilt_flight_transition_time_s_);
    // スイッチが閾値を超えていた場合はチルト飛行のために目標roll姿勢を更新
    if (command.sw > SWITCH_THRESHOLD)
    {
        // チルト飛行
        ref_roll = updateTiltFlightReferenceAngle(refRPY_deg_[0], tilt_flight_target_angle_deg_, tilt_flight_angle_step_deg_);
    }
    else
    {
        // 水平飛行
        ref_roll = updateTiltFlightReferenceAngle(refRPY_deg_[0], ref_offset_deg_[0], tilt_flight_angle_step_deg_);
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

/**
 * @brief 目標姿勢回転行列(オフセットなし)取得処理
 * @return 目標姿勢の回転行列(オフセットなし)
 */
Eigen::Matrix3f AttitudePlanner::getRefRotMatrixWithoutOffset()
{
    Eigen::Matrix3f    R;
    std::vector<float> ref_rpy_rad(3);

    ref_rpy_rad[0] = (refRPY_deg_[0] - ref_offset_deg_[0]) * DEG_TO_RAD;
    ref_rpy_rad[1] = (refRPY_deg_[1] - ref_offset_deg_[1]) * DEG_TO_RAD;
    ref_rpy_rad[2] = (refRPY_deg_[2] - ref_offset_deg_[2]) * DEG_TO_RAD;

    // 目標角度から回転行列を作成
    R = Eigen::AngleAxisf(ref_rpy_rad[0], Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(ref_rpy_rad[1], Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(ref_rpy_rad[2], Eigen::Vector3f::UnitZ());

    return R;
}

/* ====================================================================
 * Private functions
 * ==================================================================== */
/**
 * @brief 角度を目標に向けてランプ状に更新する関数
 * @param [in] current_angle_deg 現在の角度
 * @param [in] target_angle_deg 目標とする角度
 * @param [in] angle_step_deg 1周期で変化させる角度量
 * @return 更新後の角度
 */
float AttitudePlanner::updateTiltFlightReferenceAngle(float current_angle_deg, float target_angle_deg, float angle_step_deg)
{
    float diff = target_angle_deg - current_angle_deg;

    // 角度を増減する
    if (diff > 0)
    {
        // 現在角度が目標より小さいので増加させる
        current_angle_deg += angle_step_deg;
        // 超えた場合は補正
        if (current_angle_deg > target_angle_deg)
        {
            current_angle_deg = target_angle_deg;
        }
    }
    else
    {
        // 現在角度が目標より大きいので減少させる
        current_angle_deg -= angle_step_deg;
        // 下回った場合は補正
        if (current_angle_deg < target_angle_deg)
        {
            current_angle_deg = target_angle_deg;
        }
    }

    return current_angle_deg;
}

/**
 * @brief チルト飛行のための角度量を計算する関数
 * @param [in] current_angle_deg 現在の角度[deg]
 * @param [in] target_angle_deg 目標の角度[deg]
 * @param [in] update_frequency_Hz 更新周波数[Hz]
 * @param [in] transition_time_s 遷移時間[s]
 * @return 1回の更新あたりに変化させる角度量[deg]
 */
float AttitudePlanner::calculateTiltFlightAngleStep(float current_angle_deg, float target_angle_deg, float update_frequency_Hz, float transition_time_s)
{
    float angle_step_deg = 0.0f;

    // 目標角度と現在角度の差を計算
    float delta = target_angle_deg - current_angle_deg;
    // 1秒あたりの更新回数から1回の更新あたりの角度量を計算
    angle_step_deg = delta / (update_frequency_Hz * transition_time_s);

    return angle_step_deg;
}