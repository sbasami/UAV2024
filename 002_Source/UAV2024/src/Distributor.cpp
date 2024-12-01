/* ====================================================================
 * Include files
 * ==================================================================== */
#include "Distributor.h"

/* ====================================================================
 * Public functions
 * ==================================================================== */
/**
 * @brief Distributorコンストラクタ
 */
Distributor::Distributor()
{
    // Do nothing
}

/**
 * @brief 推力分配計算関数(tilt)
 * @param [in] F_attitude_N 各軸の姿勢制御に必要な力[N]
 * @param [in] F_position_N 各軸の位置制御に必要な力[N]
 */
void Distributor::calculateThrustAndServoAngle(std::vector<float> F_attitude_N, std::vector<float> F_position_N)
{
    float              F_roll_4, F_pitch_4, F_yaw_4;
    float              F_x_4, F_y_4, F_z_4;
    std::vector<float> Tv(4);
    std::vector<float> Th(4);

    // モータが4つあるので4分の1にする
    F_roll_4  = F_attitude_N[0] / 4;
    F_pitch_4 = F_attitude_N[1] / 4;
    F_yaw_4   = F_attitude_N[2] / 4;
    F_x_4     = F_position_N[0] / 4;
    F_y_4     = F_position_N[1] / 4;
    F_z_4     = F_position_N[2] / 4;

    // プロペラ座標系垂直方向の分配
    Tv[0] = F_z_4 - F_roll_4 - F_pitch_4;
    Tv[1] = F_z_4 + F_roll_4 - F_pitch_4;
    Tv[2] = F_z_4 + F_roll_4 + F_pitch_4;
    Tv[3] = F_z_4 - F_roll_4 + F_pitch_4;

    // プロペラ座標系水平方向の分配
    Th[0] = -F_x_4 / sin(PI / 4) + F_y_4 / sin(PI / 4) - F_yaw_4;
    Th[1] = F_x_4 / sin(PI / 4) + F_y_4 / sin(PI / 4) - F_yaw_4;
    Th[2] = F_x_4 / sin(PI / 4) - F_y_4 / sin(PI / 4) - F_yaw_4;
    Th[3] = -F_x_4 / sin(PI / 4) - F_y_4 / sin(PI / 4) - F_yaw_4;

    // 推力の計算[N]
    thrust_[0] = sqrt(Tv[0] * Tv[0] + Th[0] * Th[0]);
    thrust_[1] = sqrt(Tv[1] * Tv[1] + Th[1] * Th[1]);
    thrust_[2] = sqrt(Tv[2] * Tv[2] + Th[2] * Th[2]);
    thrust_[3] = sqrt(Tv[3] * Tv[3] + Th[3] * Th[3]);

    // サーボ角の計算[deg]
    servo_angle_[0] = atan2(Th[0], Tv[0]);
    servo_angle_[1] = atan2(Th[1], Tv[1]);
    servo_angle_[2] = atan2(Th[2], Tv[2]);
    servo_angle_[3] = atan2(Th[3], Tv[3]);

    // for debug
    // Serial.print(F_attitude_N[0]);
    // Serial.print("\t");
    // Serial.print(F_attitude_N[1]);
    // Serial.print("\t");
    // Serial.print(F_attitude_N[2]);
    // Serial.print("\t");
    // Serial.print(F_position_N[0]);
    // Serial.print("\t");
    // Serial.print(F_position_N[1]);
    // Serial.print("\t");
    // Serial.print(F_position_N[2]);
    // Serial.print("\n");
}

/**
 * @brief 推力操作量取得処理
 * @return 推力操作量
 */
std::vector<float> Distributor::getThrust()
{
    return thrust_;
}

/**
 * @brief サーボ角度操作量取得処理
 * @return サーボ角度操作量
 */
std::vector<float> Distributor::getServoAngle()
{
    return servo_angle_;
}