#ifndef ATTITUDEPLANNER_H
#define ATTITUDEPLANNER_H

/* ====================================================================
 * Include files
 * ==================================================================== */
// C++標準ライブラリ
#include <vector>
// Arduino標準ライブラリ
#include <Arduino.h>
// オープンソースライブラリ
#include "Eigen.h"
#include "Eigen/Geometry"
// 自作クラスの.hファイル
#include "SbusReceiver.h"

/* ====================================================================
 * Class definitions
 * ==================================================================== */
/**
 * @brief AttitudePlannerクラス
 * @par 説明
 * 以下の処理を提供する
 * - 目標姿勢の計算
 * - 目標姿勢回転行列の取得
 * - 目標オイラー角の取得
 */
class AttitudePlanner
{
   private:
    static constexpr float ROLL_ANGLE_MAX   = 20.0f;   /**< Roll軸の姿勢の最大値[deg] */
    static constexpr float ROLL_ANGLE_MIN   = -20.0f;  /**< Roll軸の姿勢の最大値[deg] */
    static constexpr float PITCH_ANGLE_MAX  = 20.0f;   /**< Pitch軸の姿勢の最大値[deg] */
    static constexpr float PITCH_ANGLE_MIN  = -20.0f;  /**< Pitch軸の姿勢の最大値[deg] */
    static constexpr float YAW_ANGLE_MAX    = 170.0f;  /**< Yaw軸の姿勢の最大値[deg] */
    static constexpr float YAW_ANGLE_MIN    = -170.0f; /**< Yaw軸の姿勢の最大値[deg] */
    static constexpr float YAW_ADD_MAX      = 0.05f;   /**< Yaw軸の姿勢加算値の最大値[deg] */
    static constexpr float YAW_ADD_MIN      = -0.05f;  /**< Yaw軸の姿勢加算値の最小値[deg] */
    static constexpr float YAW_ADD_DEADBAND = 0.01f;   /**< Yaw軸の姿勢加算値のデッドバンド[deg] */

    std::vector<float> refRPY_deg_     = {0.0f, 0.0f, 0.0f}; /**< 目標姿勢[deg] */
    std::vector<float> ref_offset_deg_ = {0.0f, 0.0f, 0.0f}; /**< 目標姿勢のオフセット値[deg] */

    float tilt_flight_target_angle_deg_    = 30.0f;   /**< チルト角度[deg] */
    float tilt_flight_update_frequency_Hz_ = 1000.0f; /**< 更新周波数[Hz] */
    float tilt_flight_transition_time_s_   = 5.0f;    /**< 遷移時間[s] */
    float tilt_flight_angle_step_deg_      = 0.0f;    /**< 角度量[deg] */

    float calculateTiltFlightAngleStep(float current_angle, float target_angle, float update_frequency, float transition_time);
    float updateTiltFlightReferenceAngle(float current_angle_deg, float target_angle_deg, float angle_step_deg);

   public:
    AttitudePlanner();
    void               SetTiltFlightTargetAngle(float tilt_flight_target_angle_deg);
    void               SetTiltFlightUpdateFrequency(float tilt_flight_update_frequency_Hz);
    void               SetTiltFlightTransitionTime(float tilt_flight_transition_time_s);
    void               calculateRefAttitude(T6L_Command command, std::vector<float> cur_rpy_deg);
    std::vector<float> getRefRPY_rad();
    std::vector<float> getRefRPY_deg();
    Eigen::Matrix3f    getRefRotMatrix();
    Eigen::Matrix3f    getRefRotMatrixWithoutOffset();
};

#endif /* ATTITUDEPLANNER_H */