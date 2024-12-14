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
   public:
    AttitudePlanner();
    void               calculateRefAttitude(T6L_Command command, std::vector<float> cur_rpy_deg);
    std::vector<float> getRefRPY_rad();
    std::vector<float> getRefRPY_deg();
    Eigen::Matrix3f    getRefRotMatrix();
    Eigen::Matrix3f    getRefRotMatrixWithoutOffset();
};

#endif /* ATTITUDEPLANNER_H */