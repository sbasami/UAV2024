#ifndef ATTITUDECONTROLLER_H
#define ATTITUDECONTROLLER_H

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


/* ====================================================================
 * Class definitions
 * ==================================================================== */
/**
 * @brief AttitudeControllerクラス
 * @par 説明
 * 以下の処理を提供する
 * - PID制御の計算
 * - PID制御計算結果の取得
 * - 偏差の取得
 */
class AttitudeController
{
private:
    static constexpr float KP_[3] = {1.0f, 1.0f, 1.0f}; /**< PID制御のPゲイン */
    static constexpr float KI_[3] = {0.0f, 0.0f, 0.0f};  /**< PID制御のIゲイン */
    static constexpr float KD_[3] = {0.1f, 0.1f, 0.1f};  /**< PID制御のDゲイン */
    
    std::vector<float> error_    = {0.0f, 0.0f, 0.0f};  /**< 偏差[rad] */
    std::vector<float> errorOld_ = {0.0f, 0.0f, 0.0f};  /**< D制御用の偏差保持[rad] */
    std::vector<float> integral_ = {0.0f, 0.0f, 0.0f};  /**< I制御用の積分値[rad] */
    std::vector<float> retPID_   = {0.0f, 0.0f, 0.0f};  /**< PID操作量 */
    float Tc_;                                          /**< 制御周期[s] */
public:
    AttitudeController(float control_cycle);
    void calculatePID(Eigen::Matrix3f ref_R, Eigen::Matrix3f cur_R);
    std::vector<float> getPID();
    std::vector<float> getErrRPY_rad();
    std::vector<float> getErrRPY_deg();
};

#endif /* ATTITUDECONTROLLER_H */