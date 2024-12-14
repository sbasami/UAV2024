#ifndef POSITIONCONTROLLER_H
#define POSITIONCONTROLLER_H

/* ====================================================================
 * Include files
 * ==================================================================== */
// C++標準ライブラリ
#include <vector>
// 自作クラスの.hファイル
#include "SbusReceiver.h"
// オープンソースライブラリ
#include "Eigen.h"
#include "Eigen/Geometry"

/* ====================================================================
 * Class definitions
 * ==================================================================== */
/**
 * @brief PositionControllerクラス
 * @par 説明
 * 以下の処理を提供する
 * - 位置制御に必要な操作量の計算(FF制御)
 * - FF制御結果の取得
 */
class PositionController
{
   private:
    static constexpr float FORCE_X_MAX_ = 1.5f;               /**< X軸制御で使える力の最大値[N] */
    static constexpr float FORCE_X_MIN_ = -1.5f;              /**< X軸制御で使える力の最小値[N] */
    static constexpr float FORCE_Y_MAX_ = 1.5f;               /**< Y軸制御で使える力の最大値[N] */
    static constexpr float FORCE_Y_MIN_ = -1.5f;              /**< Y軸制御で使える力の最小値[N] */
    static constexpr float FORCE_Z_MAX_ = 4.0f;               /**< Z軸制御で使える力の最大値[N] */
    static constexpr float FORCE_Z_MIN_ = 0.0f;               /**< Z軸制御で使える力の最小値[N] */
    static constexpr float KP_[3]       = {0.0f, 0.0f, 1.0f}; /**< PID制御のPゲイン */
    static constexpr float KI_[3]       = {0.0f, 0.0f, 0.0f}; /**< PID制御のIゲイン */
    static constexpr float KD_[3]       = {0.0f, 0.0f, 0.1f}; /**< PID制御のDゲイン */

    float              Tc_;                            /**< 制御周期[s] */
    std::vector<float> error_    = {0.0f, 0.0f, 0.0f}; /**< 偏差[rad] */
    std::vector<float> errorOld_ = {0.0f, 0.0f, 0.0f}; /**< D制御用の偏差保持[rad] */
    std::vector<float> integral_ = {0.0f, 0.0f, 0.0f}; /**< I制御用の積分値[rad] */
    std::vector<float> retPID_   = {0.0f, 0.0f, 0.0f}; /**< PID操作量 */
    std::vector<float> retFF_    = {0.0f, 0.0f, 0.0f}; /**< FF操作量 */

   public:
    PositionController(float control_cycle);
    void               calculateFF(T6L_Command command, Eigen::Matrix3f R_ref);
    void               calculatePID(std::vector<float> ref_xyz_m, std::vector<float> cur_xyz_m);
    std::vector<float> getFF();
    std::vector<float> getPID();
};

#endif /* POSITIONCONTROLLER_H */