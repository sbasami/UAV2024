#ifndef DISTRIBUTER_H
#define DISTRIBUTER_H

/* ====================================================================
 * Include files
 * ==================================================================== */
#include <Arduino.h>
// C++標準ライブラリ
#include <vector>

/* ====================================================================
 * Class definitions
 * ==================================================================== */
/**
 * @brief Distributorクラス
 * @par 説明
 * 以下の処理を提供する
 * - 各モータへの推力分配の計算
 * - 各モータ推力の取得
 */
class Distributor
{
   private:
    std::vector<float> thrust_      = {0.0f, 0.0f, 0.0f, 0.0f}; /**< 推力操作量 */
    std::vector<float> servo_angle_ = {0.0f, 0.0f, 0.0f, 0.0f}; /**< サーボ角度操作量 */
   public:
    Distributor();
    void               calculateThrustAndServoAngle(std::vector<float> F_attitude_N, std::vector<float> F_position_N);
    std::vector<float> getThrust();
    std::vector<float> getServoAngle();
};

#endif /* DISTRIBUTER_H */