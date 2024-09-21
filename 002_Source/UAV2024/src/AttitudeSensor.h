#ifndef ATTITUDESENSOR_H
#define ATTITUDESENSOR_H

/* ====================================================================
 * Include files
 * ==================================================================== */
// C++標準ライブラリ
#include <vector>
// オープンソースライブラリ
#include "BMI088.h"
#include "MadgwickAHRS.h"
#include "Eigen.h"
#include "Eigen/Geometry"

/* ====================================================================
 * Class definitions
 * ==================================================================== */
/**
 * @brief AttitudeSensorクラス
 * @par 説明
 * 以下の処理を提供する
 * - 姿勢センサの初期化
 * - 姿勢データの更新
 * - 姿勢データの取得
 * - 現在姿勢の取得
 */
class AttitudeSensor
{
   private:
    Bmi088Accel accel_;
    Bmi088Gyro  gyro_;
    Madgwick    madgwick_;

   public:
    AttitudeSensor(SPIClass &bus, uint8_t cs_pin_accel, uint8_t cs_pin_gyro);
    void               begin(float sample_frequency_Hz);
    void               update();
    std::vector<float> getAccel_mss();
    std::vector<float> getGlobalAccel_mss();
    std::vector<float> getGyro_rads();
    std::vector<float> getGyro_degs();
    std::vector<float> getRPY_rad();
    std::vector<float> getRPY_deg();
    Eigen::Quaternionf getQuaternion();
    Eigen::Matrix3f    getRotationMatrix();
};

#endif /* ATTITUDESENSOR_H */